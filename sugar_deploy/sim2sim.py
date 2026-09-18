"""SUGAR Command Generator + Command Tracker 的 MuJoCo sim2sim 主循环。

时序（都是从 SUGAR 源码核实过的常量，见 contract.py）：
    物理:      200 Hz (PHYSICS_DT=0.005s)
    Tracker:   50 Hz  (CONTROL_DECIMATION=4 个物理步一次)
    Generator: 2.5 Hz (每 GENERATOR_CALL_INTERVAL=20 个 tracker 步一次，约 0.4s)

物体状态从 ObjectStateSource 拿（sim2sim 自验证用 MuJoCo ground truth，真机换成
MoCap，见 object_state.py）。
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path

import mujoco
import numpy as np
import torch

from sugar_deploy import contract
from sugar_deploy.object_state import MujocoGroundTruthSource, ObjectStateSource
from sugar_deploy.observation import (
    CommandBuffer, RobotState, TrackerObsBuilder, build_generator_obs,
)
from sugar_deploy.policy import GeneratorPolicy, TrackerPolicy

DEFAULT_SCENE = Path(__file__).resolve().parent.parent / "assets" / "g1" / "carrybox_scene.xml"


def _quat_wxyz(mj_quat: np.ndarray) -> np.ndarray:
    """MuJoCo 的 xquat 已经是 wxyz，这里只是显式标注含义，不做转换。"""
    return mj_quat


@dataclass
class SimStats:
    control_steps: int = 0
    generator_calls: int = 0
    mean_tracker_step_ms: float = 0.0


@dataclass
class SugarSim2Sim:
    scene_path: Path
    tracker: TrackerPolicy
    generator: GeneratorPolicy
    task: contract.TaskContract
    device: str = "cpu"
    target_offset_w: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0]))
    """use_target=True 时，物体目标位置 = 初始物体位置 + 这个偏移（世界系）。
    第一版没有真实任务目标位置来源，先用一个占位偏移，行为上大概率不对，
    只用于验证"策略在跑、command chunk 会响应目标变化"，不代表复现论文精度。"""

    model: mujoco.MjModel = field(init=False)
    data: mujoco.MjData = field(init=False)
    object_source: ObjectStateSource = field(init=False)

    def __post_init__(self) -> None:
        self.model = mujoco.MjModel.from_xml_path(str(self.scene_path))
        self.model.opt.timestep = contract.PHYSICS_DT
        # SUGAR 训练时用的是 PhysX 的 ImplicitActuatorCfg——PD 是隐式求解的，
        # 对增益大小不敏感。这里用普通 <motor> 扭矩电机 + 手算 PD，
        # 配 MuJoCo 默认的显式 Euler 积分器在这组偏硬的增益（kp 最高到 99）下
        # 200Hz 物理频率直接会炸（实测过：qacc 一步之内从 9.8 飙到 2.7万+）。
        # 换成 implicitfast 积分器（MuJoCo 内部对关节阻尼项做隐式处理）解决。
        self.model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        self.data = mujoco.MjData(self.model)
        self.object_source = MujocoGroundTruthSource(self.model, self.data, self.task.object_body_name)

        self.joint_qpos_adr = np.array([
            self.model.joint(name).qposadr[0] for name in contract.JOINT_NAMES
        ])
        self.joint_qvel_adr = np.array([
            self.model.joint(name).dofadr[0] for name in contract.JOINT_NAMES
        ])
        # MJCF 默认 armature=0，SUGAR 训练时电机转子惯量（armature）是非零的
        # （见 contract.py，来自 unitree.py 的 ImplicitActuatorCfg），补上去，
        # 否则关节的等效惯量和训练时不一致，动力学响应会偏"轻飘"。
        self.model.dof_armature[self.joint_qvel_adr] = np.array(contract.JOINT_ARMATURE)
        self.torso_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, contract.ANCHOR_BODY_NAME)
        self.pelvis_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, contract.BASE_BODY_NAME)
        if self.torso_body_id < 0 or self.pelvis_body_id < 0:
            raise ValueError(
                f"MuJoCo 模型里找不到 {contract.ANCHOR_BODY_NAME}/{contract.BASE_BODY_NAME}，"
                "检查 assets/g1/g1_29dof.xml 的 body 命名"
            )

        self.obs_builder = TrackerObsBuilder()
        self.command_buffer = CommandBuffer()
        self.last_raw_action = np.zeros(contract.NUM_JOINTS, dtype=np.float32)
        self.time_steps = 0
        self.stats = SimStats()
        self._target_pos_w: np.ndarray | None = None
        self._target_quat_w = np.array([1.0, 0.0, 0.0, 0.0])

    # ------------------------------------------------------------------
    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        pelvis_qpos_adr = self.model.joint("floating_base_joint").qposadr[0]
        self.data.qpos[pelvis_qpos_adr : pelvis_qpos_adr + 3] = [0.0, 0.0, contract.DEFAULT_ROOT_HEIGHT]
        self.data.qpos[pelvis_qpos_adr + 3 : pelvis_qpos_adr + 7] = [1.0, 0.0, 0.0, 0.0]
        for adr, q in zip(self.joint_qpos_adr, contract.DEFAULT_JOINT_POS):
            self.data.qpos[adr] = q
        mujoco.mj_forward(self.model, self.data)

        self.last_raw_action[:] = 0.0
        self.command_buffer = CommandBuffer()
        self.time_steps = 0
        self.stats = SimStats()

        robot = self._read_robot_state()
        self.obs_builder.reset(robot)

        obj_pose = self.object_source.get_pose()
        self._target_pos_w = obj_pose.pos_w + self.target_offset_w
        self._target_quat_w = np.array([1.0, 0.0, 0.0, 0.0])

    # ------------------------------------------------------------------
    def _read_robot_state(self) -> RobotState:
        d = self.data
        joint_pos = d.qpos[self.joint_qpos_adr].copy()
        joint_vel = d.qvel[self.joint_qvel_adr].copy()
        base_quat_w = _quat_wxyz(d.xquat[self.pelvis_body_id].copy())
        vel6 = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, d, mujoco.mjtObj.mjOBJ_BODY, self.pelvis_body_id, vel6, 0)
        base_ang_vel_w = vel6[:3].copy()
        anchor_pos_w = d.xpos[self.torso_body_id].copy()
        anchor_quat_w = _quat_wxyz(d.xquat[self.torso_body_id].copy())
        return RobotState(
            joint_pos=joint_pos, joint_vel=joint_vel,
            base_quat_w=base_quat_w, base_ang_vel_w=base_ang_vel_w,
            anchor_pos_w=anchor_pos_w, anchor_quat_w=anchor_quat_w,
        )

    def _apply_pd(self, q_target: np.ndarray) -> None:
        d = self.data
        q = d.qpos[self.joint_qpos_adr]
        qd = d.qvel[self.joint_qvel_adr]
        kp = np.array(contract.JOINT_STIFFNESS)
        kd = np.array(contract.JOINT_DAMPING)
        effort = np.array(contract.JOINT_EFFORT_LIMIT)
        torque = kp * (q_target - q) - kd * qd
        torque = np.clip(torque, -effort, effort)
        d.ctrl[:] = torque

    def _maybe_call_generator(self, robot: RobotState) -> None:
        if not self.command_buffer.should_call_generator(self.time_steps):
            return
        obj_pose = self.object_source.get_pose()
        from sugar_deploy.observation import rotmat_to_quat_wxyz

        obj_quat_w = rotmat_to_quat_wxyz(obj_pose.rot_w)
        gen_obs = build_generator_obs(
            robot, obj_pose.pos_w, obj_quat_w,
            last_command_36=self.command_buffer.current(),
            use_target=self.generator.use_target, use_last_action=self.generator.use_last_action,
            target_obj_pos_w=self._target_pos_w if self.generator.use_target else None,
            target_obj_quat_w=self._target_quat_w if self.generator.use_target else None,
        )
        chunk = self.generator.predict(gen_obs)  # (1, 36, 36)
        self.command_buffer.set_chunk(chunk[0].cpu().numpy())
        self.stats.generator_calls += 1

    def control_step(self) -> None:
        robot = self._read_robot_state()
        self._maybe_call_generator(robot)
        command = self.command_buffer.current()
        self.command_buffer.advance()

        obj_pose = self.object_source.get_pose()
        from sugar_deploy.observation import rotmat_to_quat_wxyz

        obj_quat_w = rotmat_to_quat_wxyz(obj_pose.rot_w)
        obs_510 = self.obs_builder.build(robot, obj_pose.pos_w, obj_quat_w, command)

        t0 = time.perf_counter()
        raw_action = self.tracker.act(torch.from_numpy(obs_510).float()).cpu().numpy()[0]
        self.stats.mean_tracker_step_ms += (time.perf_counter() - t0) * 1000.0

        q_target = np.array(contract.DEFAULT_JOINT_POS) + np.array(contract.JOINT_ACTION_SCALE) * raw_action

        for _ in range(contract.CONTROL_DECIMATION):
            self._apply_pd(q_target)
            mujoco.mj_step(self.model, self.data)

        self.obs_builder.step(self._read_robot_state(), last_action=raw_action)
        self.last_raw_action = raw_action
        self.time_steps += 1
        self.stats.control_steps += 1

    # ------------------------------------------------------------------
    def run(self, control_steps: int, headless: bool = True, real_time: bool = False) -> SimStats:
        self.reset()
        if headless:
            for _ in range(control_steps):
                self.control_step()
        else:
            import mujoco.viewer

            with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                for _ in range(control_steps):
                    step_start = time.perf_counter()
                    self.control_step()
                    viewer.sync()
                    if real_time:
                        dt = contract.CONTROL_DT - (time.perf_counter() - step_start)
                        if dt > 0:
                            time.sleep(dt)
                    if not viewer.is_running():
                        break
        if self.stats.control_steps > 0:
            self.stats.mean_tracker_step_ms /= self.stats.control_steps
        return self.stats
