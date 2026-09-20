"""SUGAR Command Generator + Command Tracker 的 MuJoCo sim2sim 主循环。

时序（都是从 SUGAR 源码核实过的常量，见 contract.py）：
    物理:      200 Hz (PHYSICS_DT=0.005s)
    Tracker:   50 Hz  (CONTROL_DECIMATION=4 个物理步一次)
    Generator: 2.5 Hz (每 GENERATOR_CALL_INTERVAL=20 个 tracker 步一次，约 0.4s)

物体状态从 ObjectStateSource 拿（sim2sim 自验证用 MuJoCo ground truth，真机换成
MoCap，见 object_state.py）。
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

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
    object_source_override: ObjectStateSource | None = None
    """不传就用 __post_init__ 里默认建的 MujocoGroundTruthSource（sim2sim 自验证用）；
    传了就用调用方给的这个实例——用来在不碰这个类主体逻辑的前提下换成
    AprilTagObjectSource/MocapObjectSource 等真实感知方案，见 scripts/run_sim2sim.py
    的 --object-source 开关。"""
    pre_step_hook: Callable[[], None] | None = None
    """每个控制步最开始（读 obj pose 之前）会调用一次的可选回调，不接受参数、不要返回值。
    设计出来专门给"用 AprilTag 视觉感知"这种需要每步先做一次'相机现在在哪+检测更新'的
    object_source 用（见 run_sim2sim.py），MujocoGroundTruthSource/MocapObjectSource
    不需要这个，留 None 就行。"""

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
        self.object_source = self.object_source_override or MujocoGroundTruthSource(
            self.model, self.data, self.task.object_body_name
        )

        self.joint_qpos_adr = np.array([
            self.model.joint(name).qposadr[0] for name in contract.JOINT_NAMES
        ])
        self.joint_qvel_adr = np.array([
            self.model.joint(name).dofadr[0] for name in contract.JOINT_NAMES
        ])
        # 关键：按名字查 actuator id，不能假设 contract.JOINT_NAMES 的顺序和
        # g1_29dof.xml 里 <actuator> 块的声明顺序一样——已经不一样了（MJCF 是
        # 文件声明顺序，JOINT_NAMES 现在是 IsaacLab 运行时实测的真实顺序，两者
        # 不重合）。之前 `d.ctrl[:] = torque` 直接按位置写是错的，扭矩会发到错误
        # 的关节上，这很可能是之前站不稳的真正原因。
        self.actuator_ids = np.array([
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in contract.JOINT_NAMES
        ])
        if (self.actuator_ids < 0).any():
            missing = [n for n, i in zip(contract.JOINT_NAMES, self.actuator_ids) if i < 0]
            raise ValueError(f"MuJoCo 模型里找不到这些关节对应的 actuator: {missing}")
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
        self._generator_thread: threading.Thread | None = None

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
        self._generator_thread = None

        robot = self._read_robot_state()
        self.obs_builder.reset(robot)

        # AprilTagObjectSource 这类"主动检测型" object_source 在第一次 get_pose() 之前
        # 必须先跑过至少一次 update()，否则会直接抛异常（见 object_state.py）——
        # pre_step_hook 就是干这个的，reset() 里也要调一次，不能只在 control_step() 里调。
        if self.pre_step_hook is not None:
            self.pre_step_hook()
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
        d.ctrl[self.actuator_ids] = torque

    def _maybe_call_generator(self, robot: RobotState) -> None:
        """异步调用 Generator，不阻塞 50Hz 主循环。

        实测过同步调用的代价：CPU 上单次 ~120~160ms，GPU 上 ~60ms，都远超 20ms 的
        单步控制预算——同步调用会让机器人每 0.4s（GENERATOR_CALL_INTERVAL 步）卡一下，
        这不只是 sim2sim 可视化不流畅的问题，真机部署会是真的"机器人每 0.4 秒僵一下"。

        Command chunk 本身设计上就有 36 步的余量（只消费前 20 步，见 CommandBuffer），
        足够覆盖异步计算的延迟：后台线程算新 chunk 的这段时间里，主循环继续消费旧
        chunk 剩下的部分，算完了再整体替换，不需要主循环等待。
        """
        if not self.command_buffer.should_call_generator(self.time_steps):
            return
        if self._generator_thread is not None and self._generator_thread.is_alive():
            # 上一次还没算完（正常情况下 <150ms 远小于 400ms 的调用间隔，不该发生；
            # 真发生了就跳过这次触发，继续用当前 buffer，好过重叠调用或者卡住主循环等它）
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

        def _worker() -> None:
            chunk = self.generator.predict(gen_obs)  # (1, 36, 36)
            self.command_buffer.set_chunk(chunk[0].cpu().numpy())
            self.stats.generator_calls += 1

        self._generator_thread = threading.Thread(target=_worker, daemon=True)
        self._generator_thread.start()

    def control_step(self) -> None:
        if self.pre_step_hook is not None:
            self.pre_step_hook()
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
        # Generator 是后台线程异步跑的（见 _maybe_call_generator），进程/脚本结束前
        # 必须等它跑完，不然作为 daemon 线程会被硬中断，报 C++ 层的 abort
        # （"terminate called without an active exception"），不是干净退出。
        if self._generator_thread is not None:
            self._generator_thread.join(timeout=5.0)
        if self.stats.control_steps > 0:
            self.stats.mean_tracker_step_ms /= self.stats.control_steps
        return self.stats
