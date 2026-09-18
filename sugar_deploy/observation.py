"""把 (机器人本体状态, 物体状态, 上一次动作) 拼成 Tracker/Generator 各自需要的观测。

坐标变换和历史 buffer 的语义都是照着 SUGAR 源码核实过的：
- anchor 是 ``torso_link``，不是 pelvis（``ANCHOR_BODY_NAME`` in contract.py）。
- ``obj_pos_b``/``obj_ori_b`` 是相对 anchor 的局部系，不是世界系也不是 root 系。
- 6D 旋转表示是 "旋转矩阵前两列按行展开"，和 SUGAR 的
  ``isaaclab.utils.math.matrix_from_quat(...)[..., :2].reshape(...)`` 完全一致
  （数值上等价于 sugar_il_wrapper.py 里的 ``quat_to_6d_rotation_col``，已经对照过
  两边公式，是同一个约定，不是巧合）。
- Command chunk 的消费节奏：Generator 每 ``GENERATOR_CALL_INTERVAL``(20) 个
  tracker 控制步被调用一次，产出一个 36 长度的插值 buffer（DOWNSAMPLE_RATE=5 决定
  的插值细节在 ``GeneratorWrapper._parse_action`` 里，这边不重新实现），Tracker
  每步按 ``idx = time_steps % GENERATOR_CALL_INTERVAL`` 从这个 buffer 里取一条
  36 维 command。
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np
import torch

from sugar_deploy import contract


def quat_wxyz_to_6d(quat_wxyz: np.ndarray) -> np.ndarray:
    """(..., 4) wxyz 四元数 -> (..., 6)，旋转矩阵前两列按行展开。
    和 SUGAR `matrix_from_quat(...)[..., :2].reshape(...)` 数值上等价。"""
    w, x, y, z = quat_wxyz[..., 0], quat_wxyz[..., 1], quat_wxyz[..., 2], quat_wxyz[..., 3]
    r00 = 1 - 2 * (y * y + z * z)
    r10 = 2 * (x * y + z * w)
    r20 = 2 * (x * z - y * w)
    r01 = 2 * (x * y - z * w)
    r11 = 1 - 2 * (x * x + z * z)
    r21 = 2 * (y * z + x * w)
    return np.stack([r00, r01, r10, r11, r20, r21], axis=-1)


def rotmat_to_quat_wxyz(rot: np.ndarray) -> np.ndarray:
    """(...,3,3) 旋转矩阵 -> (...,4) wxyz 四元数。用于把 MuJoCo 的 xmat 转成
    GeneratorObs 需要的四元数格式。"""
    m00, m01, m02 = rot[..., 0, 0], rot[..., 0, 1], rot[..., 0, 2]
    m10, m11, m12 = rot[..., 1, 0], rot[..., 1, 1], rot[..., 1, 2]
    m20, m21, m22 = rot[..., 2, 0], rot[..., 2, 1], rot[..., 2, 2]
    trace = m00 + m11 + m22
    w = np.sqrt(np.clip(trace + 1.0, 1e-8, None)) / 2.0
    x = (m21 - m12) / (4.0 * w)
    y = (m02 - m20) / (4.0 * w)
    z = (m10 - m01) / (4.0 * w)
    quat = np.stack([w, x, y, z], axis=-1)
    return quat / np.linalg.norm(quat, axis=-1, keepdims=True)


def subtract_frame_transform(
    anchor_pos_w: np.ndarray, anchor_quat_wxyz: np.ndarray,
    point_pos_w: np.ndarray, point_quat_wxyz: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """世界系 -> anchor 局部系。等价于 IsaacLab 的 subtract_frame_transforms，
    只是这里只需要用到位置部分的严谨实现；姿态部分复合旋转用于算 6D 表示。"""
    anchor_rot = quat_wxyz_to_rotmat(anchor_quat_wxyz)
    pos_b = anchor_rot.T @ (point_pos_w - anchor_pos_w)
    point_rot = quat_wxyz_to_rotmat(point_quat_wxyz)
    rot_b = anchor_rot.T @ point_rot
    quat_b = rotmat_to_quat_wxyz(rot_b)
    return pos_b, quat_b


def quat_wxyz_to_rotmat(quat_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = quat_wxyz[..., 0], quat_wxyz[..., 1], quat_wxyz[..., 2], quat_wxyz[..., 3]
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_apply_inverse(quat_wxyz: np.ndarray, vec_w: np.ndarray) -> np.ndarray:
    """把世界系向量转到 quat 表示的局部系下（比如把重力向量转到机体系）。"""
    return quat_wxyz_to_rotmat(quat_wxyz).T @ vec_w


GRAVITY_W = np.array([0.0, 0.0, -1.0])


@dataclass
class RobotState:
    """一帧机器人本体状态，全部是世界系原始读数，坐标变换在 TrackerObsBuilder 里做。"""

    joint_pos: np.ndarray       # (29,)
    joint_vel: np.ndarray       # (29,)
    base_quat_w: np.ndarray     # (4,) wxyz，pelvis
    base_ang_vel_w: np.ndarray  # (3,) 世界系角速度（会转到 pelvis 局部系）
    anchor_pos_w: np.ndarray    # (3,) torso_link 世界系位置
    anchor_quat_w: np.ndarray   # (4,) wxyz，torso_link


class HistoryBuffer:
    """固定长度、旧->新的滑动窗口，对齐 IsaacLab CircularBuffer 的 flatten 顺序。"""

    def __init__(self, dim: int, length: int):
        self.dim = dim
        self.length = length
        self._buf: deque[np.ndarray] = deque(
            [np.zeros(dim, dtype=np.float32) for _ in range(length)], maxlen=length
        )

    def push(self, x: np.ndarray) -> None:
        self._buf.append(np.asarray(x, dtype=np.float32))

    def flatten(self) -> np.ndarray:
        return np.concatenate(list(self._buf), axis=0)

    def reset(self, x: np.ndarray | None = None) -> None:
        fill = np.zeros(self.dim, dtype=np.float32) if x is None else np.asarray(x, dtype=np.float32)
        self._buf = deque([fill.copy() for _ in range(self.length)], maxlen=self.length)


@dataclass
class TrackerObsBuilder:
    """维护 Tracker 需要的 5 帧历史 buffer，每个控制步调用一次 build()。"""

    default_joint_pos: np.ndarray = field(default_factory=lambda: np.array(contract.DEFAULT_JOINT_POS))
    base_ang_vel_hist: HistoryBuffer = field(
        default_factory=lambda: HistoryBuffer(3, contract.TRACKER_OBS_HISTORY_LEN))
    joint_pos_hist: HistoryBuffer = field(
        default_factory=lambda: HistoryBuffer(contract.NUM_JOINTS, contract.TRACKER_OBS_HISTORY_LEN))
    joint_vel_hist: HistoryBuffer = field(
        default_factory=lambda: HistoryBuffer(contract.NUM_JOINTS, contract.TRACKER_OBS_HISTORY_LEN))
    action_hist: HistoryBuffer = field(
        default_factory=lambda: HistoryBuffer(contract.NUM_JOINTS, contract.TRACKER_OBS_HISTORY_LEN))
    gravity_hist: HistoryBuffer = field(
        default_factory=lambda: HistoryBuffer(3, contract.TRACKER_OBS_HISTORY_LEN))
    last_action: np.ndarray = field(default_factory=lambda: np.zeros(contract.NUM_JOINTS, dtype=np.float32))

    def reset(self, robot: RobotState) -> None:
        self.base_ang_vel_hist.reset()
        self.joint_pos_hist.reset()
        self.joint_vel_hist.reset()
        self.action_hist.reset()
        self.gravity_hist.reset()
        self.last_action[:] = 0.0
        self._push_frame(robot)

    def _push_frame(self, robot: RobotState) -> None:
        base_ang_vel_b = quat_apply_inverse(robot.base_quat_w, robot.base_ang_vel_w)
        gravity_b = quat_apply_inverse(robot.base_quat_w, GRAVITY_W)
        self.base_ang_vel_hist.push(base_ang_vel_b)
        self.joint_pos_hist.push(robot.joint_pos - self.default_joint_pos)
        self.joint_vel_hist.push(robot.joint_vel)  # 默认关节速度是 0，rel = 原始值
        self.action_hist.push(self.last_action)
        self.gravity_hist.push(gravity_b)

    def step(self, robot: RobotState, last_action: np.ndarray) -> None:
        """每个控制步先用上一步应用的动作更新 last_action，再推新的一帧历史。"""
        self.last_action = np.asarray(last_action, dtype=np.float32)
        self._push_frame(robot)

    def build(self, robot: RobotState, obj_pos_w: np.ndarray, obj_quat_w: np.ndarray,
              command_36: np.ndarray) -> np.ndarray:
        """拼出 510 维 Tracker 观测，顺序严格对齐 contract.TRACKER_OBS_LAYOUT。"""
        obj_pos_b, obj_quat_b = subtract_frame_transform(
            robot.anchor_pos_w, robot.anchor_quat_w, obj_pos_w, obj_quat_w
        )
        obj_ori_b_6d = quat_wxyz_to_6d(obj_quat_b)

        parts = [
            command_36.astype(np.float32),
            self.base_ang_vel_hist.flatten(),
            self.joint_pos_hist.flatten(),
            self.joint_vel_hist.flatten(),
            self.action_hist.flatten(),
            self.gravity_hist.flatten(),
            obj_pos_b.astype(np.float32),
            obj_ori_b_6d.astype(np.float32),
        ]
        obs = np.concatenate(parts, axis=0)
        assert obs.shape == (contract.TRACKER_OBS_DIM,), obs.shape
        return obs


@dataclass
class CommandBuffer:
    """持有 Generator 最近一次产出的插值 command chunk，供 Tracker 逐步消费。"""

    buffer_36: np.ndarray = field(
        default_factory=lambda: np.zeros((36, contract.COMMAND_DIM), dtype=np.float32))
    step_in_cycle: int = 0

    def set_chunk(self, chunk_36xD: np.ndarray) -> None:
        self.buffer_36 = np.asarray(chunk_36xD, dtype=np.float32)
        self.step_in_cycle = 0

    def current(self) -> np.ndarray:
        idx = min(self.step_in_cycle, self.buffer_36.shape[0] - 1)
        return self.buffer_36[idx]

    def advance(self) -> None:
        self.step_in_cycle += 1

    def should_call_generator(self, time_steps: int) -> bool:
        return time_steps % contract.GENERATOR_CALL_INTERVAL == 0


def build_generator_obs(
    robot: RobotState, obj_pos_w: np.ndarray, obj_quat_w: np.ndarray,
    last_command_36: np.ndarray, use_target: bool, use_last_action: bool,
    target_obj_pos_w: np.ndarray | None = None, target_obj_quat_w: np.ndarray | None = None,
):
    """构造喂给 GeneratorWrapper.predict 的 GeneratorObs（n_obs_steps=1，单帧）。

    保持四元数格式（不转 6D）——GeneratorWrapper 内部自己会转 6D，两边约定一致
    （见本文件顶部说明），这里没必要重复转换。
    """
    from sugar_il.wrapper.sugar_il_wrapper import GeneratorObs

    obj_pos_b, obj_quat_b = subtract_frame_transform(
        robot.anchor_pos_w, robot.anchor_quat_w, obj_pos_w, obj_quat_w
    )
    gravity_b = quat_apply_inverse(robot.base_quat_w, GRAVITY_W)

    def _t(x: np.ndarray) -> torch.Tensor:
        return torch.as_tensor(x, dtype=torch.float32).view(1, 1, -1)

    if use_target:
        assert target_obj_pos_w is not None and target_obj_quat_w is not None, (
            "这个 checkpoint 的 use_target=True，必须提供 target_obj_pos_w/target_obj_quat_w"
            "（任务的目标物体位姿，比如 CarryBox 要搬到的目的地）"
        )
        target_pos_b, target_quat_b = subtract_frame_transform(
            robot.anchor_pos_w, robot.anchor_quat_w, target_obj_pos_w, target_obj_quat_w
        )
    else:
        target_pos_b = np.zeros(3, dtype=np.float32)
        target_quat_b = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)

    # joint_pos/project_gravity 只有 use_last_action=False 时才会真正被
    # GeneratorWrapper._prepare_obs_dict 用到，但 dataclass 字段不是 Optional，
    # 不管用不用都要填；joint_pos 沿用和 Tracker 一致的"相对默认位姿"约定。
    return GeneratorObs(
        obj_pos_b=_t(obj_pos_b),
        obj_ori_b=_t(obj_quat_b),
        joint_pos=_t(robot.joint_pos - contract.DEFAULT_JOINT_POS),
        project_gravity=_t(gravity_b),
        target_obj_pos_b=_t(target_pos_b),
        target_obj_ori_b=_t(target_quat_b),
        last_command=_t(last_command_36),
    )
