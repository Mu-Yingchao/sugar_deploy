"""SUGAR Tracker 真机部署所需的纯计算辅助函数。"""

from __future__ import annotations

import numpy as np

from sugar_deploy.observation import RobotState, quat_wxyz_to_rotmat
from sugar_deploy.real_robot_io import RealRobotState


def effort_limited_position_target(
    q_desired: np.ndarray,
    q: np.ndarray,
    qd: np.ndarray,
    kp: np.ndarray,
    kd: np.ndarray,
    effort_limit: np.ndarray,
    effort_scale: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """把策略目标换成在同一组 PD 增益下等效、且力矩受限的位置目标。

    MuJoCo 部署层使用 ``clip(kp*(q_des-q)-kd*qd, effort_limit)``。真机低层接口接收
    q/kp/kd 而不是最终 PD 力矩，因此反解一个 ``q_limited``，使电机侧同一公式得到截断后的
    力矩。返回 ``(q_limited, tau_unclipped, tau_limited)``。
    """
    arrays = [np.asarray(x, dtype=np.float64) for x in (q_desired, q, qd, kp, kd, effort_limit)]
    shape = arrays[0].shape
    if any(x.shape != shape for x in arrays):
        raise ValueError("Tracker 力矩限幅输入形状必须一致")
    if not all(np.all(np.isfinite(x)) for x in arrays):
        raise ValueError("Tracker 力矩限幅输入含 NaN/Inf")
    if not 0.0 < effort_scale <= 1.0:
        raise ValueError("effort_scale 必须在 (0, 1] 内")
    q_desired, q, qd, kp, kd, effort_limit = arrays
    if np.any(kp <= 0) or np.any(kd < 0) or np.any(effort_limit <= 0):
        raise ValueError("kp/effort_limit 必须为正，kd 不能为负")

    tau_unclipped = kp * (q_desired - q) - kd * qd
    limit = effort_limit * effort_scale
    tau_limited = np.clip(tau_unclipped, -limit, limit)
    q_limited = q + (tau_limited + kd * qd) / kp
    return q_limited, tau_unclipped, tau_limited


def blend_tracker_torque(
    baseline_tau: np.ndarray,
    tracker_tau_unclipped: np.ndarray,
    effort_limit: np.ndarray,
    mix: float,
) -> tuple[np.ndarray, np.ndarray]:
    """从已验收基线力矩无冲击混合到满额限幅 Tracker 力矩。"""
    arrays = [
        np.asarray(x, dtype=np.float64)
        for x in (baseline_tau, tracker_tau_unclipped, effort_limit)
    ]
    if any(x.shape != arrays[0].shape for x in arrays):
        raise ValueError("力矩混合输入形状必须一致")
    if not all(np.all(np.isfinite(x)) for x in arrays) or not np.isfinite(mix):
        raise ValueError("力矩混合输入含 NaN/Inf")
    if np.any(arrays[2] <= 0):
        raise ValueError("effort_limit 必须为正")
    if not 0.0 <= mix <= 1.0:
        raise ValueError("mix 必须在 [0,1] 内")
    baseline, tracker_raw, effort = arrays
    tracker_full = np.clip(tracker_raw, -effort, effort)
    applied = baseline + mix * (tracker_full - baseline)
    return tracker_full, applied


def real_state_for_tracker(state: RealRobotState) -> RobotState:
    """把真机 IMU/关节状态转成 Tracker 的 ``RobotState``。

    G1 IMU 陀螺仪给机体系角速度，而 ``RobotState`` 字段约定为世界系；先旋到世界系，使
    ``TrackerObsBuilder`` 内部再旋回机体系后得到原始陀螺仪值。首次静止测试暂用固定的 anchor
    坐标系和物体相对位姿，因此这里只需让 anchor 为单位位姿。
    """
    quat = np.asarray(state.base_quat_wxyz, dtype=np.float64)
    norm = float(np.linalg.norm(quat))
    if norm < 1e-6 or not np.isfinite(norm):
        raise ValueError("IMU 四元数无效")
    quat = quat / norm
    gyro_b = np.asarray(state.base_gyro, dtype=np.float64)
    gyro_w = quat_wxyz_to_rotmat(quat) @ gyro_b
    return RobotState(
        joint_pos=np.asarray(state.joint_pos, dtype=np.float64).copy(),
        joint_vel=np.asarray(state.joint_vel, dtype=np.float64).copy(),
        base_quat_w=quat,
        base_ang_vel_w=gyro_w,
        anchor_pos_w=np.zeros(3, dtype=np.float64),
        anchor_quat_w=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
    )
