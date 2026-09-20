"""G1 真机 DDS 底层电机数组顺序，和 ``contract.JOINT_NAMES``（IsaacLab 运行时实测顺序）
之间的映射。

**这是 sim2sim 那次"关节顺序错了、机器人瘫倒"的坑在真机上的对应版本，性质更严重**——
sim 里搞错了后果是机器人在虚拟世界里摔倒，真机上搞错了是真实电机收到发给别的关节的指令，
是安全问题，不是"表现差"的问题。这里的顺序**没有在真实硬件上跑过、没有验证过**，是照着
Unitree 官方 SDK 例程和 HDMI 部署仓库（``EGalahad/sim2real`` 的 ``utils/strings.py``）
里的 ``unitree_joint_names`` 抄下来的——这份列表本身是有实际部署项目在用、值得信任的来源，
但"抄对了列表"不等于"这份代码没 bug"，接真机前必须先跑
``scripts/real_robot_telemetry.py`` 这个只读脚本核对一遍（细节见
``REAL_HARDWARE_DEPLOYMENT.md`` 第 0 阶段），不能直接跳过验证就发指令。

两份顺序的本质区别：``UNITREE_JOINT_NAMES`` 是电机厂商按物理电机接线/索引分配的顺序
（左腿全部→右腿全部→腰→左臂全部→右臂全部，简单的按肢体分组），``contract.JOINT_NAMES``
是 IsaacLab/PhysX articulation 内部按运动学树深度遍历产生的顺序（关节类型分层、左右交替、
腰部穿插）——两者名字集合相同，顺序完全不同，不能混用。
"""

from __future__ import annotations

import numpy as np

from sugar_deploy import contract

# 来源：EGalahad/sim2real（HDMI 官方真机部署仓库）utils/strings.py 的 unitree_joint_names，
# 对应 G1 29dof（unitree_hg LowState_/LowCmd_ 的 motor_state/motor_cmd 数组下标顺序，
# 0-based，直接 motor_state[idx] 索引，不需要额外查表）。
UNITREE_JOINT_NAMES: tuple[str, ...] = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
)
assert len(UNITREE_JOINT_NAMES) == contract.NUM_JOINTS
assert set(UNITREE_JOINT_NAMES) == set(contract.JOINT_NAMES), (
    "UNITREE_JOINT_NAMES 和 contract.JOINT_NAMES 的关节名字集合对不上，先查是不是漏了/多了关节"
)

_unitree_index = {name: i for i, name in enumerate(UNITREE_JOINT_NAMES)}
_contract_index = {name: i for i, name in enumerate(contract.JOINT_NAMES)}

# 两个都是"取值用的下标数组"（gather，不是 scatter），故意只用这一种模式——scatter 赋值
# （out[perm] = x）和 gather 取值（out = x[perm]）的 perm 互为逆置换，混着写最容易写反，
# 这里统一只用 gather，两个数组分别独立按名字查出来，不是互相求逆算出来的，出错的话两条
# assert 至少有一条会先炸，不会安静地传一个转置错的矩阵下去。
#
# UNITREE_TO_CONTRACT_PERM[j] = contract.JOINT_NAMES[j] 这个关节在 UNITREE_JOINT_NAMES 里的下标。
# 用法：contract_order_array = unitree_order_array[..., UNITREE_TO_CONTRACT_PERM]
UNITREE_TO_CONTRACT_PERM: np.ndarray = np.array(
    [_unitree_index[name] for name in contract.JOINT_NAMES], dtype=np.int64
)
# CONTRACT_TO_UNITREE_PERM[i] = UNITREE_JOINT_NAMES[i] 这个关节在 contract.JOINT_NAMES 里的下标。
# 用法：unitree_order_array = contract_order_array[..., CONTRACT_TO_UNITREE_PERM]
CONTRACT_TO_UNITREE_PERM: np.ndarray = np.array(
    [_contract_index[name] for name in UNITREE_JOINT_NAMES], dtype=np.int64
)


def unitree_to_contract(arr_unitree_order: np.ndarray) -> np.ndarray:
    """(..., 29)，电机数组顺序 -> contract.JOINT_NAMES 顺序。"""
    return arr_unitree_order[..., UNITREE_TO_CONTRACT_PERM]


def contract_to_unitree(arr_contract_order: np.ndarray) -> np.ndarray:
    """(..., 29)，contract.JOINT_NAMES 顺序 -> 电机数组顺序。"""
    return arr_contract_order[..., CONTRACT_TO_UNITREE_PERM]
