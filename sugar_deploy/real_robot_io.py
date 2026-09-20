"""G1 真机低层 DDS 通信：读 ``rt/lowstate``、发 ``rt/lowcmd``。

**这份代码没有在真实硬件上跑过**——是照着 Unitree 官方 SDK 的消息格式和 HDMI 官方部署仓库
（`EGalahad/sim2real` 的 `scripts/real_bridge.py`）里验证过的真实用法写的，不是凭空猜的，
但"参考了一个真实可用的实现"不等于"这份代码没有 bug"。接真机前**必须**先跑通
``scripts/real_robot_telemetry.py`` 这个只读脚本（不发送任何指令，只订阅、打印），
核对关节顺序映射（见 ``unitree_joint_map.py``）读出来的角度和机器人实际姿态吻合，再往下走
发指令这一步——完整的分阶段安全流程见 ``REAL_HARDWARE_DEPLOYMENT.md``。

依赖 ``unitree_sdk2py``（这台机器上还没装，需要先解决 cyclonedds 编译依赖，见
HDMI README 的 FAQ：https://github.com/unitreerobotics/unitree_sdk2_python?tab=readme-ov-file#faq），
所以这个模块顶层不 import 这个包，全部放进方法内部，避免只是 `import sugar_deploy` 就因为
没装 SDK 而失败。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from sugar_deploy import contract
from sugar_deploy.unitree_joint_map import UNITREE_JOINT_NAMES, contract_to_unitree, unitree_to_contract

# Unitree SDK 的标准哨兵值：q/dq 设成这两个值、同时 kp=kd=0，表示"这个关节这一步不给主动
# 控制指令"，不是随便选的数字，是电机固件识别的特殊标记——来源同上（HDMI 部署仓库
# utils/common.py 的 UNITREE_LEGGED_CONST，和 Unitree 官方 SDK 例程一致）。
POS_STOP_F = 2146000000.0
VEL_STOP_F = 16000.0
LOWLEVEL_FLAG = 0xFF


@dataclass
class RealRobotState:
    """从 rt/lowstate 读出来、已经转成 contract.JOINT_NAMES 顺序的机器人状态。"""

    joint_pos: np.ndarray  # (29,) rad，contract 顺序
    joint_vel: np.ndarray  # (29,) rad/s，contract 顺序
    joint_tau_est: np.ndarray  # (29,) N*m，电机自己估的力矩，仅供参考/诊断用
    base_quat_wxyz: np.ndarray  # (4,) IMU 给的姿态四元数
    base_gyro: np.ndarray  # (3,) rad/s，IMU 角速度


class RealRobotIO:
    """G1（29dof，unitree_hg 消息族）低层读写。构造函数只建立 DDS 通道，**不发送任何指令、
    不释放高层运动控制**——这两步都是有实际后果的操作，故意拆成显式方法，不放进构造函数里
    静默执行。
    """

    def __init__(self, network_interface: str = "eth0", domain_id: int = 0):
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize,
            ChannelPublisher,
            ChannelSubscriber,
        )
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        ChannelFactoryInitialize(domain_id, network_interface)

        self._low_state_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self._low_state_sub.Init(handler=None, queueLen=0)
        self._low_cmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self._low_cmd_pub.Init()

        self._crc = CRC()
        self._low_cmd = unitree_hg_msg_dds__LowCmd_()
        self._low_cmd.mode_pr = 0
        self._low_cmd.mode_machine = 0  # 第一次 read_state() 之后会用真实值覆盖，见下面
        for cmd in self._low_cmd.motor_cmd:
            cmd.mode = 1
            cmd.q = POS_STOP_F
            cmd.dq = VEL_STOP_F
            cmd.kp = 0.0
            cmd.kd = 0.0
            cmd.tau = 0.0

        self._released_high_level = False

    # ------------------------------------------------------------------
    # 只读部分：随时安全调用，不影响机器人当前的控制状态
    # ------------------------------------------------------------------
    def read_state(self) -> RealRobotState:
        msg = self._low_state_sub.Read()
        if msg is None:
            raise RuntimeError(
                "还没收到过 rt/lowstate 消息——检查网络接口/domain_id 是否正确，"
                "机器人是否已经开机并且在同一个网络上。"
            )
        self._low_cmd.mode_machine = msg.mode_machine

        n = len(UNITREE_JOINT_NAMES)
        joint_pos_u = np.array([msg.motor_state[i].q for i in range(n)], dtype=np.float32)
        joint_vel_u = np.array([msg.motor_state[i].dq for i in range(n)], dtype=np.float32)
        joint_tau_u = np.array([msg.motor_state[i].tau_est for i in range(n)], dtype=np.float32)

        return RealRobotState(
            joint_pos=unitree_to_contract(joint_pos_u),
            joint_vel=unitree_to_contract(joint_vel_u),
            joint_tau_est=unitree_to_contract(joint_tau_u),
            base_quat_wxyz=np.array(msg.imu_state.quaternion, dtype=np.float32),
            base_gyro=np.array(msg.imu_state.gyroscope, dtype=np.float32),
        )

    # ------------------------------------------------------------------
    # 会实际影响机器人的部分：故意分开、要求显式调用
    # ------------------------------------------------------------------
    def release_high_level_control(self, timeout_s: float = 5.0) -> None:
        """release 掉机器人自带的高层运动控制模式（比如站立/行走的官方控制器），
        发低层指令之前必须先做这一步，不然我们发的指令会跟官方控制器打架——这是真实的
        安全问题，不是可以跳过的步骤。照抄 HDMI 部署仓库 real_bridge.py 的做法：
        反复 CheckMode + ReleaseMode 直到确认没有高层模式在跑。"""
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

        msc = MotionSwitcherClient()
        msc.SetTimeout(timeout_s)
        msc.Init()
        status, result = msc.CheckMode()
        while result.get("name"):
            msc.ReleaseMode()
            status, result = msc.CheckMode()
        self._released_high_level = True

    def send_command(
        self,
        q_target_contract: np.ndarray,
        kp_contract: np.ndarray | None = None,
        kd_contract: np.ndarray | None = None,
        tau_ff_contract: np.ndarray | None = None,
    ) -> None:
        """发一次低层关节指令，输入全部是 contract.JOINT_NAMES 顺序（跟 sim2sim.py /
        contract.py 的约定一致，内部自己转成电机数组顺序，调用方不需要关心 unitree 顺序）。

        ``kp_contract``/``kd_contract`` 不传的话用 ``contract.JOINT_STIFFNESS``/
        ``JOINT_DAMPING``（sim2sim 验证过的增益），但**真机的合适增益不一定和 sim 一样**，
        第一次上真机建议从很小的增益开始试，不要直接用满增益，见
        ``REAL_HARDWARE_DEPLOYMENT.md`` 的分阶段测试流程。

        没有调用过 ``release_high_level_control()`` 会直接报错拒绝发送，不允许"忘了 release
        但指令照样发出去"这种静默的危险状态。
        """
        if not self._released_high_level:
            raise RuntimeError(
                "还没调用 release_high_level_control()——不允许在没有释放高层控制的情况下"
                "发送低层指令，这会和机器人自带的控制器冲突。"
            )
        if kp_contract is None:
            kp_contract = np.array(contract.JOINT_STIFFNESS)
        if kd_contract is None:
            kd_contract = np.array(contract.JOINT_DAMPING)
        if tau_ff_contract is None:
            tau_ff_contract = np.zeros(contract.NUM_JOINTS)

        q_u = contract_to_unitree(np.asarray(q_target_contract, dtype=np.float64))
        kp_u = contract_to_unitree(np.asarray(kp_contract, dtype=np.float64))
        kd_u = contract_to_unitree(np.asarray(kd_contract, dtype=np.float64))
        tau_u = contract_to_unitree(np.asarray(tau_ff_contract, dtype=np.float64))

        for i in range(len(UNITREE_JOINT_NAMES)):
            cmd = self._low_cmd.motor_cmd[i]
            cmd.q = float(q_u[i])
            cmd.dq = 0.0
            cmd.tau = float(tau_u[i])
            cmd.kp = float(kp_u[i])
            cmd.kd = float(kd_u[i])

        self._low_cmd.crc = self._crc.Crc(self._low_cmd)
        self._low_cmd_pub.Write(self._low_cmd)

    def send_zero_torque(self) -> None:
        """发一次"不主动控制"的指令（q=PosStopF, kp=kd=0）——用来在 release 高层控制之后、
        真正开始位置控制之前，先确认发布通道本身工作正常，且不会给任何关节实际力矩。"""
        if not self._released_high_level:
            raise RuntimeError("还没调用 release_high_level_control()")
        for cmd in self._low_cmd.motor_cmd:
            cmd.q = POS_STOP_F
            cmd.dq = VEL_STOP_F
            cmd.kp = 0.0
            cmd.kd = 0.0
            cmd.tau = 0.0
        self._low_cmd.crc = self._crc.Crc(self._low_cmd)
        self._low_cmd_pub.Write(self._low_cmd)
