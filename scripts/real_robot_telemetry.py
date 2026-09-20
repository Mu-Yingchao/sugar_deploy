"""第 0 阶段：只读遥测，**不发送任何指令**，用来在发第一条控制指令之前确认两件事：

1. DDS 通信本身通了（能收到 rt/lowstate）。
2. sugar_deploy/unitree_joint_map.py 里的关节顺序映射是对的——机器人站着不动时，读出来的
   （已经转成 contract.JOINT_NAMES 顺序的）关节角度应该和 contract.DEFAULT_JOINT_POS
   同一量级、同一个正负号，明显对不上说明映射错了，必须先查清楚再往下走，不能带着错的映射
   去发控制指令。

这个脚本全程不 release 高层控制、不调用 send_command，物理上不可能对机器人造成任何影响，
可以在机器人正常站立/被官方遥控器控制的状态下运行。

用法（要先在机器人所在网络的机器上跑，且装好 unitree_sdk2py）：
    python scripts/real_robot_telemetry.py --interface eth0 --duration 5
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np
import tyro

from sugar_deploy import contract
from sugar_deploy.real_robot_io import RealRobotIO


@dataclass
class Args:
    interface: str = "eth0"
    """机器人网络接口名，不确定就跑 `ip a` 看哪个接口和机器人在同一网段。"""
    domain_id: int = 0
    duration: float = 5.0
    """打印这么多秒的数据然后自动退出。"""


def main(args: Args) -> None:
    print(f"[telemetry] 连接 rt/lowstate（网络接口={args.interface}, domain_id={args.domain_id}）"
          "——这个脚本只读，不会发送任何指令")
    io = RealRobotIO(network_interface=args.interface, domain_id=args.domain_id)

    t_start = time.time()
    n_reads = 0
    while time.time() - t_start < args.duration:
        try:
            state = io.read_state()
        except RuntimeError as e:
            print(f"[telemetry] {e}")
            time.sleep(0.5)
            continue
        n_reads += 1
        if n_reads % 20 == 1:  # 差不多每 0.4~0.5s 打印一次，不刷屏
            default_pos = np.array(contract.DEFAULT_JOINT_POS)
            diff = state.joint_pos - default_pos
            print(
                f"[telemetry] n={n_reads} base_quat_wxyz={np.round(state.base_quat_wxyz, 3)} "
                f"max|joint_pos-default|={np.max(np.abs(diff)):.3f}rad "
                f"({contract.JOINT_NAMES[int(np.argmax(np.abs(diff)))]})"
            )
        time.sleep(0.02)

    print(f"[telemetry] 完成，共读到 {n_reads} 帧。")
    print(
        "[telemetry] 核对方法：如果机器人现在站的姿态接近默认站姿，上面打印的 "
        "max|joint_pos-default| 应该是个不大的数（零点几弧度量级，不是几弧度）；如果差得很"
        "离谱、或者符号看着不对劲，先怀疑 unitree_joint_map.py 里的顺序映射错了，"
        "不要直接跳到发指令那一步。"
    )


if __name__ == "__main__":
    main(tyro.cli(Args))
