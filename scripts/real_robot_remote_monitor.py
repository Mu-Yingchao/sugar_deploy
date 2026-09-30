"""只读显示 G1 独立手柄 DDS 话题中的安全组合键。"""

from __future__ import annotations

from dataclasses import dataclass
import time

import tyro

from sugar_deploy.real_robot_io import RealRobotIO
from sugar_deploy.real_robot_safety import RemoteButtons


@dataclass
class Args:
    interface: str = "eth0"
    domain_id: int = 0
    duration: float = 10.0


def main(args: Args) -> None:
    print(
        f"[remote] 只读监听手柄（接口={args.interface}）；请依次按 L2+Y 和 L2+B，"
        "本脚本不会 release 或发送 lowcmd",
        flush=True,
    )
    io = RealRobotIO(args.interface, args.domain_id)
    try:
        deadline = time.monotonic() + args.duration
        last: RemoteButtons | None = None
        seen: set[str] = set()
        while time.monotonic() < deadline:
            try:
                state = io.read_state()
            except RuntimeError:
                time.sleep(0.02)
                continue
            remote_data = io.read_wireless_remote()
            if remote_data is None:
                remote_data = state.wireless_remote
            buttons = RemoteButtons.from_wireless_remote(remote_data)
            if buttons != last:
                print(
                    f"[remote] L1={int(buttons.l1)} L2={int(buttons.l2)} "
                    f"A={int(buttons.a)} B={int(buttons.b)} Y={int(buttons.y)} "
                    f"emergency={buttons.emergency_combo or '-'}",
                    flush=True,
                )
                last = buttons
            if buttons.emergency_combo:
                seen.add(buttons.emergency_combo)
            time.sleep(0.02)
        print(f"[remote] 完成；识别到的组合键：{sorted(seen) or '无'}", flush=True)
    finally:
        io.close()


if __name__ == "__main__":
    main(tyro.cli(Args))
