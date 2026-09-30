"""带手柄急停和心跳看门狗的阶段 2/3 增益测试。"""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Literal

import numpy as np
import tyro

from sugar_deploy import contract
from sugar_deploy.real_robot_safety import EmergencyStopError, SafeRealRobotController


SAFETY_CONFIRMATION = "ROBOT_SUPPORTED_ESTOP_READY"


@dataclass
class Args:
    interface: str = "eth0"
    domain_id: int = 0
    gain_scale: float = 0.05
    duration: float = 5.0
    ramp_duration: float = 0.0
    """目标为 default 时，从当前姿态平滑过渡到默认姿态的秒数；0 表示禁止移动。"""
    max_target_speed: float = 0.15
    """平滑目标轨迹的最大允许峰值速度，rad/s；超限时在 release 高层控制前拒绝启动。"""
    target: Literal["current", "default"] = "current"
    zero_torque_hold: float = 5.0
    """结束或急停后持续发送零力矩的秒数，给现场人员明确的触觉确认窗口。"""
    watchdog_test_after: float | None = None
    """仅用于悬挂低增益验收：到时故意停止心跳 0.3 秒，验证看门狗锁存零力矩。"""
    safety_confirmation: str = ""
    """必须显式传 ROBOT_SUPPORTED_ESTOP_READY，防止误启动真机低层控制。"""
    max_tilt: float = 1.0
    """机身绝对倾角急停阈值，rad；Unitree 官方 termination 默认值为 1.0。"""
    max_joint_velocity: float = 10.0
    """任一关节绝对速度急停阈值，rad/s；Unitree 官方默认值为 10。"""
    max_base_gyro: float = 6.0
    """任一机身角速度分量急停阈值，rad/s；Unitree 官方默认值为 6。"""


def main(args: Args) -> None:
    if args.safety_confirmation != SAFETY_CONFIRMATION:
        raise SystemExit(
            "拒绝启动：先确认机器人有对应阶段要求的物理支撑、第二个人已拿好手柄，"
            f"再传 --safety-confirmation {SAFETY_CONFIRMATION}"
        )
    if not 0.0 < args.gain_scale <= 1.0:
        raise SystemExit("--gain-scale 必须在 (0, 1] 内")
    if args.duration <= 0:
        raise SystemExit("--duration 必须大于 0")
    if args.ramp_duration < 0:
        raise SystemExit("--ramp-duration 不能小于 0")
    if args.max_target_speed <= 0:
        raise SystemExit("--max-target-speed 必须大于 0")
    if args.target == "default":
        if args.ramp_duration <= 0:
            raise SystemExit("target=default 必须显式设置 --ramp-duration，禁止全增益阶跃目标")
        if args.duration < args.ramp_duration:
            raise SystemExit("--duration 必须大于等于 --ramp-duration")
    if args.zero_torque_hold < 1.0:
        raise SystemExit("--zero-torque-hold 为保证安全确认不得小于 1 秒")
    if args.max_tilt <= 0 or args.max_joint_velocity <= 0 or args.max_base_gyro <= 0:
        raise SystemExit("姿态和速度安全阈值必须全部大于 0")
    if args.watchdog_test_after is not None and not 0 < args.watchdog_test_after < args.duration:
        raise SystemExit("--watchdog-test-after 必须在 (0, duration) 内")
    if args.watchdog_test_after is not None and args.gain_scale > 0.15:
        raise SystemExit("看门狗故障注入只允许在 gain-scale <= 0.15 时使用")

    controller = SafeRealRobotController.connect(
        args.interface,
        args.domain_id,
        control_period_s=contract.CONTROL_DT,
        command_timeout_s=0.10,
        state_timeout_s=0.20,
        remote_timeout_s=0.50,
        zero_torque_hold_s=args.zero_torque_hold,
        max_tilt_rad=args.max_tilt,
        max_abs_joint_velocity=args.max_joint_velocity,
        max_abs_base_gyro=args.max_base_gyro,
    )

    try:
        initial_state = controller.start()
        if args.target == "current":
            q_start = initial_state.joint_pos.copy()
            q_final = q_start.copy()
        else:
            q_start = initial_state.joint_pos.astype(np.float64, copy=True)
            q_final = np.asarray(contract.DEFAULT_JOINT_POS, dtype=np.float64)
            delta = q_final - q_start
            max_index = int(np.argmax(np.abs(delta)))
            # smoothstep 3a^2-2a^3 的导数峰值是 1.5，因此峰值目标速度如下。
            peak_speed = 1.5 * float(np.max(np.abs(delta))) / args.ramp_duration
            print(
                f"[gain-test] 默认姿态过渡：max_delta={abs(delta[max_index]):.4f}rad "
                f"({contract.JOINT_NAMES[max_index]}), ramp={args.ramp_duration:.1f}s, "
                f"planned_peak_speed={peak_speed:.4f}rad/s",
                flush=True,
            )
            if peak_speed > args.max_target_speed:
                raise SystemExit(
                    f"拒绝启动：规划峰值目标速度 {peak_speed:.4f}rad/s 超过 "
                    f"--max-target-speed {args.max_target_speed:.4f}rad/s；请延长 ramp-duration"
                )
        kp = np.asarray(contract.JOINT_STIFFNESS) * args.gain_scale
        kd = np.asarray(contract.JOINT_DAMPING) * args.gain_scale

        print(
            f"[gain-test] 开始：gain={args.gain_scale:.2f}, target={args.target}, "
            f"duration={args.duration:.1f}s",
            flush=True,
        )
        print(
            "[gain-test] 低层控制期间按 L2+Y（首选）、L2+B 或 L1+A，"
            "会由本脚本锁存并持续发送 HG 零力矩命令",
            flush=True,
        )
        deadline = time.monotonic() + args.duration
        started_at = time.monotonic()
        next_report = time.monotonic()
        watchdog_injected = False
        while time.monotonic() < deadline:
            elapsed = time.monotonic() - started_at
            if args.target == "default":
                phase = min(elapsed / args.ramp_duration, 1.0)
                blend = phase * phase * (3.0 - 2.0 * phase)
                q_target = q_start + blend * (q_final - q_start)
            else:
                q_target = q_final
            controller.set_command(q_target, kp, kd)
            controller.raise_if_estopped()
            now = time.monotonic()
            if (
                args.watchdog_test_after is not None
                and not watchdog_injected
                and now - started_at >= args.watchdog_test_after
            ):
                watchdog_injected = True
                print("[gain-test] 故意停止主循环心跳 0.3s，等待看门狗急停", flush=True)
                time.sleep(0.3)
                controller.raise_if_estopped()
            if now >= next_report:
                remaining = max(0.0, deadline - now)
                print(f"[gain-test] 剩余 {remaining:.1f}s", flush=True)
                next_report = now + 1.0
            time.sleep(contract.CONTROL_DT)
        print("[gain-test] 保持测试正常完成", flush=True)
    except EmergencyStopError as exc:
        print(f"[gain-test] 软件急停已锁存，正在发送 HG 零力矩命令：{exc}", flush=True)
        raise SystemExit(2) from None
    finally:
        print(
            f"[gain-test] 进入 {args.zero_torque_hold:.1f}s 持续零力矩收尾窗口",
            flush=True,
        )
        controller.close()
        status = controller.status
        print(
            f"[gain-test] 已完成零力矩命令发送窗口（是否实际卸力须由现场确认）；"
            f"reason={status.reason!r}, "
            f"worker_error={status.worker_error!r}",
            flush=True,
        )


if __name__ == "__main__":
    main(tyro.cli(Args))
