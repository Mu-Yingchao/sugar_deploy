"""吊架状态下记录满增益默认姿态基线，不运行 SUGAR Tracker。"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import time

import numpy as np
import tyro

from sugar_deploy import contract
from sugar_deploy.real_robot_safety import EmergencyStopError, SafeRealRobotController


SAFETY_CONFIRMATION = "ROBOT_SUSPENDED_BASELINE_READY"


@dataclass
class Args:
    interface: str = "eth0"
    domain_id: int = 0
    ramp_duration: float = 10.0
    hold_duration: float = 10.0
    max_target_speed: float = 0.15
    zero_torque_hold: float = 30.0
    max_tilt: float = 0.35
    max_joint_velocity: float = 2.0
    max_base_gyro: float = 1.0
    output: Path | None = None
    safety_confirmation: str = ""


def _save_csv(
    path: Path,
    times: list[float],
    phases: list[str],
    positions: list[np.ndarray],
    velocities: list[np.ndarray],
    efforts: list[np.ndarray],
    targets: list[np.ndarray],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = ["time_s", "phase"]
    for prefix in ("q", "dq", "tau_est", "q_target"):
        fields.extend(f"{prefix}.{name}" for name in contract.JOINT_NAMES)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(fields)
        for row in zip(times, phases, positions, velocities, efforts, targets, strict=True):
            t, phase, q, dq, tau, target = row
            writer.writerow([f"{t:.9f}", phase, *q, *dq, *tau, *target])


def _print_summary(
    times: list[float], positions: list[np.ndarray], velocities: list[np.ndarray]
) -> None:
    if not times:
        print("[baseline] 没有采集到状态样本", flush=True)
        return
    t = np.asarray(times, dtype=np.float64)
    q = np.stack(positions).astype(np.float64)
    dq = np.stack(velocities).astype(np.float64)

    reported_flat = int(np.argmax(np.abs(dq)))
    sample_index, joint_index = np.unravel_index(reported_flat, dq.shape)
    left_ankle_index = contract.JOINT_NAMES.index("left_ankle_roll_joint")
    print(
        f"[baseline] 上报速度峰值={dq[sample_index, joint_index]:+.3f}rad/s "
        f"({contract.JOINT_NAMES[joint_index]}, t={t[sample_index]:.3f}s)",
        flush=True,
    )
    print(
        f"[baseline] 左踝横滚：max|dq|={np.max(np.abs(dq[:, left_ankle_index])):.3f}rad/s, "
        f"position_span={np.ptp(q[:, left_ankle_index]):.5f}rad",
        flush=True,
    )

    dt = np.diff(t)
    valid = dt > 1e-4
    if not np.any(valid):
        print("[baseline] 有效相邻时间样本不足，无法计算位置差分速度", flush=True)
        return
    fdq = np.diff(q, axis=0)[valid] / dt[valid, None]
    fdq_flat = int(np.argmax(np.abs(fdq)))
    fd_sample, fd_joint = np.unravel_index(fdq_flat, fdq.shape)
    print(
        f"[baseline] 位置差分速度峰值={fdq[fd_sample, fd_joint]:+.3f}rad/s "
        f"({contract.JOINT_NAMES[fd_joint]})",
        flush=True,
    )
    print(
        f"[baseline] 左踝横滚位置差分 max|dq|="
        f"{np.max(np.abs(fdq[:, left_ankle_index])):.3f}rad/s",
        flush=True,
    )


def main(args: Args) -> None:
    if args.safety_confirmation != SAFETY_CONFIRMATION:
        raise SystemExit(
            "拒绝启动：确认吊架承担全部重量、双脚完全离地且第二个人准备按 L2+Y 后，再传 "
            f"--safety-confirmation {SAFETY_CONFIRMATION}"
        )
    if args.ramp_duration <= 0 or args.hold_duration <= 0:
        raise SystemExit("--ramp-duration 和 --hold-duration 必须大于 0")
    if args.max_target_speed <= 0:
        raise SystemExit("--max-target-speed 必须大于 0")
    if args.zero_torque_hold < 1.0:
        raise SystemExit("--zero-torque-hold 不得小于 1 秒")

    output = args.output
    if output is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output = Path("diagnostics") / f"baseline_{stamp}.csv"

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
    kp = np.asarray(contract.JOINT_STIFFNESS, dtype=np.float64)
    kd = np.asarray(contract.JOINT_DAMPING, dtype=np.float64)
    default = np.asarray(contract.DEFAULT_JOINT_POS, dtype=np.float64)
    times: list[float] = []
    phases: list[str] = []
    positions: list[np.ndarray] = []
    velocities: list[np.ndarray] = []
    efforts: list[np.ndarray] = []
    targets: list[np.ndarray] = []

    exit_code = 0
    try:
        initial = controller.start()
        q_start = initial.joint_pos.astype(np.float64, copy=True)
        delta = default - q_start
        max_index = int(np.argmax(np.abs(delta)))
        planned_peak_speed = 1.5 * float(np.max(np.abs(delta))) / args.ramp_duration
        print(
            f"[baseline] 默认姿态过渡：max_delta={abs(delta[max_index]):.4f}rad "
            f"({contract.JOINT_NAMES[max_index]}), "
            f"planned_peak_speed={planned_peak_speed:.4f}rad/s",
            flush=True,
        )
        if planned_peak_speed > args.max_target_speed:
            raise SystemExit(
                f"拒绝启动：规划峰值目标速度 {planned_peak_speed:.4f}rad/s 超过 "
                f"{args.max_target_speed:.4f}rad/s"
            )

        started = time.monotonic()
        duration = args.ramp_duration + args.hold_duration
        print(
            f"[baseline] 开始：{args.ramp_duration:.1f}s 平滑进入默认姿态 + "
            f"{args.hold_duration:.1f}s 保持；不运行 Tracker",
            flush=True,
        )
        while True:
            elapsed = time.monotonic() - started
            if elapsed >= duration:
                break
            phase = min(elapsed / args.ramp_duration, 1.0)
            blend = phase * phase * (3.0 - 2.0 * phase)
            q_target = q_start + blend * delta
            state = controller.latest_state
            times.append(elapsed)
            phases.append("ramp" if elapsed < args.ramp_duration else "hold")
            positions.append(state.joint_pos.astype(np.float64, copy=True))
            velocities.append(state.joint_vel.astype(np.float64, copy=True))
            efforts.append(state.joint_tau_est.astype(np.float64, copy=True))
            targets.append(q_target.copy())
            controller.set_command(q_target, kp, kd)
            controller.raise_if_estopped()
            time.sleep(contract.CONTROL_DT)
        print("[baseline] 无 Tracker 基线测试正常完成", flush=True)
    except EmergencyStopError as exc:
        exit_code = 2
        print(f"[baseline] 软件急停已锁存：{exc}", flush=True)
    finally:
        _save_csv(output, times, phases, positions, velocities, efforts, targets)
        print(f"[baseline] 原始记录已保存：{output.resolve()}", flush=True)
        _print_summary(times, positions, velocities)
        print(f"[baseline] 进入 {args.zero_torque_hold:.1f}s 零力矩收尾", flush=True)
        controller.close()
        status = controller.status
        print(
            f"[baseline] 已退出；reason={status.reason!r}, "
            f"worker_error={status.worker_error!r}",
            flush=True,
        )
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main(tyro.cli(Args))
