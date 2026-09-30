"""真机 Tracker 影子测试：运行推理和记录，但绝不把策略输出发送给电机。"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import time

import numpy as np
import torch
import tyro

from sugar_deploy import contract
from sugar_deploy.observation import TrackerObsBuilder
from sugar_deploy.policy import TrackerPolicy
from sugar_deploy.real_robot_safety import EmergencyStopError, SafeRealRobotController
from sugar_deploy.real_robot_tracker import effort_limited_position_target, real_state_for_tracker


SAFETY_CONFIRMATION = "ROBOT_SUSPENDED_TRACKER_SHADOW_READY"
INITIAL_OBJECT_POS_B = np.array([1.5039635, 0.0, -0.314], dtype=np.float64)
INITIAL_OBJECT_QUAT_B = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


@dataclass
class Args:
    tracker_checkpoint: Path = Path("/home/user/SUGAR/SUGAR/demo_ckpts/CarryBox/tracker.pt")
    interface: str = "eth0"
    domain_id: int = 0
    device: str = "cpu"
    ramp_duration: float = 10.0
    shadow_duration: float = 10.0
    effort_scale: float = 0.05
    max_target_speed: float = 0.15
    max_raw_action: float = 10.0
    zero_torque_hold: float = 30.0
    max_tilt: float = 0.35
    max_joint_velocity: float = 2.0
    max_base_gyro: float = 1.0
    output: Path | None = None
    safety_confirmation: str = ""


def _save_csv(path: Path, samples: list[tuple[object, ...]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = ["time_s"]
    for prefix in (
        "q",
        "dq",
        "tau_est",
        "raw_action",
        "q_desired",
        "tau_unclipped",
        "tau_limited",
    ):
        fields.extend(f"{prefix}.{name}" for name in contract.JOINT_NAMES)
    fields.extend(f"base_quat_wxyz.{i}" for i in range(4))
    fields.extend(f"base_gyro.{i}" for i in range(3))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(fields)
        for sample in samples:
            writer.writerow(np.concatenate(([sample[0]], *sample[1:])))


def _peak_line(label: str, values: np.ndarray, times: np.ndarray, unit: str) -> str:
    flat = int(np.argmax(np.abs(values)))
    sample_index, joint_index = np.unravel_index(flat, values.shape)
    return (
        f"[shadow] {label}={values[sample_index, joint_index]:+.3f}{unit} "
        f"({contract.JOINT_NAMES[joint_index]}, t={times[sample_index]:.3f}s)"
    )


def _print_summary(samples: list[tuple[object, ...]], default: np.ndarray) -> None:
    if not samples:
        print("[shadow] 没有采集到 Tracker 影子样本", flush=True)
        return
    times = np.asarray([sample[0] for sample in samples], dtype=np.float64)
    raw = np.stack([sample[4] for sample in samples]).astype(np.float64)
    desired = np.stack([sample[5] for sample in samples]).astype(np.float64)
    tau_unclipped = np.stack([sample[6] for sample in samples]).astype(np.float64)
    tau_limited = np.stack([sample[7] for sample in samples]).astype(np.float64)
    desired_delta = desired - default
    print(_peak_line("max raw_action", raw, times, ""), flush=True)
    print(_peak_line("max|q_desired-default|", desired_delta, times, "rad"), flush=True)
    print(_peak_line("max unclipped torque", tau_unclipped, times, "Nm"), flush=True)
    print(_peak_line("max 5% limited torque", tau_limited, times, "Nm"), flush=True)
    ankle = contract.JOINT_NAMES.index("left_ankle_roll_joint")
    print(
        f"[shadow] 左踝横滚策略输出：raw_action=[{raw[:, ankle].min():+.3f}, "
        f"{raw[:, ankle].max():+.3f}], "
        f"q_desired-default=[{desired_delta[:, ankle].min():+.3f}, "
        f"{desired_delta[:, ankle].max():+.3f}]rad, "
        f"tau_limited=[{tau_limited[:, ankle].min():+.3f}, "
        f"{tau_limited[:, ankle].max():+.3f}]Nm",
        flush=True,
    )


def main(args: Args) -> None:
    if args.safety_confirmation != SAFETY_CONFIRMATION:
        raise SystemExit(
            "拒绝启动：确认吊架承担全部重量、双脚完全离地且第二个人准备按 L2+Y 后，再传 "
            f"--safety-confirmation {SAFETY_CONFIRMATION}"
        )
    if not args.tracker_checkpoint.is_file():
        raise SystemExit(f"找不到 Tracker checkpoint：{args.tracker_checkpoint}")
    if args.ramp_duration <= 0 or args.shadow_duration <= 0:
        raise SystemExit("--ramp-duration 和 --shadow-duration 必须大于 0")
    if not 0.0 < args.effort_scale <= 0.05:
        raise SystemExit("影子测试 effort-scale 必须在 (0, 0.05] 内")
    if args.max_target_speed <= 0 or args.max_raw_action <= 0:
        raise SystemExit("速度和动作限制必须大于 0")
    if args.zero_torque_hold < 1.0:
        raise SystemExit("--zero-torque-hold 不得小于 1 秒")

    output = args.output
    if output is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output = Path("diagnostics") / f"tracker_shadow_{stamp}.csv"

    tracker = TrackerPolicy.load(args.tracker_checkpoint, device=args.device)
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
    effort = np.asarray(contract.JOINT_EFFORT_LIMIT, dtype=np.float64)
    default = np.asarray(contract.DEFAULT_JOINT_POS, dtype=np.float64)
    samples: list[tuple[object, ...]] = []
    exit_code = 0

    try:
        initial = controller.start()
        q_start = initial.joint_pos.astype(np.float64, copy=True)
        delta = default - q_start
        max_index = int(np.argmax(np.abs(delta)))
        planned_peak_speed = 1.5 * float(np.max(np.abs(delta))) / args.ramp_duration
        print(
            f"[shadow] 默认姿态过渡：max_delta={abs(delta[max_index]):.4f}rad "
            f"({contract.JOINT_NAMES[max_index]}), "
            f"planned_peak_speed={planned_peak_speed:.4f}rad/s",
            flush=True,
        )
        if planned_peak_speed > args.max_target_speed:
            raise SystemExit(
                f"拒绝启动：规划峰值目标速度 {planned_peak_speed:.4f}rad/s 超过 "
                f"{args.max_target_speed:.4f}rad/s"
            )

        ramp_started = time.monotonic()
        while True:
            elapsed = time.monotonic() - ramp_started
            if elapsed >= args.ramp_duration:
                break
            phase = min(elapsed / args.ramp_duration, 1.0)
            blend = phase * phase * (3.0 - 2.0 * phase)
            controller.set_command(q_start + blend * delta, kp, kd)
            controller.raise_if_estopped()
            time.sleep(contract.CONTROL_DT)

        robot = real_state_for_tracker(controller.latest_state)
        obs_builder = TrackerObsBuilder()
        obs_builder.reset(robot)
        command = np.zeros(contract.COMMAND_DIM, dtype=np.float32)
        command[: contract.NUM_JOINTS] = default.astype(np.float32)
        print(
            f"[shadow] 开始 {args.shadow_duration:.1f}s Tracker 影子推理；"
            "电机继续接收固定默认姿态，策略输出不会下发",
            flush=True,
        )
        shadow_started = time.monotonic()
        while True:
            elapsed = time.monotonic() - shadow_started
            if elapsed >= args.shadow_duration:
                break
            state = controller.latest_state
            robot = real_state_for_tracker(state)
            obs = obs_builder.build(
                robot, INITIAL_OBJECT_POS_B, INITIAL_OBJECT_QUAT_B, command
            )
            raw_action = tracker.act(torch.from_numpy(obs).float()).cpu().numpy()[0]
            if not np.all(np.isfinite(raw_action)):
                controller.emergency_stop("Tracker 影子输出含 NaN/Inf")
                controller.raise_if_estopped()
            raw_abs = float(np.max(np.abs(raw_action)))
            if raw_abs > args.max_raw_action:
                controller.emergency_stop(
                    f"Tracker 影子原始动作 {raw_abs:.3f} 超过限制 {args.max_raw_action:.3f}"
                )
                controller.raise_if_estopped()
            desired = default + np.asarray(contract.JOINT_ACTION_SCALE) * raw_action
            _, tau_unclipped, tau_limited = effort_limited_position_target(
                desired,
                state.joint_pos,
                state.joint_vel,
                kp,
                kd,
                effort,
                args.effort_scale,
            )
            samples.append(
                (
                    elapsed,
                    state.joint_pos.astype(np.float64, copy=True),
                    state.joint_vel.astype(np.float64, copy=True),
                    state.joint_tau_est.astype(np.float64, copy=True),
                    raw_action.astype(np.float64, copy=True),
                    desired.astype(np.float64, copy=True),
                    tau_unclipped.astype(np.float64, copy=True),
                    tau_limited.astype(np.float64, copy=True),
                    state.base_quat_wxyz.astype(np.float64, copy=True),
                    state.base_gyro.astype(np.float64, copy=True),
                )
            )
            # 影子测试的核心安全属性：这里永远只发送默认姿态，不发送 desired/q_limited。
            controller.set_command(default, kp, kd)
            controller.raise_if_estopped()
            time.sleep(contract.CONTROL_DT)
            obs_builder.step(real_state_for_tracker(controller.latest_state), raw_action)
        print("[shadow] Tracker 影子测试正常完成", flush=True)
    except EmergencyStopError as exc:
        exit_code = 2
        print(f"[shadow] 软件急停已锁存：{exc}", flush=True)
    finally:
        _save_csv(output, samples)
        print(f"[shadow] 原始记录已保存：{output.resolve()}", flush=True)
        _print_summary(samples, default)
        print(f"[shadow] 进入 {args.zero_torque_hold:.1f}s 零力矩收尾", flush=True)
        controller.close()
        status = controller.status
        print(
            f"[shadow] 已退出；reason={status.reason!r}, "
            f"worker_error={status.worker_error!r}",
            flush=True,
        )
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main(tyro.cli(Args))
