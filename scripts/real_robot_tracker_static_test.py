"""吊架状态下首次验证 SUGAR Tracker 静止 command，禁止直接用于无吊架站立。"""

from __future__ import annotations

from dataclasses import dataclass
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


SAFETY_CONFIRMATION = "ROBOT_SUSPENDED_TRACKER_READY"
INITIAL_OBJECT_POS_B = np.array([1.5039635, 0.0, -0.314], dtype=np.float64)
INITIAL_OBJECT_QUAT_B = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


@dataclass
class Args:
    tracker_checkpoint: Path = Path("/home/user/SUGAR/SUGAR/demo_ckpts/CarryBox/tracker.pt")
    interface: str = "eth0"
    domain_id: int = 0
    device: str = "cpu"
    ramp_duration: float = 10.0
    tracker_duration: float = 5.0
    tracker_blend_duration: float = 5.0
    """从已验证的默认姿态保持力矩平滑混合到 Tracker 力矩所用时间。"""
    effort_scale: float = 0.05
    zero_torque_hold: float = 30.0
    max_tilt: float = 0.35
    max_joint_velocity: float = 2.0
    max_base_gyro: float = 1.0
    max_raw_action: float = 10.0
    safety_confirmation: str = ""


def main(args: Args) -> None:
    if args.safety_confirmation != SAFETY_CONFIRMATION:
        raise SystemExit(
            "拒绝启动：首次 Tracker 测试必须双脚离地悬挂并准备好 L2+Y，再传 "
            f"--safety-confirmation {SAFETY_CONFIRMATION}"
        )
    if not args.tracker_checkpoint.is_file():
        raise SystemExit(f"找不到 Tracker checkpoint：{args.tracker_checkpoint}")
    if args.ramp_duration <= 0 or args.tracker_duration <= 0 or args.tracker_blend_duration <= 0:
        raise SystemExit("ramp-duration、tracker-duration 和 tracker-blend-duration 必须大于 0")
    # 10% 直接切换曾触发 2rad/s 速度保护；修正版在重新验收前硬限制为 5%。
    if not 0.0 < args.effort_scale <= 0.05:
        raise SystemExit("当前 Tracker 复测 effort-scale 只能在 (0, 0.05] 内")

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

    try:
        initial = controller.start()
        q_start = initial.joint_pos.astype(np.float64, copy=True)
        print(f"[tracker-test] 先用 {args.ramp_duration:.1f}s 平滑进入默认姿态", flush=True)
        started = time.monotonic()
        while True:
            elapsed = time.monotonic() - started
            if elapsed >= args.ramp_duration:
                break
            phase = min(elapsed / args.ramp_duration, 1.0)
            blend = phase * phase * (3.0 - 2.0 * phase)
            controller.set_command(q_start + blend * (default - q_start), kp, kd)
            controller.raise_if_estopped()
            time.sleep(contract.CONTROL_DT)

        robot = real_state_for_tracker(controller.latest_state)
        obs_builder = TrackerObsBuilder()
        obs_builder.reset(robot)
        command = np.zeros(contract.COMMAND_DIM, dtype=np.float32)
        command[: contract.NUM_JOINTS] = default.astype(np.float32)
        print(
            f"[tracker-test] 启用 Tracker：blend={args.tracker_blend_duration:.1f}s, "
            f"hold={args.tracker_duration:.1f}s, "
            f"effort_scale={args.effort_scale:.2f}",
            flush=True,
        )
        tracker_started = time.monotonic()
        deadline = tracker_started + args.tracker_blend_duration + args.tracker_duration
        max_seen_raw = 0.0
        max_seen_tau = 0.0
        while time.monotonic() < deadline:
            state = controller.latest_state
            robot = real_state_for_tracker(state)
            obs = obs_builder.build(
                robot, INITIAL_OBJECT_POS_B, INITIAL_OBJECT_QUAT_B, command
            )
            raw_action = tracker.act(torch.from_numpy(obs).float()).cpu().numpy()[0]
            if not np.all(np.isfinite(raw_action)):
                controller.emergency_stop("Tracker 输出含 NaN/Inf")
                controller.raise_if_estopped()
            raw_abs = float(np.max(np.abs(raw_action)))
            max_seen_raw = max(max_seen_raw, raw_abs)
            if raw_abs > args.max_raw_action:
                controller.emergency_stop(
                    f"Tracker 原始动作 {raw_abs:.3f} 超过限制 {args.max_raw_action:.3f}"
                )
                controller.raise_if_estopped()
            desired = default + np.asarray(contract.JOINT_ACTION_SCALE) * raw_action
            _, _, limited_tau = effort_limited_position_target(
                desired,
                state.joint_pos,
                state.joint_vel,
                kp,
                kd,
                effort,
                args.effort_scale,
            )
            # 不能从默认姿态保持瞬间切到策略力矩。先算出已验收的默认姿态 PD 力矩，再用
            # smoothstep 平滑混合到受 5% 限制的 Tracker 力矩，最后反解为等效 q 命令。
            baseline_tau = np.clip(
                kp * (default - state.joint_pos) - kd * state.joint_vel,
                -effort,
                effort,
            )
            blend_phase = min(
                (time.monotonic() - tracker_started) / args.tracker_blend_duration, 1.0
            )
            blend = blend_phase * blend_phase * (3.0 - 2.0 * blend_phase)
            blended_tau = (1.0 - blend) * baseline_tau + blend * limited_tau
            blended_q = state.joint_pos + (blended_tau + kd * state.joint_vel) / kp
            max_seen_tau = max(max_seen_tau, float(np.max(np.abs(blended_tau))))
            controller.set_command(blended_q, kp, kd)
            controller.raise_if_estopped()
            time.sleep(contract.CONTROL_DT)
            obs_builder.step(real_state_for_tracker(controller.latest_state), raw_action)

        print(
            f"[tracker-test] Tracker 悬挂测试完成：max|raw_action|={max_seen_raw:.3f}, "
            f"max|limited_tau|={max_seen_tau:.3f}Nm",
            flush=True,
        )
    except EmergencyStopError as exc:
        print(f"[tracker-test] 软件急停已锁存：{exc}", flush=True)
        raise SystemExit(2) from None
    finally:
        print(f"[tracker-test] 进入 {args.zero_torque_hold:.1f}s 零力矩收尾", flush=True)
        controller.close()
        status = controller.status
        print(
            f"[tracker-test] 已退出；reason={status.reason!r}, worker_error={status.worker_error!r}",
            flush=True,
        )


if __name__ == "__main__":
    main(tyro.cli(Args))
