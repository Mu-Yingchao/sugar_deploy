"""官方 Generator + Tracker 联合影子测试；策略输出绝不发送给电机。"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import queue
import threading
import time

import numpy as np
import torch
import tyro

from sugar_deploy import contract
from sugar_deploy.observation import RobotState, TrackerObsBuilder, build_generator_obs
from sugar_deploy.policy import GeneratorPolicy, TrackerPolicy
from sugar_deploy.real_robot_safety import EmergencyStopError, SafeRealRobotController
from sugar_deploy.real_robot_tracker import (
    blend_tracker_torque,
    effort_limited_position_target,
    real_state_for_tracker,
)


SAFETY_CONFIRMATION = "ROBOT_SUSPENDED_GENERATOR_SHADOW_READY"
CONTACT_SAFETY_CONFIRMATION = "ROBOT_HOISTED_CONTACT_SHADOW_READY"
MIX_SAFETY_CONFIRMATION = "ROBOT_HOISTED_TRACKER_MIX_READY"
OBJECT_POS_B = np.array([1.5039635, 0.0, -0.314], dtype=np.float64)
OBJECT_QUAT_B = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


@dataclass
class Args:
    tracker_checkpoint: Path = Path("/home/user/SUGAR/SUGAR/demo_ckpts/CarryBox/tracker.pt")
    generator_checkpoint: Path = Path("/home/user/SUGAR/SUGAR/demo_ckpts/CarryBox/generator.ckpt")
    interface: str = "eth0"
    domain_id: int = 0
    device: str = "cpu"
    generator_seed: int = 0
    target_offset_x: float = 1.0
    ramp_duration: float = 10.0
    shadow_steps: int = contract.GENERATOR_CALL_INTERVAL
    effort_scale: float = 0.05
    tracker_mix: float = 0.0
    """力矩空间中从默认姿态保持到满额限幅 Tracker 的混合比例；当前验收最多 0.02。"""
    max_target_speed: float = 0.15
    max_raw_action: float = 10.0
    max_command_joint_delta: float = 1.0
    zero_torque_hold: float = 30.0
    max_tilt: float = 0.35
    max_joint_velocity: float = 2.0
    max_base_gyro: float = 1.0
    contact_settle_duration: float = 1.0
    contact_gate_attempts: int = 3
    contact_pitch_offset_min: float = -0.23
    contact_pitch_offset_max: float = -0.10
    contact_pitch_asymmetry: float = 0.06
    contact_max_abs_roll: float = 0.06
    contact_max_joint_velocity: float = 0.15
    contact_max_base_gyro: float = 0.15
    contact_max_tilt: float = 0.20
    contact_sequence: bool = False
    """启用交互式“脚轻触后测试、重新吊起后卸力”流程。"""
    output: Path | None = None
    safety_confirmation: str = ""


def _canonical_robot() -> RobotState:
    return RobotState(
        joint_pos=np.asarray(contract.DEFAULT_JOINT_POS, dtype=np.float64),
        joint_vel=np.zeros(contract.NUM_JOINTS),
        base_quat_w=np.array([1.0, 0.0, 0.0, 0.0]),
        base_ang_vel_w=np.zeros(3),
        anchor_pos_w=np.zeros(3),
        anchor_quat_w=np.array([1.0, 0.0, 0.0, 0.0]),
    )


def _generate_and_validate_chunk(args: Args) -> tuple[TrackerPolicy, np.ndarray]:
    tracker = TrackerPolicy.load(args.tracker_checkpoint, device=args.device)
    generator = GeneratorPolicy.load(args.generator_checkpoint, device=args.device)
    if not generator.use_target or not generator.use_last_action:
        raise SystemExit(
            "CarryBox Generator 配置不符合预期：必须 use_target=True 且 use_last_action=True"
        )
    torch.manual_seed(args.generator_seed)
    target_pos_b = OBJECT_POS_B + np.array([args.target_offset_x, 0.0, 0.0])
    obs = build_generator_obs(
        _canonical_robot(),
        OBJECT_POS_B,
        OBJECT_QUAT_B,
        last_command_36=np.zeros(contract.COMMAND_DIM),
        use_target=generator.use_target,
        use_last_action=generator.use_last_action,
        target_obj_pos_w=target_pos_b,
        target_obj_quat_w=OBJECT_QUAT_B,
    )
    chunk = generator.predict(obs)[0].detach().cpu().numpy().astype(np.float64)
    if chunk.ndim != 2 or chunk.shape[1] != contract.COMMAND_DIM:
        raise SystemExit(f"Generator command 形状错误：{chunk.shape}")
    if args.shadow_steps <= 0 or args.shadow_steps > min(len(chunk), contract.GENERATOR_CALL_INTERVAL):
        raise SystemExit(
            f"--shadow-steps 必须在 [1,{min(len(chunk), contract.GENERATOR_CALL_INTERVAL)}] 内"
        )
    selected = chunk[: args.shadow_steps]
    if not np.all(np.isfinite(selected)):
        raise SystemExit("Generator command 含 NaN/Inf")
    default = np.asarray(contract.DEFAULT_JOINT_POS)
    joint_delta = selected[:, : contract.NUM_JOINTS] - default
    max_delta = float(np.max(np.abs(joint_delta)))
    if max_delta > args.max_command_joint_delta:
        raise SystemExit(
            f"Generator 参考关节偏移 {max_delta:.3f}rad 超过限制 "
            f"{args.max_command_joint_delta:.3f}rad"
        )
    if np.max(np.abs(selected[:, 29:32])) > 0.5:
        raise SystemExit("Generator 参考线速度超过 0.5m/s")
    if np.max(np.abs(selected[:, 32:35])) > 3.0:
        raise SystemExit("Generator 参考角速度超过 3.0rad/s")
    if np.min(selected[:, 35]) < -0.1 or np.max(selected[:, 35]) > 1.1:
        raise SystemExit("Generator contact_label 超出 [-0.1,1.1]")
    flat = int(np.argmax(np.abs(joint_delta)))
    frame, joint = np.unravel_index(flat, joint_delta.shape)
    print(
        f"[generator-shadow] command 已通过启动前校验：shape={chunk.shape}, "
        f"max|ref-default|={abs(joint_delta[frame, joint]):.3f}rad "
        f"({contract.JOINT_NAMES[joint]}, frame={frame})",
        flush=True,
    )
    return tracker, selected


def _save_csv(path: Path, samples: list[tuple[object, ...]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = ["time_s", "step"]
    fields.extend(f"command.{i}" for i in range(contract.COMMAND_DIM))
    for prefix in (
        "q",
        "dq",
        "raw_action",
        "q_desired",
        "tau_unclipped",
        "tau_limited",
        "baseline_tau",
        "tracker_tau_full",
        "applied_tau",
        "q_command",
    ):
        fields.extend(f"{prefix}.{name}" for name in contract.JOINT_NAMES)
    fields.extend(f"base_quat_wxyz.{i}" for i in range(4))
    fields.extend(f"base_gyro.{i}" for i in range(3))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(fields)
        for sample in samples:
            writer.writerow(np.concatenate(([sample[0], sample[1]], *sample[2:])))


def _summary(samples: list[tuple[object, ...]], default: np.ndarray) -> None:
    if not samples:
        print("[generator-shadow] 没有采集到样本", flush=True)
        return
    times = np.asarray([s[0] for s in samples])
    raw = np.stack([s[5] for s in samples])
    desired = np.stack([s[6] for s in samples])
    limited = np.stack([s[8] for s in samples])
    baseline = np.stack([s[9] for s in samples])
    applied = np.stack([s[11] for s in samples])
    delta = desired - default
    for label, values, unit in (
        ("max raw_action", raw, ""),
        ("max|q_desired-default|", delta, "rad"),
        ("max 5% limited torque", limited, "Nm"),
    ):
        flat = int(np.argmax(np.abs(values)))
        sample, joint = np.unravel_index(flat, values.shape)
        print(
            f"[generator-shadow] {label}={values[sample, joint]:+.3f}{unit} "
            f"({contract.JOINT_NAMES[joint]}, t={times[sample]:.3f}s)",
            flush=True,
        )
    ankle = contract.JOINT_NAMES.index("left_ankle_roll_joint")
    print(
        f"[generator-shadow] 左踝横滚：q_desired-default="
        f"[{delta[:, ankle].min():+.3f},{delta[:, ankle].max():+.3f}]rad, "
        f"tau_limited=[{limited[:, ankle].min():+.3f},{limited[:, ankle].max():+.3f}]Nm",
        flush=True,
    )
    correction = applied - baseline
    flat = int(np.argmax(np.abs(correction)))
    sample, joint = np.unravel_index(flat, correction.shape)
    print(
        f"[generator-shadow] max|applied-baseline|="
        f"{abs(correction[sample, joint]):.3f}Nm "
        f"({contract.JOINT_NAMES[joint]}, t={times[sample]:.3f}s)",
        flush=True,
    )


def _wait_for_operator(
    controller: SafeRealRobotController,
    default: np.ndarray,
    kp: np.ndarray,
    kd: np.ndarray,
    expected: str,
    prompt: str,
) -> None:
    """等待 stdin 确认，同时持续喂默认姿态心跳；错误输入立即锁存急停。"""
    replies: queue.SimpleQueue[str] = queue.SimpleQueue()

    def _read() -> None:
        try:
            replies.put(input().strip())
        except EOFError:
            replies.put("__EOF__")

    print(f"[generator-shadow] {prompt}", flush=True)
    print(f"[generator-shadow] 等待确认：{expected}", flush=True)
    threading.Thread(target=_read, name=f"wait-{expected}", daemon=True).start()
    while True:
        controller.set_command(default, kp, kd)
        controller.raise_if_estopped()
        try:
            reply = replies.get_nowait()
        except queue.Empty:
            time.sleep(contract.CONTROL_DT)
            continue
        if reply != expected:
            controller.emergency_stop(f"现场确认输入错误：期望 {expected!r}，收到 {reply!r}")
            controller.raise_if_estopped()
        print(f"[generator-shadow] 已收到确认：{expected}", flush=True)
        return


def _contact_gate(
    controller: SafeRealRobotController,
    default: np.ndarray,
    kp: np.ndarray,
    kd: np.ndarray,
    args: Args,
) -> str | None:
    """保持默认姿态采样接触状态；返回 None 表示通过，否则返回拒绝原因。"""
    q_samples: list[np.ndarray] = []
    dq_samples: list[np.ndarray] = []
    gyro_samples: list[np.ndarray] = []
    tilt_samples: list[float] = []
    deadline = time.monotonic() + args.contact_settle_duration
    while time.monotonic() < deadline:
        controller.set_command(default, kp, kd)
        controller.raise_if_estopped()
        state = controller.latest_state
        q_samples.append(state.joint_pos.astype(np.float64, copy=True))
        dq_samples.append(state.joint_vel.astype(np.float64, copy=True))
        gyro_samples.append(state.base_gyro.astype(np.float64, copy=True))
        quat = state.base_quat_wxyz.astype(np.float64, copy=True)
        quat /= np.linalg.norm(quat)
        tilt_samples.append(
            float(np.arccos(np.clip(1.0 - 2.0 * (quat[1] ** 2 + quat[2] ** 2), -1.0, 1.0)))
        )
        time.sleep(contract.CONTROL_DT)

    q = np.mean(np.stack(q_samples), axis=0)
    dq_peak = float(np.max(np.abs(np.stack(dq_samples))))
    gyro_peak = float(np.max(np.abs(np.stack(gyro_samples))))
    tilt_peak = float(np.max(tilt_samples))
    left_pitch = contract.JOINT_NAMES.index("left_ankle_pitch_joint")
    right_pitch = contract.JOINT_NAMES.index("right_ankle_pitch_joint")
    left_roll = contract.JOINT_NAMES.index("left_ankle_roll_joint")
    right_roll = contract.JOINT_NAMES.index("right_ankle_roll_joint")
    pitch_offsets = q[[left_pitch, right_pitch]] - default[[left_pitch, right_pitch]]
    roll_offsets = q[[left_roll, right_roll]] - default[[left_roll, right_roll]]
    print(
        "[generator-shadow] 接触门限测量："
        f"pitch_offset=[{pitch_offsets[0]:+.4f},{pitch_offsets[1]:+.4f}]rad, "
        f"roll_offset=[{roll_offsets[0]:+.4f},{roll_offsets[1]:+.4f}]rad, "
        f"max|dq|={dq_peak:.4f}rad/s, max|gyro|={gyro_peak:.4f}rad/s, "
        f"tilt={tilt_peak:.4f}rad",
        flush=True,
    )
    if np.any(pitch_offsets < args.contact_pitch_offset_min) or np.any(
        pitch_offsets > args.contact_pitch_offset_max
    ):
        return (
            f"踝 pitch 偏差 {pitch_offsets.tolist()} 不在 "
            f"[{args.contact_pitch_offset_min},{args.contact_pitch_offset_max}]rad"
        )
    if abs(float(pitch_offsets[0] - pitch_offsets[1])) > args.contact_pitch_asymmetry:
        return "左右踝 pitch 不对称超过门限"
    if float(np.max(np.abs(roll_offsets))) > args.contact_max_abs_roll:
        return "踝 roll 偏差超过门限"
    if dq_peak > args.contact_max_joint_velocity:
        return "接触状态关节速度超过门限"
    if gyro_peak > args.contact_max_base_gyro:
        return "接触状态机身角速度超过门限"
    if tilt_peak > args.contact_max_tilt:
        return "接触状态机身倾角超过门限"
    return None


def main(args: Args) -> None:
    if args.tracker_mix > 0:
        required_confirmation = MIX_SAFETY_CONFIRMATION
    else:
        required_confirmation = (
            CONTACT_SAFETY_CONFIRMATION if args.contact_sequence else SAFETY_CONFIRMATION
        )
    if args.safety_confirmation != required_confirmation:
        raise SystemExit(
            "拒绝启动：确认吊架和手柄条件后，再传 "
            f"--safety-confirmation {required_confirmation}"
        )
    for path in (args.tracker_checkpoint, args.generator_checkpoint):
        if not path.is_file():
            raise SystemExit(f"找不到 checkpoint：{path}")
    if args.ramp_duration <= 0 or args.max_target_speed <= 0:
        raise SystemExit("过渡时间和目标速度限制必须大于 0")
    if not 0.0 < args.effort_scale <= 0.05:
        raise SystemExit("影子分析 effort-scale 必须在 (0,0.05] 内")
    if not 0.0 <= args.tracker_mix <= 0.02:
        raise SystemExit("当前力矩混合 --tracker-mix 必须在 [0,0.02] 内")
    if args.tracker_mix > 0 and not args.contact_sequence:
        raise SystemExit("启用 Tracker 力矩混合时必须同时启用 --contact-sequence")
    if args.zero_torque_hold < 1.0:
        raise SystemExit("--zero-torque-hold 不得小于 1 秒")
    if args.contact_settle_duration < 0.5:
        raise SystemExit("--contact-settle-duration 不得小于 0.5 秒")
    if not 1 <= args.contact_gate_attempts <= 3:
        raise SystemExit("--contact-gate-attempts 必须在 [1,3] 内")

    # 模型加载、扩散采样和 command 校验全部在连接 DDS 之前完成。
    tracker, commands = _generate_and_validate_chunk(args)
    output = args.output
    if output is None:
        output = Path("diagnostics") / (
            f"generator_tracker_shadow_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )

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
        peak_speed = 1.5 * float(np.max(np.abs(delta))) / args.ramp_duration
        print(
            f"[generator-shadow] 默认姿态过渡：max_delta={abs(delta[max_index]):.4f}rad "
            f"({contract.JOINT_NAMES[max_index]}), planned_peak_speed={peak_speed:.4f}rad/s",
            flush=True,
        )
        if peak_speed > args.max_target_speed:
            raise SystemExit(
                f"拒绝启动：规划峰值目标速度 {peak_speed:.4f}rad/s 超过 "
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

        if args.contact_sequence:
            _wait_for_operator(
                controller,
                default,
                kp,
                kd,
                "FEET_CONTACT",
                "现在极慢下放，只让双脚底完整轻触地面；吊带必须继续承担大部分重量",
            )
            gate_reason: str | None = None
            for attempt in range(1, args.contact_gate_attempts + 1):
                gate_reason = _contact_gate(controller, default, kp, kd, args)
                if gate_reason is None:
                    break
                print(
                    f"[generator-shadow] 接触状态门限未通过 "
                    f"({attempt}/{args.contact_gate_attempts})：{gate_reason}",
                    flush=True,
                )
                if attempt < args.contact_gate_attempts:
                    _wait_for_operator(
                        controller,
                        default,
                        kp,
                        kd,
                        "CONTACT_ADJUSTED",
                        "Tracker 尚未介入。请根据门限结果微调吊架高度，吊带继续承担大部分重量",
                    )
            if gate_reason is not None:
                _wait_for_operator(
                    controller,
                    default,
                    kp,
                    kd,
                    "REHOISTED",
                    "未执行 Tracker 混合。请重新升高吊架直到双脚完全离地",
                )
                controller.emergency_stop(f"接触状态门限未通过：{gate_reason}")
                controller.raise_if_estopped()
            print("[generator-shadow] 接触状态门限通过", flush=True)

        builder = TrackerObsBuilder()
        builder.reset(real_state_for_tracker(controller.latest_state))
        print(
            f"[generator-shadow] 开始 {len(commands)} 帧联合"
            f"{'力矩混合' if args.tracker_mix > 0 else '影子推理'}；"
            + (
                f"Tracker 修正从 0 平滑升到 {args.tracker_mix * 100:.1f}%"
                if args.tracker_mix > 0
                else "电机继续保持默认姿态，Generator/Tracker 输出均不下发"
            ),
            flush=True,
        )
        started = time.monotonic()
        next_tick = started
        for step, command in enumerate(commands):
            state = controller.latest_state
            robot = real_state_for_tracker(state)
            obs = builder.build(robot, OBJECT_POS_B, OBJECT_QUAT_B, command)
            raw = tracker.act(torch.from_numpy(obs).float()).cpu().numpy()[0]
            if not np.all(np.isfinite(raw)) or np.max(np.abs(raw)) > args.max_raw_action:
                controller.emergency_stop("Generator+Tracker 影子输出无效或超过动作限制")
                controller.raise_if_estopped()
            desired = default + np.asarray(contract.JOINT_ACTION_SCALE) * raw
            _, tau_raw, tau_limited = effort_limited_position_target(
                desired, state.joint_pos, state.joint_vel, kp, kd, effort, args.effort_scale
            )
            baseline_tau = np.clip(
                kp * (default - state.joint_pos) - kd * state.joint_vel,
                -effort,
                effort,
            )
            mix_phase = (step + 1) / len(commands)
            smooth = mix_phase * mix_phase * (3.0 - 2.0 * mix_phase)
            mix = args.tracker_mix * smooth
            tracker_tau_full, applied_tau = blend_tracker_torque(
                baseline_tau, tau_raw, effort, mix
            )
            q_command = state.joint_pos + (applied_tau + kd * state.joint_vel) / kp
            samples.append(
                (
                    time.monotonic() - started,
                    step,
                    command.copy(),
                    state.joint_pos.astype(np.float64, copy=True),
                    state.joint_vel.astype(np.float64, copy=True),
                    raw.astype(np.float64, copy=True),
                    desired.astype(np.float64, copy=True),
                    tau_raw.astype(np.float64, copy=True),
                    tau_limited.astype(np.float64, copy=True),
                    baseline_tau.astype(np.float64, copy=True),
                    tracker_tau_full.astype(np.float64, copy=True),
                    applied_tau.astype(np.float64, copy=True),
                    q_command.astype(np.float64, copy=True),
                    state.base_quat_wxyz.astype(np.float64, copy=True),
                    state.base_gyro.astype(np.float64, copy=True),
                )
            )
            # tracker_mix=0 是纯影子；首次有控制的测试只允许 20 帧内平滑升到 1%。
            controller.set_command(q_command if args.tracker_mix > 0 else default, kp, kd)
            controller.raise_if_estopped()
            next_tick += contract.CONTROL_DT
            time.sleep(max(0.0, next_tick - time.monotonic()))
            builder.step(real_state_for_tracker(controller.latest_state), raw)
        print("[generator-shadow] 联合影子测试正常完成", flush=True)
        if args.contact_sequence:
            _wait_for_operator(
                controller,
                default,
                kp,
                kd,
                "REHOISTED",
                "影子推理已结束。现在重新升高吊架，直到双脚完全离地；确认后才会零力矩",
            )
    except EmergencyStopError as exc:
        exit_code = 2
        print(f"[generator-shadow] 软件急停已锁存：{exc}", flush=True)
    finally:
        _save_csv(output, samples)
        print(f"[generator-shadow] 原始记录已保存：{output.resolve()}", flush=True)
        _summary(samples, default)
        print(f"[generator-shadow] 进入 {args.zero_torque_hold:.1f}s 零力矩收尾", flush=True)
        controller.close()
        status = controller.status
        print(
            f"[generator-shadow] 已退出；reason={status.reason!r}, "
            f"worker_error={status.worker_error!r}",
            flush=True,
        )
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main(tyro.cli(Args))
