"""AprilTag 感知精度验证：跑一遍真实 sim2sim 闭环（机器人真的走向箱子、伸手），全程用
MujocoGroundTruthSource 控制（不改变已经验证过的行为），同时用同一份 MuJoCo 状态并行跑
AprilTagObjectSource（影子模式，只观测不控制），逐步记录两者的位置/姿态误差。

这是在回答"AprilTag 这条感知链路准不准、能不能撑住一次真实的走近-伸手轨迹"，而不是只测
静态单帧——静态单帧在开发过程中已经验证过（见 sugar_deploy 的 AprilTag 部署文档），这里
测的是运动中、检测有可能因为角度/遮挡短暂失败时的鲁棒性。

用法：
    python scripts/test_apriltag_perception.py --control-steps 300
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import tyro

from sugar_deploy import contract
from sugar_deploy.apriltag_sim_calibration import TAG_OFFSETS, TAG_SIZE_M
from sugar_deploy.camera_source import MujocoCameraSource
from sugar_deploy.object_state import AprilTagObjectSource
from sugar_deploy.policy import GeneratorPolicy, TrackerPolicy
from sugar_deploy.sim2sim import SugarSim2Sim

SCENE_PATH = Path(__file__).resolve().parent.parent / "assets" / "g1" / "carrybox_scene_apriltag.xml"


@dataclass
class Args:
    tracker_checkpoint: Path = Path("../SUGAR/demo_ckpts/CarryBox/tracker.pt")
    generator_checkpoint: Path = Path("../SUGAR/demo_ckpts/CarryBox/generator.ckpt")
    control_steps: int = 300
    device: str = "cpu"
    detect_every: int = 1
    """每隔几个控制步跑一次 AprilTag 检测——真机上检测本身有算力开销，跟不上 50Hz 的话
    可以调大这个值，模拟"检测频率低于控制频率、中间用零阶保持"的真实场景。"""


def main(args: Args) -> None:
    task = contract.TASK_CONTRACTS["CarryBox"]
    tracker = TrackerPolicy.load(args.tracker_checkpoint, device=args.device)
    generator = GeneratorPolicy.load(args.generator_checkpoint, device=args.device)

    runner = SugarSim2Sim(
        scene_path=SCENE_PATH, tracker=tracker, generator=generator, task=task, device=args.device,
    )
    runner.reset()

    camera_source = MujocoCameraSource(runner.model, runner.data, "chest_cam")
    shadow_source = AprilTagObjectSource(camera_source, tag_size_m=TAG_SIZE_M, tag_offsets=TAG_OFFSETS)

    pos_errors: list[float] = []
    rot_errors_deg: list[float] = []
    n_detected = 0
    n_stale_skipped = 0

    # 按 50Hz 真实节拍跑，不是为了实时可视化——是因为 Generator 的后台线程（异步调用，见
    # sim2sim.py 的 _maybe_call_generator）需要主循环有 sleep 让出 GIL 才能被正常调度，
    # 不加这个 pacing 的话 CPU 上会复现之前踩过的坑："跑得太快，后台线程根本抢不到
    # 时间片"，détection 本身没问题但机器人几乎不会动，会污染这次精度测试的轨迹多样性。
    import time

    for step in range(args.control_steps):
        step_start = time.perf_counter()
        runner.control_step()
        remain = contract.CONTROL_DT - (time.perf_counter() - step_start)
        if remain > 0:
            time.sleep(remain)

        if step % args.detect_every != 0:
            continue

        cam_pos_w, cam_rot_w = camera_source.get_camera_pose_w()
        shadow_source.set_camera_pose_w(cam_pos_w, cam_rot_w)
        detected = shadow_source.update()
        if not detected:
            n_stale_skipped += 1
            continue

        n_detected += 1
        est = shadow_source.get_pose()
        gt = runner.object_source.get_pose()

        pos_err = float(np.linalg.norm(est.pos_w - gt.pos_w))
        cos_theta = np.clip((np.trace(est.rot_w.T @ gt.rot_w) - 1.0) / 2.0, -1.0, 1.0)
        rot_err_deg = float(np.degrees(np.arccos(cos_theta)))
        pos_errors.append(pos_err)
        rot_errors_deg.append(rot_err_deg)

    if runner._generator_thread is not None:
        runner._generator_thread.join(timeout=5.0)

    n_frames = args.control_steps // args.detect_every
    print(f"[apriltag-test] 检测帧数 {n_frames}, 采纳 {n_detected} "
          f"({100.0 * n_detected / max(n_frames, 1):.1f}%), 零阶保持 {n_stale_skipped} 帧 "
          f"(其中置信度不够拒绝 {shadow_source.n_rejected_low_margin} 帧, "
          f"位置跳变过大拒绝 {shadow_source.n_rejected_jump} 帧)")
    if pos_errors:
        pos_errors_arr = np.array(pos_errors)
        rot_errors_arr = np.array(rot_errors_deg)
        print(f"[apriltag-test] 位置误差 (m): mean={pos_errors_arr.mean():.4f} "
              f"p95={np.percentile(pos_errors_arr, 95):.4f} max={pos_errors_arr.max():.4f}")
        print(f"[apriltag-test] 姿态误差 (deg): mean={rot_errors_arr.mean():.3f} "
              f"p95={np.percentile(rot_errors_arr, 95):.3f} max={rot_errors_arr.max():.3f}")
    else:
        print("[apriltag-test] 全程一次都没检测到任何已知 tag——检查相机朝向/box 是否在视野内")


if __name__ == "__main__":
    main(tyro.cli(Args))
