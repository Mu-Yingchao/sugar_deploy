"""SUGAR Tracker+Generator 的 MuJoCo sim2sim 入口。

用法：
    python scripts/run_sim2sim.py --task CarryBox \\
        --tracker-checkpoint .../tracker.pt --generator-checkpoint .../generator.ckpt

必须在装了 sugar_rl/sugar_il/rsl_rl 的那个 venv 下跑（复用 SUGAR 训练用的环境，
额外 `uv pip install mujoco tyro` 即可，不需要单独的 sugar_deploy venv）。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import tyro

from sugar_deploy import contract
from sugar_deploy.policy import GeneratorPolicy, TrackerPolicy
from sugar_deploy.sim2sim import DEFAULT_SCENE, SugarSim2Sim


@dataclass
class Args:
    task: str = "CarryBox"
    """SUGAR 六个任务之一，目前只有 CarryBox 有配好的 MuJoCo 场景
    （assets/g1/carrybox_scene.xml），其余任务需要照着这个再加一个物体 body。"""

    tracker_checkpoint: Path = Path("../SUGAR/demo_ckpts/CarryBox/tracker.pt")
    generator_checkpoint: Path = Path("../SUGAR/demo_ckpts/CarryBox/generator.ckpt")

    scene: Path | None = None
    """MuJoCo 场景 xml，不指定则按 --task 用 assets/g1/{task_lower}_scene.xml
    的默认约定（目前只有 carrybox_scene.xml 存在）。"""

    control_steps: int = 500
    """跑多少个 50Hz 控制步；500 步 = 10 秒。"""

    headless: bool = False
    """不开 MuJoCo viewer，适合服务器 smoke test。"""

    real_time: bool = True
    """按 50Hz 墙钟节拍运行；关闭后尽快执行（validate/smoke 用）。"""

    device: str = "cpu"
    """两个 policy 跑在哪个 device 上；MLP 和 DiT 都不大，CPU 通常够用。"""

    target_offset_x: float = 1.0
    """use_target=True 的任务（CarryBox/PushBox）：目标位置 = 初始物体位置 +
    (target_offset_x, 0, 0)。这是占位值，不代表论文里真实的目标分布，
    第一版只用来验证 pipeline 跑得通、command 会响应目标变化。"""

    validate_only: bool = False
    """只加载 checkpoint、校验维度，不跑仿真。"""

    object_source: str = "ground_truth"
    """物体状态来源：
    - "ground_truth"（默认）：MujocoGroundTruthSource，读仿真真值，sim2sim 契约自验证用。
    - "apriltag"：AprilTagObjectSource，胸前相机渲染 + AprilTag 检测解算物体位姿，用来在
      不接触真实硬件的前提下验证 AprilTag 感知方案能不能撑住一次真实的闭环控制（机器人
      真的用视觉估计出来的箱子位置去决定动作，不是只在旁边观测）。只有配了相机+贴了 tag
      的场景能用（目前是 carrybox_scene_apriltag.xml），--scene 不指定时会自动切过去。
      详细原理、标定方法、精度实测数据见 sugar_deploy 仓库的 AprilTag 部署文档。"""


def main(args: Args) -> None:
    if args.task not in contract.TASK_CONTRACTS:
        raise ValueError(f"未知任务 {args.task}，可选: {list(contract.TASK_CONTRACTS)}")
    task_contract = contract.TASK_CONTRACTS[args.task]
    if args.object_source not in ("ground_truth", "apriltag"):
        raise ValueError(f"未知 --object-source {args.object_source}，可选: ground_truth, apriltag")

    scene_path = args.scene
    if scene_path is None:
        suffix = "_apriltag" if args.object_source == "apriltag" else ""
        candidate = DEFAULT_SCENE.parent / f"{args.task.lower()}_scene{suffix}.xml"
        if not candidate.exists():
            raise FileNotFoundError(
                f"没有找到 {args.task} 对应的 MuJoCo 场景 ({candidate})。"
                "目前只配好了 CarryBox，其他任务需要照着 assets/g1/carrybox_scene.xml"
                "加一个对应形状的物体 body。"
            )
        scene_path = candidate

    print(f"[sugar_deploy] task={args.task} scene={scene_path}")
    tracker = TrackerPolicy.load(args.tracker_checkpoint, device=args.device)
    generator = GeneratorPolicy.load(args.generator_checkpoint, device=args.device)
    print(
        f"[sugar_deploy] generator: use_target={generator.use_target} "
        f"use_last_action={generator.use_last_action} n_action_steps={generator.n_action_steps}"
    )
    if generator.use_target != task_contract.use_target_default:
        print(
            f"[sugar_deploy] 注意：checkpoint 实际 use_target={generator.use_target}，"
            f"和 contract.py 里 {args.task} 的默认猜测 {task_contract.use_target_default} 不一致，"
            "以 checkpoint 为准，这条只是提醒。"
        )

    if args.validate_only:
        print("[sugar_deploy] VALIDATE_ONLY=PASS")
        return

    runner = SugarSim2Sim(
        scene_path=scene_path, tracker=tracker, generator=generator, task=task_contract,
        device=args.device, target_offset_w=__import__("numpy").array([args.target_offset_x, 0.0, 0.0]),
    )

    if args.object_source == "apriltag":
        # 只有在这里才 import，避免 --object-source ground_truth（默认路径）也强制要求
        # 装 pupil_apriltags。model/data 要等 SugarSim2Sim.__post_init__ 建完场景之后才
        # 存在，所以不能在构造 runner 之前传进去，只能构造完之后原地替换
        # runner.object_source（见 sim2sim.py 里 object_source_override 字段的说明）。
        from sugar_deploy.apriltag_sim_calibration import TAG_OFFSETS, TAG_SIZE_M
        from sugar_deploy.camera_source import MujocoCameraSource
        from sugar_deploy.object_state import AprilTagObjectSource

        camera_source = MujocoCameraSource(runner.model, runner.data, "chest_cam")
        # max_stale_frames 在这里特意调大：实测过（见 AprilTag 部署文档）机器人走到贴近箱子、
        # 准备伸手的最后 0.2~0.4m 时，固定俯仰角的胸前相机会因为视场角覆盖不到而看不见贴在
        # 箱子上的 tag——这不是检测算法的问题，是"固定倾角相机在极近距离下的视场盲区"，
        # 真实机械臂/人形抓取系统里很常见的问题，通常靠加一个手腕相机覆盖最后一段来解决。
        # 这里没有手腕相机，先把 max_stale_frames 调大到能撑过这段盲区（零阶保持最后一次
        # 看到的箱子位置），让 demo 能跑完展示"看得见的时候准不准"，不代表这个盲区问题已经
        # 解决了——真实部署这个问题必须正视，见部署文档"已知限制"一节。
        apriltag_source = AprilTagObjectSource(
            camera_source, tag_size_m=TAG_SIZE_M, tag_offsets=TAG_OFFSETS, max_stale_frames=200,
        )

        def _update_apriltag() -> None:
            cam_pos_w, cam_rot_w = camera_source.get_camera_pose_w()
            apriltag_source.set_camera_pose_w(cam_pos_w, cam_rot_w)
            apriltag_source.update()

        runner.object_source = apriltag_source
        runner.pre_step_hook = _update_apriltag
        print("[sugar_deploy] object_source=apriltag（胸前相机 + AprilTag 检测，不是仿真真值）")

    stats = runner.run(args.control_steps, headless=args.headless, real_time=args.real_time)
    print(
        f"[sugar_deploy] DONE control_steps={stats.control_steps} "
        f"generator_calls={stats.generator_calls} "
        f"mean_tracker_inference_ms={stats.mean_tracker_step_ms:.3f}"
    )


if __name__ == "__main__":
    main(tyro.cli(Args))
