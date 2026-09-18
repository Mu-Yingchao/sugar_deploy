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


def main(args: Args) -> None:
    if args.task not in contract.TASK_CONTRACTS:
        raise ValueError(f"未知任务 {args.task}，可选: {list(contract.TASK_CONTRACTS)}")
    task_contract = contract.TASK_CONTRACTS[args.task]

    scene_path = args.scene
    if scene_path is None:
        candidate = DEFAULT_SCENE.parent / f"{args.task.lower()}_scene.xml"
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
    stats = runner.run(args.control_steps, headless=args.headless, real_time=args.real_time)
    print(
        f"[sugar_deploy] DONE control_steps={stats.control_steps} "
        f"generator_calls={stats.generator_calls} "
        f"mean_tracker_inference_ms={stats.mean_tracker_step_ms:.3f}"
    )


if __name__ == "__main__":
    main(tyro.cli(Args))
