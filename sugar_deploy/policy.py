"""加载 SUGAR 的两级策略（Command Tracker + Command Generator），不依赖 IsaacLab。

两个 checkpoint 的加载方式完全不同：

- ``tracker.pt`` 是标准 rsl_rl ``OnPolicyRunner`` 存档（``model_state_dict`` /
  ``optimizer_state_dict`` / ``iter`` / ``infos``），架构（hidden dims/激活函数）
  在 sugar_rl 源码里是写死的常量，这里直接按 contract.py 里核实过的
  ``TRACKER_ACTOR_HIDDEN_DIMS`` 构造 ``rsl_rl.modules.ActorCritic`` 再灌权重。

- ``generator.ckpt`` 是 sugar_il ``BaseWorkspace`` 的存档。**不要自己重新拼装** ——
  一开始我按 `train_generator_workspace.yaml` 的默认值（`DDIMScheduler`、
  `use_attn_mask=True` 等）手写过一版加载逻辑，能跑，但后来翻到
  `source/sugar_rl/sugar_rl/tasks/locomanip/mdp/commands.py:47,351` 才发现：
  SUGAR **自己的** `play.py`/`inference.sh` 走的推理路径，用的是
  `sugar_il.wrapper.sugar_il_wrapper.GeneratorWrapper`——而这个 wrapper 内部实际
  用的是 `DDPMScheduler`，不是训练 yaml 写的 `DDIMScheduler`；`use_target`/
  `use_last_action` 也是从 checkpoint 顶层 `cfg.use_target` 读的，不是从
  `obs_encoder` 子节点读。这些细节自己重新猜一遍很容易猜错（我自己就猜错了
  scheduler 类型），所以这里改成**直接复用 `GeneratorWrapper`**，不重新实现——
  这本来就是 SUGAR 官方验证过、`play.py` 实际在用的代码，没有理由自己再写一遍。

  唯一的技术前提：这个类在 `sugar_il` 包能被 import 的环境下跑（也就是复用
  SUGAR 训练用的那个 venv，见仓库 README）。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import torch

from sugar_deploy import contract


@dataclass
class TrackerPolicy:
    """Command Tracker：(robot_state, object_state, command) -> 29 维关节目标动作。"""

    model: torch.nn.Module
    device: torch.device

    @classmethod
    def load(cls, checkpoint_path: str | Path, device: str = "cpu") -> "TrackerPolicy":
        from rsl_rl.modules import ActorCritic

        device_t = torch.device(device)
        payload = torch.load(checkpoint_path, map_location=device_t, weights_only=False)
        if "model_state_dict" not in payload:
            raise ValueError(
                f"{checkpoint_path} 不像一个 rsl_rl OnPolicyRunner checkpoint"
                f"（没有 model_state_dict key，实际 keys={list(payload.keys())}）"
            )
        state_dict = payload["model_state_dict"]

        # critic 维度只在构造时用来对齐 state_dict shape，推理时用不到，
        # 890 是从 demo checkpoint 的 critic.0.weight shape 读出来的，不同 checkpoint
        # 理论上可能不同（取决于 privileged obs 组成），这里做了个形状自适配。
        critic_dim = state_dict["critic.0.weight"].shape[1]
        dummy_obs = {
            "policy_obs": torch.zeros(1, contract.TRACKER_OBS_DIM, device=device_t),
            "critic_obs": torch.zeros(1, critic_dim, device=device_t),
        }
        obs_groups = {"policy": ["policy_obs"], "critic": ["critic_obs"]}

        model = ActorCritic(
            obs=dummy_obs,
            obs_groups=obs_groups,
            num_actions=contract.TRACKER_ACTION_DIM,
            actor_obs_normalization=False,
            critic_obs_normalization=False,
            actor_hidden_dims=list(contract.TRACKER_ACTOR_HIDDEN_DIMS),
            critic_hidden_dims=list(contract.TRACKER_ACTOR_HIDDEN_DIMS),
            activation=contract.TRACKER_ACTIVATION,
        )
        # rsl_rl 的 ActorCritic.load_state_dict 重写了一层、永远 return True，
        # 拿不到真正的 missing/unexpected——绕过这层重写，直接调 nn.Module 原始实现来校验。
        result = torch.nn.Module.load_state_dict(model, state_dict, strict=False)
        if result.missing_keys or result.unexpected_keys:
            raise RuntimeError(
                f"Tracker state_dict 没对齐：missing={result.missing_keys} "
                f"unexpected={result.unexpected_keys}；"
                "多半是 contract.py 里的 hidden_dims/激活函数和这个 checkpoint 实际训练时不一样，"
                "检查 checkpoint 里 actor.*.weight 的 shape 确认。"
            )
        model.to(device_t)
        model.eval()
        return cls(model=model, device=device_t)

    @torch.no_grad()
    def act(self, obs_510: torch.Tensor) -> torch.Tensor:
        """obs_510: (B, 510)，按 contract.TRACKER_OBS_LAYOUT 的顺序拼好的观测。
        返回 (B, 29) 的原始网络输出（还没乘 action_scale / 加 default_joint_pos）。"""
        if obs_510.dim() == 1:
            obs_510 = obs_510.unsqueeze(0)
        return self.model.act_inference({"policy_obs": obs_510.to(self.device)})


@dataclass
class GeneratorPolicy:
    """Command Generator 的薄封装——直接复用 SUGAR 自己 `play.py` 用的
    `sugar_il.wrapper.sugar_il_wrapper.GeneratorWrapper`，不重新实现加载/推理逻辑。"""

    wrapper: object  # sugar_il.wrapper.sugar_il_wrapper.GeneratorWrapper
    use_target: bool
    use_last_action: bool
    n_action_steps: int
    n_obs_steps: int

    @classmethod
    def load(cls, checkpoint_path: str | Path, device: str = "cpu") -> "GeneratorPolicy":
        from sugar_il.wrapper.sugar_il_wrapper import GeneratorWrapper

        wrapper = GeneratorWrapper.load(str(checkpoint_path), device=device)
        return cls(
            wrapper=wrapper,
            use_target=wrapper.use_target,
            use_last_action=wrapper.use_last_action,
            n_action_steps=wrapper.n_action_steps,
            n_obs_steps=wrapper.n_obs_steps,
        )

    def predict(self, obs):
        """obs: ``sugar_il.wrapper.sugar_il_wrapper.GeneratorObs``，由
        observation.py 的 ``build_generator_obs()`` 构造。返回
        (B, n_action_steps*5-4, 36) 的插值后 command chunk（DOWNSAMPLE_RATE=5,
        插值细节见 GeneratorWrapper._parse_action，这里不重新实现）。"""
        return self.wrapper.predict(obs)
