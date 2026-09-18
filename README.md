# sugar_deploy

[SUGAR](https://github.com/tianshuwu/SUGAR)（Command Tracker + Command Generator 两级策略）的 MuJoCo
sim2sim / 真机部署代码。和 SUGAR 训练仓库分开放（原因见下），不依赖 IsaacSim/IsaacLab 跑起来。

## 为什么单独一个仓库，不塞进 SUGAR 训练仓库

- 训练依赖 IsaacSim（重、GPU 专用）；部署只需要 `torch` + `mujoco`（轻，CPU 就能跑，未来真机上更是不能依赖 IsaacSim）。
- SUGAR 训练仓库要跟上游 `tianshuwu/SUGAR` 保持能 merge 的状态，部署这边要接的东西（DDS、MoCap、真机 SDK）跟上游没关系，混在一起会让两边都不好维护。
- 参考的是你自己 `SONIC_MimicLite/gear_sonic_deploy` 已经验证过的做法——那个也是独立于训练仓库的部署项目。

## 现状（诚实汇报，不夸大）

跑 `scripts/run_sim2sim.py --task CarryBox`，用官方 `demo_ckpts/CarryBox` 的 `tracker.pt` +
`generator.ckpt`：

- ✅ 两个 checkpoint 都能正确加载（见下面"契约怎么核实的"），维度、state_dict key 完全对齐，没有静默出错。
- ✅ 整条数据流（机器人本体历史观测、anchor 系物体位姿变换、Generator 低频出 command、Tracker 高频消费 command）跑得通，形状全部核实过。
- ✅ 物理不再发散（见下面"踩过的坑"），扭矩量级合理（几到几十牛米，在电机 effort limit 以内）。
- ⚠️ **机器人还站不稳**：大概 1 秒内从站立姿态逐渐瘫软倒地，不是瞬间爆炸式的错误，更像残余的 sim2sim 保真度问题或者还有没找到的细节偏差，具体在下面"已知问题 / 下一步排查"里写了排查思路。**目前不能当成"复现成功"，只能当成"管线打通、数值在合理量级"。**

## 契约怎么核实的（为什么这些数字可信）

这不是照着论文或者猜的，是逐条从 `tianshuwu/SUGAR` 实际代码里读出来的（版本见 `contract.py` 头部注释），
核心事实和对应源码位置：

| 契约 | 来源 |
|---|---|
| 29 关节顺序 | `descriptions/robots/g1/g1_29dof_rev_1_0_with_rubber_hand.urdf` 的 `<joint>` 声明顺序 |
| PD 增益/action_scale | `source/sugar_rl/sugar_rl/assets/robots/unitree.py`（`ImplicitActuatorCfg` + `0.25*effort/stiffness` 公式） |
| Tracker 510 维观测组成 | `base_inference_env_cfg.py` 的 `TrackerCfg` observation group，逐项核对，历史长度 5 |
| Command 36 维结构 | `sugar_il/wrapper/sugar_il_wrapper.py:250-262` 的 `_parse_action` 注释 |
| anchor=torso_link（不是 pelvis） | `commands.py:1658-1663` |
| Generator 调用频率（每 20 步一次） | `base_inference_env_cfg.py:156` + `commands.py:1290-1301` |
| Generator 的真实推理路径 | `commands.py:47,351` 直接 import `sugar_il.wrapper.sugar_il_wrapper.GeneratorWrapper`——这是 `play.py`/`inference.sh` 实际在用的代码，不是训练 yaml 里写的默认配置（两者不一样，见下面踩坑记录） |

`policy.py` 里 Generator 的加载**直接复用 `GeneratorWrapper`**，不重新实现——这是刻意的选择，见 `policy.py`
文件头的说明。

## 用法

依赖 SUGAR 训练用的那个 venv（要能 `import sugar_rl`/`import sugar_il`/`import rsl_rl`）：

```bash
source /home/yingchaomu/下载/sugar-venv/bin/activate
cd /home/yingchaomu/下载/sugar_deploy
uv pip install -e .          # 装 sugar_deploy 自己（mujoco/tyro 等）

# 先只校验 checkpoint 能不能加载，不跑仿真
python scripts/run_sim2sim.py --task CarryBox \
    --tracker-checkpoint /home/yingchaomu/下载/SUGAR/demo_ckpts/CarryBox/tracker.pt \
    --generator-checkpoint /home/yingchaomu/下载/SUGAR/demo_ckpts/CarryBox/generator.ckpt \
    --validate-only

# headless smoke test（服务器/无显示器）
python scripts/run_sim2sim.py --task CarryBox \
    --tracker-checkpoint .../tracker.pt --generator-checkpoint .../generator.ckpt \
    --headless --no-real-time --control-steps 250

# 本机看 MuJoCo 窗口（有 DISPLAY）
python scripts/run_sim2sim.py --task CarryBox \
    --tracker-checkpoint .../tracker.pt --generator-checkpoint .../generator.ckpt \
    --control-steps 1000
```

拿我们自己训出来的 checkpoint 跑（等服务器上对应任务训完 Generator 之后）：

```bash
python scripts/run_sim2sim.py --task CarryBox \
    --tracker-checkpoint /data0/SUGAR_repro/SUGAR/outputs/CarryBox_server_repro/ckpts/tracker.pt \
    --generator-checkpoint /data0/SUGAR_repro/SUGAR/outputs/CarryBox_server_repro/ckpts/generator.ckpt
```

## 目录结构

```text
sugar_deploy/
  contract.py      关节顺序/PD增益/action_scale/观测布局等硬常量，全部标注了代码来源
  policy.py        TrackerPolicy（rsl_rl ActorCritic）+ GeneratorPolicy（复用 GeneratorWrapper）
  object_state.py  ObjectStateSource 接口：MujocoGroundTruthSource（sim2sim 自验证）/ MocapObjectSource（真机占位）
  observation.py   历史 buffer、anchor 坐标变换、6D 旋转表示、command buffer
  sim2sim.py       主循环：50Hz Tracker / 每 20 步一次 Generator，PD 力矩控制
scripts/
  run_sim2sim.py   tyro CLI 入口
assets/g1/
  g1_29dof.xml     G1 29dof MuJoCo 模型，复用自 SONIC_MimicLite/gear_sonic_deploy/g1/
                   （关节顺序和 SUGAR 的 URDF 逐一对应过，见 contract.py 头部）
  carrybox_scene.xml  G1 + CarryBox 箱子的组合场景，目前只有这一个任务配好
```

## 踩过的坑

1. **Generator checkpoint 的 `cfg._target_` 指向一个不存在的 `controller` 模块**——已发布的
   demo checkpoint 是用改名前的内部代码（`controller`）训的，state_dict 的 key 名字没变，import
   路径变了。不能用 `BaseWorkspace.create_from_checkpoint()`。最终方案是直接复用 SUGAR 自己
   `play.py` 在用的 `GeneratorWrapper.load()`，而不是自己重新拼装（见 `policy.py` 文件头，这也
   顺带避免了下面第 2 条我自己踩过的坑）。
2. **训练 yaml 里 `noise_scheduler` 写的是 `DDIMScheduler`，但真实推理路径用的是
   `DDPMScheduler`**——自己第一版手写加载逻辑时用错了，是从 `commands.py` 追出 `GeneratorWrapper`
   之后才发现的。这也是为什么最后决定不自己重新实现，直接复用官方 wrapper。
3. **MuJoCo 显式 Euler 积分器 + 训练时的偏硬 PD 增益（kp 最高到 99）在 200Hz 下直接数值发散**
   （一步之内 qacc 从 9.8 飙到 2.7 万+）。SUGAR 训练时用的是 PhysX 的隐式 PD 求解
   （`ImplicitActuatorCfg`），对增益大小不敏感；MuJoCo 这边换成 `mjINT_IMPLICITFAST` 积分器解决。
   同时发现 MJCF 默认 `armature=0`，和训练时非零的电机转子惯量不一致，也一并补上了
   （`sim2sim.py` 的 `__post_init__`）。

## 已知问题 / 下一步排查

机器人目前站不稳，大概 1 秒内瘫软倒地。这**不是**数值发散（qacc 稳定在几百到几千的合理量级），
更像是行为不对或者还有精度偏差。可能的方向，按怀疑程度排序：

1. **箱子尺寸/位置是占位值**（`carrybox_scene.xml` 里 0.28×0.20×0.20m，没有真实来源——见文件里的
   注释），如果和训练数据里的箱子差异较大，obj_pos_b/obj_ori_b 观测会系统性偏离训练分布。优先级
   最高：找到/量出 `descriptions/objects/small_box/obj_aligned.usd` 的真实包围盒替换掉。
2. **`use_target=True` 的目标位置也是占位值**（`--target-offset-x`，默认物体初始位置 +1m），如果
   和训练时的目标分布差太远，Generator 生成的 command 可能整体不合理。
3. MuJoCo 的 `<motor>` 扭矩电机 + 显式 PD 计算，和 PhysX `ImplicitActuatorCfg` 在**高频动态响应**
   上即使增益数值相同也不会 100% 等价（隐式 vs 显式求解的相位/阻尼特性有差异）——这是本来就知道的
   sim2sim gap，可能需要额外调阻尼或者小幅调整 kp/kd 做补偿，不代表契约本身有错。
4. 还没有独立验证过关节顺序是不是真的对（`contract.py` 里写明了这是静态代码分析推出来的，没有从
   跑起来的 IsaacLab env 里 `print(env.scene["robot"].joint_names)` 交叉验证过）——如果有条件跑一次
   官方 `inference.sh` 并在 `play.py` 里加一行 print，这是排除"关节顺序错位"这个可能性最快的办法。

排查顺序建议：先解决 1（箱子尺寸），这个最容易验证也最可能是主因；如果换了真实箱子尺寸还是站不稳，
再去交叉验证关节顺序（4）；两个都排除了再考虑增益补偿（3）。

## 还没做的

- 只有 CarryBox 的 MuJoCo 场景，其余五个任务（KickBox/PushBox/PickBottle/StandBottle/SitChair）
  需要照着 `assets/g1/carrybox_scene.xml` 配对应形状的物体（`descriptions/objects/{big_box,bottle,chair}/`）。
- `MocapObjectSource` 只有接口，没接任何真实 MoCap 协议——真机部署前必须先接好。
- 真机侧的 DDS/Unitree SDK 通信完全没写，目前只有 MuJoCo sim2sim。真机开始接的时候建议参考
  `SONIC_MimicLite/gear_sonic_deploy` 里 `deploy.sh sim|real|<interface>` 的模式。
