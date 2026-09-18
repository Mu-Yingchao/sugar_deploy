# sugar_deploy

[SUGAR](https://github.com/tianshuwu/SUGAR)（Command Tracker + Command Generator 两级策略）的 MuJoCo
sim2sim / 真机部署代码。和 SUGAR 训练仓库分开放（原因见下），不依赖 IsaacSim/IsaacLab 跑起来。

## 为什么单独一个仓库，不塞进 SUGAR 训练仓库

- 训练依赖 IsaacSim（重、GPU 专用）；部署只需要 `torch` + `mujoco`（轻，CPU 就能跑，未来真机上更是不能依赖 IsaacSim）。
- SUGAR 训练仓库要跟上游 `tianshuwu/SUGAR` 保持能 merge 的状态，部署这边要接的东西（DDS、MoCap、真机 SDK）跟上游没关系，混在一起会让两边都不好维护。
- 参考的是你自己 `SONIC_MimicLite/gear_sonic_deploy` 已经验证过的做法——那个也是独立于训练仓库的部署项目。

## 现状

跑 `scripts/run_sim2sim.py --task CarryBox`，用官方 `demo_ckpts/CarryBox` 的 `tracker.pt` +
`generator.ckpt`：

- ✅ 两个 checkpoint 都能正确加载（见下面"契约怎么核实的"），维度、state_dict key 完全对齐，没有静默出错。
- ✅ 整条数据流（机器人本体历史观测、anchor 系物体位姿变换、Generator 低频出 command、Tracker 高频消费 command）跑得通，形状全部核实过。
- ✅ 物理不再发散（见下面"踩过的坑"），扭矩量级合理（几到几十牛米，在电机 effort limit 以内）。
- ✅ **机器人站稳了**：`pelvis_z` 全程稳定维持在 0.76~0.80，不再瘫软倒地。之前一直摔倒的根因是**关节
  顺序错了**——`contract.py` 里原来按 URDF 文件声明顺序推断的关节顺序，和 IsaacLab 运行时真实用的顺序
  完全不一样（2026-09-18 用官方 `inference.sh` 的日志实测确认并修正，见"踩过的坑"第 4 条）。
- ✅ **Generator 调用不再卡顿**：改成后台线程异步调用后，50Hz 真实节拍下每步耗时稳定在预算内（见"踩过
  的坑"第 5 条），CPU/GPU 都不需要额外配置。
- ✅ **机器人会真的走过去伸手**：`carrybox_scene.xml` 里箱子的尺寸和初始距离原来都是没有依据的占位值，
  跟真实箱子模型/真实训练数据的分布差得远，机器人因此要么伸空手摔倒、要么直接原地不动（见"踩过的坑"第
  6 条）。用真实 USD 网格和真实参考轨迹重新校准后，600 步（12 秒）实测机器人会从起点走到箱子旁边、
  站稳不摔、手伸到箱子可触及范围内并发生接触。
- ⚠️ **还没稳定抓住搬到目标点**：箱子会被碰到、轻微顶起，但没有观察到稳定抓住并搬运到终点的连贯动作，
  当前最主要的待排查项，怀疑方向见"已知问题 / 下一步排查"。
- ⚠️ **GUI viewer 模式退出不干净**（见"踩过的坑"第 7 条）：仿真本身跑完、结果正确，但关闭 MuJoCo
  viewer 窗口时这台机器上偶发卡死或段错误，是这台机器的显示环境问题，不影响仿真结果本身；需要干净
  退出码的场景（比如脚本化调用）用 `--headless` 模式。

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

# headless smoke test（服务器/无显示器）——注意这个命令只用来验证"跑不跑得通"，不代表真实性能：
# --no-real-time 下主循环尽可能快地跑，CPU 上会出现 GIL 争抢，日志里 generator_calls 会明显低于
# control_steps/20 的理论值（比如 250 步只有 2 次，不是 12 次左右），这是测试方式本身的问题，不是
# bug——真实使用（下面这条 GUI 命令，或者 --headless --real-time）里主循环会有 sleep 让出 GIL，
# 后台线程能正常跑满，见"踩过的坑"第 5 条
python scripts/run_sim2sim.py --task CarryBox \
    --tracker-checkpoint .../tracker.pt --generator-checkpoint .../generator.ckpt \
    --headless --no-real-time --control-steps 250

# 本机看 MuJoCo 窗口（有 DISPLAY）——正常退出前会先打印 DONE 和正确的统计结果，但关窗口这一步在
# 这台机器上可能卡死或段错误退出（exit code 139），是显示环境的问题不是仿真结果的问题，
# 见"踩过的坑"第 7 条；需要干净退出码就用上面的 --headless
python scripts/run_sim2sim.py --task CarryBox \
    --tracker-checkpoint .../tracker.pt --generator-checkpoint .../generator.ckpt \
    --control-steps 1000
```

拿我们自己训出来的 checkpoint 跑（等服务器上对应任务训完 Tracker + Generator 之后，`ckpts/` 目录
会出现和 `demo_ckpts/<Task>/` 一样命名的 `tracker.pt` + `generator.ckpt`）：

```bash
python scripts/run_sim2sim.py --task CarryBox \
    --tracker-checkpoint /data0/SUGAR_repro/SUGAR/outputs/CarryBox_server_repro/ckpts/tracker.pt \
    --generator-checkpoint /data0/SUGAR_repro/SUGAR/outputs/CarryBox_server_repro/ckpts/generator.ckpt
```

**核实过路径格式是对的，但截至 2026-09-18 六个任务没有一个训完，这条命令目前对任何任务都还跑不了**
（SSH 到服务器 `ls /data0/SUGAR_repro/SUGAR/outputs/<Task>_server_repro/ckpts/` 确认过，全部是空
目录）——CarryBox/PushBox/PickBottle/SitChair 还在 Refiner 阶段；KickBox/StandBottle 已经训完
Refiner、进入 Tracker 阶段（`logs/` 下能看到 `tracker/` 目录和 `refiner.pt`），但都还没到 Generator
阶段。跑之前先用上面这条 `ls` 命令确认 `ckpts/` 里已经有文件，再替换 `--task` 和路径里的任务名。

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
                   （29 个关节名字和 SUGAR 的 URDF 逐一对应，但顺序不代表 contract.JOINT_NAMES
                   的顺序——MJCF/URDF 是文件声明顺序，JOINT_NAMES 是 IsaacLab 运行时实测的真实
                   顺序，两者不一样，sim2sim.py 里都是按名字查 id，不依赖顺序对齐，见"踩过的坑"）
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
4. **（2026-09-18，这是之前机器人一直站不稳的真正原因）关节顺序错了**——最早 `contract.py` 里的
   `JOINT_NAMES` 是按 G1 URDF `<joint type="revolute">` 的**文件声明顺序**推断的（"先左腿全部关节，
   再右腿全部，再腰，再左臂，再右臂"），文件头当时就标注过"这是静态分析推出来的，没有运行时验证"。
   后来跑官方 `inference.sh CarryBox`（非 headless）时，日志里 IsaacLab 自己打出了真实的动作关节
   顺序（`Resolved joint names for the action term JointPositionAction`），是按关节类型分层、左右
   交替、腰部穿插的完全不同顺序，跟 URDF 声明顺序对不上。这意味着 Tracker 输出的 29 维动作，之前
   一直被系统性地发到了错误的关节上——网络算出来的"抬左腿"，实际被当成了"转腰"之类完全不相关的动作，
   机器人当然站不住。改正 `JOINT_NAMES` 之后还连带发现一个隐藏的第二个 bug：`sim2sim.py` 里
   `d.ctrl[:] = torque` 是按位置直接写 MuJoCo 的执行器数组的，隐含假设了 `contract.JOINT_NAMES` 的
   顺序和 `g1_29dof.xml` 里 `<actuator>` 块的声明顺序一致——这个假设在旧（错误）顺序下"恰好"成立
   （因为旧顺序本来就是照抄 URDF/MJCF 文件顺序的），但换成实测的真实顺序后就不成立了，得改成按名字
   查 `actuator_ids` 再写（`sim2sim.py` 的 `_apply_pd`）。两处一起修完，机器人从"1 秒内瘫软倒地"
   变成"3 秒仿真里 `pelvis_z` 稳定在 0.78~0.79"，直接验证了这就是根因。

   **教训**：任何"关节顺序"之类的契约，只要有条件，一定要从跑起来的系统里实测拿到，不要相信任何
   基于 URDF/MJCF/代码静态分析的推断——两者不一致的情况比想象中常见，而且不一致时不会报错，只会让
   策略表现得莫名其妙地烂，很难第一时间定位到是这个原因。

5. **（2026-09-18）MuJoCo 窗口很卡，根因不是"没用 GPU"，是同步调用架构**——用户反馈本地跑
   viewer 特别卡，直觉怀疑是没挂 GPU。实测下来：MuJoCo 物理步本身 <1ms，不是瓶颈；真正的瓶颈是
   `Generator`（DiT + 16 步 diffusion 采样）每 `GENERATOR_CALL_INTERVAL=20` 步（约 0.4s）同步调用
   一次，CPU 上单次要 120~160ms，GPU 上也要 ~57~60ms——两者都远超 50Hz 单步 20ms 的预算，导致主
   循环每 0.4 秒卡顿一次（GPU 只是把卡顿幅度从 ~150ms 降到 ~60ms，卡顿本身没消除）。MuJoCo 物理
   仿真本身是纯 CPU 的，"给它挂 GPU"这个方向从一开始就是错的。真正的修复是把 `_maybe_call_generator`
   改成后台线程异步调用（`sim2sim.py`），不阻塞主循环；command chunk 本身有 36 步余量（只消费前
   20 步，见 `CommandBuffer`），刚好够盖住这段异步计算延迟。**实测验证**（50Hz 真实节拍、
   200 步 CarryBox）：CPU 上 200 步内最慢一步 19.27ms（在 20ms 预算内），GPU 上最慢一步
   74.60ms（但只在 step 0，是模型/CUDA 冷启动，之后每步都在预算内）；两种 device 下
   `generator_calls` 都精确等于期望值 10（200/20）。**结论：CPU 默认配置足够，不需要 `--device
   cuda`**——之前的卡顿是架构问题不是硬件问题，async 化之后 CPU/GPU 都不卡。

   **教训**：性能问题先测量再下结论，不要直觉先入为主。"是不是没用 GPU" 这类猜测很容易先入为主地
   把排查方向带偏——这里真机部署也会踩同一个坑（真机上没有"viewer 卡不卡"这种直观信号，只会表现成
   机器人动作一顿一顿，更难发现是这个原因），async 化对真机部署同样是必要的，不只是仿真可视化的
   优化。

6. **（2026-09-18）场景里箱子的尺寸和初始距离都是没有依据的占位值，导致"机器人不动"或"伸空手摔倒"**——
   详细排查过程、真实数值怎么量出来的、实测前后对比，见下面"已知问题 / 下一步排查"第 1、2 条，这里
   不重复。教训是一样的：**任何"占位场景参数"只要仓库里有真实数据/真实资产可以核对，就不要长期停留在
   占位状态**——`descriptions/objects/*/obj_aligned.usd` 和 `data/<Task>/` 这两类真实资产/真实轨迹
   数据其实从一开始就在仓库里，没有及时去读是这个坑拖了一段时间的原因。

7. **GUI viewer 模式下，关闭 MuJoCo 窗口时进程偶发卡死或段错误（SIGSEGV, exit code 139）**——
   用 `scripts/run_sim2sim.py`（不带 `--headless`）跑完之后，终端会先正确打印
   `[sugar_deploy] DONE ...`，说明仿真本身和统计结果都是对的，但进程在退出 `mujoco.viewer.launch_
   passive` 的 `with` 块之后没有干净退出。排查过是不是我们自己加的后台线程（`_maybe_call_generator`
   异步调用）导致的：写了一个完全不含 sugar_deploy 代码、纯 MuJoCo 的最小复现（建模型、开 passive
   viewer、跑 50 步、退出 `with` 块），同样卡住不退出——说明**这是这台机器的显示环境（`DISPLAY=:1`，
   非原生 X，具体是哪种远程/虚拟显示服务还没查）和 MuJoCo GLFW 窗口销毁交互的问题，不是我们的异步线程
   改动引入的**，和之前"IsaacSim 窗口关不掉需要强制退出"大概率是同一类问题的不同表现。**不影响仿真
   结果的正确性**，只影响进程能不能干净退出：需要干净退出码的场景（脚本化调用、CI）用 `--headless`；
   交互查看仍然用 GUI viewer，但要接受它可能需要手动 `kill` 收尾，这个问题本身还没有根因定位到具体是
   GLFW/驱动的哪一层，留作后续。

## 已知问题 / 下一步排查

**（2026-09-18 更新）**之前遇到过"机器人要么一动不动，要么弯腰去够箱子但没够到、还直接摔倒抽搐"，
排查出两个原因都是场景占位参数没校准，不是策略或契约本身的问题：

1. **箱子尺寸是占位值，比真实箱子小了 1.4~2.7 倍**（原来 0.28×0.20×0.20m，用 pip 装的轻量
   `usd-core`——不需要完整 IsaacSim/Omniverse——读 `obj_aligned.usd` 网格顶点包围盒实测出真实尺寸
   是 0.40×0.55×0.54m）。机器人是照着训练时见过的、接近齐腰高的箱子学的弯腰幅度，箱子摆太矮太小，
   手会伸空，弯腰角度也和真实箱子对不上，这是"够不到箱子、还摔倒"的主因。已修（`carrybox_scene.xml`
   第 16 行的 `size`）。
2. **机器人-箱子的初始距离是占位值，比训练分布近了 2~5 倍**（原来 0.45m，直接读了
   `data/CarryBox/` 下 15 条真实参考轨迹的 `obj_trans[0]` 相对 pelvis 位置，实测真实初始距离全部
   落在 1.1~2.6m，从没有 <1m 的样本）。策略在"箱子已经贴脸"这种训练时没见过的分布外输入下，退化成
   了保守的原地不动——这是"机器人完全不动"的主因，不是策略坏了，是喂的观测本身就不合理。已修
   （`carrybox_scene.xml` 第 13 行，箱子挪到 1.5m 外）。

**两处都修完后重新实测**（CPU、50Hz 真实节拍、600 步/12s，`data/CarryBox` 典型片段时长量级）：
机器人从原地走出 1.5m 到箱子附近（pelvis_xy 从 (0,0) 走到 (1.5, 0.02)），全程 pelvis_z 稳定在
0.76~0.80 没有摔倒，手腕-箱子距离在 step 150~300 期间降到 ~0.48m（进入手臂可触及范围），箱子被
碰到并轻微顶起（高度 0.268→0.29m，位置从 x=1.5 挪到 x≈1.83m）。**行为已经从"完全不响应"变成了
"走过去、伸手、发生接触"**，但还没有观察到稳定抓住并搬到目标点（目标 x=2.5m，箱子最终停在
x≈1.83m 不再前进）——这部分剩余 gap 更可能是接触/抓取保持力这类精细问题，候选方向：

3. MuJoCo 的 `<motor>` 扭矩电机 + 显式 PD 计算，和 PhysX `ImplicitActuatorCfg` 在**高频动态响应**
   上即使增益数值相同也不会 100% 等价（隐式 vs 显式求解的相位/阻尼特性有差异）——这是本来就知道的
   sim2sim gap，可能需要额外调阻尼或者小幅调整 kp/kd 做补偿，不代表契约本身有错。
4. 箱子摩擦系数/接触参数（`friction="1.0 0.01 0.0001"`）也是没有核实过真实值的占位设置，和 PhysX
   默认摩擦模型不一定等价，抓取全靠摩擦力夹持时摩擦系数偏差会直接影响抓不抓得住。
5. `--target-offset-x` 默认值（+1m，纯 x 方向）虽然量级接近真实数据（15 条样本位移 0.8~2.2m），
   但方向是瞎猜的（真实数据里位移方向并不固定沿 x），可能不在 Generator 熟悉的目标模式里。

## 完全没有物体感知，机器人会不会做搬箱子动作（实测，不是猜的）

**（2026-09-18 更新）下面这组实测是在"机器人站不稳"那个 bug 修复之前做的**（当时关节顺序还是错的，
机器人本身就没法正常站立/行走），现在关节顺序、箱子尺寸、初始距离三处都修完之后，机器人已经能正常
走到箱子旁边伸手了（见上面"已知问题"），下面这组实验的具体表现（"手臂大幅摆动不收敛"）大概率已经
不能代表现状，结论部分（架构层面"没有物体感知就不会有连贯动作"的判断）仍然成立，但具体实验数据建议
重新跑一遍再引用。先保留原始记录：

直接做了个对照实验回答这个问题：同一个 checkpoint、同一个初始状态，一组用 MuJoCo 场景里箱子的
真实位置（`MujocoGroundTruthSource`），另一组把物体状态源换成一个不管真实情况、永远返回
"物体在机器人胸口原点"这个物理上不可能的固定值（模拟"没有任何输入物体信息的方法"）。

**结果**：两组的手臂关节轨迹都是类似幅度的大摆动、不收敛、看不出"伸手→抓→抬起"这种连贯动作
（比如左肩 pitch 在几步内从 -0.44 跳到 2.87 再跳回 0.2，两组都这样）；有真实箱子的那组，箱子本身
在前 15 步左右直接从 0.5m 掉到地上（没被稳定抓住），后面箱子位置的变化看起来更像是被倒地的机器人
身体带着蹭动，不是被稳定托举搬运。

**诚实的结论**：**现在这版 sim2sim 因为还没解决"机器人站不稳"这个更基础的问题（见上面"已知问题"），
"有真实物体信息" vs "完全没有"这两组看起来都不连贯，没能干净地分离出"少了物体信息"单独造成的影响**
——不是我不想给你一个干脆的实验结论，是当前这版实现本身的保真度还没到能看出这个差异的程度。

**但可以给一个有把握的架构层面的判断**：SUGAR 的 Command Generator 是**闭环、依据当前物体状态决策
下一步做什么**的设计（这是它区别于"照着录好的参考轨迹硬播"的整个论文卖点），不是那种"不管输入是什么，
都会播放一段预先学好的'搬箱子'固定动作序列"的系统。喂一个物理上不存在/不合理的物体位置，等于让
Generator 长期停留在训练时从没见过的输入区域，它没有"识别出没有物体、于是退化成某种安全的默认搬箱子
哑动作"这种机制——更可能的情况是持续产生和真实任务不对应的、不收敛的动作（就像上面两组实测都表现出来的
那种大幅摆动），而不是清晰地完成或者清晰地"什么都不做"。**结论仍然是：没有物体感知，机器人不会正确/
连贯地做出搬箱子这个动作，但它也不会保持静止——会有动作，只是动作和真实任务对不上。**

想要一个更干净的实验（能明确分离"有没有物体信息"这一个变量），前提是先把上面"已知问题"里机器人站不稳
那个问题解决掉——到时候可以用同样的方法（换 `object_source` 实现）再测一次，结论会更可信。

## 真机物体感知方案：难度分级

`ObjectStateSource`（`object_state.py`）是特意做成可插拔接口的，换下面哪个方案，`observation.py`/
`sim2sim.py` 都不用改，只要新写一个 `ObjectStateSource` 子类。按投入产出比从低到高排：

| 档位 | 方案 | 需要什么 | 精度/鲁棒性 | 预计工作量（做出"能跑的第一版"，不是打磨完善） |
|---|---|---|---|---|
| 0 | **固定预设坐标**（人工每次把箱子摆在量好的同一个位置，代码里写死这个坐标，训练/测试全程不再更新） | 一把尺子 | 极差：不是真感知，物体挪一下位置就全错，交互过程中"抓没抓住"这类闭环判断完全失效 | 几分钟。**只建议当成排查"是不是物体感知的问题"的对照实验用，不建议当成真实部署方案** |
| 1 | **AprilTag/ArUco 标签 + G1 自带的 D435 深度相机** | 打印标签贴物体上，`opencv-python` 的 `cv2.aruco` 模块（或 `apriltag` 库），相机内参（D435 出厂自带或 `realsense-viewer` 读）+ 外参（URDF `d435_joint` 的挂载位置可以当起点，更准可以做一次简单的手眼标定） | 刚体、已知标签尺寸时可以做到毫米级位置精度；被完全遮挡时会丢失，标签本身如果被手挡住会失效 | **1~3 天**：半天到 1 天先用一个独立脚本验证"D435 拿到 RGB 图 → 检测到标签 → 解出 6D pose"能跑通；半天做相机系到机器人系的坐标变换；半天到 1 天接进 `ObjectStateSource` 子类并在 sim2sim/真机上联调。这是**最快能拿到一个可用方案**的路径 |
| 1.5 | **无标签但用深度做简单形状拟合**（比如用色彩/深度分割把箱子从背景里抠出来，拿点云去拟合一个长方体/圆柱体求位姿） | D435 深度流 + 简单点云处理（`open3d` 之类） | 比标签方案更脆弱（依赖背景干净、物体和背景有区分度），但不用在物体上贴东西 | **3~7 天**，比标签方案多一层"稳定分割"要调 |
| 2 | **在线跑 FoundationPose**（SUGAR 论文 Stage 1 离线处理视频用的同一个工具，改成实时吃 D435 的 RGB-D 流） | FoundationPose 本身的运行环境（需要 GPU）、`descriptions/objects/*/obj_aligned.usd` 这几个物体已经有的 mesh 可以直接喂给它 | 无需贴标签、理论精度可以很高，但**遮挡下的跟踪丢失/漂移是真实风险**——搬箱子这个任务手本身就经常挡住箱子，这大概率正是论文选择用 MoCap 而不是在线视觉的原因 | **1~2 周**：不只是"跑起来"，还要处理实时性（能不能追上控制频率）、跟踪丢失后怎么重新初始化，工作量明显比标签方案大 |
| 3 | **真动作捕捉系统**（Vicon/OptiTrack 等） | 多摄像头动捕硬件、场地标定 | 论文原始方案，精度和鲁棒性最好 | 如果买现成商用系统：硬件到位后标定安装大概几天到一两周，但**采购成本高**（通常几万到几十万人民币起）；如果想着自己拼一套低成本动捕（比如多个普通摄像头三角化），那是一个独立的、相当有难度的工程项目，不建议为了"物体感知"这一个子问题单独去做 |

**建议**：先做档位 0（固定坐标）配合先解决"机器人站不稳"那个更基础的问题，验证清楚 sim2sim 这条链路
本身没问题之后，直接上**档位 1（AprilTag）**——这是性价比最高的选择，大概率就是几天的工作量，能让你
真正拿到一版"喂真实变化的物体位置"的可用系统，档位 2（FoundationPose）可以作为后续摆脱标签的技术
升级方向，不建议一开始就啃。

## 还没做的

- 只有 CarryBox 的 MuJoCo 场景，其余五个任务（KickBox/PushBox/PickBottle/StandBottle/SitChair）
  需要照着 `assets/g1/carrybox_scene.xml` 配对应形状的物体（`descriptions/objects/{big_box,bottle,chair}/`）。
- `MocapObjectSource` 只有接口，没接任何真实 MoCap 协议——真机部署前必须先接好。
- 真机侧的 DDS/Unitree SDK 通信完全没写，目前只有 MuJoCo sim2sim。真机开始接的时候建议参考
  `SONIC_MimicLite/gear_sonic_deploy` 里 `deploy.sh sim|real|<interface>` 的模式。
