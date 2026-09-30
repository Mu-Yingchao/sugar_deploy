# 真机部署：DDS 通信层 + 分阶段上机测试流程

这份文档是 `sugar_deploy` 从"只有 sim2sim"走向"能在真实 G1 上跑"的起点。参考的是 HDMI
官方部署仓库 [`EGalahad/sim2real`](https://github.com/EGalahad/sim2real) 里验证过的
DDS 通信写法（训练框架仍然是 SUGAR，这里只是借鉴部署这一层的代码模式，见
`APRILTAG_DEPLOYMENT.md` 里 SUGAR vs HDMI 的评估）。

**在打开这份文档之前你应该已经读完 `README.md` 和 `APRILTAG_DEPLOYMENT.md`**——那两份
文档里的 sim2sim 验证、关节顺序踩过的坑、AprilTag 感知方案是这份文档的前置知识，这里不
重复背景，只讲真机这一层新增的东西。

## ⚠️ 先读这个：真机比 sim 多一类风险

sim2sim 里关节顺序错了，后果是机器人在虚拟世界里瘫倒，重启一下仿真就行。**真机上同样的错误
是真实电机收到发给别的关节的指令**，轻则姿态失控摔倒，重则损坏机器人或者伤到旁边的人。这份
文档里的每一步都是按"先假设自己哪里写错了，用最小代价先验证"的顺序排的，**不要跳步骤**，
尤其不要跳过第 0、1 阶段直接去发位置控制指令。

## 🛑 紧急停止（每次通电测试前都要重新确认，不是读一遍就够）

**之前这份文档在这里写得不够——只说了"要有物理支撑"，没说清楚出问题了具体怎么停。补上，
下面这些是查官方文档确认过的，不是我猜的。**

### 官方高层模式下的阻尼/零力矩键

- **当前这台真机实测**：`L2+B` 进入阻尼模式，`L2+Y` 进入零力矩模式；两者不是一回事。
- **固件 V1.0.4 及以上**：官方资料把 `L2+B` 记为阻尼模式（damping mode）。
- **固件 V1.0.2**（老一些的固件）：按 `L1 + A`。
- 先确认你这台机器人固件版本对应哪个组合——两个记混了，出事的时候按错键等于没按。
- 进入阻尼模式后，电机**不是瞬间断电摔倒，是主动卸力、带阻尼地缓慢倒下**（官方原文："The
  robot will enter damping mode and slowly fall to the ground"）——但依然是会倒的，不是
  "安全悬停"，触发前必须已经有物理支撑（见下面三种方法），不能假设机器人会自己稳住。

**（2026-09-22 真机实测纠正）这不是能覆盖低层 DDS 指令的硬件急停。**在
`release_high_level_control()` 之后持续发送 `rt/lowcmd` 的 15% 增益保持测试中，按官方组合键
完全没有卸力、也没有语音反馈；测试结束后只能通过重启恢复官方手柄控制。因此不能再声称这个通道
"独立于我们自己的代码"。它只在官方高层控制仍掌权时作为阻尼模式键使用。

低层控制期间仍要求第二个人专职拿着遥控器，因为下面新增的安全循环会直接从独立的
`rt/wirelesscontroller` DDS 话题读取组合键并自行锁存零力矩；首选 `L2+Y`，同时把 `L2+B` 和
旧固件 `L1+A` 也当作急停。此时真正执行急停的是我们的
`SafeRealRobotController`，不是官方高层控制器。真机已经用只读监视器确认原始按键数据，软件也
能识别并锁存组合键。

**2026-09-22 的首次闭环急停验收没有通过：终端虽然报告锁存 `L2+Y`，机器人在 5 秒收尾窗口
内仍有保持力。原因定位为旧实现把 `unitree_go` 的 `PosStopF/VelStopF` 哨兵误用于 G1 的
`unitree_hg` 命令。改成 Unitree 官方 HG 零力矩格式
`mode=1, q=dq=kp=kd=tau=0` 后，已重新在悬挂、15% 当前姿态保持状态下实测：安全线程识别
`L2+Y`，在持续发送零力矩命令的 60 秒窗口内，现场确认关节保持力消失。该修正版的“按键检测 →
锁存 → HG 零力矩输出 → 物理卸力”链路已通过，但它仍是软件安全层，不是独立硬件断能。**

**机身上没有查到任何物理急停按钮**——官方文档里唯一的急停手段就是遥控器组合键，电源键是
"长按 2 秒以上"关机，不是瞬时急停，不要在紧急情况下指望电源键。

### 触发阻尼模式前，机器人必须处于官方文档说明的三种支撑状态之一

1. **坐姿**：机器人坐在椅子上，手臂/腿自然摆放。
2. **悬挂**：用肩带把机器人吊起来，**脚不沾地**。
3. **站立但有人工扶持**：操作员一手放在机器人两腿之间，另一手扶住背部支架——这个姿势下
   人是主要的支撑力，阻尼模式触发后人要能承住机器人的重量，不是象征性扶一下。

阶段 1（下面会讲到）第一次 release 高层控制、发零力矩指令，**必须是这三种状态之一**，不能是
机器人自己正常站立的状态——阻尼模式生效那一刻机器人会开始倒，没有支撑就是直接摔。

### 低层控制的软件急停和看门狗

查过 `unitree_sdk2py` 相关文档和 Unitree 官方资料，**没有找到关于"控制脚本崩溃/网络断开后，
电机会不会自动进入安全状态"的官方说明**——不确定这件事的话就应该按"不会"来设计操作流程，
不要心存侥幸。这意味着：

- `real_robot_safety.py` 的 `SafeRealRobotController` 用独立发送线程直接监听原始手柄数据，并在
  `L2+Y`（首选）、`L2+B` 或 `L1+A`、手柄数据流超时、主控制循环心跳超时、`lowstate` 超时、发送异常、
  `Ctrl+C`/正常退出时锁存零力矩。锁存后同一个实例不能恢复位置控制，必须退出并重新检查。
- 后台线程能防主策略循环阻塞/抛异常，但**不能**防整个 Python 进程被 `SIGKILL`、操作系统或电脑
  崩溃、网卡/DDS 完全断开；这些故障仍需要真正独立的硬件断能链路。不要把软件看门狗写成
  "硬件级急停"。
- 每次测试前明确分工：**至少两人在场**，一人操作电脑/跑脚本，另一人专职拿遥控器 + 在机器人
  旁边做物理保护，不能一个人同时干两件事。

先做只读按键检查（不会 release、不会发送指令）：

```bash
python scripts/real_robot_remote_monitor.py --interface eth0 --duration 10
```

低层增益测试统一使用 `scripts/real_robot_gain_test.py`，不要再直接写裸 `send_command()` 循环。
脚本默认在正常结束或急停后继续发送 5 秒零力矩，方便现场人员明确触摸确认；随后进程退出，固件
可能重新表现为阻尼状态，所以要在这 5 秒窗口内判断，不要只在进程退出后判断。当前修正必须先在
悬挂、15% 当前姿态保持下重新验收，明确确认这 5 秒内关节变为零力矩，才能恢复后续测试。
手柄急停物理卸力验证通过后，还应在悬挂、15% 以下做一次主循环卡死故障注入：加
`--watchdog-test-after 1`，脚本会在 1 秒时故意停止心跳 0.3 秒；预期终端报告“控制心跳超时”并
锁存零力矩。该参数硬编码拒绝在 15% 以上使用。

### 正常（非紧急）关机顺序

先用遥控器组合键进入阻尼模式（这时候人要扶住/机器人要在支撑状态），把机器人稳定放倒/放稳，
再长按电源键 2 秒以上关机——不要在机器人还站着/没有支撑的时候直接长按电源键关机。

## 0. 现在有什么、还没有什么

| | 状态 |
|---|---|
| `unitree_joint_map.py`：contract.JOINT_NAMES ↔ 真机 DDS 电机数组顺序的映射 | ✅ **已在真实 G1 上核对通过**（阶段 0 遥测确认过关节映射对得上实际姿态） |
| `real_robot_io.py`：`rt/lowstate` 订阅 / `rt/lowcmd` 发布，CRC、release 高层控制 | ✅ 遥测、位置保持和修正后的 HG 零力矩均已在真机跑通 |
| `scripts/real_robot_telemetry.py`：只读遥测，不发任何指令 | ✅ 已跑通 |
| 位置控制指令发送、release 高层控制、小增益位置保持 | ✅ 阶段 1（零力矩）、阶段 2（小增益位置保持）都已实测通过 |
| 阶段 3：满增益默认站姿 | 🟡 3.1 满增益悬挂、3.2 默认姿态过渡、3.3 轻触/保守部分承重已通过；3.4 独立站立等待 Tracker 动态闭环，禁止用固定 PD 强行验收 |
| Tracker/Generator 接入真机闭环 | 🟡 checkpoint、真机观测、影子推理和 1% 力矩混合已实现；真实视觉输入和完整主循环未实现，见阶段 4 |
| AprilTag 相机外参/tag 偏移真机标定 | 🟡 仿真链路已实现；真机相机尚未枚举，外参和 tag→箱体偏移未标定，见阶段 4.1～4.3 |
| 低层控制软件急停/心跳看门狗 | ✅ `L2+Y` 按键检测、锁存和物理卸力真机通过，其他超时路径离线故障注入通过；仍不能替代独立硬件断能 |

## 1. 阶段 0 完整逐步指导（唯一不需要任何物理防护的一步，从这里开始）

### 1.1 在哪台机器上跑

**（2026-09-21 更新）部署场地 G1 旁边有一台专用的 4090 部署电脑**——这台大概率就是应该用的
那台机器：一是要在能收到机器人 DDS 广播的网络里（下面确认一下），二是后面接 Tracker/
Generator 之后需要 GPU 跑推理（Generator 是扩散模型，sim2sim 里我们实测过 CPU 单次采样
120~160ms，远超 50Hz 的 20ms 预算，真机这一层大概率也需要 GPU，见 sugar_deploy 主 README
"踩过的坑"第 5 条），4090 正好用得上。**先确认这台机器和 G1 之间的网络已经连好**（网线/
网络配置有没有现成的，还是需要你自己接）——这个我不知道你现场具体怎么接的，需要你确认。

确认网络通不通：机器人开机之后，在这台 4090 机器上能不能 `ping` 通机器人本体地址——Unitree
系列机器人**通常**用 `192.168.123.0/24` 这个网段（比如机器人本体是 `.161` 这类，具体数字
按你机器人本体上贴的标签或者购机文档给的为准，这里说的是行业里常见的默认约定，不保证你这台
一定是这个），`ip a` 看这台 4090 机器自己在这个网段里有没有分到地址，有就说明网络通了。

#### 为什么先选 4090 网线连接，不是板载部署

两个都是选项（G1 板载电脑 vs 4090 台式机网线连接），阶段 0~3 先用 4090，理由：

1. **算力**：Generator 是扩散模型，sim2sim 里实测 CPU 单次采样 120~160ms，远超 50Hz(20ms)
   预算——G1 板载电脑（通常是 Jetson 级别）算力明显弱于桌面级 CPU，直接跑现在这套没优化过的
   PyTorch 推理大概率撑不住。4090 能直接复用已经在 sim2sim 里验证过的代码，不用先做
   ONNX/TensorRT 导出这类额外工程。HDMI 的部署仓库有专门的 `onboard_jetpack5_inference_
   backends.md` 文档说明板载部署要做这层优化才可行——说明这条路可以走，但工作量比现在直接
   上 4090 大一截，不必现在就投入。
2. **HDMI 的真实前车之鉴**：同一份仓库的 `docs/robot_io.md` 明确写过"如果关节剧烈抖动、或者
   高动态动作明显比预期差，先怀疑 ZMQ 网络 I/O 延迟不稳定，不是先怀疑策略或增益"——说明外部
   PC + 网络中继这条路径确实有延迟抖动的真实风险，不是猜的。
3. **CarryBox 任务本身的物理约束**：训练数据里机器人-箱子初始距离是 1.1~2.6m，机器人要真的
   走这么远——如果 4090 是台式机、机器人拖一根网线走这么远，线缆管理会是现场的真实问题（够不够
   长、会不会被绊到、转身缠线），板载方案没有这个问题。

**阶段 0~3**（遥测、release 控制、增益测试、站姿保持）动作范围很小，网线不是问题，先用 4090
把 DDS 通信和关节映射验证对。等要接 Generator 跑真正会走动的 CarryBox 时，再看现场网线长度/
空间能不能覆盖整个任务范围——不行的话再考虑板载 + TensorRT 优化，不需要现在就决定。

### 1.2 确认网络接口名字

```bash
ip a
```

找那个显示已连接、IP 地址在机器人网段里的接口（常见名字 `eth0`，也可能是 `enp0s3` 这种，
不同发行版命名不一样），记下这个名字，下面命令里的 `--interface` 都要传这个。如果找不到任何
接口有 IP（网线没插好、或者机载电脑网络服务没起来），先解决这个，不要往下走。

### 1.3 确认机型

`unitree_joint_map.py`/`real_robot_io.py` 现在只覆盖 **G1 29dof**（`unitree_hg` 消息族，
`LowState_`/`LowCmd_` 里 29 个 `motor_state`/`motor_cmd`）——先确认你这台就是 29dof 型号
（不是 H1/H1-2/Go2，也不是 G1 的其他关节数配置），不是的话 `UNITREE_JOINT_NAMES` 这份列表
不能直接用，需要重新核对。

### 1.4 装依赖

在 4090 部署电脑上：

```bash
git clone https://github.com/Mu-Yingchao/sugar_deploy.git   # 如果这台机器上还没有这个仓库
cd sugar_deploy
python3 -m venv .venv && source .venv/bin/activate   # 或者用你已有的 venv/conda 环境都行
pip install -e ".[unitree]"
```

**这里先建一个新 venv 只是为了阶段 0 能尽快跑起来，不代表最终就用这个**——阶段 4 之后要接
Tracker/Generator，需要 `sugar_rl`/`sugar_il`/`rsl_rl`（和 sim2sim 用的是同一个环境，见
主 README 的"用法"一节），如果这台 4090 机器后续会装完整的 SUGAR 训练/推理环境，建议提前
规划成同一个 venv（比如就叫 `sugar-venv`，跟你本机那个一致），不用到时候再合并两套环境。

大概率会在装 `unitree_sdk2py` 这一步卡住，报类似这样的错：
```
Could not locate cyclonedds. Try to set CYCLONEDDS_HOME or CMAKE_PREFIX_PATH
```
这是因为 `unitree_sdk2py` 依赖编译好的 `cyclonedds`，pip 装不了这一层，需要先手动编译一遍
（这台机器要有 `cmake`/`gcc` 这类基础编译工具，没有的话先 `apt install build-essential cmake`）：

```bash
cd ~
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd cyclonedds && mkdir build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install
export CYCLONEDDS_HOME="$HOME/cyclonedds/install"
```

`export CYCLONEDDS_HOME=...` 这一行只在当前终端会话有效，重开一个终端要重新 export 一次，
嫌麻烦可以加进 `~/.bashrc`。编译好之后回到 `sugar_deploy` 目录重新装一次：

```bash
cd ~/sugar_deploy
pip install -e ".[unitree]"
```

这次应该能顺利装完。装完之后验证一下（不连机器人，只确认包本身能 import）：

```bash
python -c "import unitree_sdk2py; print('unitree_sdk2py OK')"
```

### 1.5 跑只读遥测脚本

确认机器人已经开机（可以是正常站立、被官方遥控器控制的状态，这一步不会干扰它）：

```bash
python scripts/real_robot_telemetry.py --interface eth0 --duration 10
```
（把 `eth0` 换成 1.2 里确认的真实接口名。）

**这个脚本从头到尾只订阅 `rt/lowstate`，不发布任何消息**——翻一下
`sugar_deploy/scripts/real_robot_telemetry.py` 和它调用的
`RealRobotIO.read_state()`（`sugar_deploy/real_robot_io.py`）就能确认这一点，这不是我口头
保证，是可以直接读代码验证的。

### 1.6 怎么看结果、下一步

**如果打印出来一堆 `[telemetry] ...` 行，10 秒后正常打印"完成"退出**：
```
[telemetry] n=1 base_quat_wxyz=[...] max|joint_pos-default|=0.xxxrad (some_joint_name)
```
看 `max|joint_pos-default|` 这个数——如果机器人当时站的是接近默认站姿（不是蹲着或者摆了个
奇怪姿势），这个数应该是零点几弧度这个量级（几度到二三十度），**不应该是好几个弧度、也不该
是一个看起来完全不合理的关节**（比如报的是某个手腕关节差了 1.5 弧度，但你看着机器人手腕根本
没有明显偏离默认姿态，这种情况就要怀疑映射错了）。连续跑几次、机器人摆几个不同姿态都试一下，
每次报的"差得最多的关节"和你目视观察到的实际情况应该对得上。

**核对通过**（差值量级合理、指出来的关节和实际观察吻合）→ 可以进入 `REAL_HARDWARE_DEPLOYMENT.md`
下面"阶段 1"，但阶段 1 开始机器人会失去自身平衡辅助，**必须先把机器人挂上吊架或者放倒在软垫
上**，这一步不能跳，见下一节详细说明。

**核对不通过**（差值离谱、或者指出来的关节和实际不符）→ **不要往下做**，回到
`sugar_deploy/unitree_joint_map.py`，把 `UNITREE_JOINT_NAMES` 这份列表和你机器人实际的
DDS 消息定义/SDK 文档核对一遍（不同批次固件、不同购机配置有细微差异不是不可能），核对对了
再重新跑这个脚本确认。

**如果脚本报错**（比如"还没收到过 rt/lowstate 消息"）→ 见文档最后的故障排查表。

## 2. 后续阶段：分阶段上机流程

**每一阶段开始前，确认机器人处于对应阶段要求的物理安全状态**（见每阶段说明），不要图省事
跳过支撑/防护直接测下一阶段。上面第 1 节就是下面的"阶段 0"，这里从阶段 1 开始往后排。

### 阶段 1：release 高层控制 + 零力矩指令

**⚠️ 这一步机器人会失去自带的平衡/站立辅助控制，会像断电一样瘫软下去。机器人必须先处于
上面"紧急停止"一节说的三种支撑状态之一（坐姿/悬挂/人工扶持），绝对不能在正常站立状态下
做这一步**——`release_high_level_control()` 一旦生效，接下来 `send_zero_torque()` 发的是
"这个关节不主动控制"的指令，机器人自己的重量会让关节自由下垂/整个瘫倒，这是预期行为，不是
bug，但只有在有物理支撑、不会摔到硬地面/伤到人的情况下才能做。

**跑之前确认**：第二个人已经拿着遥控器、就位，随时能按这台真机已验证的零力矩组合键 `L2+Y`。

还没有现成脚本，先写一个最小的手动测试（确认机器人已经在安全支撑状态下再跑）：

```python
from sugar_deploy.real_robot_io import RealRobotIO
import time

io = RealRobotIO(network_interface="eth0")
io.release_high_level_control()
print("已 release 高层控制，机器人现在应该会瘫软——确认是在支撑状态下再继续")
for _ in range(50):
    io.send_zero_torque()
    time.sleep(0.02)
print("零力矩指令发送正常")
```

确认这一步之后：机器人瘫软的方式看起来物理上合理（不是某个关节反着转/抽搐），DDS 发布通道
本身工作正常。

### 阶段 2：小增益位置保持（仍然要有物理支撑）

**跑之前确认**：还是三种支撑状态之一（脚离地或者只是轻触地面，不承重），遥控器还是有人拿着。

用**远小于** `contract.JOINT_STIFFNESS` 的增益尝试让当前姿态保持住。下面这份是**能直接跑
的完整代码**，不是要你自己填的伪代码——增益从满增益的 5% 开始（这个折扣本身是保守起点，
不是量出来的"正确"值，真机 PD 特性和 sim 不一定一样，第一次测必须从这么小开始）：

```python
from sugar_deploy.real_robot_io import RealRobotIO
from sugar_deploy import contract
import numpy as np
import time

io = RealRobotIO(network_interface="eth0")
io.release_high_level_control()

state = io.read_state()
q_target = state.joint_pos.copy()  # 保持当前姿态，不是走向 DEFAULT_JOINT_POS
kp_small = np.array(contract.JOINT_STIFFNESS) * 0.05  # 满增益的 5%，保守起点
kd_small = np.array(contract.JOINT_DAMPING) * 0.05

for _ in range(250):  # 50Hz 跑 5 秒
    io.send_command(q_target_contract=q_target, kp_contract=kp_small, kd_contract=kd_small)
    time.sleep(0.02)

io.send_zero_torque()
print("阶段 2 测试结束，已切回零力矩")
```

确认：指令发出去之后关节没有剧烈抖动/发散，姿态大致能保持住。**如果出现剧烈抖动，立刻停止
发送指令**（Ctrl+C，或者手动调用 `io.send_zero_torque()`），不要试图调大增益去"压住"抖动，
先回去检查是不是哪里的增益方向/单位不对。确认稳定之后，再考虑把 `0.05` 这个折扣逐步调大
（比如 0.05→0.15→0.3→...），不要一次跳到满增益。

上面的裸循环只保留作原理说明；后续真机复测应改用带急停/看门狗的等价命令：

```bash
python scripts/real_robot_gain_test.py \
  --interface eth0 --gain-scale 0.05 --target current --duration 5 \
  --safety-confirmation ROBOT_SUPPORTED_ESTOP_READY
```

### 阶段 3：满增益默认站姿

阶段 3 的完成情况、急停修正、Tracker/脚踝调查及暂时转入阶段 4 的交接总结，见
[阶段 3 实测总结与阶段 4 接续计划](STAGE3_SUMMARY_AND_STAGE4_PLAN.md)（2026-09-23）。
下文保留历次试验的历史叙述；当前结论以总结中的证据边界为准，尤其不要将历史试验命令当成继续加档的指示。

**（2026-09-22 更新）之前这里写得太粗——从"5% 增益、机器人还挂着不承重"直接跳到"满增益+
脚着地承重"，中间跨度太大，一步出问题不好判断是哪个环节的问题。拆成更小的子步骤，每一步
都要能单独判断"过还是不过"再往下走，不要图快合并着做。**

每个子步骤开始前都要重新确认：遥控器有人拿着、人在机器人旁边、支撑状态符合当前子步骤要求。
安全脚本已在 15% 档确认能从独立 DDS 手柄话题识别 `L2+Y`、锁存并用修正后的 HG 命令让关节
实际卸力。后续低层测试首选按 `L2+Y`。官方模式的语音播报不作为低层急停判据，因为高层模式
已经被 release。

**3.1 支撑状态下，增益逐步加大**：机器人仍然是悬挂/人工扶持（脚不承重），把阶段 2 用过的
`0.05` 这个增益折扣系数逐步调大——`0.05 → 0.15 → 0.3 → 0.5 → 0.75 → 1.0`，每一档都跑
个几秒确认没有抖动/发散再加下一档，任何一档出现异常就停在那一档往回退，不要跳档。目标关节角
在这一步**仍然用刚读到的当前姿态**（`state.joint_pos`），不是 `DEFAULT_JOINT_POS`——先确认
"保持住当前姿态"这个最基础的闭环在满增益下也稳，再引入"走向另一个姿态"这个新变量。

**实测状态（2026-09-22）：✅ 已完成。**机器人双脚离地悬挂，`0.05、0.15、0.3、0.5、0.75、
1.0` 各档当前姿态保持均稳定，无抖动、发散、猛转、异响或异常对抗。

每一档使用同一个安全脚本，只改 `--gain-scale`，且一档一档人工确认：

```bash
python scripts/real_robot_gain_test.py \
  --interface eth0 --gain-scale 0.15 --target current --duration 5 \
  --safety-confirmation ROBOT_SUPPORTED_ESTOP_READY
```

**3.2 支撑状态下，满增益走向默认站姿**：3.1 稳定之后，把目标换成
`contract.DEFAULT_JOINT_POS`，增益用满（`contract.JOINT_STIFFNESS`/`JOINT_DAMPING`，
sim2sim 验证过的那组值，真机不一定完全适用，但当起点），机器人仍然不承重——观察姿态变化过程
是不是平顺，不是猛地一甩过去。**禁止把默认姿态作为阶跃目标直接下发**；安全脚本要求显式给出
平滑过渡时间，并用 S 曲线插值，同时对规划峰值目标速度做启动前硬限制。

当前真机在 3.1 完成后的只读测量中，当前姿态到默认姿态最大差值为 `0.8253rad`（约 `47.3°`，
`left_elbow_joint`）。首次 3.2 使用 10 秒过渡、随后保持 2 秒，规划峰值目标速度约
`0.124rad/s`：

```bash
python scripts/real_robot_gain_test.py \
  --interface eth0 --gain-scale 1.0 --target default --duration 12 \
  --ramp-duration 10 --max-target-speed 0.15 \
  --safety-confirmation ROBOT_SUPPORTED_ESTOP_READY
```

**实测状态（2026-09-22）：✅ 已完成。**实际启动帧最大差值 `0.8255rad`（左肘），10 秒 S 曲线
过渡的规划峰值目标速度 `0.1238rad/s`。移动连续平顺，无猛转、抖动、异响或异常对抗，最终默认
姿态合理；随后零力矩收尾，进程正常退出且 `worker_error=None`。

**3.3 逐步转移承重**：3.2 稳定之后，才开始让脚接触地面。G1 的 `unitree_hg::LowState` 没有
足底载荷字段，现有 DDS 遥测无法量化承重比例；而当前控制器只是固定关节 PD，并没有基于 IMU/
足底接触的动态平衡。因此本阶段必须拆成多次人工验收，吊架始终保留张力和防坠能力，绝不能一次
松吊架或直接尝试完全承重。

第一次只做**双脚轻触地面**：先在双脚离地状态启动并平滑进入默认姿态，稳定后极慢地下放吊架，
只让两只脚底完整接触地面，吊带仍承担大部分重量。确认无倾斜趋势、脚底打滑、膝盖塌陷、抖动或
异常对抗后，立即重新升高到双脚明确离地，再按 `L2+Y` 结束。禁止在脚仍承重时主动结束脚本。

**实测状态（2026-09-22）：✅ 第一次双脚轻触测试已完成。**机器人先平滑进入默认姿态，再让
双脚轻触地面且吊带继续承担大部分重量，现场观察一切正常；随后先重新吊到双脚离地，再用
`L2+Y` 锁存零力矩。程序正常退出，`worker_error=None`。这不代表更高承重比例或完全站立已经
通过。

**第二次小幅部分承重（2026-09-22）：✅ 已完成。**在双脚完整接触后额外下放约 `1cm`，吊带
始终绷紧，短时部分承重表现正常；随后先恢复双脚完全离地，再用 `L2+Y` 锁存零力矩，程序正常
退出且 `worker_error=None`。在没有吊架载荷计的条件下，3.3 到此按保守范围验收完成；禁止继续
靠主观“再放一点”逼近吊带松弛，因为无法量化载荷，也不能据此推断机器人具备独立平衡能力。

该测试使用更严格的软件终止阈值：绝对倾角 `0.35rad`、关节速度 `2rad/s`、机身角速度
`1rad/s`；超限会锁存零力矩，所以吊架必须始终能立即承接全部重量。控制保持时限设得足够长，
计划结束仍以“先重新吊起，再按 `L2+Y`”为准。

**3.4 完全站立**：当前固定关节 PD 没有动态平衡能力，不能仅凭 3.3 的轻触/部分承重结果就松开
吊架。必须先实现并验证闭环站立平衡控制，或切换到经过验证的官方站立控制器；在此之前 3.4
明确阻塞，不能靠人工“试着松手”验收。

**Tracker 前置工作（2026-09-22）**：已从 SUGAR 官方来源下载 CarryBox `tracker.pt`，并用官方
IsaacLab v2.3.0 锁定的 `rsl-rl-lib==3.0.1` 严格加载。510→29 维离线推理无 NaN，CPU 平均约
`2.16ms`。固定静止 command 在 MuJoCo 中连续 10 秒保持稳定；同时发现 Tracker 会故意产生超出
关节范围的远目标，由仿真的 effort limit 截断成所需力矩，因此真机层新增了等效显式力矩限幅，
禁止把原始策略目标直接下发。

真机 Tracker 测试仍属于 3.4 的**悬挂前置验证**，不是完全站立。首次 10% 直接切换失败后，
当前复测要求双脚离地，策略力矩硬限制为额定值的 5%，并用 5 秒从默认姿态保持力矩平滑混合到
Tracker 力矩，再保持 5 秒：

```bash
python scripts/real_robot_tracker_static_test.py \
  --interface eth0 --effort-scale 0.05 --tracker-blend-duration 5 --tracker-duration 5 \
  --safety-confirmation ROBOT_SUSPENDED_TRACKER_READY
```

只有该测试及后续吊架兜底的逐档验收通过，才可重新评估 3.4；不得跳过直接松吊架。

**首次结果（2026-09-22）：❌ 10% 未通过。**Tracker 启用后自动检测到某关节速度
`2.152rad/s`，超过 `2.000rad/s` 软限制，安全层立即锁存零力矩，收尾正常且
`worker_error=None`。没有放宽安全阈值；代码已改为从默认姿态保持力矩用 5 秒 S 曲线混合到
Tracker 力矩，复测上限降为 5%，并在后续超速日志中打印具体关节名。10% 必须等 5% 通过后再评估。

**第二次结果（2026-09-22）：❌ 5% 平滑切入仍未通过。**触发关节为
`left_ankle_roll_joint`，上报速度峰值 `3.282rad/s`，超过 `2.000rad/s` 软限制；安全层立即锁存，
完成 30 秒 HG 零力矩收尾且 `worker_error=None`。现场未观察到明显运动；瞬时速度超限不等同于
大角度位移，因此既不能据此判定硬件故障，也不能把阈值调高后继续。下一步先运行无 Tracker 的
默认姿态基线诊断，记录每周期的 `q/dq/tau_est/q_target`，并用相邻位置差分速度核对上报速度：

```bash
python scripts/real_robot_baseline_diagnostic.py \
  --interface eth0 --ramp-duration 10 --hold-duration 10 \
  --safety-confirmation ROBOT_SUSPENDED_BASELINE_READY
```

若基线也出现左踝速度尖峰，优先排查 DDS 遥测或悬空踝关节控制；若基线正常，再做只计算、不向
电机发送 Tracker 输出的影子测试。原因明确之前不得再次启用 Tracker 闭环。

**无 Tracker 基线结果（2026-09-22）：✅ 正常。**10 秒平滑进入满增益默认姿态、随后保持
10 秒，全程未触发保护。全关节上报速度峰值为 `0.541rad/s`（左肘），对应位置差分速度峰值
`0.474rad/s`；左踝横滚上报 `max|dq|=0.267rad/s`、位置差分 `max|dq|=0.215rad/s`，位置总跨度
仅 `0.01179rad`，均明显低于 `2.000rad/s` 限制。30 秒 HG 零力矩收尾正常，
`worker_error=None`。这说明满增益默认姿态保持本身没有复现异常，下一步只能做 Tracker 影子测试，
不得直接再次让 Tracker 接管电机。

影子测试仍保持满增益默认姿态，只让 Tracker 读取实时状态并记录其原始动作、期望角度和 5% 限幅
前后力矩；代码中不会把 Tracker 的目标发给电机：

```bash
python scripts/real_robot_tracker_shadow_test.py \
  --interface eth0 --ramp-duration 10 --shadow-duration 10 --effort-scale 0.05 \
  --safety-confirmation ROBOT_SUSPENDED_TRACKER_SHADOW_READY
```

**影子测试结果（2026-09-22）：❌ 当前手工静止 command 不可用于闭环。**影子推理本身正常完成、
电机始终只收到默认姿态命令，未触发安全保护；但是 Tracker 原始动作达到 `-9.063`，对应左踝俯仰
期望偏移 `-3.975rad`。左踝横滚期望偏移范围为 `[-2.266,+0.049]rad`，5% 限幅后仍要求
`[-2.500,+1.416]Nm`，足以解释此前接管时的左踝加速。全局未限幅力矩最高达
`233.345Nm`（右膝）；影子阶段并未把这些值发给电机。30 秒零力矩收尾正常，
`worker_error=None`。

核对 IsaacLab v2.3.0 官方 `CircularBuffer` 后还修正了一处历史初始化差异：首帧应复制填满 5 帧，
而不是“四帧零值 + 当前帧”。用同一份影子实测状态离线重放后，极端输出数值基本不变，因此该差异
必须修复但不是本次异常的主因。当前主要问题是手工拼出的“默认关节角 + 零速度 + 零接触”静止
command 没有证据属于 CarryBox Tracker 的训练分布；官方推理配置始终由 `generator.ckpt` 产生
36 维 command。下一步必须接入官方 Generator 先做离线/影子验证，禁止继续使用这个手工 command
驱动真机 Tracker。

**官方 Generator 离线核对（2026-09-22）：✅ 主因基本确认。**CarryBox checkpoint 的真实配置为
`use_target=True`、`use_last_action=True`、`n_obs_steps=1`、`n_action_steps=8`；一次输出经官方插值后
为 `(36,36)` command chunk。以同一物体相对位姿、目标沿 x 方向偏移 1m、固定随机种子生成有效
command，再用本次影子实测的真机状态离线驱动 Tracker：Generator 的参考关节角相对默认姿态最大
偏移 `0.733rad`（右肩 yaw），Tracker 的全局最大目标偏移为 `0.772rad`（左髋 roll），左踝 roll
目标范围缩小到 `[-0.265,+0.029]rad`。相比手工 command 导致的左踝 roll `-2.266rad` 和左踝
pitch `-3.975rad`，输出已回到合理量级。因此后续只能使用 Generator command 做影子测试；下一次
仍不允许 Tracker 控制电机。

联合影子测试只消费官方 Generator 初始 chunk 的前 20 帧（官方下一次 Generator 调用前的
`0.4s`），并在连接 DDS 之前完成扩散采样和 command 硬限制校验：

```bash
python scripts/real_robot_generator_tracker_shadow_test.py \
  --interface eth0 --shadow-steps 20 \
  --safety-confirmation ROBOT_SUSPENDED_GENERATOR_SHADOW_READY
```

**首次联合影子结果（2026-09-22，双脚离地）：⚠️ 未允许闭环。**20 帧推理正常完成且没有任何策略
输出下发，实时最大关节速度仅 `0.130rad/s`；但 Tracker 目标在 0.4 秒内自行放大：左踝 pitch
偏移最低 `-1.342rad`，左踝 roll 最低 `-0.900rad`，左踝 roll 的 5% 分析力矩达到
`-2.500Nm` 限幅。机器人状态本身基本不动，说明这不是实机已经发生的大动作。与前一次影子状态
相比，悬空默认保持下右踝 pitch 还存在约 `0.131rad` 的状态差异；CarryBox 策略训练和推理时具有
足底接触并准备走向 1.5m 外的箱子，双脚完全悬空不是其正常状态分布。因此不得据此开启控制；下一步
只允许在吊架仍承担大部分重量、双脚完整轻触地面的条件下重复联合影子测试，以验证足底几何约束是否
让输出恢复。策略输出仍不得下发。

轻触复测使用交互式双确认，脚仍接触地面时程序会持续保持默认姿态，不会按定时器自动卸力。终端出现
提示后分别输入 `FEET_CONTACT` 和 `REHOISTED`；第二项只能在双脚已经完全离地后确认：

```bash
python scripts/real_robot_generator_tracker_shadow_test.py \
  --interface eth0 --shadow-steps 20 --contact-sequence \
  --safety-confirmation ROBOT_HOISTED_CONTACT_SHADOW_READY
```

**轻触地面联合影子结果（2026-09-22）：⚠️ 影子流程通过，尚未允许闭环。**按交互流程完成“离地
进入默认姿态 → 双脚轻触且吊带主要承重 → 20 帧影子推理 → 重新吊起 → 零力矩”，进程正常退出，
`worker_error=None`。机器人实测状态稳定：最大关节速度仅 `0.036rad/s`，机身倾角约
`0.122rad`，最大陀螺仪分量 `0.022rad/s`。左踝 roll 的策略目标从离地测试的最低
`-0.900rad` 改善为 `[+0.030,+0.230]rad`，说明足底约束确实显著改变了策略判断；但最大目标转移
到右踝 pitch `+1.433rad`，左右踝 pitch 的 5% 分析力矩都达到 `+2.500Nm` 限幅，左右髋 roll
也接近/达到各自限幅。这里的大位置目标是策略借 PD 请求受 effort limit 截断的支撑力矩，不应直接
解释为电机真的要转到该角度；影子测试没有发送这些目标。下一次若做控制，只能从“已验证默认姿态
力矩 → Tracker 满额限幅力矩”的极小混合比例开始，而不是再次把控制器整体切换成 5% 绝对力矩。

首次 1% 测试定义为力矩空间无冲击混合：
`tau = tau_default + alpha * (clip(tau_tracker, effort_limit) - tau_default)`，其中 `alpha` 在 20 帧
内用 S 曲线从 0 升到 `0.01`。这会保留约 99% 已验收的默认姿态保持力矩；它不是把全部保持力矩
替换成“额定力矩的 1%”。首次验收时程序硬限制 `tracker_mix<=0.01`，并继续使用轻触/重新吊起双确认：

```bash
python scripts/real_robot_generator_tracker_shadow_test.py \
  --interface eth0 --shadow-steps 20 --contact-sequence --tracker-mix 0.01 \
  --safety-confirmation ROBOT_HOISTED_TRACKER_MIX_READY
```

**1% 力矩混合结果（2026-09-22）：✅ 遥测通过，等待现场观察确认。**完整执行交互流程且未触发
保护，进程正常退出、`worker_error=None`。混合修正从 `0.003Nm` 平滑升到最大 `0.548Nm`
（右膝）；实际最大关节速度仅 `0.0274rad/s`（腰 yaw），0.4 秒内最大关节位移仅
`0.000515rad`（约 `0.030°`，左肩 yaw）。机身倾角约 `0.110rad`，首尾变化仅
`0.000024rad`，最大陀螺仪分量 `0.0224rad/s`。四个踝关节位移均小于 `0.000012rad`。
这些数据说明 1% 混合没有引入可测的姿态扰动；仍需现场确认无抖动、异响、打滑或异常对抗后才算
完整通过，且该结果不能直接外推到更高混合比例。

**现场确认：✅ 1% 完整通过。**现场未观察到抖动、异响、脚底滑动或异常对抗。操作者同时指出
本次吊架下放量小于上一次；遥测也支持两次接触工况不同：本次左右踝 pitch 相对默认姿态约为
`-8.1°/-9.2°`，上次为 `-9.6°/-10.7°`，机身倾角约 `6.3°` 对 `7.0°`。现有 DDS 没有足底
载荷且吊架没有载荷计，所以后续不能只靠“轻触”一词假定承重一致。提高混合比例前应增加启动状态
门限，记录并检查双踝 pitch 偏差、左右 roll、机身倾角及吊带始终绷紧；每一档都保持同样的接触
操作，不同时增加混合比例和下放量。

1% 完整通过后，下一档上限开放到 2%。在收到 `FEET_CONTACT` 后先保持默认姿态采样 1 秒，只有
以下状态门限全部满足才允许混入 Tracker：左右踝 pitch 相对默认姿态均处于
`[-0.23,-0.10]rad`、左右差不超过 `0.06rad`、任一踝 roll 偏差不超过 `0.06rad`、全关节
`max|dq|<=0.15rad/s`、机身 `max|gyro|<=0.15rad/s`、倾角不超过 `0.20rad`。若门限失败，程序
保持默认姿态，最多允许三次 `CONTACT_ADJUSTED` 微调复测；三次仍失败才等待 `REHOISTED`。它不会
在脚接触地面时自动卸力，也不会在门限通过前执行 Tracker 混合。

```bash
python scripts/real_robot_generator_tracker_shadow_test.py \
  --interface eth0 --shadow-steps 20 --contact-sequence --tracker-mix 0.02 \
  --safety-confirmation ROBOT_HOISTED_TRACKER_MIX_READY
```

**首次 2% 尝试（2026-09-22）：⏭️ 门限拒绝，未执行混合。**测得左右踝 pitch 偏差为
`[-0.0869,-0.1148]rad`，其中左踝小于接触下限 `0.10rad`，说明该侧接触比已验收的1%工况更轻；
其余 roll、关节速度、机身角速度和倾角门限均通过。程序没有发送任何 Tracker 修正，现场重新吊起后
才零力矩退出，`worker_error=None`。这不是2%测试失败，而是2%尚未开始。

任何一步出现意料之外的抖动/发散/倾倒趋势，第一反应是让拿遥控器的人按急停，同时操作脚本的人
立即终止控制。

### 阶段 4 及之后：接入 Tracker / Generator / AprilTag

阶段 4 的目标不是“把三个模块同时打开看看能不能走”，而是把真实物体观测、Generator 参考轨迹、
Tracker 力矩和真机状态组成一条**可观测、可回放、失去视觉就停止推进**的闭环。阶段 3.1～3.3
已经完成；3.4 不能靠固定关节 PD 独立验收，最终的动态站立能力要在 Tracker 闭环中验证。

此前文档写的“给 Tracker 手工固定站立 command”不再执行。真机已经证明手工拼出的静止 command
不属于 CarryBox Tracker 的可靠输入分布；较大的 Tracker 位置目标也不能单独判错，因为策略可能
用远 PD 目标请求经 effort limit 截断的支撑力矩。后续只使用官方 Generator 产生的有效 command，
验收重点是限幅力矩、实际 `q/dq`、IMU、接触和任务方向。

#### 4.0 当前实现边界与禁止事项

- ✅ Tracker/Generator checkpoint 加载、观测历史、真机状态转换、显式 effort limit、力矩空间
  平滑混合和安全控制器已经实现并有测试。
- ✅ 官方 Generator + Tracker 联合影子流程、轻触门限和 1% 力矩混合已运行；1% 只证明微小修正
  没有引入可测扰动，不代表完整策略可落地。
- ⚪ 真机相机、相机外参、真实 tag 偏移、视觉时间戳/失效处理、完整真机主循环尚未验收。
- **禁止**继续把脚踝的原始 `q_desired-current` 单独当成故障判据；也禁止把当前脚本里硬编码的
  虚拟箱子位姿用于更高比例真机控制。
- **禁止**在 AprilTag 丢失或位姿过期时继续生成新动作；不能用很大的 `max_stale_frames` 掩盖
  近场盲区。

#### 4.1 相机硬件识别（只读，不连接 DDS）

G1 头部相机如果通过颈后 USB-C 接到开发计算单元，它枚举在机器人 PC2，而不是当前外部控制电脑。
[官方 G1 接口说明](https://support.unitree.com/home/en/G1_developer/about_G1)把颈后
`No.6/7/8` 标为 USB 3.0 host、5V/1.5A，`No.9` 标为 USB 3.2 host + DP Alt Mode；
[宇树官方遥操作相机说明](https://github.com/unitreerobotics/xr_teleoperate/wiki/Camera_and_Image)
对多路 RealSense/Hub 推荐 `No.9`。**单个原装头部相机线束究竟预留给
哪个编号，现有公开资料没有给出可以覆盖所有 G1 批次的唯一答案，应以机身端口编号、该批次装配图和
宇树技术支持确认为准，不能凭插头能插进去就试。**绝不能插到 XT30 电源口或 5577 I/O 口。

本机 2026-09-22 的只读检查结果：外部电脑没有枚举到 Intel RealSense，唯一 `/dev/video0` 是
Iriun 虚拟输出设备；PC2 `192.168.123.164` 可以 ping 通，但尚无可用 SSH 认证，所以还没有检查
PC2 上的 USB 枚举。虚拟环境也尚未安装 `pyrealsense2`。

接线前先断开机器人运动测试并放稳；确认线头是头部相机的 USB-C 数据线、确认端口编号后再连接。
取得 PC2 登录后，在 PC2 上依次执行：

```bash
lsusb
ls -l /dev/video*
python -m pip install pyrealsense2
python scripts/real_robot_camera_probe.py --list-only
python scripts/real_robot_camera_probe.py \
  --frame-count 60 --output diagnostics/g1_head_camera.png
```

验收条件：枚举结果明确显示 RealSense 型号和唯一序列号，USB 链路为 USB 3.x，连续读取至少 60 帧，
分辨率/内参有限且合理，保存图片方向正确、没有明显损坏或整帧黑屏。探针脚本不导入 Unitree SDK、
不连接 DDS、不会发送电机指令。如果官方 `videohub_pc4`/TeleImager 已占用相机，先记录占用情况，
不要直接杀死或禁用随机服务；决定是复用官方图像流，还是经确认后让本项目独占设备。

#### 4.2 准备并检测 AprilTag（仍然只读）

使用 `tag36h11`，为箱子的正面、顶面和侧面准备三个不同 ID（建议从 `0/1/2` 开始）。打印时：

1. 关闭打印机“适合页面/自动缩放”，按 100% 比例打印，保持图案平整且四周有完整白色静区。
2. 用卡尺测量**最外圈黑色方框的外边缘到外边缘**，把实测米数作为 `tag_size_m`；不能用纸张、
   白边或设计文件的名义尺寸代替实测值。
3. 暂时不要永久贴箱子；先把单个 tag 正对相机放在约 0.5m、1.0m、1.5m 处，只验证 ID、
   `decision_margin`、检测率和距离比例。
4. 记录相机序列号、图像模式、tag family、ID、实测黑框边长；这些构成标定配置，不能运行时猜。

通过条件：静止 10 秒内无错误 ID，正常光照下采纳率稳定，估计距离随量具距离变化方向和比例正确。
此时只报告 tag 在相机坐标系中的位姿，不生成 Generator 指令。

#### 4.3 标定相机外参和 tag→箱体偏移

严格按 `APRILTAG_DEPLOYMENT.md` 第 2、6.3、6.4 节操作。采用推荐的 torso 局部参考系：
`anchor_pos=(0,0,0)`、`anchor_quat=identity`，每帧使用“相机相对 torso_link”的固定外参。
至少使用 3 个不同距离/方位的测量姿态拟合并留一组独立复核；不能直接把 MuJoCo 的 `chest_cam`
名义外参或文档示例中的零偏移复制到真机。

验收报告至少包含：位置残差 mean/p95/max、姿态残差 mean/p95/max、每个 tag 的独立结果、相机
时间戳间隔，以及遮挡后恢复时是否出现大跳变。任何米级、轴向颠倒或约 90°/180° 的固定误差都先
按坐标系/尺寸错误处理，不进入策略测试。

#### 4.4 真实视觉 → Generator → Tracker 影子主循环

实现真机版主循环，对应 `sim2sim.py::control_step()`：

1. `real_robot_io.read_state()` 提供关节和 IMU；torso 参考系按 4.3 的固定局部系约定处理。
2. AprilTag 检测在独立线程低频运行，控制/影子循环只读取带采集时间戳的最近完整快照，禁止半帧
   更新；必须同时限制“连续丢帧次数”和“墙钟时间年龄”。
3. Generator 异步产生 chunk；每个 chunk 记录它使用的物体位姿、目标位姿、随机种子和时间戳。
4. Tracker 以 50Hz 推理，但本阶段始终向电机发送已验收的默认姿态保持，策略输出只落盘。
5. 检测丢失、数据过期、Generator 超时、非有限值、观测/命令越界时停止推进策略状态，并保持
   已验收的安全命令；如果已进入运动阶段则锁存安全停止，不能继续消费旧轨迹。

把同一份记录输入仿真离线回放，核对动作方向、接触时序、effort limit 饱和比例和左右对称性。
原始 `q_desired` 大不是单独失败条件；实际速度超限、持续力矩饱和、与仿真响应显著不一致、视觉
跳变进入策略或任务方向明显错误才是不通过。

#### 4.5 真机分级闭环

只有 4.1～4.4 全部通过后才重新发送 Tracker 修正。每档都使用同一份真实视觉场景、同一安全门限
和吊架兜底，逐档执行“影子 → 极小力矩混合 → 提高混合比例”；每次只改变一个变量。比例序列和
每档时长要根据 4.4 的仿真/影子结果另行确定，不能从 1% 直接推断全比例安全，也不再为了通过一个
主观“轻触深度”门限反复下放机器人。

在能够量化或可靠判定接触、完整策略能恢复平衡、视觉失效能触发安全停止之前，吊带不得松弛。
CarryBox 的最后 0.2～0.4m 存在已知头部相机近场盲区；进入完整任务前必须通过多面 tag、第二视角
或明确的“停住等待重新观测”解决，不能仅扩大过期帧容忍值。

#### 4.6 阶段 4 完成定义

阶段 4 不是“机器人动了一次”。完成必须同时满足：真实标定配置可复现；视觉和控制日志可按时间戳
对齐；相同输入的仿真/实机响应差异有解释；急停/视觉丢失/进程异常均安全收尾；吊架下逐级闭环无
抖动、猛转、持续撞限幅或方向错误。独立无吊架 CarryBox 属于后续任务验收，必须另做风险评审。

## 3. 故障排查

| 现象 | 大概率原因 |
|---|---|
| `read_state()` 抛"还没收到过 rt/lowstate 消息" | 网络接口不对、domain_id 不对、机器人没开机、或者不在同一网段 |
| `pip install unitree_sdk2py` 报 cyclonedds 找不到 | 见第 1 节，需要先手动编译 cyclonedds |
| 阶段 0 遥测读出来的角度和实际姿态对不上 | `unitree_joint_map.py` 顺序映射错了，不要往下走，先查 |
| 阶段 2/3 关节剧烈抖动 | 增益方向/单位不对，或者控制频率跟不上；立即按手柄组合键，由 `SafeRealRobotController` 从原始按键数据锁存零力矩。官方高层阻尼键本身不能覆盖低层 DDS 指令 |
| `send_command()` 抛"还没调用 release_high_level_control()" | 这是故意设的安全拦截，不是 bug，按流程先 release |
| 低层测试时按手柄没有反应 | 不要继续加增益；确认使用安全脚本，并先运行只读 `real_robot_remote_monitor.py` 验证原始组合键数据 |
| 任何时候不确定要不要继续 | 停止测试、保持物理支撑；没有“先试试看”这个选项 |
