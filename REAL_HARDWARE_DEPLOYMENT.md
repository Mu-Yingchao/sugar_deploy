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

## 0. 现在有什么、还没有什么

| | 状态 |
|---|---|
| `unitree_joint_map.py`：contract.JOINT_NAMES ↔ 真机 DDS 电机数组顺序的映射 | ✅ 已实现，用关节名字做过 round-trip 验证（不是数值巧合，是真的映射对了），但**顺序表本身来自 HDMI 部署仓库，没有在真实 G1 上核对过** |
| `real_robot_io.py`：`rt/lowstate` 订阅 / `rt/lowcmd` 发布，CRC、release 高层控制 | ✅ 已实现，照着 HDMI `real_bridge.py` 的真实用法写的，**没在真实硬件上跑过** |
| `scripts/real_robot_telemetry.py`：只读遥测，不发任何指令 | ✅ 已实现，**这是你应该跑的第一个脚本** |
| 位置控制指令发送、release 高层控制 | ✅ 代码写了（`send_command`/`release_high_level_control`），**下面第 2 阶段之前不要调用** |
| Tracker/Generator 接入真机闭环 | ⚪ 未实现，等第 0~3 阶段都过了再做 |
| AprilTag 相机外参/tag 偏移真机标定 | ⚪ 未做，方法见 `APRILTAG_DEPLOYMENT.md` 第 6.3/6.4 节，要等机器人能稳定站/走之后再做意义更大 |

## 1. 阶段 0 完整逐步指导（唯一不需要任何物理防护的一步，从这里开始）

### 1.1 在哪台机器上跑

这一步需要在**能直接收到机器人 DDS 广播**的机器上跑，不是随便哪台能上网的电脑都行——
两种常见情况选一种：

- **机器人自带的机载电脑**（很多 Unitree 机型自己带一台 Jetson/NUC，出厂就在同一个网络里，
  SSH 上去跑就行，最省事）。
- **一台用网线直连机器人网口的笔记本/台式机**——G1 通常有一个专门给二次开发用的以太网口，
  网线插上之后这台笔记本就在机器人的局域网里了。

不确定自己是哪种情况的话，先确认：机器人开机、网线插好（或者机载电脑本身就在跑）之后，能不能
`ping` 通机器人给的默认网关/机器人本体地址——Unitree 系列机器人**通常**用
`192.168.123.0/24` 这个网段（比如机器人本体是 `.161`、机载电脑是 `.18` 这类，具体数字按你
机器人本体上贴的标签或者购机文档给的为准，这里说的是行业里常见的默认约定，不是保证你这台
一定是这个），先 `ip a` 看这台机器自己在这个网段里分到的地址，能看到就说明网络通了。

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

在 1.1 选定的那台机器上：

```bash
git clone https://github.com/Mu-Yingchao/sugar_deploy.git   # 如果这台机器上还没有这个仓库
cd sugar_deploy
python3 -m venv .venv && source .venv/bin/activate   # 或者用你已有的 venv/conda 环境都行
pip install -e ".[unitree]"
```

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

**⚠️ 这一步机器人会失去自带的平衡/站立辅助控制，会像断电一样瘫软下去。机器人必须先挂在
吊架上，或者摆成躺姿放在软垫上，绝对不能在正常站立状态下做这一步**——`release_high_level_
control()` 一旦生效，接下来 `send_zero_torque()` 发的是"这个关节不主动控制"的指令，机器人
自己的重量会让关节自由下垂/整个瘫倒，这是预期行为，不是 bug，但只有在有物理支撑、不会摔到
硬地面/伤到人的情况下才能做。

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

机器人挂在吊架上（脚离地或者只是轻触地面，不承重），用**远小于** `contract.JOINT_STIFFNESS`
的增益（比如打个 5~10% 折扣都不够，建议先从个位数的 kp 开始）尝试让当前姿态保持住：

```python
state = io.read_state()
io.send_command(q_target_contract=state.joint_pos, kp_contract=..., kd_contract=...)  # 增益自己填一个很小的数组
```

确认：指令发出去之后关节没有剧烈抖动/发散，姿态大致能保持住。**如果出现剧烈抖动，立刻停止
发送指令**（比如 Ctrl+C，或者切回 `send_zero_torque()`），不要试图调大增益去"压住"抖动，
先回去检查是不是哪里的增益方向/单位不对。

### 阶段 3：满增益默认站姿

增益换成 `contract.JOINT_STIFFNESS`/`JOINT_DAMPING`（sim2sim 验证过的那组值，真机不一定
完全适用，但可以当起点），目标关节角用 `contract.DEFAULT_JOINT_POS`。这一步开始可以考虑让
机器人脚着地承重，**但要有人在旁边随时准备扶住**，不要第一次满增益测试就完全放手。

### 阶段 4 及之后：接入 Tracker / Generator / AprilTag

这几步现在都还没写代码，大致方向（到时候再展开细化）：

1. 写一个真机版的主循环，结构上对应 `sim2sim.py` 的 `control_step()`，但 `_apply_pd()`
   换成 `real_robot_io.send_command()`，`_read_robot_state()` 换成 `real_robot_io.read_state()`
   （加 anchor/torso 相关的量需要额外做，`read_state()` 目前只给关节角/IMU，torso_link 的
   位置需要正运动学或者按 `APRILTAG_DEPLOYMENT.md` 第 2 节"以机体为参考系"的简化方案处理）。
2. 先只测 Tracker（不接 Generator），喂一个固定的"站立不动"command，确认能站稳。
3. 再接 Generator，验证走路。
4. 最后接 `AprilTagObjectSource` + `RealSenseCameraSource`（`APRILTAG_DEPLOYMENT.md` 第
   6 节的标定步骤要先做），跑 CarryBox。

## 3. 故障排查

| 现象 | 大概率原因 |
|---|---|
| `read_state()` 抛"还没收到过 rt/lowstate 消息" | 网络接口不对、domain_id 不对、机器人没开机、或者不在同一网段 |
| `pip install unitree_sdk2py` 报 cyclonedds 找不到 | 见第 1 节，需要先手动编译 cyclonedds |
| 阶段 0 遥测读出来的角度和实际姿态对不上 | `unitree_joint_map.py` 顺序映射错了，不要往下走，先查 |
| 阶段 2/3 关节剧烈抖动 | 增益方向/单位不对，或者控制频率跟不上（`send_command` 发送频率如果远低于 sim2sim 验证过的 50Hz，PD 会不稳） |
| `send_command()` 抛"还没调用 release_high_level_control()" | 这是故意设的安全拦截，不是 bug，按流程先 release |
