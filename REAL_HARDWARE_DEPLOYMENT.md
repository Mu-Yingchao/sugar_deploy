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

## 1. 前置确认

1. **网络接口**：找到你的开发机（或者机载电脑）和机器人通信用的网口名字，`ip a` 看一下，
   通常是 `eth0`，不一定，确认好传给下面脚本的 `--interface`。
2. **确认机型**：`unitree_joint_map.py`/`real_robot_io.py` 目前只支持 **G1 29dof**
   （`unitree_hg` 消息族）——如果你的 G1 是别的关节数配置，`UNITREE_JOINT_NAMES` 这份列表
   要重新核对，不能直接用。
3. **装 SDK**（在能连到机器人网络的那台机器上）：
   ```bash
   pip install -e ".[unitree]"
   ```
   大概率会卡在编译 `cyclonedds` 这一步（`Could not locate cyclonedds`），需要先手动编译：
   ```bash
   git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
   cd cyclonedds && mkdir build install && cd build
   cmake .. -DCMAKE_INSTALL_PREFIX=../install
   cmake --build . --target install
   export CYCLONEDDS_HOME="$(pwd)/../install"
   ```
   然后重新跑 `pip install -e ".[unitree]"`。这个 FAQ 来源同样是 HDMI 部署仓库。

## 2. 上机测试：分阶段流程

**每一阶段开始前，确认机器人处于对应阶段要求的物理安全状态**（见每阶段说明），不要图省事
跳过支撑/防护直接测下一阶段。

### 阶段 0：只读遥测（不需要任何物理防护，机器人可以正常站着/被遥控器控制）

```bash
python scripts/real_robot_telemetry.py --interface eth0 --duration 10
```

这个脚本**不发送任何指令**，物理上不可能影响机器人，可以随时跑。核对打印出来的
`max|joint_pos-default|`（机器人如果是默认站姿附近，这个数应该是零点几弧度量级，不是几个
弧度、也不该是明显不合理的符号）——这是在验证 `unitree_joint_map.py` 的顺序映射对不对，
**这一步没过之前，后面所有阶段都不要做**。如果对不上，先去 `unitree_joint_map.py` 核对每个
关节名字对应的下标，不要猜。

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
