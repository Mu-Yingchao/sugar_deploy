# AprilTag 物体感知方案：实施方案与步骤

`sugar_deploy` 目前走 SUGAR 这条训练路线，真机部署（DDS/Unitree SDK 通信层）到时候参考
HDMI 的 [`EGalahad/sim2real`](https://github.com/EGalahad/sim2real)（细节见 README 的
SUGAR vs HDMI 评估）。物体感知这一块，SUGAR 和 HDMI 官方都是外部 MoCap（Vicon），都没有
AprilTag 方案——这份文档记录的是我们自己在 `sugar_deploy` 里从零实现、并且已经在仿真里
验证过的 AprilTag 方案，包括代码在哪、怎么验证的、真机部署还要做什么、以及实测发现的一个
真实限制（近场视野盲区）。

## 0. 现状一览

| | 状态 |
|---|---|
| 检测/位姿解算代码 | ✅ 已实现，`sugar_deploy/object_state.py` 的 `AprilTagObjectSource` |
| 相机帧抽象（sim 渲染 / 真实 RealSense） | ✅ 已实现，`sugar_deploy/camera_source.py` |
| Sim 内验证场景（相机 + 3 个 tag） | ✅ `assets/g1/carrybox_scene_apriltag.xml` |
| Sim 内标定方法 | ✅ `scripts/calibrate_apriltag_offsets.py`，已跑通 |
| 精度实测（sim，影子模式，不影响真实控制） | ✅ 检测到时位置误差 mean 5~6mm / p95 ~13mm，姿态误差 mean ~1°，见第 4 节 |
| 完整闭环跑通（AprilTag 直接驱动控制，不是仅观测） | ✅ `scripts/run_sim2sim.py --object-source apriltag`，见第 5 节 |
| **已知限制：近场视野盲区** | ⚠️ 实测发现，机器人贴近箱子准备伸手的最后 0.2~0.4m，固定胸前相机会看不到 tag，见第 6 节——**这是部署前必须计划好怎么处理的真实问题，不是理论上的顾虑** |
| 真机 RealSense 接入 | ⚪ 代码写了（`RealSenseCameraSource`），**没在真实硬件上测过** |
| 真机外参/tag 偏移标定 | ⚪ 未做，见第 7 节的步骤 |

## 1. 架构：两层可插拔接口

```
真实/仿真相机  --CameraSource.get_frame()-->  RGB 帧 + 内参
                                                    |
                                          AprilTagObjectSource
                                       (pupil_apriltags 检测 + 位姿解算)
                                                    |
                                    ObjectStateSource.get_pose() --> ObjectPose
                                                    |
                                    observation.py / sim2sim.py（不用改）
```

- **`CameraSource`**（`camera_source.py`）：只有一个方法 `get_frame() -> CameraFrame`（RGB +
  内参 + 时间戳）。`MujocoCameraSource` 从 MuJoCo 场景离屏渲染；`RealSenseCameraSource` 从
  真实 D435 读。`AprilTagObjectSource` 只依赖这个接口，不关心帧是渲染出来的还是拍出来的。
- **`ObjectStateSource`**（`object_state.py`，本来就有的接口）：`AprilTagObjectSource` 是这个
  接口的第三个实现，和 `MujocoGroundTruthSource`（sim 真值）、`MocapObjectSource`（真机
  MoCap）并列。`sim2sim.py` 的主循环只认这个接口，换感知方案不用碰主循环代码。

这两层拆开是为了让"检测/位姿解算对不对"这个问题能在**不接触任何真实硬件**的情况下，用
`MujocoCameraSource` 渲染出来的图跑通、核对精度——本文档第 3~6 节全部是这么做的。真机部署
只需要把 `MujocoCameraSource` 换成 `RealSenseCameraSource`（第 7 节），`AprilTagObjectSource`
本身不用改一行。

## 2. 参考系怎么选（真机部署前必须想清楚的一件事）

**先澄清一个容易搞混的地方**：SUGAR 论文原文说 $o_t^O$ 是物体相对机器人 **"root frame"**
的位姿，但训练代码（`commands.py:499` 的注释、`anchor_body_name` 这个变量名本身、以及三处
任务配置文件都写死的 `anchor_body_name="torso_link"`）用的是 **"anchor frame"** 这个概念，
实际值是 `torso_link`，不是 IsaacLab articulation 真正的 root（那是 `robot_base_pos_w`/
`robot_base_quat_w`，取自 `root_pos_w`/`root_quat_w`，对应 pelvis，`commands.py:1666-1670`
能看到两者是代码里明确区分开的两个不同属性）。这大概率是论文行文时的不严谨表述——`
sugar_deploy` 跟的是训练代码实际用的 torso_link，不是论文字面上的"root"，因为训好的
checkpoint 是照 torso_link 这个参考系训的，用错参考系观测会系统性偏离训练分布。

`observation.py` 里物体相对机器人的观测，用的是
`subtract_frame_transform(robot.anchor_pos_w, robot.anchor_quat_w, obj_pos_w, obj_quat_w)`——
这一步只做**减法**，`anchor_pos_w`（torso_link 位置）的绝对值从来没被单独使用过，全链路只
关心"物体相对 anchor 差多少"。这意味着：

- **不需要一个漂移无关的真世界坐标系**，只需要"物体位姿"和"anchor 位姿"用**同一个参考系**
  表达，这个参考系本身是什么都无所谓。
- sim2sim 里这个参考系就是 MuJoCo 的仿真世界系（`d.xpos`/`d.cam_xpos` 天然一致，免费拿到）。
- **真机部署推荐的简化方案**：既然 `torso_link` 的相机是刚性固定挂载（G1 这个 29dof 型号
  没有独立头部/颈部关节，见 `g1_29dof.xml` 里 `chest_cam` 直接挂在 `torso_link` 下），可以
  **不做任何里程计/状态估计，直接把"当前 torso_link 帧"当成参考系**：
  - `anchor_pos_w = (0,0,0)`，`anchor_quat_w = identity`（每一步都这样，不追踪机器人在
    世界里挪了多远）
  - `AprilTagObjectSource.set_camera_pose_w(...)` 每一步固定传"相机相对 torso_link 的标定
    外参"（一次性标定好的常量，不随时间变化）
  - 这样完全不需要处理里程计漂移问题，因为压根不用里程计
  - `base_quat_w`（pelvis 姿态，重力方向用）单独来自 IMU，这个和上面无关，正常给

  这是**推荐**方案，不是唯一方案——如果机器人本身已经有可靠的里程计/状态估计，也可以用那个
  当参考系，把 `anchor_pos_w` 换成真实里程计位置，`AprilTagObjectSource` 对应传"相机的里程计
  系位姿"（每一步变化，不是常量）。两种方案 `AprilTagObjectSource` 的代码完全不用改，区别
  只在调用方每一步给 `set_camera_pose_w()` 传什么。

## 3. Sim 内怎么验证（已经做过，命令都能跑）

依赖同一个 SUGAR 训练用的 venv（多装了 `pupil-apriltags`/`pillow`，已经在 `pyproject.toml`
里加好了）：

```bash
source /home/yingchaomu/下载/sugar-venv/bin/activate
cd /home/yingchaomu/下载/sugar_deploy
pip install -e .
```

### 3.1 精度验证（影子模式：不影响控制，只对比）

跑一遍真实的 CarryBox 闭环（机器人真的走向箱子、伸手），全程还是用
`MujocoGroundTruthSource` 控制，同时并行跑 `AprilTagObjectSource` 记录两者的误差：

```bash
python scripts/test_apriltag_perception.py \
    --tracker-checkpoint /home/yingchaomu/下载/SUGAR/demo_ckpts/CarryBox/tracker.pt \
    --generator-checkpoint /home/yingchaomu/下载/SUGAR/demo_ckpts/CarryBox/generator.ckpt \
    --control-steps 300
```

实测结果（多次运行，Generator 是扩散模型、每次采样有随机性，数字会有波动但同一量级）：

```
检测帧数 300, 采纳 122~160 (41%~53%), 零阶保持 140~178 帧（置信度/跳变拒绝 0 帧）
位置误差 (m): mean=0.005~0.008  p95=0.013~0.014  max=0.017~0.020
姿态误差 (deg): mean=~1.0  p95=~1.9~2.0  max=~2.5~3.0
```

**检测到 tag 的时候，精度是好的**（毫米级位置、1° 量级姿态），这一点在 sim 里可以确认。
"采纳率只有 41%~53%"不是精度问题，是第 6 节说的视野覆盖问题（tag 不在视野里的时候本来就
不该采纳，这是正确行为，不是 bug）。

### 3.2 完整闭环跑通（AprilTag 直接驱动控制）

不只是观测，是让 Tracker/Generator 真的拿 AprilTag 估计出来的箱子位置去决定动作：

```bash
python scripts/run_sim2sim.py --task CarryBox \
    --tracker-checkpoint /home/yingchaomu/下载/SUGAR/demo_ckpts/CarryBox/tracker.pt \
    --generator-checkpoint /home/yingchaomu/下载/SUGAR/demo_ckpts/CarryBox/generator.ckpt \
    --object-source apriltag --headless --control-steps 500
```

跑得通，`generator_calls` 符合预期（500 步 / 20 = 25 次）。去掉 `--headless` 能在本机看
MuJoCo 窗口（同样可能遇到 README 里"踩过的坑"第 7 条那个关窗口卡死/段错误的显示环境问题，
不影响这里的结果）。

### 3.3 怎么标定 tag 偏移（如果要改贴 tag 的位置/加新任务）

`carrybox_scene_apriltag.xml` 里 3 个 tag 相对箱子几何中心的偏移，不是手算的，是拿
MuJoCo 的真值反解出来的：

```bash
python scripts/calibrate_apriltag_offsets.py
```

原理：把机器人摆到几个"能看到目标 tag"的位置，渲染 + 检测拿到"tag 在相机系下的位姿"，
再用 `MujocoGroundTruthSource` 给的箱子真值反解出"tag 局部系下，箱子中心在哪、差多少
旋转"。输出可以直接粘贴进 `sugar_deploy/apriltag_sim_calibration.py`。真机标定是同一个
原理，只是"真值从哪来"换成动捕/精密量具，见第 7.4 节。

## 4. 检测代码本身的两个坑（已经踩过、已经修）

写 `AprilTagObjectSource` 时踩了两个坑，记录下来避免自己/别人重踩：

1. **MuJoCo 相机的内部坐标系约定和 AprilTag/OpenCV 不一样**——MuJoCo 的 `cam_xmat` 是
   OpenGL 风格：局部 -Z 朝画面里看，+Y 图像朝上；`pupil_apriltags`（跟 OpenCV 一致）解出来
   的位姿是局部 +Z 朝画面里看，+Y 图像朝下。直接拿 `cam_xmat` 去合成检测结果，位置误差能
   到几米、方向完全对不上（实测过：3.7m 误差，不是精度问题，是系选错了）。修法是在
   `MujocoCameraSource.get_camera_pose_w()` 里统一做一次转换（`diag(1,-1,-1)`），往上层
   暴露出来的永远是"OpenCV 约定"的相机位姿，`AprilTagObjectSource` 不用关心这件事。
2. **AprilTag 单 tag 位姿解算在某些角度下会给出置信度不低但完全错的解**——这是 AprilTag
   单应性解算本身已知的歧义问题，不是这份代码的 bug。第一版精度测试里 p95 误差被这类异常值
   拉到 0.54m/90°，加了两道门槛后收敛：`min_decision_margin`（置信度太低直接当没检测到）
   和 `max_jump_m`（新解出的位置和上一次采纳的位置差太远，当异常拒绝，因为物体不可能在一次
   检测间隔内跳这么远）。这两个参数不是可有可无的调参项，是实测过必须要有的防线。

## 5. 已知限制：近场视野盲区（实测发现，真机部署前必须计划）

跑完整闭环时（3.2 节）发现：机器人走到贴近箱子、准备伸手的最后 0.2~0.4m 时，固定俯仰角的
胸前相机会看不到贴在箱子前面/顶面的 tag——渲染出来的画面直接确认过（见下面两张对比），
不是检测算法失败，是几何上 tag 已经不在视场范围内了：

- 机器人离箱子 1.0~1.1m 时，顶面 tag 清晰在画面中央
- 机器人离箱子 1.3m（也就是几乎贴上箱子）时，tag 已经完全滚出画面下沿，画面里只有箱子
  表面，看不到任何标记

这是**固定倾角单相机在近距离操作任务上的通病**，不是这份代码写得不好——真实机械臂/人形
抓取系统里非常常见的问题，行业里的标准解法是：

- **加一个手腕相机**，专门覆盖最后抓取阶段的近场盲区，胸前/头部相机负责中远距离的接近导航——
  这是最推荐的方案，也是很多人形/机械臂系统的标准配置。
- 或者让相机能主动低头/俯仰跟随（这个 G1 型号没有独立头部关节，做不到，除非额外加一个云台）。
- 短期内没有条件加硬件时的权宜之计：把 `max_stale_frames` 调大（`run_sim2sim.py`
  `--object-source apriltag` 目前就是这么处理的，调到 200），让机器人在盲区内用"最后一次
  看到的位置"零阶保持完成动作——**这只在盲区窗口本身不太长、物体在这段时间里不会挪动太远
  时才可接受**，实测过盲区持续时长本身也有波动（不同次运行因为 Generator 采样随机性，有时
  很快重新进入视野，有时会一直盲到测试结束），不是一个稳定可预期的短窗口，真机部署不建议
  只靠调大这个参数了事。

## 6. 真机部署步骤

### 6.1 硬件准备

1. **确认你这台 G1 有没有自带可用的 RGB 相机**（EDU 配置一般有 Intel RealSense D435/D435i，
   具体型号看你买的配置单）。没有的话需要额外加装，挂载位置尽量刚性固定（不要用容易松动的
   支架，外参标定一次就要保持稳定）。
2. **打印 AprilTag**。**（2026-09-22）已经生成好可以直接打印的文件**，在
   `assets/g1/print_tags/apriltag_print_ready.pdf`（3 页，id 0/1/2，`tag36h11` family，
   来源 `AprilRobotics/apriltag-imgs`）——每页黑边方框标称边长 10.16cm，页面上直接印了这个
   标称值和一个红框标出量哪一圈。**打印设置选"100% 实际大小"/"无缩放"，不要用"适应页面"**
   （那个会按纸张自动缩放，尺寸就不对了）。打印机、纸张都有误差，**打印完必须用卡尺/直尺
   实测黑边方框的实际边长**（红框标出的那一圈，外边缘到外边缘），这个实测值才是后面要填的
   `tag_size_m`，不能直接用标称的 10.16cm（sim 里就是因为标称和实际没对上踩过一次坑，见
   第 4 节代码注释里 `TAG_SIZE_M = 0.15 * 256/480` 那行）。
3. **在真实箱子上贴 tag**，建议贴 2~3 个面（比如正面 + 顶面）——原因见 5 节，单面在近距离
   会失去视野，多面至少能保证一部分时间可见；另外抓取时手也可能挡住某一面。用哪个 tag_id
   贴哪个面自己定，贴完之后到 6.4 节标定的时候要对应记清楚，不需要和 sim 里的贴法（id0=前面/
   id1=顶面/id2=侧面）完全一样。

### 6.2 软件依赖

```bash
source /home/yingchaomu/下载/sugar-venv/bin/activate
cd /home/yingchaomu/下载/sugar_deploy
pip install -e ".[realsense]"   # 装 pyrealsense2，sim2sim 主链路不需要这个 extra
```

第一次接硬件，先单独验证 SDK 能不能拿到相机（不涉及 sugar_deploy 代码）：

```bash
python -c "
from sugar_deploy.camera_source import RealSenseCameraSource
cam = RealSenseCameraSource()
frame = cam.get_frame()
print('frame shape:', frame.rgb.shape)
print('intrinsics:', frame.intrinsics)
"
```

打印出来的 `fx`/`fy` 应该在合理范围（640x480 下一般 380~620，具体看机型/固件）——如果是
明显不对的数字（比如 0 或者几千），说明 SDK 没有正确初始化，不要往下接。

### 6.3 相机外参标定（相机相对 torso_link 的固定变换）

按第 2 节的推荐方案（以机体为参考系，不追踪世界位置），这一步标定的是一个**常量**：相机
坐标系相对 `torso_link` 坐标系的旋转 `R` 和平移 `t`（OpenCV 约定：Z 朝画面里看，Y 朝下）。

最简单的标定方法（不需要专业设备）：

1. 找一个已知尺寸、能看清楚位姿的标定板（可以直接用同款 AprilTag，贴在一个已经用卷尺/CAD
   图纸量出"相对 torso_link 位置"的固定点上，比如贴在正前方 1m 处地面标记好的位置）。
2. 用相机拍这个标定板，检测出"标定板在相机系下的位姿"（`pupil_apriltags` 直接给）。
3. 你已经知道"标定板在 torso_link 系下的位姿"（量出来的），两者结合直接解出"相机在
   torso_link 系下的位姿"，也就是要标定的外参。
4. 换 2~3 个不同标定板位置重复，取平均或者选残差最小的一组，减小测量误差影响。

如果机器人 URDF/MJCF 里相机挂载点的名义坐标已知（比如这次 sim 里 `g1_29dof.xml` 加的
`chest_cam` 那样，直接写在 XML 里的 `pos`/`xyaxes`），可以先拿名义值当初始猜测，再用上面
的方法微调校正真实安装误差（螺丝孔位置、支架公差这些名义值覆盖不到的部分）。

### 6.4 tag 相对物体的偏移标定

原理和 3.3 节的 `calibrate_apriltag_offsets.py` 完全一样，只是"箱子真值"这次不是 MuJoCo
给的，需要你自己想办法拿到（比如先用卷尺量出"tag 贴的位置相对箱子设计中心的名义偏移"作为
起点，再用检测结果和已知的箱子摆放位置反解校正）：

1. 把真实箱子摆在一个你能精确量出位置的地方（比如贴着一面墙、放在量好格线的地面上）。
2. 用已经标定好外参的相机拍一张，检测 tag，算出"tag 在 torso_link 系下的位姿"。
3. 你知道箱子摆放的真实位置（步骤 1 量的），结合上面反解出"tag 局部系下，箱子中心在哪、
   差多少旋转"——和 `calibrate_apriltag_offsets.py` 里的公式完全一样：
   ```python
   pos_offset = tag_rot.T @ (box_pos_true - tag_pos)
   rot_offset = tag_rot.T @ box_rot_true
   ```
4. 贴了几个面就重复几次，每个 tag_id 一组独立的偏移。

### 6.5 接入 sim2sim 主循环

真机部署代码本身（DDS/Unitree SDK 通信层）还没写，等接的时候，感知这部分只需要：

```python
# ⚠️ 这段是结构示例，不是能直接复制运行的代码——下面标了 TODO 的几处，必须先做完 6.3/6.4
# 节的真机标定，拿到真实数字填进去，不能直接照抄这段跑（tag_size_m 的 0.10 也只是占位，
# 换成你 6.1 节实际量出来的黑框边长）。
import numpy as np
from sugar_deploy.camera_source import RealSenseCameraSource
from sugar_deploy.object_state import AprilTagObjectSource, TagObjectOffset

camera_source = RealSenseCameraSource()
tag_offsets = [
    TagObjectOffset(
        tag_id=0,
        pos_offset=np.array([0.0, 0.0, 0.0]),  # TODO: 6.4 节标定出来的真实值，别用这个占位
        rot_offset=np.eye(3),                   # TODO: 同上
    ),
    # 贴了几个面就加几条
]
object_source = AprilTagObjectSource(
    camera_source, tag_size_m=0.10,  # TODO: 6.1 节量出来的实际黑框边长，替换这个占位值
    tag_offsets=tag_offsets,
    max_stale_frames=150,  # TODO: 按真实控制频率和能接受的盲区时长定，这只是一个起点建议
)

# 每个控制步（或者按检测能跟上的频率，见下面 6.6）：
cam_pos_torso = np.array([0.0, 0.0, 0.0])  # TODO: 6.3 节标定出来的固定外参（常量，不随时间变）
cam_rot_torso = np.eye(3)                   # TODO: 同上
object_source.set_camera_pose_w(cam_pos_torso, cam_rot_torso)
object_source.update()
# 主循环的 robot.anchor_pos_w / anchor_quat_w 按第 2 节的约定固定填 (0,0,0) / identity
```

### 6.6 检测频率

`pupil_apriltags` 检测本身有算力开销（sim 里 640x480、CPU 上大概几毫秒到十几毫秒一帧，
具体看机载算力），如果真机主控算力有限、跟不上 50Hz 控制频率，`AprilTagObjectSource.update()`
是单独暴露出来的方法，不强制每个控制步都调——可以按更低频率调用（比如 10~15Hz），中间的
控制步继续用 `get_pose()` 读零阶保持的值，这个类本身不替调用方决定这个节奏，怎么调用是
主循环的事。

## 7. 故障排查

| 现象 | 大概率原因 | 怎么查 |
|---|---|---|
| `get_pose()` 一直抛"还没检测到任何已知 tag" | tag 不在视野里，或者 `tag_id` 没在 `tag_offsets` 里配置，或者 `tag_family` 不对 | 单独跑 `camera_source.get_frame()` 存图看一眼，确认 tag 确实在画面里且清晰 |
| 检测到但位置误差很大（米级） | 内参不对，或者 `tag_size_m` 和实物不一致，或者相机坐标系约定搞反了（见第 4 节第 1 条） | 先用一个已知精确距离的物体测距，反推内参对不对；`tag_size_m` 拿卡尺重新量一遍黑框边长 |
| 精度偶尔跳变得很离谱（但大部分时候准） | AprilTag 单 tag 位姿歧义（见第 4 节第 2 条） | 确认 `min_decision_margin`/`max_jump_m` 有设置且没被改成 0 或者极大值 |
| 机器人快摸到箱子时开始用明显过期的位置 | 近场视野盲区（见第 5 节） | 检查这个阶段相机画面里箱子/tag 是不是已经滚出画面；短期内只能调大 `max_stale_frames` 权宜，长期建议加手腕相机 |
| `RealSenseCameraSource()` 初始化就报错 | SDK 没装对，或者相机没插好/没通电 | 先用 `realsense-viewer`（RealSense 官方工具）确认相机本身能正常出图，再排查 Python 这边 |

## 8. 还没做的

- 真机 RealSense 接入完全没在真实硬件测过（`RealSenseCameraSource` 是照官方 API 文档写的）。
- 手腕相机覆盖近场盲区——目前只有胸前一个相机，第 5 节讨论过但没实现。
- 真机 DDS/Unitree SDK 部署代码本身，参考 HDMI 的 `EGalahad/sim2real`（见 README 的评估）。
- 其余 5 个任务（KickBox/PushBox/PickBottle/StandBottle/SitChair）目前都还只有 CarryBox
  一个 AprilTag 验证场景，需要照着 `carrybox_scene_apriltag.xml` 给对应物体加相机+贴图。
