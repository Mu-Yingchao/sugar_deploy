"""物体（箱子/瓶子/椅子）6D 位姿的获取接口。

SUGAR 论文本身的真机实验里，物体状态就是靠外部 MoCap 拿的，不是机器人自己拿摄像头
感知的——这个仓库第一版沿用同样的假设：物体状态是一个可插拔的外部输入，不在这里解决
"机器人自己看见物体在哪"这个感知问题。

``MujocoGroundTruthSource`` 是 sim2sim 自验证用的：直接读 MuJoCo 仿真里物体 body 的
真实状态，效果上等价于 SUGAR 在 IsaacSim 里训练/推理时的做法（用仿真给的 ground truth
物体状态，不是重建出来的）。真机部署时把它换成 ``MocapObjectSource`` 或
``AprilTagObjectSource``（或者你自己的感知方案）即可，上层 observation.py / sim2sim.py
不需要跟着改。

``AprilTagObjectSource`` 是 AprilTag 视觉方案的实现，配合 ``camera_source.py`` 的
``CameraSource`` 抽象使用，详细的标定步骤和精度实测数据见 sugar_deploy 仓库的 AprilTag
部署文档。
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np


@dataclass
class ObjectPose:
    """世界系下的物体位姿/速度。旋转用 3x3 矩阵（和 SUGAR 训练数据 obj_rot 的
    约定一致，不是四元数——这是原指南踩过的一个坑，这里直接对齐，不留隐患）。"""

    pos_w: np.ndarray       # (3,) 世界系位置
    rot_w: np.ndarray       # (3,3) 世界系旋转矩阵
    lin_vel_w: np.ndarray   # (3,) 世界系线速度
    ang_vel_w: np.ndarray   # (3,) 世界系角速度


class ObjectStateSource(ABC):
    """物体状态来源的抽象接口，sim2sim / 真机部署共用同一套下游代码。"""

    @abstractmethod
    def get_pose(self) -> ObjectPose:
        """返回当前时刻的物体位姿。调用方（observation.py）负责做坐标变换和
        30/50Hz 的时序处理，这里只管"此刻物体在哪"。"""
        raise NotImplementedError


class MujocoGroundTruthSource(ObjectStateSource):
    """从 MuJoCo 仿真直接读物体 body 的真实状态，用于 sim2sim 自验证。"""

    def __init__(self, mj_model, mj_data, body_name: str):
        import mujoco

        self.model = mj_model
        self.data = mj_data
        self.body_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if self.body_id < 0:
            raise ValueError(f"MuJoCo 模型里找不到 body '{body_name}'")

    def get_pose(self) -> ObjectPose:
        import mujoco

        pos_w = self.data.xpos[self.body_id].copy()
        rot_w = self.data.xmat[self.body_id].reshape(3, 3).copy()
        # cvel: (6,) = [ang_vel(3), lin_vel(3)]，世界系原点表达，mj_objectVelocity
        # 转成 body 质心处的速度更准确
        vel6 = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, self.body_id, vel6, 0)
        ang_vel_w = vel6[:3].copy()
        lin_vel_w = vel6[3:].copy()
        return ObjectPose(pos_w=pos_w, rot_w=rot_w, lin_vel_w=lin_vel_w, ang_vel_w=ang_vel_w)


class MocapObjectSource(ObjectStateSource):
    """真机部署用：从外部 MoCap 系统订阅物体位姿。

    这是一个占位实现，没有接任何真实 MoCap 协议——按论文原始设置，真机部署本来就
    依赖外部 MoCap，具体是哪一套系统（Vicon/OptiTrack/其他）、怎么订阅、坐标系怎么
    对齐到机器人世界系，这些是你的 MoCap 系统决定的，需要你按自己的现场环境接。

    最简单的接入方式：起一个线程/进程订阅 MoCap 的位姿流，每次收到新数据就调用
    ``update()`` 写进来，``get_pose()`` 直接返回最近一次收到的值（做零阶保持）。
    """

    def __init__(self, initial_pose: ObjectPose | None = None):
        self._pose = initial_pose

    def update(self, pose: ObjectPose) -> None:
        """MoCap 回调/订阅线程里调用，喂入最新收到的物体位姿。"""
        self._pose = pose

    def get_pose(self) -> ObjectPose:
        if self._pose is None:
            raise RuntimeError(
                "MocapObjectSource 还没收到过任何数据——真机部署前必须先接好 MoCap 订阅，"
                "并在主循环开始前至少调用一次 update()。"
            )
        return self._pose


@dataclass(frozen=True)
class TagObjectOffset:
    """单个 AprilTag 相对物体参考点（比如箱子几何中心）的静态位姿偏移，贴 tag 时量一次、
    写死在配置里，不是运行时算出来的。

    约定跟着 ``pupil_apriltags`` 的 tag 坐标系定义走：原点在 tag 图案中心，z 轴指向 tag
    图案正面外侧（也就是垂直穿出纸面朝向观察者的方向），x/y 轴在 tag 平面内。贴 tag 的时候
    按这个约定量：``pos_offset`` 是"物体参考点相对 tag 原点，用 tag 坐标系的轴表达"，
    ``rot_offset`` 是"tag 坐标系到物体参考点坐标系"的旋转。
    """

    tag_id: int
    pos_offset: np.ndarray  # (3,)
    rot_offset: np.ndarray  # (3,3)


class AprilTagObjectSource(ObjectStateSource):
    """用 AprilTag 视觉检测算物体位姿——真机部署时替代 MoCap 的方案，也可以在 sim2sim 里
    配合 ``camera_source.MujocoCameraSource`` 单独验证检测/位姿解算代码本身对不对。

    在物体不同的面上贴多个 tag（``tag_offsets`` 传多条）是为了防止抓取过程中手挡住某一个
    面——任意一个可见的 tag 都能独立解出完整的物体位姿，多个同时可见时取
    ``decision_margin``（pupil_apriltags 给的检测置信度）最高的那个，不做多 tag 融合
    （融合能提高精度，但会明显增加复杂度，第一版没必要）。

    短暂检测不到任何已知 tag 时做零阶保持（沿用上一次的位姿），和 ``MocapObjectSource``
    一样的降级策略；连续太多帧检测不到会在 ``get_pose()`` 里报错，不会静默返回过期很久
    的数据给下游。
    """

    def __init__(
        self,
        camera_source,
        tag_size_m: float,
        tag_offsets: list[TagObjectOffset],
        tag_family: str = "tag36h11",
        max_stale_frames: int = 15,
        min_decision_margin: float = 30.0,
        max_jump_m: float = 0.3,
    ):
        """``min_decision_margin``/``max_jump_m`` 是两道异常拒绝的门槛，不是可有可无的
        调参项——实测过（见 sugar_deploy 的 AprilTag 部署文档"精度实测"一节）单 tag 位姿
        解算在某些角度/距离下会给出**检测置信度不低、但位姿完全错的解**（AprilTag 单应性
        解算本身的已知歧义问题，不是这份代码的 bug），300 步的真实走近-伸手轨迹里 p95 位置
        误差被这类异常值拉到 0.54m、姿态误差拉到 90°，加这两道门槛后才收敛到能用的水平：

        - ``min_decision_margin``：检测置信度低于这个值直接当成"没检测到"处理，不进入
          位姿解算这一步。
        - ``max_jump_m``：新解出的位置如果和上一次*采纳*的位置差超过这个距离，当成异常
          拒绝掉（沿用上一次的位姿，不更新），因为物体不可能在一次检测间隔内跳这么远——
          这是"检测置信度也骗不过去"的那批歧义解的主要拦截手段。
        """
        from pupil_apriltags import Detector

        self.camera_source = camera_source
        self.tag_size_m = tag_size_m
        self.tag_offsets = {t.tag_id: t for t in tag_offsets}
        self.detector = Detector(families=tag_family, nthreads=2)
        self.max_stale_frames = max_stale_frames
        self.min_decision_margin = min_decision_margin
        self.max_jump_m = max_jump_m

        self._camera_pose_w: tuple[np.ndarray, np.ndarray] | None = None
        self._pose: ObjectPose | None = None
        self._stale_count = 0
        self.last_tag_id: int | None = None
        self.last_decision_margin: float = 0.0
        self.n_rejected_low_margin = 0
        self.n_rejected_jump = 0

    def set_camera_pose_w(self, pos_w: np.ndarray, rot_w: np.ndarray) -> None:
        """告诉这个 source"相机现在在哪"，要求和 ``robot.anchor_pos_w`` 用同一个参考系表达
        （具体是不是真的惯性系不重要，下游只做相对减法，见部署文档"参考系怎么选"一节）。

        sim2sim 里每个控制步用 ``MujocoCameraSource.get_camera_pose_w()`` 给的真实值喂进来；
        真机部署如果采用"以机体为参考系"的简化约定（推荐，见部署文档），这里固定传相机相对
        ``torso_link`` 的标定外参就行，不需要额外的里程计/状态估计。
        """
        self._camera_pose_w = (np.asarray(pos_w, dtype=np.float64), np.asarray(rot_w, dtype=np.float64))

    def update(self) -> bool:
        """抓一帧、跑检测、更新内部位姿。返回这一帧是否检测到了至少一个已知 tag。

        单独暴露成一个方法（不是塞进 ``get_pose()`` 里懒加载）是为了让调用方能控制检测节奏——
        AprilTag 检测本身有算力开销，真机上如果跟不上主循环频率，可以按更低频率调用
        ``update()``，中间的控制步继续用 ``get_pose()`` 读零阶保持的值，这个类不替调用方
        决定这个节奏。
        """
        if self._camera_pose_w is None:
            raise RuntimeError("update() 之前必须先调用 set_camera_pose_w() 告诉相机现在在哪")
        cam_pos_w, cam_rot_w = self._camera_pose_w

        frame = self.camera_source.get_frame()
        gray = _rgb_to_gray(frame.rgb)
        intr = frame.intrinsics
        detections = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=(intr.fx, intr.fy, intr.cx, intr.cy),
            tag_size=self.tag_size_m,
        )
        candidates = [
            d for d in detections
            if d.tag_id in self.tag_offsets and d.decision_margin >= self.min_decision_margin
        ]
        if not candidates:
            if any(d.tag_id in self.tag_offsets for d in detections):
                self.n_rejected_low_margin += 1
            self._stale_count += 1
            return False

        best = max(candidates, key=lambda d: d.decision_margin)
        offset = self.tag_offsets[best.tag_id]
        tag_rot_cam = np.asarray(best.pose_R, dtype=np.float64)
        tag_pos_cam = np.asarray(best.pose_t, dtype=np.float64).reshape(3)

        obj_rot_cam = tag_rot_cam @ offset.rot_offset
        obj_pos_cam = tag_pos_cam + tag_rot_cam @ offset.pos_offset

        obj_rot_w = cam_rot_w @ obj_rot_cam
        obj_pos_w = cam_rot_w @ obj_pos_cam + cam_pos_w

        if self._pose is not None:
            jump = float(np.linalg.norm(obj_pos_w - self._pose.pos_w))
            if jump > self.max_jump_m:
                self.n_rejected_jump += 1
                self._stale_count += 1
                return False

        self._pose = ObjectPose(
            pos_w=obj_pos_w,
            rot_w=obj_rot_w,
            # AprilTag 检测本身不产出速度，且下游 observation.py 目前也没有任何地方读
            # ObjectPose.lin_vel_w/ang_vel_w（只有 MujocoGroundTruthSource 会填这两个字段），
            # 这里填零不影响现有行为——真要用到速度再另外做有限差分。
            lin_vel_w=np.zeros(3),
            ang_vel_w=np.zeros(3),
        )
        self._stale_count = 0
        self.last_tag_id = best.tag_id
        self.last_decision_margin = float(best.decision_margin)
        return True

    def get_pose(self) -> ObjectPose:
        if self._pose is None:
            raise RuntimeError(
                "AprilTagObjectSource 还没检测到任何已知 tag——检查 tag 是否在相机视野内、"
                "tag_id 是否在 tag_offsets 里配置过、tag_size_m 是否和实际打印/贴图尺寸一致。"
            )
        if self._stale_count > self.max_stale_frames:
            raise RuntimeError(
                f"AprilTag 连续 {self._stale_count} 次 update() 没检测到已知 tag，超过 "
                f"max_stale_frames={self.max_stale_frames}，可能是被遮挡太久或者已经不在"
                "视野里——不能再零阶保持下去，上层需要决定怎么处理（比如让机器人暂停动作"
                "等 tag 重新出现，而不是继续用过期的位姿控制）。"
            )
        return self._pose


def _rgb_to_gray(rgb: np.ndarray) -> np.ndarray:
    """AprilTag 检测只需要灰度图，标准 ITU-R BT.601 加权（和 OpenCV cvtColor 一致）。"""
    return (rgb[..., 0] * 0.299 + rgb[..., 1] * 0.587 + rgb[..., 2] * 0.114).astype(np.uint8)
