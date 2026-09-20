"""RGB 相机帧的获取接口：sim2sim 用 MuJoCo 离屏渲染，真机用 RealSense。

设计成和 ``ObjectStateSource``（见 ``object_state.py``）同样的可插拔模式——
``AprilTagObjectSource`` 只依赖 ``CameraSource.get_frame()`` 这一个方法，不关心
帧到底是渲染出来的还是真相机拍的。这样 AprilTag 检测/位姿解算那部分代码可以先用
``MujocoCameraSource`` 在纯仿真里跑通、核对精度，真机部署时只需要把
``MujocoCameraSource`` 换成 ``RealSenseCameraSource``，上层不用改一行。
"""

from __future__ import annotations

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass
class CameraIntrinsics:
    """针孔相机内参，单位像素（fx/fy/cx/cy）。"""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


@dataclass
class CameraFrame:
    rgb: np.ndarray  # (H, W, 3) uint8
    intrinsics: CameraIntrinsics
    timestamp: float


class CameraSource(ABC):
    @abstractmethod
    def get_frame(self) -> CameraFrame:
        raise NotImplementedError


class MujocoCameraSource(CameraSource):
    """从 MuJoCo 场景里一个具名 ``<camera>`` 元素离屏渲染 RGB 帧。

    内参从 MuJoCo 相机的 ``fovy``（垂直视场角）反算——MuJoCo 只存视场角，不存像素焦距，
    按标准针孔模型 ``fy = height / (2 * tan(fovy/2))`` 换算，假设方形像素（``fx = fy``），
    这和大多数 RealSense RGB 模式的实际内参在数量级上是一致的，但不是真实标定值——
    仅用于 sim 内验证检测代码本身对不对，不代表真机精度。
    """

    def __init__(self, model, data, camera_name: str, width: int = 640, height: int = 480):
        import mujoco

        self.model = model
        self.data = data
        self.camera_name = camera_name
        self.width = width
        self.height = height

        self.camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        if self.camera_id < 0:
            raise ValueError(f"MuJoCo 模型里找不到相机 '{camera_name}'")

        fovy_deg = float(model.cam_fovy[self.camera_id])
        fy = height / 2.0 / np.tan(np.deg2rad(fovy_deg) / 2.0)
        fx = fy
        self.intrinsics = CameraIntrinsics(
            fx=fx, fy=fy, cx=width / 2.0, cy=height / 2.0, width=width, height=height
        )

        self.renderer = mujoco.Renderer(model, height=height, width=width)

    def get_frame(self) -> CameraFrame:
        self.renderer.update_scene(self.data, camera=self.camera_name)
        rgb = self.renderer.render()
        return CameraFrame(rgb=rgb, intrinsics=self.intrinsics, timestamp=time.time())

    # MuJoCo 的相机外参（cam_xmat）是图形学/OpenGL 约定：局部 -Z 朝画面里看，+Y 是图像
    # 朝上；但 pupil_apriltags（跟 OpenCV 一致）算出来的 tag 位姿是视觉/OpenCV 约定：
    # 局部 +Z 朝画面里看，+Y 是图像朝下。两个"相机局部系"其实不是同一个系，直接拿
    # cam_xmat 去合成 AprilTag 解出来的 pose_t/pose_R 会得到完全错误的结果（实测过：
    # 3.5mm 误差 vs 2.6m 误差，符号和数量级都不对，不是精度问题，是系选错了）。
    # 这个矩阵把"OpenCV 相机局部系下的向量"转成"MuJoCo 相机局部系下的同一个向量"：
    # x 不变，y/z 取反。
    _CV_TO_MJ = np.diag([1.0, -1.0, -1.0])

    def get_camera_pose_w(self) -> tuple[np.ndarray, np.ndarray]:
        """相机当前在（和 ``robot.anchor_pos_w`` 同一个参考系下的）世界位姿，**旋转部分
        已经转成 OpenCV 相机系约定**——``AprilTagObjectSource`` 直接拿这个位姿去合成
        ``pupil_apriltags`` 输出的 tag 位姿，两边约定必须一致，转换只需要在这一处做一次，
        不需要 ``AprilTagObjectSource`` 关心相机帧到底是渲染出来的还是真相机拍的。

        MuJoCo 每步 forward/step 之后会自动算好 ``cam_xpos``/``cam_xmat``（相机跟随它
        挂载的 body 一起动），这里直接读，不需要额外计算——真机上没有这个免费的量，
        真机部署时这一步要么用固定外参（相机刚性挂载、不跟踪机器人世界位置，见部署文档
        的"参考系怎么选"一节，这种情况下外参本来就该按 OpenCV 约定标定/给出，不需要
        额外转换），要么用机器人自己的状态估计。
        """
        pos_w = self.data.cam_xpos[self.camera_id].copy()
        rot_w_mj = self.data.cam_xmat[self.camera_id].reshape(3, 3).copy()
        rot_w_cv = rot_w_mj @ self._CV_TO_MJ
        return pos_w, rot_w_cv


class RealSenseCameraSource(CameraSource):
    """真机用：从 Intel RealSense（D435/D435i 等）读 RGB 帧 + SDK 自带的真实内参。

    **还没在真实硬件上跑过**——调用方式是照 ``pyrealsense2`` 官方 API 写的，第一次接硬件
    时先单独跑一下 ``get_frame()`` 打印 ``intrinsics``，核对 fx/fy 是否在合理范围
    （640x480 下一般 380~620 之间，具体看机型/固件），确认真的拿到相机自己的标定值，
    而不是某个默认/占位值，再往下接 ``AprilTagObjectSource``。
    """

    def __init__(self, width: int = 640, height: int = 480, fps: int = 30):
        import pyrealsense2 as rs

        self._rs = rs
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
        profile = self.pipeline.start(config)

        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_stream.get_intrinsics()
        self.intrinsics = CameraIntrinsics(
            fx=intr.fx, fy=intr.fy, cx=intr.ppx, cy=intr.ppy, width=width, height=height
        )

    def get_frame(self) -> CameraFrame:
        frames = self.pipeline.wait_for_frames()
        color = frames.get_color_frame()
        if not color:
            raise RuntimeError("RealSense 没有拿到彩色帧（wait_for_frames 超时或掉帧）")
        rgb = np.asanyarray(color.get_data())
        return CameraFrame(rgb=rgb, intrinsics=self.intrinsics, timestamp=time.time())

    def close(self) -> None:
        self.pipeline.stop()
