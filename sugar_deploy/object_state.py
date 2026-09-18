"""物体（箱子/瓶子/椅子）6D 位姿的获取接口。

SUGAR 论文本身的真机实验里，物体状态就是靠外部 MoCap 拿的，不是机器人自己拿摄像头
感知的——这个仓库第一版沿用同样的假设：物体状态是一个可插拔的外部输入，不在这里解决
"机器人自己看见物体在哪"这个感知问题。

``MujocoGroundTruthSource`` 是 sim2sim 自验证用的：直接读 MuJoCo 仿真里物体 body 的
真实状态，效果上等价于 SUGAR 在 IsaacSim 里训练/推理时的做法（用仿真给的 ground truth
物体状态，不是重建出来的）。真机部署时把它换成 ``MocapObjectSource``（或者你自己的
感知方案）即可，上层 observation.py / sim2sim.py 不需要跟着改。
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

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
