"""标定 ``carrybox_scene_apriltag.xml`` 里每个 AprilTag 相对箱子几何中心的静态位姿偏移。

原理：把机器人摆到几个能看到目标 tag 的位置，渲染 chest_cam、跑检测拿到"tag 在相机系下
的位姿"，再用 MuJoCo 给的箱子真值（``MujocoGroundTruthSource``）反解出"tag 局部系下，
箱子中心在哪、朝向差多少"——这个偏移只跟"tag 贴在箱子的什么位置、什么朝向"有关，跟机器人
站在哪、相机拍到的是哪一帧都无关，所以理论上从任意一个能看清 tag 的角度算出来的结果应该
是一样的（细微差异来自渲染/检测噪声）。

真机上做同一件事的等价操作是：把箱子摆在动捕/其它已知真值系统能给出精确位姿的地方，用
真实相机拍真实贴好的 tag，跑同一套解算逻辑反推偏移——原理完全一样，只是"真值从哪来"不同
（这里是 MuJoCo 仿真给的，真机是动捕/精密量具给的）。

用法（不需要跑策略，只是几何标定，跑得很快）：
    python scripts/calibrate_apriltag_offsets.py
输出可以直接粘贴进 sugar_deploy/apriltag_sim_calibration.py 的 TAG_OFFSETS。
"""

from __future__ import annotations

import mujoco
import numpy as np
from pupil_apriltags import Detector

from sugar_deploy.apriltag_sim_calibration import TAG_SIZE_M
from sugar_deploy.camera_source import MujocoCameraSource
from sugar_deploy.sim2sim import DEFAULT_SCENE

SCENE_PATH = DEFAULT_SCENE.parent / "carrybox_scene_apriltag.xml"

# 每个 tag 挑一个"确认能看到它"的机器人姿态（pelvis 世界位置 + 四元数），
# 都是试出来的，不是算出来的——真机标定同理，找一个能看清 tag 的角度就行，不用精确摆位。
CALIBRATION_POSES: dict[int, tuple[list[float], list[float]]] = {
    0: ([0.0, 0.0, 0.76], [1.0, 0.0, 0.0, 0.0]),        # 正对箱子前面
    1: ([1.0, 0.0, 0.76], [1.0, 0.0, 0.0, 0.0]),        # 靠近后能看到顶面
    2: ([1.5, 1.0, 0.76], [0.707, 0.0, 0.0, -0.707]),   # 侧面站位看 +y 面
}


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    pelvis_adr = model.joint("floating_base_joint").qposadr[0]
    box_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "object")

    camera_source = MujocoCameraSource(model, data, "chest_cam")
    detector = Detector(families="tag36h11", nthreads=2)

    for tag_id, (pos, quat) in CALIBRATION_POSES.items():
        data.qpos[pelvis_adr : pelvis_adr + 3] = pos
        data.qpos[pelvis_adr + 3 : pelvis_adr + 7] = quat
        mujoco.mj_forward(model, data)

        box_pos_true = data.xpos[box_id].copy()
        box_rot_true = data.xmat[box_id].reshape(3, 3).copy()
        cam_pos_w, cam_rot_w = camera_source.get_camera_pose_w()

        frame = camera_source.get_frame()
        gray = (frame.rgb[..., 0] * 0.299 + frame.rgb[..., 1] * 0.587 + frame.rgb[..., 2] * 0.114).astype(
            np.uint8
        )
        intr = frame.intrinsics
        results = detector.detect(
            gray, estimate_tag_pose=True, camera_params=(intr.fx, intr.fy, intr.cx, intr.cy), tag_size=TAG_SIZE_M
        )
        matches = [r for r in results if r.tag_id == tag_id]
        if not matches:
            print(f"tag {tag_id}: 这个姿态下没检测到，换个 CALIBRATION_POSES 里的位置试试")
            continue
        best = matches[0]

        tag_rot_w = cam_rot_w @ np.asarray(best.pose_R)
        tag_pos_w = cam_rot_w @ np.asarray(best.pose_t).ravel() + cam_pos_w

        pos_offset = tag_rot_w.T @ (box_pos_true - tag_pos_w)
        rot_offset = tag_rot_w.T @ box_rot_true
        # 检测噪声会让 rot_offset 不是完美正交矩阵，SVD 重新正交化一下
        u, _, vt = np.linalg.svd(rot_offset)
        rot_offset = u @ vt

        print(f"tag {tag_id} (decision_margin={best.decision_margin:.1f}):")
        print("  pos_offset =", np.array2string(pos_offset, separator=", ", precision=4))
        print("  rot_offset =", np.array2string(rot_offset, separator=", ", precision=4))


if __name__ == "__main__":
    main()
