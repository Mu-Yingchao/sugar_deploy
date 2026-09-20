"""``assets/g1/carrybox_scene_apriltag.xml`` 这个 sim 验证场景专用的 AprilTag 标定常量。

**这些数字是拿这个 sim 场景自己的 ground truth 反解出来的（不是手算的，方法见
scripts/calibrate_apriltag_offsets.py），只对这一个 MuJoCo 场景成立**——真机部署换了真
箱子、真相机之后，这几个数字必须用同样的方法针对真实硬件重新标定一遍，不能照抄。
详细原理和真机标定步骤见 sugar_deploy 仓库的 AprilTag 部署文档。
"""

from __future__ import annotations

import numpy as np

from sugar_deploy.object_state import TagObjectOffset

# carrybox_scene_apriltag.xml 里贴图画布 480px，tag 自己的黑边方框只占中间 256px
# （480 外圈 80px 是额外加的白色 quiet zone，tag 自身 10x10 源图最外 1px 也是白边，
# 8/10*320=256）；geom 物理全宽 0.15m -> 黑边方框物理宽度 = 0.15 * 256/480。
TAG_SIZE_M = 0.15 * 256 / 480

TAG_OFFSETS: list[TagObjectOffset] = [
    TagObjectOffset(
        tag_id=0,  # 贴在箱子前面（法线 -x，机器人从 x=0 走向箱子 x=1.5 时正对着看到）
        pos_offset=np.array([0.0052, 0.0026, 0.1971]),
        rot_offset=np.array([
            [0.0211, -0.0008, 0.9998],
            [0.0115, -0.9999, -0.0011],
            [0.9997, 0.0115, -0.0211],
        ]),
    ),
    TagObjectOffset(
        tag_id=1,  # 贴在顶面（法线 +z），机器人走近箱子低头时能看到
        pos_offset=np.array([-0.0007, 0.0003, 0.2691]),
        rot_offset=np.array([
            [1.0000, -0.0008, 0.0019],
            [-0.0008, -1.0000, -0.0011],
            [0.0019, 0.0011, -1.0000],
        ]),
    ),
    TagObjectOffset(
        tag_id=2,  # 贴在一侧（法线 +y），验证"任意可见面都能独立解出位姿"
        pos_offset=np.array([-0.0002, 0.0008, 0.2734]),
        rot_offset=np.array([
            [1.0000, 0.0000, 0.0001],
            [-0.0001, -0.0019, 1.0000],
            [0.0000, -1.0000, -0.0019],
        ]),
    ),
]
