"""SUGAR Tracker/Generator 的观测-动作契约。

这里的每一个数字都是从 tianshuwu/SUGAR 仓库的实际代码里核实出来的，不是抄论文猜的：
来源见各常量旁的注释（文件路径 + 行号），核实方法记录在
sugar_deploy 仓库 README 的"契约来源"一节。

**关节顺序**：SUGAR 的 29 个受控关节顺序 == G1 URDF
(`descriptions/robots/g1/g1_29dof_rev_1_0_with_rubber_hand.urdf`) 里
`type="revolute"` 关节的声明顺序，这个顺序同时也和 `unitree.py` 里定义好没被用上的
`joint_sdk_names`（面向真机 SDK 的顺序）逐一对上——也就是说仿真里的动作顺序和真机
SDK 的关节顺序是一致的，不需要做重映射。但这是静态代码分析推出来的，不是从跑起来的
env 里 print 出来的，第一次接真实 policy/真机之前务必用
`scripts/dump_joint_order.py` 之类的方式跑一次 `env.scene["robot"].joint_names`
交叉验证。
"""

from __future__ import annotations

from dataclasses import dataclass, field


# 29 个受控关节，顺序来源：
# SUGAR/descriptions/robots/g1/g1_29dof_rev_1_0_with_rubber_hand.urdf
# 里 <joint type="revolute"> 的声明顺序（已核对：SONIC_MimicLite/gear_sonic_deploy/
# g1/g1_29dof.xml 的关节顺序和这个逐一相同，可以直接拿来当 MuJoCo 模型用，
# 不用像 g1_29dof_with_hand.xml 那样处理手指关节交错的问题）。
JOINT_NAMES: tuple[str, ...] = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
)
NUM_JOINTS = len(JOINT_NAMES)
assert NUM_JOINTS == 29


@dataclass(frozen=True)
class ActuatorGroup:
    """一组共享同一套 PD 参数的关节。数值来源：SUGAR source/sugar_rl/sugar_rl/assets/
    robots/unitree.py:112-249（ImplicitActuatorCfg），action_scale 按
    unitree.py:285-296 的公式 `0.25 * effort_limit_sim / stiffness` 反算。"""

    joint_names: tuple[str, ...]
    stiffness: float
    damping: float
    armature: float
    effort_limit: float

    @property
    def action_scale(self) -> float:
        return 0.25 * self.effort_limit / self.stiffness


ACTUATOR_GROUPS: tuple[ActuatorGroup, ...] = (
    ActuatorGroup(  # hip pitch/yaw, 膝盖同款电机型号但下面单独一组（不同 effort）
        ("left_hip_pitch_joint", "right_hip_pitch_joint",
         "left_hip_yaw_joint", "right_hip_yaw_joint"),
        stiffness=40.179, damping=2.558, armature=0.01018, effort_limit=88,
    ),
    ActuatorGroup(
        ("left_hip_roll_joint", "right_hip_roll_joint",
         "left_knee_joint", "right_knee_joint"),
        stiffness=99.098, damping=6.309, armature=0.02510, effort_limit=139,
    ),
    ActuatorGroup(
        ("left_ankle_pitch_joint", "left_ankle_roll_joint",
         "right_ankle_pitch_joint", "right_ankle_roll_joint"),
        stiffness=28.501, damping=1.814, armature=0.00722, effort_limit=50,
    ),
    ActuatorGroup(
        ("waist_roll_joint", "waist_pitch_joint"),
        stiffness=28.501, damping=1.814, armature=0.00722, effort_limit=50,
    ),
    ActuatorGroup(
        ("waist_yaw_joint",),
        stiffness=40.179, damping=2.558, armature=0.01018, effort_limit=88,
    ),
    ActuatorGroup(
        ("left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
         "left_elbow_joint", "left_wrist_roll_joint",
         "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
         "right_elbow_joint", "right_wrist_roll_joint"),
        stiffness=14.251, damping=0.907, armature=0.00361, effort_limit=25,
    ),
    ActuatorGroup(
        ("left_wrist_pitch_joint", "left_wrist_yaw_joint",
         "right_wrist_pitch_joint", "right_wrist_yaw_joint"),
        stiffness=16.778, damping=1.068, armature=0.00425, effort_limit=5,
    ),
)


def _build_per_joint_arrays() -> dict[str, list[float]]:
    stiffness = [0.0] * NUM_JOINTS
    damping = [0.0] * NUM_JOINTS
    armature = [0.0] * NUM_JOINTS
    effort_limit = [0.0] * NUM_JOINTS
    action_scale = [0.0] * NUM_JOINTS
    index = {name: i for i, name in enumerate(JOINT_NAMES)}
    seen = set()
    for group in ACTUATOR_GROUPS:
        for name in group.joint_names:
            i = index[name]
            stiffness[i] = group.stiffness
            damping[i] = group.damping
            armature[i] = group.armature
            effort_limit[i] = group.effort_limit
            action_scale[i] = group.action_scale
            seen.add(name)
    missing = set(JOINT_NAMES) - seen
    if missing:
        raise RuntimeError(f"ACTUATOR_GROUPS 没覆盖到这些关节: {missing}")
    return {
        "stiffness": stiffness, "damping": damping, "armature": armature,
        "effort_limit": effort_limit, "action_scale": action_scale,
    }


_PER_JOINT = _build_per_joint_arrays()
JOINT_STIFFNESS: tuple[float, ...] = tuple(_PER_JOINT["stiffness"])
JOINT_DAMPING: tuple[float, ...] = tuple(_PER_JOINT["damping"])
JOINT_ARMATURE: tuple[float, ...] = tuple(_PER_JOINT["armature"])
JOINT_EFFORT_LIMIT: tuple[float, ...] = tuple(_PER_JOINT["effort_limit"])
JOINT_ACTION_SCALE: tuple[float, ...] = tuple(_PER_JOINT["action_scale"])

# 默认关节位置（弧度）。来源：unitree.py:118-127，其余关节默认 0.0。
_DEFAULT_OVERRIDES: dict[str, float] = {
    "left_hip_pitch_joint": -0.312, "right_hip_pitch_joint": -0.312,
    "left_knee_joint": 0.669, "right_knee_joint": 0.669,
    "left_ankle_pitch_joint": -0.363, "right_ankle_pitch_joint": -0.363,
    "left_elbow_joint": 0.6, "right_elbow_joint": 0.6,
    "left_shoulder_roll_joint": 0.2, "left_shoulder_pitch_joint": 0.2,
    "right_shoulder_roll_joint": -0.2, "right_shoulder_pitch_joint": 0.2,
}
DEFAULT_JOINT_POS: tuple[float, ...] = tuple(
    _DEFAULT_OVERRIDES.get(name, 0.0) for name in JOINT_NAMES
)
DEFAULT_ROOT_HEIGHT = 0.76  # unitree.py init_state.pos，仅作 sim2sim 初始化参考


# --- 控制频率。来源：source/sugar_rl/.../carry_box_inference_env_cfg.py:77-86 ---
PHYSICS_DT = 0.005          # sim.dt，200 Hz
CONTROL_DECIMATION = 4      # 每 4 个物理步应用一次动作
CONTROL_DT = PHYSICS_DT * CONTROL_DECIMATION  # 0.02s，50 Hz —— Tracker 频率

# --- Generator 调用节奏。来源：base_inference_env_cfg.py:156, commands.py:1290-1301 ---
GENERATOR_CALL_INTERVAL = 20  # 每 20 个 tracker 控制步调用一次 Generator（0.4s @ 50Hz）

# --- Tracker 观测：历史长度与 concat 顺序。
# 来源：base_inference_env_cfg.py 的 TrackerCfg（policy 组），逐项核实过。
# 顺序非常重要——必须严格按这个顺序拼观测向量，rsl_rl 是按 concat 顺序吃维度的。
TRACKER_OBS_HISTORY_LEN = 5
TRACKER_OBS_LAYOUT: tuple[tuple[str, int, int], ...] = (
    # (term 名字, 单帧维度, 历史长度；历史长度=0 表示不做历史 stacking)
    ("generated_command", 36, 0),
    ("base_ang_vel", 3, TRACKER_OBS_HISTORY_LEN),
    ("joint_pos_rel", NUM_JOINTS, TRACKER_OBS_HISTORY_LEN),
    ("joint_vel_rel", NUM_JOINTS, TRACKER_OBS_HISTORY_LEN),
    ("last_action", NUM_JOINTS, TRACKER_OBS_HISTORY_LEN),
    ("project_gravity", 3, TRACKER_OBS_HISTORY_LEN),
    ("obj_pos_b", 3, 0),
    ("obj_ori_b", 6, 0),
)
TRACKER_OBS_DIM = sum(dim * max(hist, 1) for _, dim, hist in TRACKER_OBS_LAYOUT)
assert TRACKER_OBS_DIM == 510, f"算出来是 {TRACKER_OBS_DIM}，和 checkpoint 里 actor 第一层 (512,510) 对不上"

TRACKER_ACTION_DIM = NUM_JOINTS
TRACKER_ACTOR_HIDDEN_DIMS = (512, 256, 128)  # 从 tracker.pt 的 actor.{0,2,4,6}.weight shape 反推
TRACKER_ACTIVATION = "elu"

# anchor body：观测里 obj_pos_b / obj_ori_b / project_gravity 用的参考系
# 注意不是 pelvis！commands.py:1658-1663 明确用的是 torso_link。
ANCHOR_BODY_NAME = "torso_link"
# base（pelvis）单独用于 base_ang_vel（commands.py:1666-1667 走的是 root_quat_w=pelvis）
BASE_BODY_NAME = "pelvis"

# --- Command c_t：36 维，来源 sugar_il/wrapper/sugar_il_wrapper.py:250-262 ---
COMMAND_DIM = 36
COMMAND_LAYOUT: tuple[tuple[str, int], ...] = (
    ("ref_joint_pos", NUM_JOINTS),          # 29, 目标关节角 (rad)
    ("ref_anchor_lin_vel_b", 3),            # anchor(torso_link) 局部系线速度
    ("ref_anchor_ang_vel_b", 3),            # anchor(torso_link) 局部系角速度
    ("contact_label", 1),                   # 原始网络输出，未做阈值化
)
assert sum(d for _, d in COMMAND_LAYOUT) == COMMAND_DIM


@dataclass(frozen=True)
class TaskContract:
    """每个任务不同的部分：物体资产、Generator 的 use_target 开关等。

    use_target 的值**必须以 checkpoint 自带的 cfg 为准**，这里的默认值只是
    train.sh 里的规律（CarryBox/PickBox/PushBox=True，其余=False），用于在没有
    checkpoint 可读时给个合理默认；真正加载 checkpoint 时 policy.py 会优先读
    checkpoint 里 payload['cfg'] 的实际值，不信这个默认。
    """

    name: str
    object_body_name: str  # 物体在 MJCF 里对应的 body 名字
    use_target_default: bool


TASK_CONTRACTS: dict[str, TaskContract] = {
    "CarryBox": TaskContract("CarryBox", "object", use_target_default=True),
    "PushBox": TaskContract("PushBox", "object", use_target_default=True),
    # 注意：官方 SUGAR/train.sh 的 use_target case 语句是
    #   "CarryBox" | "PickBox" | "PushBox")   USE_TARGET="True"
    #   "PickBottle" | "StandBottle" | "SitChair")  USE_TARGET="False"
    # KickBox 两边都没覆盖到（"PickBox" 是 CarryBox 训练 checkpoint 里查到的内部旧代号
    # 'pick_box'，不是 KickBox 的别名），train.sh 跑 KickBox 时 USE_TARGET 会是空字符串，
    # 传给 hydra 大概率直接报错——这是上游一个真实的 case 覆盖缺口，不是这里瞎猜的。
    # 下面这个 False 是按"踢箱子没有明确的目标落点，不像 Carry/Push 那样"猜的，
    # 不是确认过的事实。等服务器上 KickBox 真的跑到 Generator 阶段，
    # 以 checkpoint 自带 cfg 里的实际值为准（policy.py 就是这么设计的）。
    "KickBox": TaskContract("KickBox", "object", use_target_default=False),
    "PickBottle": TaskContract("PickBottle", "object", use_target_default=False),
    "StandBottle": TaskContract("StandBottle", "object", use_target_default=False),
    "SitChair": TaskContract("SitChair", "object", use_target_default=False),
}
