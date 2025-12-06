MAX_SPEED = 7.0
MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 12

init_viewpoint_coord = [0, -2.05, 2.6]
init_viewpoint_rotation = [-0.37570441069953675, 0.3756483724608402, 0.8471921246378744, 1.748990849844424]


# IK chain configuration
BASE_ELEMENTS = [
    "base_link",
    "base_link_Torso_joint",
    "Torso",
    "torso_lift_joint",
    "torso_lift_link",
    "torso_lift_link_TIAGo front arm_joint",
    "TIAGo front arm",
]

LAST_LINK_VECTOR = [0.004, 0.0, -0.1741]

ARM_JOINT_NAMES = [
    "torso_lift_joint",  # can be disabled later if needed
    "arm_1_joint",
    "arm_2_joint",
    "arm_3_joint",
    "arm_4_joint",
    "arm_5_joint",
    "arm_6_joint",
    "arm_7_joint",
]


# Motor order and initial targets
JOINT_NAMES = [
    "head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
    "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint",
    "arm_6_joint", "arm_7_joint", "wheel_left_joint", "wheel_right_joint"
]

JOINT_TARGET_POS = [
    0.24,    # head_2_joint
    -0.67,   # head_1_joint
    0.09,    # torso_lift_joint
    1.0,    # arm_1_joint
    1.0,     # arm_2_joint
    0.0,     # arm_3_joint
    1.0,     # arm_4_joint
    1.5,     # arm_5_joint
    -1.39,   # arm_6_joint
    1.41,    # arm_7_joint
    float("inf"),  # wheel_left_joint
    float("inf"),  # wheel_right_joint
]

JOINT_RESCUE_POS_2 = [
    0.24,    # head_2_joint
    -0.67,   # head_1_joint
    0.09,    # torso_lift_joint
    1.0,    # arm_1_joint
    1.0,     # arm_2_joint
    -1.0,     # arm_3_joint
    1.0,     # arm_4_joint
    1.5,     # arm_5_joint
    -1.39,   # arm_6_joint
    1.41,    # arm_7_joint
    float("inf"),  # wheel_left_joint
    float("inf"),  # wheel_right_joint
]

JOINT_RESCUE_POS_3 = [
    0.24,    # head_2_joint
    -0.67,   # head_1_joint
    0.09,    # torso_lift_joint
    1.0,    # arm_1_joint
    1.0,     # arm_2_joint
    1.0,     # arm_3_joint
    2.0,     # arm_4_joint
    1.5,     # arm_5_joint
    -1.39,   # arm_6_joint
    1.41,    # arm_7_joint
    float("inf"),  # wheel_left_joint
    float("inf"),  # wheel_right_joint
]

ARM_SENSOR_NAMES = [
    "arm_1_joint",
    "arm_2_joint",
    "arm_3_joint",
    "arm_4_joint",
    "arm_5_joint",
    "arm_6_joint",
    "arm_7_joint",
    "torso_lift_joint",
]