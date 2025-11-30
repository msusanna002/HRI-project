from controller import Supervisor, Keyboard, Camera, RangeFinder
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import math
import os
import numpy as np



# Workaround for ikpy on newer NumPy versions
if not hasattr(np, "float"):
    np.float = float

MAX_SPEED = 7.0

MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 12

robot_view_target = None
vp_pos_field = None
vp_rot_field = None
square_red_pos = None
puzzle_outline = None
para_1_red = None

init_viewpoint = [0, -2.05, 2.6]
init_rotation = [-0.37570441069953675, 0.3756483724608402, 0.8471921246378744, 1.748990849844424]

# ---------------------------------------------------------------------
# Helper: current chain joint vector from Webots motors
# ---------------------------------------------------------------------
def distance_to_target(target_xyz):
    pos, _ = get_end_effector_pose_from_sensors()
    dx = pos[0] - target_xyz[0]
    dy = pos[1] - target_xyz[1]
    dz = pos[2] - target_xyz[2]
    return math.sqrt(dx*dx + dy*dy + dz*dz)


def get_current_chain_positions():
    """
    Build a list of joint positions for all links in my_chain.
    Disabled links get 0. Active links map to the Webots motors.
    Uses motor target positions only, no sensors.
    """
    current = [0.0] * len(my_chain.links)

    for i, link in enumerate(my_chain.links):
        if not my_chain.active_links_mask[i]:
            continue

        link_name = link.name

        if link_name in names:
            motor_index = names.index(link_name)
            motor = robot_parts[motor_index]
            # Use last commanded position; good enough for IK initial guess
            current[i] = motor.getTargetPosition()
        else:
            current[i] = 0.0

    return current

def get_end_effector_pose():
    # 1) get current joint configuration
    joints = get_joint_positions_from_sensors()

    # 2) run forward kinematics
    T = my_chain.forward_kinematics(joints)  # 4x4 homogeneous matrix

    # 3) extract position
    x = T[0, 3]
    y = T[1, 3]
    z = T[2, 3]

    # 4) extract orientation (rotation matrix)
    R = T[:3, :3]

    return (x, y, z), R

def get_joint_positions_from_sensors():
    joints = [0.0] * len(my_chain.links)
    for i, link in enumerate(my_chain.links):
        if not my_chain.active_links_mask[i]:
            continue

        link_name = link.name
        if link_name in arm_sensors:
            joints[i] = arm_sensors[link_name].getValue()
        else:
            joints[i] = 0.0

    return joints


def get_end_effector_pose_from_sensors():
    joints = get_joint_positions_from_sensors()
    T = my_chain.forward_kinematics(joints)
    pos = (T[0, 3], T[1, 3], T[2, 3])
    R = T[:3, :3]
    return pos, R

# ---------------------------------------------------------------------
# IKPY-based arm motion
# ---------------------------------------------------------------------
def move_arm_to_position(target_xyz, orientation=None, orientation_mode=None,
                         error_tolerance=0.005):
    """
    Use IKPY to move the TIAGo arm so that the end-effector reaches `target_xyz`
    in robot base coordinates.

    Parameters
    ----------
    target_xyz : list/tuple of length 3
        [x, y, z] in the same base frame as the IK chain (usually base_link).
    orientation : list/tuple of length 3 or 4, optional
        Optional target orientation vector, passed to IKPY.
        For example [0, 0, 1] to keep the tool roughly vertical.
    orientation_mode : str, optional
        IKPY orientation mode, e.g. "Y" or "all". None means no orientation.
    error_tolerance : float
        If the distance error after solution is greater than this, you can
        decide to re-run IK or log a warning.
    """
    # 1) Get current joint configuration as initial guess
    initial_position = get_joint_positions_from_sensors()

    # 2) Run IK
    if orientation is not None and orientation_mode is not None:
        ik_results = my_chain.inverse_kinematics(
            target_xyz,
            initial_position=initial_position,
            target_orientation=orientation,
            orientation_mode=orientation_mode,
        )
    else:
        ik_results = my_chain.inverse_kinematics(
            target_xyz,
            initial_position=initial_position,
        )

    # 3) Optional: compute position error in task space (rough check)
    #    (Using the requested target and forward kinematics on ik_results)
    fk_result = my_chain.forward_kinematics(ik_results)
    end_eff_pos = fk_result[:3, 3]
    err = math.sqrt(
        (end_eff_pos[0] - target_xyz[0]) ** 2 +
        (end_eff_pos[1] - target_xyz[1]) ** 2 +
        (end_eff_pos[2] - target_xyz[2]) ** 2
    )

    if err > error_tolerance:
        print(f"[IK] Warning: end-effector error {err:.4f} > tolerance {error_tolerance:.4f}")

    # 4) Apply joint angles to Webots motors
    for i, link in enumerate(my_chain.links):
        if not my_chain.active_links_mask[i]:
            continue

        link_name = link.name

        # Only send commands to joints that correspond to Webots motors
        if link_name in names:
            motor_index = names.index(link_name)
            motor = robot_parts[motor_index]
            target_angle = ik_results[i]
            motor.setPosition(target_angle)
            # You can log if needed:
            # print(f"Setting {link_name} to {target_angle:.3f}")

    return err

def move_arm_to_position_blocking(target_xyz,
                                  orientation=None,
                                  orientation_mode=None,
                                  pos_tolerance=0.005,
                                  max_steps=1000):
    # Command the move
    err_ik = move_arm_to_position(target_xyz, orientation, orientation_mode,
                         error_tolerance=pos_tolerance)
    
    if err_ik > pos_tolerance:
        print("[IK] Target likely unreachable, aborting blocking wait")
        return

    # Wait until you are close enough or time out
    steps = 0
    while robot.step(time_step) != -1:
        err = distance_to_target(target_xyz)
        if err < pos_tolerance:
            # Reached the target
            break
        steps += 1
        if steps > max_steps:
            print("[IK] Warning: timeout waiting for target", target_xyz)
            break

def motor_blocking(target_angle):
    sensor = arm_sensors["arm_4_joint"]   # not "..._sensor"
    motor  = robot_parts[names.index("arm_4_joint")]

    while robot.step(time_step) != -1:
        curr_rotation = sensor.getValue()
        err = abs(curr_rotation - target_angle)
        if err < 0.01:
            break
    


def sanity_test_ik():
    # Choose some joint vector inside bounds
    q = get_current_chain_positions()
    print("Sanity test: q =", q)

    # Forward kinematics
    T = my_chain.forward_kinematics(q)
    target = T[:3, 3]

    # Inverse kinematics back to the same position, no orientation constraint
    q_ik = my_chain.inverse_kinematics(target, initial_position=q)

    T_ik = my_chain.forward_kinematics(q_ik)
    pos_ik = T_ik[:3, 3]

    err = math.sqrt(
        (pos_ik[0] - target[0])**2 +
        (pos_ik[1] - target[1])**2 +
        (pos_ik[2] - target[2])**2
    )

    print("Target from FK:", target)
    print("Back from IK:", pos_ik)
    print("Position error in sanity test:", err)

def arm_movement(targetObject, lift, tableHeight, robotNode):
    targetPosWC = None
    if targetObject is not None:
        targetPosWC = targetObject.getField("translation").getSFVec3f() 
    else: 
        print (targetObject, "is not valid")
    targetPosRC = world_to_robot(targetPosWC,robotNode)


    target_x = targetPosRC[0]
    target_y = targetPosRC[1]
    target_z= targetPosRC[2]

    pos, _ = get_end_effector_pose()
    current_x, current_y, current_z = pos
    safe_z = tableHeight + lift

    lifted_target = [target_x, target_y, max(target_z, safe_z)]

    lift_rotation = 0.8
    robot_parts[6].setPosition(lift_rotation)
    motor_blocking(lift_rotation)

    move_arm_to_position_blocking(
        lifted_target,
        orientation=[0, 0, 1],
        orientation_mode="Z",
        pos_tolerance=0.2
    )

    move_arm_to_position_blocking(
    targetPosRC,
    orientation=[0, 0, 1],
    orientation_mode="Z",
    )

# ---------------------------------------------------------------------
# Helper: make head look at the Viewpoint
# ---------------------------------------------------------------------
def normalize_angle(a):
    # keep angle in [-pi, pi]
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def make_head_look_at_target(robot_parts, robot_node, target_pos):
    # Fields on robot and viewpoint
    base_translation_field = robot_node.getField("translation")
    base_rotation_field    = robot_node.getField("rotation")
    

    if base_translation_field is None or base_rotation_field is None or target_pos is None:
        print("Could not get translation/rotation/position fields.")
        return

    # Robot base pose
    base_pos = base_translation_field.getSFVec3f()   # [x, y, z]
    base_rot = base_rotation_field.getSFRotation()   # [ax, ay, az, angle]
          # [x, y, z]

    head_pos = [
    base_pos[0],
    base_pos[1],
    base_pos[2] + 0.8
]

    # --- Extract body yaw from axis-angle (assuming planar, axis ~ [0,0,±1]) ---
    ax, ay, az, angle = base_rot
    # default if axis is weird: assume yaw = 0
    if abs(az) > 1e-3:
        body_yaw = angle * (1.0 if az >= 0.0 else -1.0)
    else:
        body_yaw = 0.0

 
    # Vector from base to camera (world frame)
    dx = target_pos[0] - head_pos[0]
    dy = -target_pos[1] - head_pos[1]
    dz = target_pos[2] - head_pos[2]

    # Guard against degenerate case 
    if dx == 0 and dy == 0:
        print("Head target direction is undefined (camera exactly above/below in XY).")
        return

    # --- World yaw to camera and relative yaw for the head ---
    world_yaw = math.atan2(dy, dx)           # yaw of vector base→camera in world frame
    rel_yaw   = normalize_angle(world_yaw + body_yaw)  # yaw relative to body orientation


    # vertical angle
    dist_xy = math.sqrt(dx * dx + dy * dy)
    pitch   = math.atan2(dz, dist_xy)        # positive if camera is above

    # Desired joint angles (head joints)
    # pan = relative yaw, tilt = opposite sign of pitch (depends on joint convention)
    desired_pan  = rel_yaw
    desired_tilt = -pitch

    head_tilt = robot_parts[0]  # "head_2_joint"
    head_pan  = robot_parts[1]  # "head_1_joint"

    # --- Get joint limits from Webots ---
    pan_min  = head_pan.getMinPosition()
    pan_max  = head_pan.getMaxPosition()
    tilt_min = head_tilt.getMinPosition()
    tilt_max = head_tilt.getMaxPosition()

    BIG = 1e6
    if abs(pan_min) > BIG:  pan_min = -BIG
    if abs(pan_max) > BIG:  pan_max = BIG
    if abs(tilt_min) > BIG: tilt_min = -BIG
    if abs(tilt_max) > BIG: tilt_max = BIG

    # --- Clamp to limits ---
    clamped_yaw  = max(pan_min,  min(pan_max,  desired_pan))
    clamped_tilt = max(tilt_min, min(tilt_max, desired_tilt))

    if abs(clamped_yaw - desired_pan) > 1e-3 or abs(clamped_tilt - desired_tilt) > 1e-3:
        print(
            "Head target out of range. "
            f"Requested pan={desired_pan:.2f}, tilt={desired_tilt:.2f} "
            f"→ clamped to pan={clamped_yaw:.2f}, tilt={clamped_tilt:.2f}"
        )

    # Apply the (possibly clamped) positions
    head_pan.setPosition(-clamped_yaw)
    head_tilt.setPosition(-clamped_tilt)
    

# ---------------------------------------------------------------------
# Helper: world to robot coordinaet translation
# ---------------------------------------------------------------------

def axis_angle_to_rotation_matrix(axis, angle):
    """Convert Webots axis angle [ax, ay, az, angle] to a 3x3 rotation matrix."""
    ax, ay, az = axis
    norm = math.sqrt(ax*ax + ay*ay + az*az)
    if norm == 0:
        return np.eye(3)

    ux, uy, uz = ax / norm, ay / norm, az / norm
    c = math.cos(angle)
    s = math.sin(angle)
    one_c = 1.0 - c

    R = np.array([
        [c + ux*ux*one_c,     ux*uy*one_c - uz*s, ux*uz*one_c + uy*s],
        [uy*ux*one_c + uz*s,  c + uy*uy*one_c,    uy*uz*one_c - ux*s],
        [uz*ux*one_c - uy*s,  uz*uy*one_c + ux*s, c + uz*uz*one_c   ]
    ])
    return R


def world_to_robot(world_xyz, robot_node):
    """
    Convert a point from Webots world coordinates to the robot base_link frame.

    world_xyz: [x, y, z] in world coords
    returns: [x, y, z] in robot base coords
    """
    # Robot base pose in world frame
    base_translation_field = robot_node.getField("translation")
    base_rotation_field    = robot_node.getField("rotation")

    base_pos = np.array(base_translation_field.getSFVec3f())   # p_B^W
    ax, ay, az, angle = base_rotation_field.getSFRotation()    # axis angle

    # Rotation from base frame to world frame
    R_WB = axis_angle_to_rotation_matrix([ax, ay, az], angle)

    # Inverse rotation gives world to base
    R_BW = R_WB.T

    # Position of point in world frame
    p_W = np.array(world_xyz)

    # Relative to base origin in world frame
    p_rel_W = p_W - base_pos

    # Express that vector in base frame
    p_B = R_BW.dot(p_rel_W)

    return [float(p_B[0]), float(p_B[1]), float(p_B[2])]


# ---------------------------------------------------------------------
# Helper: handles keyboard input
# ---------------------------------------------------------------------
def check_keyboard(robot_parts, keyboard, robot_node, head_pan_sensor, head_tilt_sensor):
    global last_space_down, robot_view_target  
    global init_viewpoint, init_rotation, vp_pos_field, vp_rot_field        

    key = keyboard.getKey()
    speeds_left = 0.0
    speeds_right = 0.0

    if key == Keyboard.UP:
        speeds_left = MAX_SPEED
        speeds_right = MAX_SPEED

    elif key == Keyboard.DOWN:
        speeds_left = -MAX_SPEED
        speeds_right = -MAX_SPEED

    elif key == Keyboard.RIGHT:
        speeds_left = MAX_SPEED
        speeds_right = -MAX_SPEED

    elif key == Keyboard.LEFT:
        speeds_left = -MAX_SPEED
        speeds_right = MAX_SPEED

    elif key == ord('0'):
        vp_pos_field.setSFVec3f(init_viewpoint)
        vp_rot_field.setSFRotation(init_rotation)

    elif key == ord('1'):
        arm_movement(puzzle_outline, 0.3, 0.43, robot_node)
        
    elif key == ord('2'):
        arm_movement(square_red, 0.3, 0.43, robot_node) 

    elif key == ord('3'):
        arm_movement(para_1_red, 0.3, 0.43, robot_node)
        
        
    space_now = (key == ord(' '))
    if space_now and not last_space_down:
        print("SPACE pressed: making head look at camera")
        if robot_node is not None and viewpoint is not None:
            viewpoint_pos_field = viewpoint.getField("position")
            cam_pos  = viewpoint_pos_field.getSFVec3f() 
            robot_view_target = cam_pos
            make_head_look_at_target(robot_parts, robot_node, robot_view_target)
        else:
            print("Cannot look at camera: robot_node or viewpoint is None.")

    # update the state for the next timestep
    last_space_down = space_now

    robot_parts[MOTOR_LEFT].setVelocity(speeds_left)
    robot_parts[MOTOR_RIGHT].setVelocity(speeds_right)



# ---------------------------------------------------------------------
# Main program
# ---------------------------------------------------------------------
robot = Supervisor()
time_step = int(robot.getBasicTimeStep())

HERE = os.path.dirname(__file__)
URDF_PATH = os.path.join(HERE, "tiago_urdf.urdf")

# If the URDF doesn't exist yet, create it once
if not os.path.exists(URDF_PATH):
    print("URDF not found, creating:", URDF_PATH)
    with open(URDF_PATH, "w") as f:
        f.write(robot.getUrdf())

# Enable RGB + depth cameras (devices, not supervisor nodes)
rgb = robot.getDevice("Astra rgb")
rgb.enable(time_step)

depth = robot.getDevice("Astra depth")
depth.enable(time_step)

# Robot part names (same order as C code)


base_elements = [
    "base_link",
    "base_link_Torso_joint",
    "Torso",
    "torso_lift_joint",
    "torso_lift_link",
    "torso_lift_link_TIAGo front arm_joint",
    "TIAGo front arm",
]
last_link_vector = [0.004, 0.0, -0.1741]

my_chain = Chain.from_urdf_file(
    "tiago_urdf.urdf",
    base_elements=base_elements,
    last_link_vector=last_link_vector,
)

# print("IK chain links:")
# for i, link in enumerate(my_chain.links):
#     print(i, link.name, "bounds=", link.bounds)

arm_joint_names = {
    "torso_lift_joint",  # disable later if unstable
    "arm_1_joint",
    "arm_2_joint",
    "arm_3_joint",
    "arm_4_joint",
    "arm_5_joint",
    "arm_6_joint",
    "arm_7_joint",
}


# Deactivate every link that is not in arm_joint_names
for i, link in enumerate(my_chain.links):
    if link.name not in arm_joint_names:
        my_chain.active_links_mask[i] = False

# Disable torso_lift_joint
for i, link in enumerate(my_chain.links):
    if link.name == "torso_lift_joint":
        my_chain.active_links_mask[i] = False


names = [
    "head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
    "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint",
    "arm_6_joint", "arm_7_joint", "wheel_left_joint", "wheel_right_joint"
]

# Target positions for initialization
target_pos = [
    0.24, #head_2_joint
    -0.67, #head_1_joint
    0.09, #torso_lift_joint
    0.07, #arm_1_joint
    1.0 , #arm_2_joint
    0, #arm_3_joint
    1, #arm_4_joint
    1.5, #arm_5_joint
    -1.39, #arm_6_joint
    1.41, #arm_7_joint
    float('inf'), 
    float('inf')]

arm_sensor_names = [
    "arm_1_joint_sensor",
    "arm_2_joint_sensor",
    "arm_3_joint_sensor",
    "arm_4_joint_sensor",
    "arm_5_joint_sensor",
    "arm_6_joint_sensor",
    "arm_7_joint_sensor",
    "torso_lift_joint",
]

robot_parts = []
arm_sensors = {}

# Configure motors
for i in range(N_PARTS):
    motor = robot.getDevice(names[i])
    robot_parts.append(motor)
    motor.setVelocity(motor.getMaxVelocity() / 2.0)
    motor.setPosition(target_pos[i])

    if names[i].startswith("arm_"):
        ps = motor.getPositionSensor()
        ps.enable(time_step)
        arm_sensors[names[i]] = ps



# Position sensors for pan/tilt
head_pan_sensor  = robot.getDevice("head_1_joint_sensor")
head_tilt_sensor = robot.getDevice("head_2_joint_sensor")

head_pan_sensor.enable(time_step)
head_tilt_sensor.enable(time_step)


# Get the robot node (this robot, as a Pose node)
robot_node = robot.getSelf()
if robot_node is None:
    print("Error: getSelf() returned None, something is wrong with the controller binding.")


viewpoint = robot.getFromDef("USER_CAMERA")
if viewpoint is None:
    print("Error: could not find USER_CAMERA Viewpoint. Check the DEF name in the world file.")
else:
    vp_pos_field = viewpoint.getField("position")
    vp_rot_field = viewpoint.getField("orientation")


vp_pos_field.setSFVec3f(init_viewpoint)
vp_rot_field.setSFRotation(init_rotation) 

# Enable keyboard
keyboard = Keyboard()
keyboard.enable(time_step) 


initial_time = robot.getTime()

print("Use arrow keys to drive. Press SPACE to turn head toward the camera.")

# initial view direction of robot
square_red = robot.getFromDef("SQUARE_RED")
if square_red is None:
    print("Could not find SQUARE_RED node")

puzzle_outline = robot.getFromDef("PUZZLE_OUTLINE")
if puzzle_outline is None:
    print("Could not find PUZZLE_OUTLINE node")

para_1_red = robot.getFromDef("PARALELLOGRAM_2_RED")
if para_1_red is None:
    print("Could not find PARALELLOGRAM_2_RED node")

piece_pos = 0
last_robot_view_target = 0



# current_chain_positions = get_current_chain_positions()
# print ("initial chain positions", current_chain_positions)

# pos, R = get_end_effector_pose()
# print("End effector position (base frame):", pos)
# print("Rotation matrix:\n", R)




# Main loop
while robot.step(time_step) != -1:
    # piece_pos = translation_field.getSFVec3f()  # [x, y, z]
    # print("PARA_PIECE_1_A position:", piece_pos)
    # robot_view_target = piece_pos
    check_keyboard(robot_parts, keyboard, robot_node, head_pan_sensor, head_tilt_sensor)
    # if robot_view_target and not last_robot_view_target:
    #     make_head_look_at_target(robot_parts, robot_node, robot_view_target)
    #     last_robot_view_target = robot_view_target

    # # "Hello" waving movement (arm joint 8)
    # t = robot.getTime() - initial_time
    # robot_parts[8].setPosition(0.3 * math.sin(5.0 * t) - 0.3)
