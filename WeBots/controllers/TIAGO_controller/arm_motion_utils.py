

from coord_utils import world_to_robot
import numpy as np
import math
import config as cfg
import scene_objects as scene

my_chain = None
arm_sensors = None
names = None
robot_parts = None
robot = None
time_step = None# Workaround for ikpy on newer NumPy versions
if not hasattr(np, "float"):
    np.float = float

def init_ik(chain, arm_sensors_map, names_list, robot_parts_list, robot_obj, ts):
    global my_chain, arm_sensors, names, robot_parts, robot, time_step
    my_chain = chain
    arm_sensors = arm_sensors_map
    names = names_list
    robot_parts = robot_parts_list
    robot = robot_obj
    time_step = ts

# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------
# Workaround for ikpy on newer NumPy versions
if not hasattr(np, "float"):
    np.float = float

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
