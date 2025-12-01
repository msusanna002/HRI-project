

from coord_utils import world_to_robot, robot_to_world
import numpy as np
import math
import config as cfg
import scene_objects as scene
import numpy as np

held_piece = None
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
                                  target_string,
                                  orientation=None,
                                  orientation_mode=None,
                                  pos_tolerance=0.005,
                                  max_steps=1000,
                                  min_z=0.52):
    # Command the move
    err_ik = move_arm_to_position(
        target_xyz,
        orientation,
        orientation_mode,
        error_tolerance=pos_tolerance
    )
    
    if err_ik > pos_tolerance:
        print("[IK] Target", target_string, " likely unreachable, aborting blocking wait")
        return

    steps = 0
    while robot.step(time_step) != -1:
        # SAFETY CHECK: make sure end-effector doesn't go below min_z
        pos, _ = get_end_effector_pose_from_sensors()
        current_z = pos[2]   # assuming chain frame shares z with world (usual TIAGo case)

        if current_z < min_z:
            print(f"[IK] SAFETY: end-effector z={current_z:.3f} < min_z={min_z:.3f}, aborting move.")
            # Optionally try to move straight up to safety:
            safe_target = [pos[0], pos[1], min_z]
            move_arm_to_position(safe_target, orientation, orientation_mode)
            break

        # Normal convergence check
        err = distance_to_target(target_xyz)
        if err < pos_tolerance:
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

def grab_piece(piece_node, robot_node):
    """
    Start 'holding' this piece. Call this when the arm has reached it.
    """
    global held_piece
    held_piece = piece_node
    # Do one immediate update so it snaps into place
    update_held_piece(robot_node)


def drop_piece():
    """
    Stop holding the current piece. It will stay where it is.
    """
    global held_piece
    held_piece = None


def update_held_piece(robot_node):
    """
    If we are holding a piece, keep its translation under the end-effector.
    Call this once every timestep from the main loop.
    """

    # print("Updating held piece position to:", held_piece.getDef() if held_piece else "None")
    if held_piece is None:
        return

    # End-effector pose in robot base frame
    pos_B, _ = get_end_effector_pose_from_sensors()

    # Convert to world coords
    pos_W = robot_to_world(pos_B, robot_node)

    # Slight offset down so the piece sits under the tool instead of inside it
    pos_W[2] += 0.02

    held_piece.getField("translation").setSFVec3f(pos_W)

# def grab_piece(piece_node, robot_node):
#     """
#     Grab piece with NO offset.
#     Snap the piece directly to the end-effector position.
#     """
#     global held_piece
#     held_piece = piece_node

#     # Get end-effector position in robot frame
#     hand_pos_B, _ = get_end_effector_pose_from_sensors()

#     # Convert to world coordinates
#     hand_pos_W = robot_to_world(hand_pos_B, robot_node)

#     # Move piece directly to the hand position
#     piece_node.getField("translation").setSFVec3f(hand_pos_W)

#     print("[grab_piece] snapped piece to the hand")


def arm_movement(targetObject, robotNode, x_offset=0.00, y_offset=0.00,
                 lift=0.30, tableHeight=0.40, do_grab=False, do_drop=False):
    """
    Move the arm to a given targetObject in three phases:
      1) go up to a safe height (>= min_z),
      2) move horizontally above the target (with x/y offsets),
      3) move down to a grasp height that accounts for tool length and piece height,
         but never below min_z.

    x_offset, y_offset:
        planar offsets in ROBOT frame that are added to the target x, y.
    lift:
        vertical margin (in meters) above the grasp height for the approach.
    tableHeight:
        table height in WORLD coordinates, used for safety.
    """

    # Safety height in world coordinates
    MIN_Z_WORLD = max(tableHeight, 0.40)

    if targetObject is None:
        print("[arm_movement] ERROR: targetObject is None")
        return
    
    name = targetObject.getDef()
    target_label = f"piece {name}"

    # 1) Get target position in WORLD coordinates
    targetPosWC = list(targetObject.getField("translation").getSFVec3f())
    # Clamp target z so we never aim below the allowed height
    if targetPosWC[2] < MIN_Z_WORLD:
        print(f"[arm_movement] Clamping target Z from {targetPosWC[2]:.3f} to {MIN_Z_WORLD:.3f}")
        targetPosWC[2] = MIN_Z_WORLD

    # 2) Convert clamped world coordinates to ROBOT coordinates
    targetPosRC = world_to_robot(targetPosWC, robotNode)
    target_x, target_y, target_z = targetPosRC

    # Apply planar offsets in ROBOT frame
    grasp_x = target_x + x_offset
    grasp_y = target_y + y_offset

    # Tool and object height compensation in z
    GRIPPER_LENGTH = 0.03          # tune this for your TIAGo model
    OBJECT_HALF_HEIGHT = 0.02   # tune this for your piece dimensions

    # target_z is likely object center; compute surface contact height
    grasp_z_surface = target_z + OBJECT_HALF_HEIGHT - GRIPPER_LENGTH

    # 3) Get current end-effector pose (in the same frame as IK chain)
    pos, _ = get_end_effector_pose_from_sensors()
    current_x, current_y, current_z = pos

    # Safety height in robot coordinates (assuming z aligned)
    table_point_world = [targetPosWC[0], targetPosWC[1], MIN_Z_WORLD]
    table_point_robot = world_to_robot(table_point_world, robotNode)
    _, _, table_z_robot = table_point_robot

    MIN_Z = table_z_robot + OBJECT_HALF_HEIGHT

    # -----------------------------------------------
    # Phase 1: go straight up from current position
    # -----------------------------------------------
    phase1_z = max(current_z, MIN_Z + lift)
    phase1_target = [current_x, current_y, phase1_z]

    move_arm_to_position_blocking(
        phase1_target,
        target_string=f"{target_label} - phase 1 lift", 
        orientation=[0, 0, 1],
        min_z=MIN_Z,
    )

    # -----------------------------------------------
    # Phase 2: move horizontally above the target
    # -----------------------------------------------
    phase2_z = max(grasp_z_surface + lift, MIN_Z + lift)
    phase2_target = [grasp_x, grasp_y, phase2_z]

    move_arm_to_position_blocking(
        phase2_target,
        target_string=f"{target_label} - above",
        pos_tolerance=0.1,
        min_z=MIN_Z,
    )

    # -----------------------------------------------
    # Phase 3: descend to grasp height
    # -----------------------------------------------
    final_z = max(grasp_z_surface, MIN_Z)
    phase3_target = [grasp_x, grasp_y, final_z]

    move_arm_to_position_blocking(
        phase3_target,
        target_string=f"{target_label} - grasp",
        pos_tolerance=0.01,
        min_z=MIN_Z,
        orientation=[0, 0, 1],
        orientation_mode="Z",
    )

    if do_grab:
        grab_piece(targetObject, robotNode)
    if do_drop:
        drop_piece()

def move_piece(targetObject, destination, robotNode):
    """
    Move a piece from its current position to a destination position.
    Both targetObject and destination are Webots nodes.
    """
    if targetObject is None or destination is None:
        print("[move_piece] ERROR: targetObject or destination is None")
        return

    scene.current_object = targetObject
    # 1) Move above the piece and grab it
    arm_movement(targetObject, robotNode)
    grab_piece(targetObject, robotNode)
    # 3) Drop the piece
    arm_movement(destination, robotNode)
    drop_piece(targetObject, robotNode)

    scene.current_object = scene.viewpoint

# ---------------------------------------------------------------------
# Non-blocking arm task
# ---------------------------------------------------------------------
class ArmMovementTask:
    def __init__(self, targetObject, robotNode,
                 x_offset=0.0, y_offset=0.0,
                 lift=0.30, tableHeight=0.40,
                 target_label="arm_task"):
        self.targetObject = targetObject
        self.robotNode = robotNode
        self.phase = 1
        self.done = False

        self.phase1_commanded = False
        self.phase2_commanded = False
        self.phase3_commanded = False

        if targetObject is None:
            print("[ArmMovementTask] ERROR: targetObject is None")
            self.done = True
            return

        # 1. Read target position in WORLD
        obj_translation_field = targetObject.getField("translation")
        target_world = obj_translation_field.getSFVec3f()
        target_world_x = target_world[0]
        target_world_y = target_world[1]
        target_world_z = target_world[2]

        # 2. Convert WORLD -> ROBOT frame
        target_robot = world_to_robot(
            [target_world_x, target_world_y, target_world_z],
            robotNode
        )

        GRIPPER_LENGTH = 0.03          # tune this for your TIAGo model
        OBJECT_HALF_HEIGHT = 0.02

        grasp_x = target_robot[0] + x_offset
        grasp_y = target_robot[1] + y_offset
        grasp_z_surface = target_robot[2] + OBJECT_HALF_HEIGHT - GRIPPER_LENGTH

        MIN_Z = tableHeight + 0.05   # safety margin
        lifted_z = max(grasp_z_surface + lift, MIN_Z)
        self.MIN_Z = MIN_Z
        self.target_label = target_label

        # 3. Read current end effector position (robot frame)
        pos, _ = get_end_effector_pose_from_sensors()
        current_x = pos[0]
        current_y = pos[1]

        # Phase targets in ROBOT frame
        self.phase1_target = [current_x, current_y, lifted_z]
        self.phase2_target = [grasp_x, grasp_y, lifted_z]
        self.phase3_target = [grasp_x, grasp_y, max(grasp_z_surface, MIN_Z)]

    def step(self):
        if self.done:
            return


        # ----- Phase 1 -----
        if self.phase == 1:
            if not self.phase1_commanded:
                # Send IK once
                move_arm_to_position(
                    self.phase1_target,
                    orientation=[0, 0, 1],
                    error_tolerance=0.1,
                )
                self.phase1_commanded = True

            # Only FK + distance check each tick
            err = distance_to_target(self.phase1_target)
            if err < 0.1:
                print("[ArmTask] Phase 1 reached")
                self.phase = 2
            return

        # ----- Phase 2 -----
        if self.phase == 2:
            if not self.phase2_commanded:
                move_arm_to_position(
                    self.phase2_target,
                    error_tolerance=0.05,
                )
                self.phase2_commanded = True

            err = distance_to_target(self.phase2_target)
            if err < 0.05:
                print("[ArmTask] Phase 2 reached")
                self.phase = 3
            return

        # ----- Phase 3 -----
        if self.phase == 3:
            if not self.phase3_commanded:
                move_arm_to_position(
                    self.phase3_target,
                    orientation=[0, 0, 1],
                    orientation_mode="Z",
                    error_tolerance=0.01,
                )
                self.phase3_commanded = True

            err = distance_to_target(self.phase3_target)
            if err < 0.01:
                print("[ArmTask] Phase 3 reached, task done")
                self.done = True
            return
        

class MovePieceTask:
    """
    High-level non-blocking task:
      1) Move arm to the piece.
      2) Grab the piece.
      3) Move arm to the destination.
      4) Drop the piece.

    Call step() once per simulation step from the main loop.
    """
    def __init__(self, targetObject, destination, robotNode,
                 x_offset=0.0, y_offset=0.0,
                 lift=0.30, tableHeight=0.40):
        self.targetObject = targetObject
        self.destination = destination
        self.robotNode = robotNode

        if targetObject is None or destination is None:
            print("[MovePieceTask] ERROR: targetObject or destination is None")
            self.done = True
            self.current_subtask = None
            return

        # Start by looking at / focusing on the picked object,
        # just like your old move_piece did.
        scene.current_object = targetObject

        # First subtask: move to the piece
        self.state = "to_source"
        self.current_subtask = ArmMovementTask(
            targetObject,
            robotNode,
            x_offset=x_offset,
            y_offset=y_offset,
            lift=lift,
            tableHeight=tableHeight,
            target_label="move_piece_to_source",
        )
        self.done = False

    def step(self):
        if self.done or self.current_subtask is None:
            return

        # Advance current arm subtask
        self.current_subtask.step()

        # If the current arm motion is not finished, we are done for this tick
        if not self.current_subtask.done:
            return

        # If we just finished going to the source piece, grab it and start going to destination
        if self.state == "to_source":
            print("[MovePieceTask] Reached source, grabbing piece")
            grab_piece(self.targetObject, self.robotNode)

            # Next subtask: move to destination
            self.state = "to_destination"
            self.current_subtask = ArmMovementTask(
                self.destination,
                self.robotNode,
                # You can tune offsets and lift for the placement motion too
                x_offset=0.0,
                y_offset=0.0,
                lift=0.30,
                tableHeight=0.40,
                target_label="move_piece_to_destination",
            )
            return

        # If we just finished going to the destination, drop and finish
        if self.state == "to_destination":
            print("[MovePieceTask] Reached destination, dropping piece")
            drop_piece()
            # Reset camera / focus, like in the old move_piece
            scene.current_object = scene.viewpoint
            self.done = True
            self.current_subtask = None
            return
