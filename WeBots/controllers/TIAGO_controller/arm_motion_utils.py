

from coord_utils import world_to_robot, robot_to_world
import numpy as np
import math
import config as cfg
import scene_objects as scene
import numpy as np
import config as config

held_piece = None
my_chain = None
arm_sensors = None
names = None
robot_parts = None
robot = None
time_step = None
# Workaround for ikpy on newer NumPy versions
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
import traceback

def debug_log(*args):
    # simple helper that always flushes
    print(*args, flush=True)


held_piece_def = None  # replace the node-based held_piece with DEF-based

# ... you already have `robot` global set in init_ik

import traceback

def debug_log(*args):
    print(*args, flush=True)

def send_joint_pose(pose):
    """
    Directly command a joint-space pose, ignoring IK.
    Used for 'rescue' when the arm is stuck or IK is bad.

    `pose` can be:
      - dict[str, float]: {joint_name: angle}
      - list/tuple[float]: angles in the same order as `names` / `robot_parts`
    """
    # Case 1: dict mapping joint names to angles
    if isinstance(pose, dict):
        for jname, angle in pose.items():
            if jname in names:
                idx = names.index(jname)
                motor = robot_parts[idx]
                motor.setPosition(angle)
            else:
                print(f"[send_joint_pose] WARNING: joint name '{jname}' not in known names, skipping.")
        return

    # Case 2: list or tuple of angles in fixed order
    if isinstance(pose, (list, tuple)):
        if len(pose) != len(robot_parts):
            print(f"[send_joint_pose] WARNING: pose length {len(pose)} "
                  f"does not match number of robot parts {len(robot_parts)}. "
                  f"Will only set the first {min(len(pose), len(robot_parts))} joints.")

        for idx, angle in enumerate(pose):
            if idx >= len(robot_parts):
                break
            motor = robot_parts[idx]
            motor.setPosition(angle)
        return

    # Fallback for unexpected types
    print(f"[send_joint_pose] ERROR: Unsupported pose type {type(pose)}. "
          f"Expected dict or list/tuple.")


def rescue_arm(iter, reason="unknown", ):
    """
    Try to unlock the arm by sending it to SAFE_ARM_POSE.
    You can log the reason for debugging.
    """
    print(f"[RESCUE] Triggered rescue_arm due to: {reason}")

    pose = None
    if (iter%3 == 0): pose = config.JOINT_TARGET_POS
    if (iter%3 == 1): pose = config.JOINT_RESCUE_POS_2
    if (iter%3 == 2): pose = config.JOINT_RESCUE_POS_3
    print("iter=" , iter, "resque Pose = ", pose)
    send_joint_pose(pose)

# def get_held_objects_children_field():
#     """
#     Returns the MFNode 'children' field of the HELD_OBJECTS Group inside the gripper.
#     We go:
#       TIAGO_ROBOT
#         -> endEffectorSlot (Slot)
#            -> endPoint (Solid)
#               -> children[ i ] ... including DEF HELD_OBJECTS Group
#     """
#     if robot is None:
#         debug_log("[get_held_objects_children_field] ERROR: robot is None")
#         return None

#     tiago_node = robot.getFromDef("TIAGO_ROBOT")
#     if tiago_node is None:
#         debug_log("[get_held_objects_children_field] ERROR: TIAGO_ROBOT not found")
#         return None

#     # endEffectorSlot is SFNode Slot
#     slot_field = tiago_node.getField("endEffectorSlot")
#     if slot_field is None:
#         debug_log("[get_held_objects_children_field] ERROR: no endEffectorSlot field")
#         return None

#     slot_node = slot_field.getSFNode()
#     if slot_node is None:
#         debug_log("[get_held_objects_children_field] ERROR: endEffectorSlot is empty")
#         return None

#     # Slot has an SFNode endPoint: the Solid at the wrist
#     end_point_field = slot_node.getField("endPoint")
#     if end_point_field is None:
#         debug_log("[get_held_objects_children_field] ERROR: Slot has no 'endPoint' field")
#         return None

#     solid_node = end_point_field.getSFNode()
#     if solid_node is None:
#         debug_log("[get_held_objects_children_field] ERROR: endPoint is empty")
#         return None

#     solid_children = solid_node.getField("children")
#     if solid_children is None:
#         debug_log("[get_held_objects_children_field] ERROR: Solid has no 'children' field")
#         return None

#     # Find DEF HELD_OBJECTS in Solid's children
#     count = solid_children.getCount()
#     debug_log("[get_held_objects_children_field] Solid children count:", count)

#     held_group_node = None
#     for i in range(count):
#         n = solid_children.getMFNode(i)
#         def_name = n.getDef()
#         debug_log(f"[get_held_objects_children_field] child {i} DEF:", def_name)
#         if def_name == "HELD_OBJECTS":
#             held_group_node = n
#             break

#     if held_group_node is None:
#         debug_log("[get_held_objects_children_field] ERROR: HELD_OBJECTS group not found")
#         return None

#     held_children = held_group_node.getField("children")
#     if held_children is None:
#         debug_log("[get_held_objects_children_field] ERROR: HELD_OBJECTS has no 'children' field")
#         return None

#     debug_log("[get_held_objects_children_field] SUCCESS: got HELD_OBJECTS.children field")
#     return held_children


# def _clone_node_to_field(node, parent_field):
#     """
#     Clone 'node' under the given MFNode field by exporting its string
#     and importing a copy *without* the DEF prefix.
#     Returns the new Node reference (the cloned node), or None on error.
#     """
#     try:
#         if node is None or parent_field is None:
#             debug_log("[_clone_node_to_field] ERROR: node or parent_field is None")
#             return None

#         field_type = parent_field.getTypeName()
#         debug_log("[_clone_node_to_field] parent_field type:", field_type)
#         if field_type != "MFNode":
#             debug_log("[_clone_node_to_field] ERROR: parent_field is not MFNode")
#             return None

#         node_def = node.getDef()
#         debug_log("[_clone_node_to_field] Cloning DEF:", node_def)

#         node_string = node.exportString()
#         debug_log("[_clone_node_to_field] Before import, children count:",
#                   parent_field.getCount())

#         # Strip leading "DEF XYZ" so the clone has no DEF and no conflict occurs.
#         # This is a simple, robust text-level hack.
#         s = node_string.lstrip()
#         lines = s.split('\n', 1)
#         first_line = lines[0]
#         rest = lines[1] if len(lines) > 1 else ""
#         tokens = first_line.split(None, 2)  # ["DEF", "NAME", "NodeType { ..."]
#         if len(tokens) >= 3 and tokens[0] == "DEF":
#             first_line = tokens[2]  # drop "DEF NAME"
#         stripped_string = first_line + ("\n" + rest if rest else "")

#         # Import the clone
#         parent_field.importMFNodeFromString(-1, stripped_string)
#         count_after = parent_field.getCount()
#         debug_log("[_clone_node_to_field] After import, children count:", count_after)

#         # The new node is the last element in the MFNode list
#         if count_after > 0:
#             new_node = parent_field.getMFNode(count_after - 1)
#             debug_log("[_clone_node_to_field] New cloned node:", new_node)
#             return new_node
#         else:
#             debug_log("[_clone_node_to_field] ERROR: children count still 0 after import")
#             return None

#     except Exception as e:
#         debug_log("[_clone_node_to_field] EXCEPTION:", repr(e))
#         traceback.print_exc()
#         return None



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
def move_arm_to_position(target_xyz,
                         orientation=None,
                         orientation_mode=None,
                         error_tolerance=0.005,
                         max_retries=3,
                         lift_step=0.03):
    """
    Use IKPY to move the TIAGo arm so that the end-effector reaches `target_xyz`
    in robot base coordinates.

    - If IK error is too large, we "rise arm and try again" by raising the
      target in z by `lift_step` up to `max_retries` times.
    - Only the best valid solution (if any) is sent to the motors.
    - Returns the best position error (float). If all attempts fail, returns +inf.
    """
    # 1) Current joint configuration as initial guess
    initial_position = get_joint_positions_from_sensors()

    def _solve_ik(target):
        if orientation is not None and orientation_mode is not None:
            return my_chain.inverse_kinematics(
                target,
                initial_position=initial_position,
                target_orientation=orientation,
                orientation_mode=orientation_mode,
            )
        else:
            return my_chain.inverse_kinematics(
                target,
                initial_position=initial_position,
            )

    base_target = np.array(target_xyz, dtype=float)

    best_err = float("inf")
    best_ik = None
    best_target = None

    for k in range(max_retries + 1):
        attempt_target = base_target.copy()
        attempt_target[2] += k * lift_step  # "rise arm and try again"

        try:
            ik_results = _solve_ik(attempt_target.tolist())
        except Exception as e:
            print(f"[IK] inverse_kinematics exception on attempt {k}:", repr(e))
            continue

        if ik_results is None or len(ik_results) != len(my_chain.links):
            print(f"[IK] invalid IK result on attempt {k}")
            continue

        fk_result = my_chain.forward_kinematics(ik_results)
        end_eff_pos = fk_result[:3, 3]
        dx = end_eff_pos[0] - attempt_target[0]
        dy = end_eff_pos[1] - attempt_target[1]
        dz = end_eff_pos[2] - attempt_target[2]
        err = math.sqrt(dx * dx + dy * dy + dz * dz)

        if not np.isfinite(err):
            print(f"[IK] Non-finite error on attempt {k}, skipping")
            continue


        # Track best attempt so far
        if err < best_err:
            best_err = err
            best_ik = ik_results
            best_target = attempt_target.copy()

        # If this attempt is already good enough, stop retrying
        if err <= error_tolerance:
            break

    # If no valid IK solution at all, do not command motors
    if best_ik is None or not np.isfinite(best_err):
        print("[IK] All attempts failed; not commanding motors")
        return float("inf")

    # If the best solution is still not within tolerance, we still have a
    # choice: send it (if it is "reasonable") or refuse. Here we send it,
    # but with a warning:
    if best_err > error_tolerance:
        print(f"[IK] Best error {best_err:.4f} > tolerance {error_tolerance:.4f}, "
              f"using lifted target {best_target} anyway")

    # 4) Apply best joint angles to Webots motors
    for i, link in enumerate(my_chain.links):
        if not my_chain.active_links_mask[i]:
            continue

        link_name = link.name
        if link_name in names:
            motor_index = names.index(link_name)
            motor = robot_parts[motor_index]
            target_angle = best_ik[i]

            if not np.isfinite(target_angle):
                print(f"[IK] Non-finite joint value for {link_name}: {target_angle}, skipping")
                continue

            motor.setPosition(target_angle)

    return best_err


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

# ---------------------------------------------------------------------
# Grab and Drop helpers        
# --------------------------------------------------------------------- 
def get_gripper_node():
    """
    Returns the CustomTiagoGripper PROTO instance stored in TIAGO_ROBOT.endEffectorSlot.
    In the world file this is:
      endEffectorSlot DEF GRIPPER_LINK CustomTiagoGripper { }
    which is an SFNode, not an MFNode list.
    """
    if robot is None:
        print("[get_gripper_node] ERROR: global robot is None (init_ik not called?)")
        return None

    tiago_node = robot.getFromDef("TIAGO_ROBOT")
    if tiago_node is None:
        print("[get_gripper_node] ERROR: could not find TIAGO_ROBOT by DEF")
        return None

    slot_field = tiago_node.getField("endEffectorSlot")
    if slot_field is None:
        print("[get_gripper_node] ERROR: TIAGO_ROBOT has no 'endEffectorSlot' field")
        return None

    print("[get_gripper_node] endEffectorSlot type:", slot_field.getTypeName())
    # Should print "SFNode"

    slot_node = slot_field.getSFNode()
    if slot_node is None:
        print("[get_gripper_node] ERROR: endEffectorSlot is empty")
        return None

    print("[get_gripper_node] Found gripper slot node DEF:", slot_node.getDef())
    return slot_node




held_piece_def = None
held_piece_clone = None

# def attach_piece_to_gripper(piece_node, robotNode):
#     global held_piece_def, held_piece_clone

#     try:
#         if piece_node is None:
#             debug_log("[attach_piece_to_gripper] piece_node is None, aborting")
#             return

#         held_piece_def = piece_node.getDef()
#         debug_log("[attach_piece_to_gripper] piece DEF:", held_piece_def)

#         # Hide original piece by moving it down
#         orig_trans_field = piece_node.getField("translation")
#         if orig_trans_field is not None:
#             orig_pos = orig_trans_field.getSFVec3f()
#             debug_log("[attach_piece_to_gripper] original piece world pos:", orig_pos)
#             orig_trans_field.setSFVec3f([orig_pos[0], orig_pos[1], orig_pos[2] - 5.0])
#             debug_log("[attach_piece_to_gripper] moved original piece down by 5m")

#         # Get HELD_OBJECTS.children MFNode field
#         held_children = get_held_objects_children_field()
#         if held_children is None:
#             debug_log("[attach_piece_to_gripper] ERROR: could not get HELD_OBJECTS children field")
#             return

#         debug_log("[attach_piece_to_gripper] HELD_OBJECTS children count before clone:",
#                   held_children.getCount())

#         # Clone the node under HELD_OBJECTS
#         clone_node = _clone_node_to_field(piece_node, held_children)
#         if clone_node is None:
#             debug_log("[attach_piece_to_gripper] ERROR: cloning under HELD_OBJECTS failed")
#             return

#         held_piece_clone = clone_node

#         # Remove physics on the clone
#         phys_field = clone_node.getField("physics")
#         if phys_field is not None:
#             phys_node = phys_field.getSFNode()
#             debug_log("[attach_piece_to_gripper] clone physics node:", phys_node)
#             if phys_node is not None:
#                 phys_field.setSFNode(None)
#                 debug_log("[attach_piece_to_gripper] clone physics removed")

#         # Set clone's local translation to origin of HELD_OBJECTS frame
#         trans_field = clone_node.getField("translation")
#         if trans_field is not None:
#             trans_field.setSFVec3f([0.0, 0.0, 0.0])
#             debug_log("[attach_piece_to_gripper] clone local translation set to 0,0,0")

#         debug_log("[attach_piece_to_gripper] SUCCESS: Attached visual clone of DEF",
#                   held_piece_def, "under HELD_OBJECTS")

#         return clone_node

#     except Exception as e:
#         debug_log("[attach_piece_to_gripper] EXCEPTION:", repr(e))
#         import traceback
#         traceback.print_exc()
#         return






held_piece = None

def grab_piece(piece_node, robotNode):
    """
    Simple grab: remember the piece, remove its physics.
    Its pose will be updated every timestep by update_held_piece().
    """
    global held_piece

    if piece_node is None:
        print("[grab_piece] ERROR: piece_node is None")
        return

    held_piece = piece_node

    # Remove physics so it doesn't fight with the gripper
    phys_field = piece_node.getField("physics")
    if phys_field is not None:
        phys_node = phys_field.getSFNode()
        if phys_node is not None:
            phys_field.setSFNode(None)

    # print("[grab_piece] Now holding", piece_node.getDef())



def drop_piece():
    """
    Release the piece: stop following the hand.
    (Optional: add physics back, and leave it where it is.)
    """
    global held_piece

    if held_piece is None:
        print("[drop_piece] Nothing to drop")
        return

    # Optionally restore physics here if your PROTO exposes it
    # phys_field = held_piece.getField("physics")
    # if phys_field is not None and phys_field.getSFNode() is None:
    #     # Create a Physics node via importMFNodeFromString:
    #     phys_field.importMFNodeFromString(-1, "Physics {}")

    # print("[drop_piece] Dropped", held_piece.getDef())
    held_piece = None



held_piece = None

def update_held_piece(robot_node):
    """
    If we are holding a piece, keep its translation under the end-effector.
    Call this once every timestep from the main loop.
    """
    if held_piece is None:
        return

    # End-effector pose in robot base frame
    pos_B, _ = get_end_effector_pose_from_sensors()

    # Convert to world coords
    pos_W = robot_to_world(pos_B, robot_node)

    # Slight offset down so the piece sits under the tool instead of inside it
    pos_W[2] += 0.02

    held_piece.getField("translation").setSFVec3f(pos_W)




def arm_movement(targetObject, robotNode, x_offset=0.00, y_offset=0.00,
                 lift=0.10, tableHeight=0.40, do_grab=False, do_drop=False):
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
    phase1_z = max(current_z, MIN_Z)
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
    phase2_z = max(grasp_z_surface , MIN_Z)
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
        grab_piece(targetObject, robot)
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
    def __init__(self, targetObjectVec, robotNode,
                 x_offset=0.0, y_offset=0.0,
                 lift=0.30, tableHeight=0.40,
                 target_label="arm_task"):
        self.targetObject = targetObjectVec
        self.robotNode = robotNode
        self.phase = 1
        self.done = False

        self.phase1_commanded = False
        self.phase2_commanded = False
        self.phase3_commanded = False

        self.last_err = None
        self.stuck_steps = 0
        self.STUCK_MAX_STEPS = 40
        self.RESCUE_ERROR_LIMIT = 0.2
        self.in_rescue = False
        self.rescue_wait_steps = 0

        self.rescue_count = 0
        self.MAX_RESCUES = 3

        if targetObjectVec is None:
            print("[ArmMovementTask] ERROR: targetObject is None")
            self.done = True
            return

        # 1. Read target position in WORLD
        target_world_x = targetObjectVec[0]
        target_world_y = targetObjectVec[1]
        target_world_z = targetObjectVec[2]

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
        
    def _check_rescue(self, err):
        """
        Common rescue logic for all phases.
        Returns True if a rescue was triggered or task ended.
        """
        if not np.isfinite(err):
            print(f"[ArmTask] Non-finite error in {self.target_label}")
            return self._trigger_rescue("non-finite error")

        # track stuck steps when error is not improving
        if self.last_err is not None and err >= self.last_err - 1e-4:
            self.stuck_steps += 1
            # print(self.stuck_steps)
        else:
            self.stuck_steps = 0

        self.last_err = err

        # If error is large and we are stuck for too long, rescue
        if self.stuck_steps > self.STUCK_MAX_STEPS:
            print(f"[ArmTask] Stuck with error {err:.4f} in {self.target_label}")
            return self._trigger_rescue(f"stuck, err={err:.4f}")

        return False


    
    def _trigger_rescue(self, reason):
        """
        Called when the arm is considered stuck or the error is invalid.
        Sends a neutral pose and prepares to redo the CURRENT phase
        after a short waiting period.
        """
        print(f"[ArmTask] Rescue triggered in {self.target_label}: {reason}")
        rescue_arm(self.rescue_count, reason=reason)  # this calls send_joint_pose(config.JOINT_TARGET_POS)

        self.rescue_count += 1
        if self.rescue_count > self.MAX_RESCUES:
            print(f"[ArmTask] Too many rescues in {self.target_label}, giving up.")
            self.done = True
            return True  # tell caller to stop

        # Put the task into "rescue wait" mode so the arm can reach neutral
        self.in_rescue = True
        self.rescue_wait_steps = 50   # tune this number as needed

        # Reset phase command flag so the current phase will recompute IK
        if self.phase == 1:
            self.phase1_commanded = False
        elif self.phase == 2:
            self.phase2_commanded = False
        elif self.phase == 3:
            self.phase3_commanded = False

        # Reset stuck tracking
        self.stuck_steps = 0
        self.last_err = None

        # Do NOT mark task done; we intend to retry
        return True



    def step(self):
        if self.done:
            return

         # If we are currently in rescue mode, just wait a few steps to let
        # the arm move to its neutral pose before re-commanding the phase.
        if self.in_rescue:
            if self.rescue_wait_steps > 0:
                self.rescue_wait_steps -= 1
                return  # keep waiting
            # done waiting, exit rescue mode and retry current phase
            print(f"[ArmTask] Rescue wait finished in {self.target_label}, retrying phase {self.phase}")
            self.in_rescue = False
            # fall through to the normal phase logic below

        # ----- Phase 1 -----
        if self.phase == 1:
            if not self.phase1_commanded:
                # Send IK once
                # send_joint_pose(config.JOINT_TARGET_POS)
                move_arm_to_position(
                    self.phase1_target,
                    orientation=[0, 0, 1],
                    error_tolerance=0.2,
                )
                self.phase1_commanded = True

            # Only FK + distance check each tick
            err = distance_to_target(self.phase1_target)
            if self._check_rescue(err):
                return
            if err < 0.1:
                # print("[ArmTask] Phase 1 reached")
                self.phase = 2
            return

        # ----- Phase 2 -----
        if self.phase == 2:
            if not self.phase2_commanded:
                move_arm_to_position(
                    self.phase2_target,
                    error_tolerance=0.1,
                )
                self.phase2_commanded = True

            err = distance_to_target(self.phase2_target)
            if self._check_rescue(err):
                return 
            if err < 0.05:
                # print("[ArmTask] Phase 2 reached")
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
            if self._check_rescue(err):
                return
            if err < 0.01:
                # print("[ArmTask] Phase 3 reached, task done")
                self.done = True
            return
        
