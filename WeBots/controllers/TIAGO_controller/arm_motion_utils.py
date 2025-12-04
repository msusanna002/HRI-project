

# includes functions to move the TIAGo arm using IKPY

from coord_utils import world_to_robot, robot_to_world
import numpy as np
import math
import config as cfg
import scene_objects as scene
import numpy as np
import config as config

# variables initialized in init_ik()
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

#init function to set global variables
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

# debug print function
def debug_log(*args):
    print(*args, flush=True)

# set the joint positions directly, ignoring IK
def send_joint_pose(pose):
    """
    Send joint positions to the robot arm motors.
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
        # Warn if lengths don't match
        if len(pose) != len(robot_parts):
            print(f"[send_joint_pose] WARNING: pose length {len(pose)} "
                  f"does not match number of robot parts {len(robot_parts)}. "
                  f"Will only set the first {min(len(pose), len(robot_parts))} joints.")
        # Set joint angles in order
        for idx, angle in enumerate(pose):
            # Stop if we run out of robot parts
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
    Try to unlock the arm by forcing it to set poses.
    """
    print(f"[RESCUE] Triggered rescue_arm due to: {reason}")

    pose = None
    if (iter%3 == 0): pose = config.JOINT_TARGET_POS
    if (iter%3 == 1): pose = config.JOINT_RESCUE_POS_2
    if (iter%3 == 2): pose = config.JOINT_RESCUE_POS_3
    print("iter=" , iter, "resque Pose = ", pose)
    send_joint_pose(pose)


def distance_to_target(target_xyz):
    """
    Compute Euclidean distance from current end-effector pose to target_xyz.
    """
    pos, _ = get_end_effector_pose_from_sensors()
    dx = pos[0] - target_xyz[0]
    dy = pos[1] - target_xyz[1]
    dz = pos[2] - target_xyz[2]
    return math.sqrt(dx*dx + dy*dy + dz*dz)

def get_joint_angles_from_sensors():
    """
    Returns a list of joint positions from the arm's sensors.
    Arms sensors are defined in the configuration.
    Disabled links get 0. Active links map to the Webots sensors.
    """
    # Initialize joint list
    joints = [0.0] * len(my_chain.links)

    # For each active link, get the sensor value
    for i, link in enumerate(my_chain.links):
        #check if link is active
        if not my_chain.active_links_mask[i]:
            continue

        link_name = link.name
        # Get sensor value
        if link_name in arm_sensors:
            joints[i] = arm_sensors[link_name].getValue()

        else:
             # if snsor missing, leave joint at 0
            joints[i] = 0.0

    return joints


def get_end_effector_pose_from_sensors():
    """
    Returns the current end-effector pose as (position, rotation matrix).
    Position is a tuple (x, y, z).
    Rotation matrix is a 3x3 numpy array.
    """
    joints = get_joint_angles_from_sensors()

    # get end effector pose via forward kinematics
    T = my_chain.forward_kinematics(joints)

    pos = (T[0, 3], T[1, 3], T[2, 3])
    R = T[:3, :3]

    return pos, R

def build_ik_seed_from_motor_pose(motor_pose):
    """
    Convert a 'motor_pose' list in the same order as `robot_parts` into a
    full IK seed for my_chain.inverse_kinematics.

    - Starts from current sensor-based joints (which are guaranteed to match
      my_chain.links).
    - Overrides entries for joints where motor_pose provides a finite value.
    """
    # Start with the current valid chain vector
    seed = np.array(get_joint_angles_from_sensors(), dtype=float)

    # If the pose is shorter/longer than robot_parts, just use what we can
    n_motors = min(len(motor_pose), len(robot_parts))

    for motor_idx in range(n_motors):
        val = motor_pose[motor_idx]
        if not np.isfinite(val):
            # ignore 'inf' markers, keep current joint value
            continue

        motor = robot_parts[motor_idx]
        motor_name = names[motor_idx]

        # Find the corresponding chain index for this motor name
        for link_idx, link in enumerate(my_chain.links):
            if not my_chain.active_links_mask[link_idx]:
                continue
            if link.name == motor_name:
                seed[link_idx] = float(val)
                break

    return seed

# ---------------------------------------------------------------------
# IKPY-based arm motion
# ---------------------------------------------------------------------
def _solve_ik_with_orientation(target, initial_joint_angles, orientation, orientation_mode):
    """Helper to solve IK with optional orientation constraints."""
    if orientation is not None and orientation_mode is not None:
        return my_chain.inverse_kinematics(
            target,
            initial_position=initial_joint_angles,
            target_orientation=orientation,
            orientation_mode=orientation_mode,
        )
    else:
        return my_chain.inverse_kinematics(
            target,
            initial_position=initial_joint_angles,
        )


def _compute_ik_error(ik_results, target):
    """Compute FK-based error between IK solution and target position."""
    if ik_results is None or len(ik_results) != len(my_chain.links):
        return None
    
    fk_result = my_chain.forward_kinematics(ik_results)
    end_eff_pos = fk_result[:3, 3]
    
    dx = end_eff_pos[0] - target[0]
    dy = end_eff_pos[1] - target[1]
    dz = end_eff_pos[2] - target[2]
    
    err = math.sqrt(dx * dx + dy * dy + dz * dz)
    return err if np.isfinite(err) else None


def _apply_joint_angles(ik_results):
    """Apply IK solution to robot motors."""
    for i, link in enumerate(my_chain.links):
        if not my_chain.active_links_mask[i]:
            continue
        
        link_name = link.name
        if link_name in names:
            motor_index = names.index(link_name)
            motor = robot_parts[motor_index]
            target_angle = ik_results[i]

            if not np.isfinite(target_angle):
                print(f"[IK] Non-finite joint value for {link_name}: {target_angle}, skipping")
                continue

            motor.setPosition(target_angle)


def solve_move_arm_to_position(target_xyz,
                            orientation=None,
                            orientation_mode=None,
                            error_tolerance=0.005,
                            initial_joint_angles=None):
    """
    Compute IK solution for moving the arm's end-effector to target_xyz.
    Returns the IK solution and error, but does NOT apply it to motors.
    This allows non-blocking operation when called from state machines.
    
    Returns:
        tuple: (ik_results, error) or (None, float("inf")) on failure
    """
    base_target = np.array(target_xyz, dtype=float)

    if initial_joint_angles is None:
        initial_joint_angles = get_joint_angles_from_sensors()


    try:
        ik_results = _solve_ik_with_orientation(
            base_target.tolist(),
            initial_joint_angles,
            orientation,
            orientation_mode
        )
    except Exception as e:
        print(f"[IK] inverse_kinematics exception:", repr(e))
        return None, float("inf")
    
    err = _compute_ik_error(ik_results, base_target.tolist())
    if err is None:
        print("[IK] Failed to compute error; returning None")
        return None, float("inf")

    # Warn if tolerance not met
    if err > error_tolerance:
        print(f"[IK] Error {err:.4f} > tolerance {error_tolerance:.4f}")


    return ik_results, err


def move_arm_to_position(target_xyz,
                        orientation=None,
                        orientation_mode=None,
                        error_tolerance=0.005):
    """
    Compute and apply IK solution to move the arm to target_xyz.
    
    Args:
        target_xyz: Target position [x, y, z]
        orientation: Optional target orientation
        orientation_mode: Optional orientation mode (e.g., "Z")
        error_tolerance: Maximum acceptable position error
    """
    ik_results, err = solve_move_arm_to_position(
        target_xyz,
        orientation=orientation,
        orientation_mode=orientation_mode,
        error_tolerance=error_tolerance
    )
    
    if ik_results is not None:
        _apply_joint_angles(ik_results)


# ---------------------------------------------------------------------
# Grab and Drop helpers        
# --------------------------------------------------------------------- 

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



def drop_piece():
    """
    Release the piece: stop following the hand.
    """
    global held_piece

    if held_piece is None:
        print("[drop_piece] Nothing to drop")
        return

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
        self.rescue_step_counter = 0
        self.RESCUE_JOINT_TOL = 0.02
        self.RESCUE_MAX_STEPS = 400
        self.MAX_RESCUES = 3
        self.rescue_joint_target = None

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

    def _current_phase_target(self):
        if self.phase == 1:
            return self.phase1_target, dict(orientation=[0, 0, 1], orientation_mode=None)
        elif self.phase == 2:
            return self.phase2_target, dict(orientation=None, orientation_mode=None)
        else:  # phase 3
            return self.phase3_target, dict(orientation=[0, 0, 1], orientation_mode="Z")
    
    def _trigger_rescue(self, reason):
        """
        Choose a rescue pose that is a good IK seed for the current phase target.
        Only move to a rescue pose if IK from that seed gives acceptable error.
        """
        print(f"[ArmTask] Rescue triggered in {self.target_label}: {reason}")

        target, orient_kwargs = self._current_phase_target()
        candidate_poses = [
            config.JOINT_RESCUE_POS_1,
            config.JOINT_RESCUE_POS_2,
            config.JOINT_RESCUE_POS_3,
        ]

        best = None

        for pose in candidate_poses:
            seed = build_ik_seed_from_motor_pose(pose)

            ik_result, err = solve_move_arm_to_position(
                target,
                error_tolerance=self.RESCUE_ERROR_LIMIT,
                initial_joint_angles=seed,
                **orient_kwargs,
            )

            if ik_result is None or not np.isfinite(err):
                continue

            # Keep the best feasible candidate
            if best is None or err < best["err"]:
                best = {"pose": pose, "seed": seed, "err": err}

        # If no candidate gives a decent rescue seed, give up
        if best is None:
            print(f"[ArmTask] No valid rescue seed found in {self.target_label}, giving up.")
            self.done = True
            return True

        if best["err"] > self.RESCUE_ERROR_LIMIT:
            print(f"[ArmTask] Best rescue seed error {best['err']:.4f} still too large, giving up.")
            self.done = True
            return True

        # Now we *know* that starting from best["seed_pose] the IK for our phase target is reasonable.
        # Use that as a physical rescue pose to unlock the arm.
        send_joint_pose(best["pose"])

        # Put the task into "rescue wait" mode
        self.in_rescue = True
        self.rescue_joint_target = np.array(best["seed"], dtype=float)
        self.rescue_step_counter = 0

        # Reset phase command flag so the current phase recomputes IK from the new state
        if self.phase == 1:
            self.phase1_commanded = False
        elif self.phase == 2:
            self.phase2_commanded = False
        elif self.phase == 3:
            self.phase3_commanded = False

        # Reset stuck tracking
        self.stuck_steps = 0
        self.last_err = None

        self.rescue_count += 1
        if self.rescue_count > self.MAX_RESCUES:
            print(f"[ArmTask] Too many rescues in {self.target_label}, giving up.")
            self.done = True
            return True

        return True




    def step(self):
        if self.done:
            return

         # If we are currently in rescue mode, just wait a few steps to let
        # the arm move to its neutral pose before re-commanding the phase.
        if self.in_rescue:
            # Read current chain joint angles
            joints = np.array(get_joint_angles_from_sensors(), dtype=float)
            self.rescue_step_counter += 1

            if self.rescue_joint_target is not None:
                diff = joints - self.rescue_joint_target
                # You can use max-abs error or L2 norm; max-abs is easy to reason about
                joint_err = float(np.max(np.abs(diff)))

                # Optional: debug print
                # print(f"[Rescue] joint_err = {joint_err:.4f}")

                if joint_err < self.RESCUE_JOINT_TOL:
                    print(f"[ArmTask] Rescue pose reached in {self.target_label}, retrying phase {self.phase}")
                    self.in_rescue = False
                    self.rescue_joint_target = None
                    self.stuck_steps = 0
                    self.last_err = None
                    # Next call to step() will run the normal phase logic
                    return

            # Safety: if something goes wrong and we never converge, bail out
            if self.rescue_step_counter > self.RESCUE_MAX_STEPS:
                print(f"[ArmTask] Rescue did not converge in {self.target_label}, giving up.")
                self.done = True
                return

            # Still moving toward rescue pose, skip phase logic this tick
            return

        # ----- Phase 1 -----
        if self.phase == 1:
            if not self.phase1_commanded:
                # Send IK once
                # send_joint_pose(config.JOINT_TARGET_POS)
                error_tolerance = 0.2
                ik_result,err = solve_move_arm_to_position(
                    self.phase1_target,
                    orientation=[0, 0, 1],
                    error_tolerance=error_tolerance,
                )
                if ik_result is not None and err <= error_tolerance:
                    _apply_joint_angles(ik_result)
                else:
                    self._trigger_rescue("IK failure in phase 1")
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
                error_tolerance = 0.1
                ik_result, err = solve_move_arm_to_position(
                    self.phase2_target,
                    error_tolerance=error_tolerance,
                )
                self.phase2_commanded = True
            
                if ik_result is not None and err <= error_tolerance:
                    _apply_joint_angles(ik_result)
                else:
                    self._trigger_rescue("IK failure in phase 2")

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
                error_tolerance = 0.01
                ik_result,err = solve_move_arm_to_position(
                    self.phase3_target,
                    orientation=[0, 0, 1],
                    orientation_mode="Z",
                    error_tolerance=error_tolerance,
                )
                if ik_result is not None and err <= error_tolerance:
                    _apply_joint_angles(ik_result)
                else:
                    self._trigger_rescue("IK failure in phase 3")
                self.phase3_commanded = True

            err = distance_to_target(self.phase3_target)
            if self._check_rescue(err):
                return
            if err < 0.01:
                # print("[ArmTask] Phase 3 reached, task done")
                self.done = True
            return
        
