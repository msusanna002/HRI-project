from arm_motion_utils import ArmMovementTask, grab_piece, drop_piece
import scene_objects as scene
import arm_motion_utils as arm_utils
from head_motion_utils import make_head_look_at_target

def gather_gaze_data(initial_time, tracking_time):
    """
    Freeze the current arm configuration and make the robot
    look at the person for `tracking_time` seconds of simulation.

    initial_time:  float, usually robot.getTime() at the start
    tracking_time: float, duration in seconds to keep gazing
    """
    # Always fetch from arm_utils at call time
    robot = arm_utils.robot
    robot_parts = arm_utils.robot_parts
    time_step = arm_utils.time_step

    if robot is None:
        print("[gather_gaze_data] ERROR: robot is None (init_ik not called?)")
        return

    # Save the previous gaze target so we can restore it afterwards
    previous_object = scene.current_object

    # Get the robot node once (for head control)
    robot_node = robot.getSelf()

    # Set gaze target to the person (adjust to your actual person node)
    # Change `scene.person` if your person node has another name
    scene.current_object = scene.viewpoint

    end_time = initial_time + tracking_time
    number_of_logs = 3
    log_index = 0   

    if tracking_time > 0:
        log_step = tracking_time / number_of_logs
        next_log_time = initial_time + log_step
    else:
        log_step = 0.0
        next_log_time = float("inf")

    while robot.step(time_step) != -1:
        now = robot.getTime()
        if now >= end_time:
            break

        # Keep head oriented toward the person while we "freeze"
        make_head_look_at_target(robot_parts, robot_node, scene.current_object)

        if log_step > 0 and now >= next_log_time:
            print(f"{log_index} / {number_of_logs} of gaze tracking")
            next_log_time += log_step
            log_index += 1

    # Restore previous gaze target after tracking
    print(f"{number_of_logs} / {number_of_logs} of gaze tracking")
    scene.current_object = previous_object


class MovePieceTask:
    """
    High-level non-blocking task:
      1) Move arm to the piece.
      2) Grab the piece.
      3) Move arm to the destination.
      4) Drop the piece.

    Call step() once per simulation step from the main loop.
    """
    def __init__(self, targetObject, dest_vec, robotNode,
                 x_offset=0.0, y_offset=0.0,
                 lift=0.30, tableHeight=0.40):
        self.targetObject = targetObject
        self.destination = dest_vec
        self.robotNode = robotNode

        if targetObject is None or dest_vec is None:
            print("[MovePieceTask] ERROR: targetObject or destination is None")
            self.done = True
            self.current_subtask = None
            return

        # Start by looking at / focusing on the picked object,
        # just like your old move_piece did.
        scene.current_object = targetObject

        targetObjectVec = list(targetObject.getField("translation").getSFVec3f())

        # First subtask: move to the piece
        self.state = "to_source"
        self.current_subtask = ArmMovementTask(
            targetObjectVec,
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
            # print("[MovePieceTask] Reached source, grabbing piece")
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
            # print("[MovePieceTask] Reached destination, dropping piece")
            drop_piece()
            # Next subtask: move to destination
            self.state = "look_at_person"
            # Reset camera / focus, like in the old move_piece
            return
        
        # If we just finished going to the destination, drop and finish
        if self.state == "look_at_person":
            print("[MovePieceTask] Dropped piece, looking at person")
            # Do NOT change scene.current_object here; gather_gaze_data handles it.
            initial_time = arm_utils.robot.getTime()
            gather_gaze_data(initial_time, 1.5)
            self.done = True
            self.current_subtask = None
            return
