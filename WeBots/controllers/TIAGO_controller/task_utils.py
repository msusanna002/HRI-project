from arm_motion_utils import ArmMovementTask, grab_piece, drop_piece
import scene_objects as scene
import arm_motion_utils as arm_utils
from head_motion_utils import make_head_look_at_target
import random



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
            self.done = True
            self.current_subtask = None
            # Reset camera / focus, like in the old move_piece
            return