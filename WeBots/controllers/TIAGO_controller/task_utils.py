from arm_motion_utils import ArmMovementTask, grab_piece, drop_piece
import scene_objects as scene
import arm_motion_utils as arm_utils
from head_motion_utils import make_head_look_at_target
import random


class MovePieceTask:
    """
    High-level non-blocking task:
      1) Move arm to the piece.
      2) Pick up the piece.
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

        # track state
        self.done = False
        self.current_subtask = None
        self.state = None
        

        # check for None inputs
        if targetObject is None or dest_vec is None:
            print("[MovePieceTask] ERROR: targetObject or destination is None")
            self.done = True
            self.current_subtask = None
            self.state = None
            return

        # Set the current object for head tracking
        scene.current_object = targetObject

        # Get target object's position
        targetObjectVec = list(targetObject.getField("translation").getSFVec3f())

        # First state: move to the source
        self.state = "moving_to_source"
        self.start_next_arm_task(targetObjectVec, robotNode,
                             x_offset=x_offset, y_offset=y_offset,
                             lift=lift, tableHeight=tableHeight,
                             target_label="move_piece_to_source")
        

    def start_next_arm_task(self, target_vec, robot_node, x_offset=0.0, y_offset=0.0, lift=0.30, tableHeight=0.40, target_label="move_piece"):
            self.current_subtask = ArmMovementTask(
                target_vec,
                robot_node,
                x_offset=x_offset,
                y_offset=y_offset,
                lift=lift,
                tableHeight=tableHeight,
                target_label=target_label,
            )    

    def step(self):
        if self.done:
            return

        # Handle each state
        if self.state == "moving_to_source":
            self.current_subtask.step()
            if self.current_subtask.done:
                self.state = "picking_up_piece"
            return

        if self.state == "picking_up_piece":
            grab_piece(self.targetObject, self.robotNode)
            self.state = "moving_to_target"
            self.start_next_arm_task(self.destination, self.robotNode, target_label="move_piece_to_destination")
            return

        if self.state == "moving_to_target":
            self.current_subtask.step()
            if self.current_subtask.done:
                self.state = "dropping_piece"
            return

        if self.state == "dropping_piece":
            drop_piece()
            self.done = True
            self.current_subtask = None
            return
