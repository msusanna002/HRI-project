from head_motion_utils import make_head_look_at_target
from arm_motion_utils import ArmMovementTask
from task_utils import MovePieceTask
from controller import Keyboard
import scene_objects as scene
import config as cfg
from demo_utils import create_move_all_pieces_task

def check_keyboard(robot_parts, keyboard, robot_node):
    """
    Check the keyboard state and return an arm movement task if needed.
    """  
    global init_viewpoint, init_rotation

    arm_task = None

    key = keyboard.getKey()

    #reset viewpoint
    if key == ord('0'):
        vp_pos_field = scene.viewpoint.getField("position")
        vp_rot_field = scene.viewpoint.getField("orientation")
        vp_pos_field.setSFVec3f(cfg.init_viewpoint_coord)
        vp_rot_field.setSFRotation(cfg.init_viewpoint_rotation)

    # for testing and demo purposes: move specific pieces to their target positions
    elif key == ord('1'):
        print("Move red small triangle to its target position")
        arm_task = MovePieceTask(
            scene.small_triangle_red,          # the piece
            scene.small_triangle_red.getField("targetPos").getSFVec3f(),   # the target position
            robot_node
        ) 

    # for testing and demo purposes: move all pieces to their target positions
    if key == ord('M'):  # pick any key
        print("[keyboard] Starting MoveAllPiecesTask")
        return create_move_all_pieces_task(robot_node)


    return arm_task