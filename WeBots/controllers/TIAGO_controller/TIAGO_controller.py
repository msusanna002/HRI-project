from controller import Supervisor, Keyboard, Camera, RangeFinder
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import math
import os
import numpy as np
import scene_objects as scene
import config as cfg
from keyboard_utils import check_keyboard
from robot_setup import create_chain, setup_motors_and_sensors
from scene_objects import init_scene_objects
from arm_motion_utils import init_ik, update_held_piece
from demo_utils import move_arm_to_all_pieces
from head_motion_utils import make_head_look_at_target
import arm_motion_utils



robot = Supervisor()
time_step = int(robot.getBasicTimeStep())

HERE = os.path.dirname(__file__)
URDF_PATH = os.path.join(HERE, "tiago_urdf.urdf")

# If the URDF doesn't exist yet, create it once
if not os.path.exists(URDF_PATH):
    print("URDF not found, creating:", URDF_PATH)
    with open(URDF_PATH, "w") as f:
        f.write(robot.getUrdf())

# IK chain
my_chain = create_chain("tiago_urdf.urdf")

# Enable motors and sensors
robot_parts, arm_sensors, head_pan_sensor, head_tilt_sensor = \
    setup_motors_and_sensors(robot, time_step)

# Enable RGB + depth cameras (devices, not supervisor nodes)
rgb = robot.getDevice("Astra rgb")
rgb.enable(time_step)
depth = robot.getDevice("Astra depth")
depth.enable(time_step)

# Get the robot node (this robot, as a Pose node)
robot_node = robot.getSelf()
if robot_node is None:
    print("Error: getSelf() returned None, something is wrong with the controller binding.")

init_scene_objects(robot)
init_ik(my_chain, arm_sensors, cfg.JOINT_NAMES, robot_parts, robot, time_step)

#set viewopoint to initial config
vp_pos_field = scene.viewpoint.getField("position")
vp_rot_field = scene.viewpoint.getField("orientation")
vp_pos_field.setSFVec3f(cfg.init_viewpoint_coord)
vp_rot_field.setSFRotation(cfg.init_viewpoint_rotation) 

# Enable keyboard
keyboard = Keyboard()
keyboard.enable(time_step) 

initial_time = robot.getTime()

print("Use arrow keys to drive. Press SPACE to turn head toward the camera.")

# move_arm_to_all_pieces(robot_node, x_offset=0.3, y_offset=0.43)
scene.current_object = scene.square_red
current_arm_task = None
# Main loop
while robot.step(time_step) != -1:
        # 1. Read keyboard and maybe start a new task
    new_task = check_keyboard(robot_parts, keyboard, robot_node)

    # If a new task was requested and nothing is running, start it
    if new_task is not None and current_arm_task is None:
        current_arm_task = new_task

    # 2. Advance the current arm task one step
    if current_arm_task is not None:
        current_arm_task.step()
        if current_arm_task.done:
            current_arm_task = None

    # 3. Always keep held piece glued to the hand
    arm_motion_utils.update_held_piece(robot_node)
    make_head_look_at_target(robot_parts, robot_node, scene.current_object)

