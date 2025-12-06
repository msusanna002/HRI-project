
import math
import numpy as np

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

def robot_to_world(robot_xyz, robot_node):
    """
    Convert a point from robot base_link coordinates back to Webots world coordinates.
    """
    base_translation_field = robot_node.getField("translation")
    base_rotation_field    = robot_node.getField("rotation")

    base_pos = np.array(base_translation_field.getSFVec3f())   # p_B^W
    ax, ay, az, angle = base_rotation_field.getSFRotation()

    # Rotation from base frame to world frame
    R_WB = axis_angle_to_rotation_matrix([ax, ay, az], angle)

    p_B = np.array(robot_xyz)
    p_W = base_pos + R_WB.dot(p_B)
    return [float(p_W[0]), float(p_W[1]), float(p_W[2])]
