import math

def normalize_angle(a):
    # keep angle in [-pi, pi]
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def make_head_look_at_target(robot_parts, robot_node, target_node):
    # Fields on robot and viewpoint
    base_translation_field = robot_node.getField("translation")
    base_rotation_field    = robot_node.getField("rotation")

    if target_node is None:
        print("Target node is None, cannot look at target.")
        return
    # if target is a node, get its position
    if target_node.getField("translation") is not None:
        target_pos = target_node.getField("translation").getSFVec3f()
    #if tarhet is viewpoint
    elif target_node.getField("position") is not None:
        target_pos = target_node.getField("position").getSFVec3f()
    
    if base_translation_field is None or base_rotation_field is None:
        print("Could not get translation/rotation/position fields.")
        return

    # Robot base pose
    base_pos = base_translation_field.getSFVec3f()   # [x, y, z]
    base_rot = base_rotation_field.getSFRotation()   # [ax, ay, az, angle]
          # [x, y, z]

    head_pos = [
    base_pos[0],
    base_pos[1],
    base_pos[2] + 0.8
]

    # --- Extract body yaw from axis-angle (assuming planar, axis ~ [0,0,±1]) ---
    ax, ay, az, angle = base_rot
    # default if axis is weird: assume yaw = 0
    if abs(az) > 1e-3:
        body_yaw = angle * (1.0 if az >= 0.0 else -1.0)
    else:
        body_yaw = 0.0

 
    # Vector from base to camera (world frame)
    dx = target_pos[0] - head_pos[0]
    dy = -target_pos[1] - head_pos[1]
    dz = target_pos[2] - head_pos[2]

    # Guard against degenerate case 
    if dx == 0 and dy == 0:
        print("Head target direction is undefined (camera exactly above/below in XY).")
        return

    # --- World yaw to camera and relative yaw for the head ---
    world_yaw = math.atan2(dy, dx)           # yaw of vector base→camera in world frame
    rel_yaw   = normalize_angle(world_yaw + body_yaw)  # yaw relative to body orientation


    # vertical angle
    dist_xy = math.sqrt(dx * dx + dy * dy)
    pitch   = math.atan2(dz, dist_xy)        # positive if camera is above

    # Desired joint angles (head joints)
    # pan = relative yaw, tilt = opposite sign of pitch (depends on joint convention)
    desired_pan  = rel_yaw
    desired_tilt = -pitch

    head_tilt = robot_parts[0]  # "head_2_joint"
    head_pan  = robot_parts[1]  # "head_1_joint"

    # --- Get joint limits from Webots ---
    pan_min  = head_pan.getMinPosition()
    pan_max  = head_pan.getMaxPosition()
    tilt_min = head_tilt.getMinPosition()
    tilt_max = head_tilt.getMaxPosition()

    BIG = 1e6
    if abs(pan_min) > BIG:  pan_min = -BIG
    if abs(pan_max) > BIG:  pan_max = BIG
    if abs(tilt_min) > BIG: tilt_min = -BIG
    if abs(tilt_max) > BIG: tilt_max = BIG

    # --- Clamp to limits ---
    clamped_yaw  = max(pan_min,  min(pan_max,  desired_pan))
    clamped_tilt = max(tilt_min, min(tilt_max, desired_tilt))

    if abs(clamped_yaw - desired_pan) > 1e-3 or abs(clamped_tilt - desired_tilt) > 1e-3:
        print(
            "Head target out of range. "
            f"Requested pan={desired_pan:.2f}, tilt={desired_tilt:.2f} "
            f"→ clamped to pan={clamped_yaw:.2f}, tilt={clamped_tilt:.2f}"
        )

    # Apply the (possibly clamped) positions
    head_pan.setPosition(-clamped_yaw)
    head_tilt.setPosition(-clamped_tilt)


    