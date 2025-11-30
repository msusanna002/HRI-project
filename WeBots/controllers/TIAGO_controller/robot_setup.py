# robot_setup.py

from ikpy.chain import Chain
import config as cfg


def create_chain(urdf_path="tiago_urdf.urdf"):
    """Create and configure the IK chain."""
    my_chain = Chain.from_urdf_file(
        urdf_path,
        base_elements=cfg.BASE_ELEMENTS,
        last_link_vector=cfg.LAST_LINK_VECTOR,
    )

    # Activate only arm joints, then explicitly disable torso lift if desired
    for i, link in enumerate(my_chain.links):
        active = link.name in cfg.ARM_JOINT_NAMES
        my_chain.active_links_mask[i] = active

    # Optionally disable torso lift joint entirely
    for i, link in enumerate(my_chain.links):
        if link.name == "torso_lift_joint":
            my_chain.active_links_mask[i] = False

    return my_chain


def setup_motors_and_sensors(robot, time_step):
    """
    Configure motors and arm sensors.
    Returns:
        robot_parts  - list of motors in cfg.JOINT_NAMES order
        arm_sensors  - dict name -> position sensor
        head_pan_sensor, head_tilt_sensor
    """
    robot_parts = []
    arm_sensors = {}

    # Motors
    for i, name in enumerate(cfg.JOINT_NAMES):
        motor = robot.getDevice(name)
        robot_parts.append(motor)

        # Set initial velocity and position
        motor.setVelocity(motor.getMaxVelocity() / 2.0)
        motor.setPosition(cfg.JOINT_TARGET_POS[i])

        # Attach arm joint sensors
        if name.startswith("arm_") or name == "torso_lift_joint":
            ps = motor.getPositionSensor()
            ps.enable(time_step)
            arm_sensors[name] = ps

    # Head sensors
    head_pan_sensor = robot.getDevice("head_1_joint_sensor")
    head_tilt_sensor = robot.getDevice("head_2_joint_sensor")
    head_pan_sensor.enable(time_step)
    head_tilt_sensor.enable(time_step)

    return robot_parts, arm_sensors, head_pan_sensor, head_tilt_sensor
