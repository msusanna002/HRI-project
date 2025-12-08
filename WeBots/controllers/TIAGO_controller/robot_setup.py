# robot_setup.py

from ikpy.chain import Chain
import config as cfg


def create_chain(urdf_path="tiago_urdf.urdf"):
    """Create and configure the IK chain."""

    # Load the IK chain from the URDF file
    my_chain = Chain.from_urdf_file(
        urdf_path,
        base_elements=cfg.BASE_ELEMENTS,
        last_link_vector=cfg.LAST_LINK_VECTOR,
    )

    # Activate only arm joints
    for i, link in enumerate(my_chain.links):
        # create a mask where only arm joints are active
        active = link.name in cfg.ARM_JOINT_NAMES
        my_chain.active_links_mask[i] = active

    # Explicitly disable torso lift joint
    for i, link in enumerate(my_chain.links):
        if link.name == "torso_lift_joint":
            my_chain.active_links_mask[i] = False

    print ("IK chain created with active links:", my_chain.active_links_mask)

    return my_chain


def setup_motors_and_sensors(robot, time_step):
    robot_parts = []
    arm_sensors = {}
    head_sensors = {}

    # for each motor in JOINT_NAMES set it up 
    for i, name in enumerate(cfg.JOINT_NAMES):
        motor = robot.getDevice(name)

        # store motor name in robot_parts list
        robot_parts.append(motor)

        if name.startswith("wheel_"):
            # Wheels: velocity mode, but start stopped
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)
        else:
            # All other joints: position control to initial pose
            motor.setVelocity(motor.getMaxVelocity() / 2.0)
            motor.setPosition(cfg.JOINT_TARGET_POS[i])

            # enable position sensors for arm joints and head joints
            if name.startswith("arm_") or name.startswith("head_"):
                ps = motor.getPositionSensor()
                ps.enable(time_step)
                if name.startswith("arm_"):
                    arm_sensors[name] = ps
                if name.startswith("head_"):
                    head_sensors[name] = ps

    return robot_parts, arm_sensors, head_sensors

