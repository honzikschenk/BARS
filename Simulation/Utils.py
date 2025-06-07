import mujoco # type: ignore

def set_joint_position(model, data, joint_name, desired_position):
    """
    Set the position of a joint in the Mujoco model with internal PID controller.
    
    Args:
        model: The Mujoco model.
        data: The Mujoco data.
        joint_name: The name of the joint to set.
        desired_position: The desired position for the joint.
    """
    # Get the joint index from the model
    joint_index = model.actuator(joint_name).id

    data.ctrl[joint_index] = desired_position

def get_joint_position(model, data, joint_name):
    """
    Get the current position of a joint in the Mujoco model.
    
    Args:
        model: The Mujoco model.
        data: The Mujoco data.
        joint_name: The name of the joint to get.
    
    Returns:
        The current position of the joint.
    """
    # Get the joint index from the model
    joint_index = model.joint(joint_name).id

    return data.qpos[joint_index]

def get_body_position_global(model, data, body_name):
    """
    Get the global position of a body in the Mujoco model.
    Note: The y-axis is flipped (forward is negative y).
    
    Args:
        model: The Mujoco model.
        data: The Mujoco data.
        body_name: The name of the body to get.
    
    Returns:
        The global position of the body.
    """
    # Get the body index from the model
    body_index = model.body(body_name).id

    return data.xpos[body_index]

def move_to_position(model, data, desired_pose, positions):
    """
    Move a body to a desired position in the Mujoco model.
    
    Args:
        model: The Mujoco model.
        data: The Mujoco data.
        desired_pose: The desired global position for the entire robot.
    """
    desired_positions = positions[desired_pose]
    for joint_name, desired_position in desired_positions.items():
        set_joint_position(model, data, joint_name, desired_position)
