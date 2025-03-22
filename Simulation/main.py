# Run mjpython main.py

import time
import json

import mujoco

m = mujoco.MjModel.from_xml_path('./bars.xml')
data = mujoco.MjData(m)

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



with mujoco.viewer.launch_passive(m, data) as viewer:
  # Load default positions from a JSON file
  with open('default_positions.json', 'r') as f:
    default_positions = json.load(f)

  # Set joints to their default positions
  for joint_name, desired_position in default_positions.items():
    set_joint_position(m, data, joint_name, desired_position)

  with viewer.lock():
    # viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = 0
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = 0

  time_since_last_step = time.time()

  while viewer.is_running():
    step_start = time.time()

    # Add any slower periodic tasks here
    if(time.time() - time_since_last_step > 0.1):
      time_since_last_step = time.time()

      print(get_body_position_global(m, data, 'Pelvis'))

    # Check for reseting sim if robot has fallen
    if(get_body_position_global(m, data, 'Pelvis')[2] < 0.3):
      mujoco.mj_resetData(m, data)

      # Set the joints to their default positions again
      for joint_name, desired_position in default_positions.items():
        set_joint_position(m, data, joint_name, desired_position)

    # Add instant control tasks here

    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    mujoco.mj_step(m, data)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)
