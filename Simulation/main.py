# Run mjpython main.py

import time
import json

import mujoco
import mujoco.viewer

m = mujoco.MjModel.from_xml_path('./bars.xml')
data = mujoco.MjData(m)

def set_joint_position(model, data, joint_name, desired_position):
    """
    Set the position of a joint in the Mujoco model with PID.
    
    Args:
        model: The Mujoco model.
        data: The Mujoco data.
        joint_name: The name of the joint to set.
        desired_position: The desired position for the joint.
    """
    # Get the joint index from the model
    joint_index = model.actuator(joint_name).id

    data.ctrl[joint_index] = desired_position



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

  crouch_value = 0
  going_up = False
  time_since_last_step = time.time()

  while viewer.is_running():
    step_start = time.time()

    if(time.time() - time_since_last_step > 0.1):
      time_since_last_step = time.time()
      if(going_up):
        crouch_value -= 0.1
        if(crouch_value <= 0):
          going_up = False
      else:
        crouch_value += 0.1
        if(crouch_value >= 1.5):
          going_up = True

      # Set the position of the left and right ankle, knee, and hip joints
      set_joint_position(m, data, 'left_ankle', {True: 0.5, False: 0.5}[crouch_value <= 1] * crouch_value + 0.2)
      set_joint_position(m, data, 'left_knee', {True: 1.0, False: 1.0}[crouch_value <= 1] * crouch_value + 0.1)
      set_joint_position(m, data, 'left_pitch_hip', -0.5 * crouch_value - 0.2)

      set_joint_position(m, data, 'right_ankle', {True: 0.5, False: 0.5}[crouch_value <= 1] * crouch_value + 0.2)
      set_joint_position(m, data, 'right_knee', {True: 1.0, False: 1.0}[crouch_value <= 1] * crouch_value + 0.1)
      set_joint_position(m, data, 'right_pitch_hip', -0.5 * crouch_value - 0.2)

    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    mujoco.mj_step(m, data)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)
