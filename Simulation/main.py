# Run mjpython main.py

import time
import json

import mujoco
import threading
import mujoco.viewer

m = mujoco.MjModel.from_xml_path('./bars.xml')
data = mujoco.MjData(m)

def set_joint_positions(model, data, joint_name, desired_position):
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

def user_input_handler():
  global default_positions

  crouch_value = 0
  going_up = False
  while True:
    if(going_up):
      crouch_value -= 0.1
      if(crouch_value <= 0):
        going_up = False
    else:
      crouch_value += 0.1
      if(crouch_value >= 1.5):
        going_up = True

    # Set the position of the left and right ankle, knee, and hip joints
    default_positions['left_ankle'] = {True: 0.5, False: 0.5}[crouch_value <= 1] * crouch_value + 0.2
    default_positions['left_knee'] = {True: 1.0, False: 1.0}[crouch_value <= 1] * crouch_value + 0.1
    default_positions['left_pitch_hip'] = -0.5 * crouch_value - 0.2

    default_positions['right_ankle'] = {True: 0.5, False: 0.5}[crouch_value <= 1] * crouch_value + 0.2
    default_positions['right_knee'] = {True: 1.0, False: 1.0}[crouch_value <= 1] * crouch_value + 0.1
    default_positions['right_pitch_hip'] = -0.5 * crouch_value - 0.2

    time.sleep(0.1)
    
    # user_input = input("Enter a value between 0 (standing) and 1 (crouching), or 'exit': ")
    # if user_input.lower() == 'exit':
    #   break
    # try:
    #   crouch_value = float(user_input)
    #   if 0 <= crouch_value <= 2:
    #     # Map the crouch value to joint positions
    #     default_positions['left_ankle'] = {True: 0.5, False: 0.5}[crouch_value <= 1] * crouch_value + 0.2
    #     default_positions['left_knee'] = {True: 1.0, False: 1.0}[crouch_value <= 1] * crouch_value + 0.1
    #     default_positions['left_pitch_hip'] = -0.5 * crouch_value - 0.2

    #     default_positions['right_ankle'] = {True: 0.5, False: 0.5}[crouch_value <= 1] * crouch_value + 0.2
    #     default_positions['right_knee'] = {True: 1.0, False: 1.0}[crouch_value <= 1] * crouch_value + 0.1
    #     default_positions['right_pitch_hip'] = -0.5 * crouch_value - 0.2
    #   else:
    #     print("Value out of range. Please enter a value between 0 and 1.")
    # except ValueError:
    #   print("Invalid input. Please enter a float between 0 and 1.")

with mujoco.viewer.launch_passive(m, data) as viewer:
  # Set a joint to a specific position
  # Load default positions from a JSON file
  with open('default_positions.json', 'r') as f:
    default_positions = json.load(f)

  # Set joints to their default positions
  for joint_name, desired_position in default_positions.items():
    set_joint_positions(m, data, joint_name, desired_position)

  # Start the user input handler in a separate thread
  input_thread = threading.Thread(target=user_input_handler, daemon=True)
  input_thread.start()

  # Example modification of a viewer option: toggle contact points every two seconds.
  with viewer.lock():
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = 0
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = 0

  while viewer.is_running():
    step_start = time.time()

    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    mujoco.mj_step(m, data)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)

    for joint_name, desired_position in default_positions.items():
      set_joint_positions(m, data, joint_name, desired_position)
