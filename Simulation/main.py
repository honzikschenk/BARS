# Run mjpython main.py

import time
import json
import numpy as np

import ReinforcementModel
import Utils

import mujoco # type: ignore

m = mujoco.MjModel.from_xml_path('./bars.xml')
data = mujoco.MjData(m)

with mujoco.viewer.launch_passive(m, data) as viewer:
  # Load default positions from a JSON file
  with open('default_positions.json', 'r') as f:
    default_positions = json.load(f)

  # Set joints to their default positions
  for joint_name, desired_position in default_positions.items():
    Utils.set_joint_position(m, data, joint_name, desired_position)

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

      print(Utils.get_body_position_global(m, data, 'Pelvis'))

    # Check for reseting sim if robot has fallen
    if(Utils.get_body_position_global(m, data, 'Pelvis')[2] < 0.3):
      mujoco.mj_resetData(m, data)

      # Set the joints to their default positions again
      for joint_name, desired_position in default_positions.items():
        Utils.set_joint_position(m, data, joint_name, desired_position)

    # Add instant control tasks here
    # Get and apply a random action
    action = ReinforcementModel.get_random_action()
    ReinforcementModel.apply_action(m, data, action)

    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    mujoco.mj_step(m, data)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)
