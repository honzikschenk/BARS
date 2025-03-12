# Run mjpython main.py

import time
import json

import mujoco
import mujoco.viewer

m = mujoco.MjModel.from_xml_path('./bars.xml')
d = mujoco.MjData(m)

with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()

  # Set a joint to a specific position
  # Load default positions from a JSON file
  with open('default_positions.json', 'r') as f:
    default_positions = json.load(f)

  # Set joints to their default positions
  for joint_name, desired_position in default_positions.items():
    joint_qpos_addr = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    d.qpos[joint_qpos_addr] = desired_position

  while viewer.is_running() and time.time() - start < 30:
    step_start = time.time()

    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    mujoco.mj_step(m, d)

    # Example modification of a viewer option: toggle contact points every two seconds.
    with viewer.lock():
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)
