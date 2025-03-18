# Run mjpython main.py

import time
import json

import mujoco
import mujoco.viewer

m = mujoco.MjModel.from_xml_path('./bars.xml')
data = mujoco.MjData(m)

p = 1
i = 0
d = 0

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
    # joint_index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    joint_index = model.actuator(joint_name).id

    data.ctrl[joint_index] = desired_position
    
    # # Get the current position of the joint
    # current_position = data.qpos[joint_index]
    
    # # Calculate the error
    # error = desired_position - current_position
    
    # # PID control (simplified)
    # global p, i, d
    # p_term = p * error
    # i_term = i * (error + data.qvel[joint_index])
    # d_term = d * (error - data.qvel[joint_index])
    
    # # Set the control signal for the joint
    # data.ctrl[joint_index] = p_term + i_term + d_term
    # # data.qpos[joint_qpos_addr] = desired_position

with mujoco.viewer.launch_passive(m, data) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()

  # Set a joint to a specific position
  # Load default positions from a JSON file
  with open('default_positions.json', 'r') as f:
    default_positions = json.load(f)

  # Set joints to their default positions
  for joint_name, desired_position in default_positions.items():
    set_joint_positions(m, data, joint_name, desired_position)

  while viewer.is_running() and time.time() - start < 30:
    step_start = time.time()

    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    mujoco.mj_step(m, data)

    # Example modification of a viewer option: toggle contact points every two seconds.
    with viewer.lock():
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = 0
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = 0

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)

    # for joint_name, desired_position in default_positions.items():
    #   set_joint_positions(m, data, joint_name, desired_position)
