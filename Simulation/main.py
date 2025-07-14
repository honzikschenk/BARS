# Run mjpython main.py

import time
import json

import Utils

import mujoco  # type: ignore

m = mujoco.MjModel.from_xml_path("./bars.xml")
data = mujoco.MjData(m)

state = 0

with mujoco.viewer.launch_passive(m, data) as viewer:
    # Load default positions from a JSON file
    with open("positions.json", "r") as f:
        positions = json.load(f)

    Utils.move_to_position(m, data, "standing", positions)

    with viewer.lock():
        # viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = 0
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = 0

    # m.opt.timestep = 0.0001

    time_since_last_step = time.time()
    time_since_start = time.time()

    time_since_last_state = time.time()

    while viewer.is_running():
        step_start = time.time()

        # # Add any slower periodic tasks here
        # if(time.time() - time_since_last_step > 0.1):
        #   time_since_last_step = time.time()

        # if state == 0 and time.time() - time_since_last_state > 2.0:
        #     state = 1
        #     time_since_last_state = time.time()
        #     print("Switching to state 1")

        #     Utils.move_to_position(m, data, "left_weight", positions)
            
        # elif state == 1 and time.time() - time_since_last_state > 3.0:
        #     state = 0
        #     time_since_last_state = time.time()
        #     print("Switching to state 2")

        #     Utils.move_to_position(m, data, "standing", positions)

        # Check for reseting sim if robot has fallen
        if Utils.get_body_position_global(m, data, "Pelvis")[2] < 0.3:
            time_lasted = time.time() - time_since_start

            # Reset sim and reward clock
            mujoco.mj_resetData(m, data)
            time_since_start = time.time()

            # Set the joints to their default positions again
            Utils.move_to_position(m, data, "standing", positions)


        # mj_step can be replaced with code that also evaluates
        # a policy and applies a control signal before stepping the physics.
        mujoco.mj_step(m, data)

        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
