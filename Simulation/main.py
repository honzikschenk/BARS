# Run python main.py
# Enhanced gait generation with interactive tuning

import time
import json
import numpy as np

import Utils
from CenterOfMass import CenterOfMassCalculator, add_com_visualization_to_model
from GaitGenerator import QuasiStaticGaitGenerator
from InteractiveGaitTuner import InteractiveGaitTuner

import mujoco  # type: ignore
import mujoco.viewer

# Create enhanced model with COM visualization
print("Setting up enhanced robot model with COM visualization...")
enhanced_xml = add_com_visualization_to_model("./bars.xml")
with open("./bars_enhanced.xml", "w") as f:
    f.write(enhanced_xml)

# Load the enhanced model
m = mujoco.MjModel.from_xml_path("./bars_enhanced.xml")
data = mujoco.MjData(m)

# Initialize the new gait system
print("Initializing gait generation system...")
com_calculator = CenterOfMassCalculator(m, data)
gait_generator = QuasiStaticGaitGenerator(m, data, com_calculator)
gait_tuner = InteractiveGaitTuner(m, data, gait_generator, com_calculator)

# Simulation state
simulation_start_time = time.time()
last_reset_time = time.time()
last_display_update = time.time()
display_update_interval = 0.1  # Update display text every 100ms

print("Starting interactive gait simulation...")
print("Press 'H' in the viewer to show/hide help")

with mujoco.viewer.launch_passive(m, data) as viewer:
    # Load default positions from a JSON file  
    with open("positions.json", "r") as f:
        positions = json.load(f)

    # Set initial standing position
    Utils.move_to_position(m, data, "standing", positions)

    # Configure viewer settings
    with viewer.lock():
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = 1  # Show contact points
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = 1  # Show contact forces
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_COM] = 1           # Show center of mass

    # Key callback function for interactive controls
    def key_callback(keycode):
        """Handle keyboard input for gait tuning."""
        key_map = {
            32: "space",    # Space bar
            72: "h",        # H key
            83: "s",        # S key  
            67: "c",        # C key
            82: "r",        # R key
            49: "1",        # Number keys
            50: "2",
            51: "3", 
            52: "4",
            53: "5",
            81: "q",        # Preset keys
            87: "w",
            69: "e",
            84: "t",
            265: "up",      # Arrow keys
            264: "down",
            262: "right",
            263: "left",
            43: "+",        # Plus/minus
            45: "-"
        }
        
        key_name = key_map.get(keycode, f"unknown_{keycode}")
        handled = gait_tuner.handle_keyboard_input(key_name)
        if handled:
            print(f"Handled key: {key_name}")

    # Set up key callback (this might need adjustment based on MuJoCo version)
    try:
        viewer.add_key_callback(key_callback)
    except AttributeError:
        print("Note: Key callbacks not available in this MuJoCo version")
        print("Use the following keys for control:")
        print("- Keys 1-5: Select gait parameter")
        print("- Up/Down arrows: Adjust parameter")  
        print("- Space: Start/stop walking")
        print("- Q/W/E/T: Load preset gaits")

    # Display text overlay
    display_text = ""

    while viewer.is_running():
        step_start = time.time()
        current_time = time.time() - simulation_start_time

        # Update gait tuner (this handles gait generation and applies joint positions)
        gait_tuner.update(current_time)

        # Update COM visualization site position
        com_pos = com_calculator.calculate_com()
        try:
            # Try to update COM marker position if the site exists
            site_id = m.site("com_marker").id
            data.site_xpos[site_id] = com_pos
        except:
            pass  # Site doesn't exist or can't be updated

        # Check for robot falling and reset if needed
        pelvis_height = Utils.get_body_position_global(m, data, "Pelvis")[2]
        if pelvis_height < 0.3:
            time_lasted = time.time() - last_reset_time
            print(f"Robot fell after {time_lasted:.1f} seconds. Resetting...")

            # Reset the simulation
            gait_tuner.reset_robot_position()
            last_reset_time = time.time()
            simulation_start_time = time.time()  # Reset simulation time

        # Update display text periodically
        if time.time() - last_display_update > display_update_interval:
            display_text = gait_tuner.get_display_info(current_time)
            last_display_update = time.time()

        # Display the information text (if supported by viewer)
        try:
            with viewer.lock():
                # Set overlay text if available
                if hasattr(viewer, 'add_overlay'):
                    viewer.add_overlay(mujoco.mjtGridPos.mjGRID_TOPLEFT, "Gait Tuner", display_text)
        except:
            pass  # Overlay not supported

        # Step the physics simulation
        mujoco.mj_step(m, data)

        # Synchronize with viewer
        viewer.sync()

        # Time keeping for real-time simulation
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

print("Simulation ended.")
