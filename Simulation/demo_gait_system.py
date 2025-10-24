#!/usr/bin/env python3
"""
Demo script for the gait generation system that doesn't require a display.
This shows how the gait generation system works without the MuJoCo viewer.
"""

import time
import json
import numpy as np

import Utils
from CenterOfMass import CenterOfMassCalculator, add_com_visualization_to_model
from GaitGenerator import QuasiStaticGaitGenerator, GaitLibrary
from InteractiveGaitTuner import InteractiveGaitTuner

import mujoco  # type: ignore

def test_gait_generation():
    """Test the gait generation system without visualization."""
    
    print("=== GAIT GENERATION SYSTEM DEMO ===")
    print()
    
    # Create enhanced model with COM visualization
    print("1. Setting up enhanced robot model...")
    enhanced_xml = add_com_visualization_to_model("./bars.xml")
    with open("./bars_enhanced.xml", "w") as f:
        f.write(enhanced_xml)
    
    # Load the enhanced model
    m = mujoco.MjModel.from_xml_path("./bars_enhanced.xml")
    data = mujoco.MjData(m)
    
    print("   ✓ Model loaded with COM visualization")
    
    # Initialize the gait system
    print("2. Initializing gait generation system...")
    com_calculator = CenterOfMassCalculator(m, data)
    gait_generator = QuasiStaticGaitGenerator(m, data, com_calculator)
    gait_tuner = InteractiveGaitTuner(m, data, gait_generator, com_calculator)
    
    print("   ✓ Center of mass calculator initialized")
    print("   ✓ Quasi-static gait generator initialized")
    print("   ✓ Interactive gait tuner initialized")
    
    # Test initial standing position
    print("3. Testing initial configuration...")
    with open("positions.json", "r") as f:
        positions = json.load(f)
    
    Utils.move_to_position(m, data, "standing", positions)
    mujoco.mj_step(m, data)
    
    # Test COM calculation
    com_pos = com_calculator.calculate_com()
    print(f"   ✓ Initial COM position: ({com_pos[0]:.3f}, {com_pos[1]:.3f}, {com_pos[2]:.3f})")
    
    # Test stability analysis
    is_stable = com_calculator.is_statically_stable()
    stability_margin = com_calculator.get_stability_margin()
    print(f"   ✓ Initial stability: {'Stable' if is_stable else 'Unstable'} (margin: {stability_margin:.3f}m)")
    
    # Test gait parameter adjustment
    print("4. Testing gait parameter adjustment...")
    original_params = {
        "step_length": gait_generator.step_length,
        "step_height": gait_generator.step_height,
        "phase_duration": gait_generator.phase_duration
    }
    print(f"   Original parameters: {original_params}")
    
    # Update parameters
    gait_generator.update_gait_parameters(
        step_length=0.12,
        step_height=0.04,
        phase_duration=1.8
    )
    print(f"   ✓ Updated step_length: {gait_generator.step_length}")
    print(f"   ✓ Updated step_height: {gait_generator.step_height}")
    print(f"   ✓ Updated phase_duration: {gait_generator.phase_duration}")
    
    # Test preset gaits
    print("5. Testing preset gait configurations...")
    presets = {
        "Slow": GaitLibrary.get_walking_gait_slow(),
        "Normal": GaitLibrary.get_walking_gait_normal(),
        "Fast": GaitLibrary.get_walking_gait_fast(),
        "Wide": GaitLibrary.get_wide_stance_gait()
    }
    
    for name, preset in presets.items():
        gait_generator.update_gait_parameters(**preset)
        print(f"   ✓ {name} gait: step_length={preset['step_length']}, duration={preset['phase_duration']}")
    
    # Test gait cycle simulation
    print("6. Testing gait cycle simulation...")
    gait_generator.update_gait_parameters(**GaitLibrary.get_walking_gait_normal())
    
    # Start walking
    simulation_start = time.time()
    gait_generator.start_walking(0.0)
    
    print("   Walking gait phases:")
    sim_duration = 15.0  # Simulate for 15 seconds
    dt = 0.1  # 100ms steps
    
    last_phase = -1
    phase_changes = 0
    stability_checks = []
    
    for step in range(int(sim_duration / dt)):
        sim_time = step * dt
        
        # Get current gait phase
        phase_info = gait_generator.get_current_phase_info(sim_time)
        
        # Check for phase changes
        if phase_info["phase_number"] != last_phase:
            print(f"     Time {sim_time:.1f}s: Phase {phase_info['phase_number']} - {phase_info['phase']}")
            last_phase = phase_info["phase_number"]
            phase_changes += 1
        
        # Get joint positions and apply them
        joint_positions = gait_generator.get_current_pose(sim_time)
        gait_tuner.apply_joint_positions(joint_positions)
        
        # Step simulation
        mujoco.mj_step(m, data)
        
        # Check stability every second
        if step % 10 == 0:  # Every 1 second
            com_info = com_calculator.get_com_info()
            stability_checks.append({
                "time": sim_time,
                "stable": com_info["is_stable"],
                "margin": com_info["stability_margin"],
                "com_pos": com_info["position"].copy()
            })
    
    print(f"   ✓ Completed {phase_changes} phase transitions")
    
    # Analyze stability during walking
    stable_count = sum(1 for check in stability_checks if check["stable"])
    stability_percentage = (stable_count / len(stability_checks)) * 100 if stability_checks else 0
    
    print(f"   ✓ Stability analysis: {stability_percentage:.1f}% stable over {len(stability_checks)} checks")
    
    # Test COM trajectory
    print("7. Testing center of mass tracking...")
    com_positions = [check["com_pos"] for check in stability_checks]
    if com_positions:
        com_array = np.array(com_positions)
        com_range_x = np.max(com_array[:, 0]) - np.min(com_array[:, 0])
        com_range_y = np.max(com_array[:, 1]) - np.min(com_array[:, 1])
        com_range_z = np.max(com_array[:, 2]) - np.min(com_array[:, 2])
        
        print(f"   ✓ COM movement range: X={com_range_x:.3f}m, Y={com_range_y:.3f}m, Z={com_range_z:.3f}m")
    
    # Test saving and loading
    print("8. Testing gait save/load functionality...")
    test_filename = "test_gait.json"
    gait_generator.save_gait_to_file(test_filename)
    print(f"   ✓ Saved gait to {test_filename}")
    
    # Modify parameters and then load
    gait_generator.update_gait_parameters(step_length=0.99)  # Unusual value
    print(f"   Modified step_length to: {gait_generator.step_length}")
    
    gait_generator.load_gait_from_file(test_filename)
    print(f"   ✓ Loaded gait, step_length restored to: {gait_generator.step_length}")
    
    # Test interactive interface info
    print("9. Testing interactive interface...")
    gait_tuner.selected_param = 0  # Select first parameter
    display_info = gait_tuner.get_display_info(10.0)
    info_lines = len(display_info.split('\n'))
    print(f"   ✓ Generated display info with {info_lines} lines")
    
    # Test parameter adjustment simulation
    original_step_length = gait_generator.step_length
    gait_tuner.handle_keyboard_input("1")  # Select step_length
    gait_tuner.handle_keyboard_input("up") # Increase it
    new_step_length = gait_generator.step_length
    print(f"   ✓ Parameter adjustment: {original_step_length:.3f} → {new_step_length:.3f}")
    
    print()
    print("=== DEMO COMPLETED SUCCESSFULLY ===")
    print()
    print("Key Features Demonstrated:")
    print("✓ Enhanced robot model with COM visualization")
    print("✓ Quasi-static gait generation with configurable parameters")
    print("✓ Real-time center of mass calculation and stability analysis")
    print("✓ Multiple preset gait configurations")
    print("✓ Smooth phase transitions and joint interpolation")
    print("✓ Interactive parameter tuning interface")
    print("✓ Gait save/load functionality")
    print("✓ Comprehensive stability monitoring")
    
    print()
    print("Ready for interactive use! Run 'mjpython main.py' with a display to use the full interface.")
    
    return True

if __name__ == "__main__":
    try:
        test_gait_generation()
    except Exception as e:
        print(f"Error during demo: {e}")
        import traceback
        traceback.print_exc()
        exit(1)