#!/usr/bin/env python3
"""
Text-based interactive gait tuner that works without display.
Shows real-time gait information in the terminal.
"""

import time
import sys
import os
import mujoco
from CenterOfMass import CenterOfMassCalculator, add_com_visualization_to_model
from GaitGenerator import QuasiStaticGaitGenerator, GaitLibrary
from InteractiveGaitTuner import InteractiveGaitTuner
import Utils

def clear_screen():
    """Clear the terminal screen."""
    os.system('cls' if os.name == 'nt' else 'clear')

def print_header():
    """Print the application header."""
    print("=" * 60)
    print("           BARS GAIT GENERATION SYSTEM")
    print("=" * 60)

def print_controls():
    """Print the control instructions."""
    print("\nControls:")
    print("  1-5: Select parameter | +/-: Adjust | Space: Start/Stop")
    print("  q/w/e/t: Presets | h: Toggle help | Ctrl+C: Exit")
    print("-" * 60)

def run_text_interface():
    """Run the text-based interface."""
    
    # Setup
    print("Initializing gait system...")
    
    # Create enhanced model
    enhanced_xml = add_com_visualization_to_model("./bars.xml")
    with open("./bars_enhanced.xml", "w") as f:
        f.write(enhanced_xml)
    
    # Load model and initialize systems
    m = mujoco.MjModel.from_xml_path("./bars_enhanced.xml")
    data = mujoco.MjData(m)
    
    com_calculator = CenterOfMassCalculator(m, data)
    gait_generator = QuasiStaticGaitGenerator(m, data, com_calculator)
    gait_tuner = InteractiveGaitTuner(m, data, gait_generator, com_calculator)
    
    # Load initial positions
    with open("positions.json", "r") as f:
        positions = json.load(f)
    
    Utils.move_to_position(m, data, "standing", positions)
    
    simulation_start_time = time.time()
    last_reset_time = time.time()
    show_help = True
    
    print("System initialized! Starting simulation...")
    time.sleep(1)
    
    try:
        while True:
            current_time = time.time() - simulation_start_time
            
            # Update gait system
            gait_tuner.update(current_time)
            
            # Check for robot falling
            pelvis_height = Utils.get_body_position_global(m, data, "Pelvis")[2]
            if pelvis_height < 0.3:
                time_lasted = time.time() - last_reset_time
                gait_tuner.reset_robot_position()
                last_reset_time = time.time()
                simulation_start_time = time.time()
                current_time = 0
            
            # Step physics
            mujoco.mj_step(m, data)
            
            # Update display every 0.5 seconds
            if int(current_time * 2) != int((current_time - 0.01) * 2):
                clear_screen()
                print_header()
                
                if show_help:
                    print_controls()
                
                # Get current status
                phase_info = gait_generator.get_current_phase_info(current_time)
                stability_info = gait_generator.get_stability_info()
                com_info = com_calculator.get_com_info()
                
                # Display gait parameters
                print("\nGait Parameters:")
                params = ["step_length", "step_height", "step_width", "phase_duration", "shift_duration"]
                for i, param in enumerate(params):
                    value = getattr(gait_generator, param)
                    marker = ">" if i == gait_tuner.selected_param else " "
                    print(f"  {marker} {i+1}. {param}: {value:.3f}")
                
                # Display status
                print(f"\nGait Status: {'WALKING' if gait_generator.is_walking else 'STOPPED'}")
                if gait_generator.is_walking:
                    print(f"Phase: {phase_info['phase']} ({phase_info['progress']:.1%})")
                    print(f"Support: {phase_info['support_foot']}")
                
                # Display stability
                print(f"\nStability:")
                print(f"  Stable: {'YES' if stability_info['is_stable'] else 'NO'}")
                print(f"  Margin: {stability_info['stability_margin']:.3f}m")
                print(f"  Contacts: {stability_info['num_contacts']}")
                
                # Display COM
                pos = com_info['position']
                print(f"\nCenter of Mass: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
                
                # Display statistics
                stats = gait_tuner.get_stability_statistics()
                print(f"Stability: {stats['stability_percentage']:.1f}% over {stats['samples']} samples")
                
                print(f"\nSimulation Time: {current_time:.1f}s")
                print(f"Pelvis Height: {pelvis_height:.3f}m")
                
                # Show instructions
                print("\nPress keys to control (Note: May require Enter after each key):")
                print("Space=start/stop, 1-5=select param, +=increase, -=decrease")
                print("q=slow, w=normal, e=fast, t=wide, h=toggle help")
            
            # Simple time keeping
            time.sleep(0.01)  # 100 Hz update rate
            
    except KeyboardInterrupt:
        print("\n\nShutting down gait system...")
        print("Goodbye!")

def demo_mode():
    """Run a simple demo showing gait transitions."""
    
    print("=" * 60)
    print("         AUTOMATED GAIT DEMONSTRATION")
    print("=" * 60)
    
    # Setup (same as interactive mode)
    enhanced_xml = add_com_visualization_to_model("./bars.xml")
    with open("./bars_enhanced.xml", "w") as f:
        f.write(enhanced_xml)
    
    m = mujoco.MjModel.from_xml_path("./bars_enhanced.xml")
    data = mujoco.MjData(m)
    
    com_calculator = CenterOfMassCalculator(m, data)
    gait_generator = QuasiStaticGaitGenerator(m, data, com_calculator)
    gait_tuner = InteractiveGaitTuner(m, data, gait_generator, com_calculator)
    
    # Demo different presets
    presets = [
        ("Normal Gait", GaitLibrary.get_walking_gait_normal()),
        ("Slow Gait", GaitLibrary.get_walking_gait_slow()),
        ("Fast Gait", GaitLibrary.get_walking_gait_fast()),
        ("Wide Stance", GaitLibrary.get_wide_stance_gait())
    ]
    
    for preset_name, preset_params in presets:
        print(f"\nTesting {preset_name}...")
        gait_generator.update_gait_parameters(**preset_params)
        gait_generator.start_walking(0.0)
        
        # Run for a few seconds
        for step in range(200):  # 20 seconds
            sim_time = step * 0.1
            gait_tuner.update(sim_time)
            mujoco.mj_step(m, data)
            
            if step % 50 == 0:  # Every 5 seconds
                stability_info = gait_generator.get_stability_info()
                phase_info = gait_generator.get_current_phase_info(sim_time)
                print(f"  Time {sim_time:.1f}s: Phase={phase_info['phase']}, Stable={stability_info['is_stable']}")
        
        # Get final statistics
        stats = gait_tuner.get_stability_statistics()
        print(f"  Final stability: {stats['stability_percentage']:.1f}%")
        
        gait_generator.stop_walking()
        time.sleep(1)
    
    print("\nDemo completed!")

if __name__ == "__main__":
    import json
    
    if len(sys.argv) > 1 and sys.argv[1] == "demo":
        demo_mode()
    else:
        print("Text-based Gait Interface")
        print("Note: This interface shows real-time updates but keyboard input may be limited")
        print("For full interactivity, use the MuJoCo viewer version")
        print("Run with 'demo' argument for automated demonstration")
        print()
        
        try:
            run_text_interface()
        except ImportError as e:
            print(f"Missing dependency: {e}")
            print("Please install required packages: pip install mujoco numpy")
        except Exception as e:
            print(f"Error: {e}")
            print("Make sure you're in the Simulation directory with the required files")