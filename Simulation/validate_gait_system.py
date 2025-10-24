#!/usr/bin/env python3
"""
Simple validation test for the gait generation system.
This creates a minimal test to verify stability calculations work correctly.
"""

import numpy as np
import mujoco
from CenterOfMass import CenterOfMassCalculator
from GaitGenerator import QuasiStaticGaitGenerator
import Utils

def test_stability_calculation():
    """Test that stability calculations work correctly in different poses."""
    
    print("=== STABILITY CALCULATION VALIDATION ===")
    
    # Load model
    m = mujoco.MjModel.from_xml_path("./bars.xml")
    data = mujoco.MjData(m)
    
    # Initialize systems
    com_calculator = CenterOfMassCalculator(m, data)
    gait_generator = QuasiStaticGaitGenerator(m, data, com_calculator)
    
    # Test positions
    test_poses = {
        "standing": {
            "left_pitch_hip": -0.2, "left_roll_hip": 0.0, "left_yaw_hip": 0.0,
            "left_knee": 0.45, "left_ankle": 0.35,
            "right_pitch_hip": -0.2, "right_roll_hip": 0.0, "right_yaw_hip": 0.0,
            "right_knee": 0.45, "right_ankle": 0.35
        },
        "wide_stance": {
            "left_pitch_hip": -0.2, "left_roll_hip": -0.5, "left_yaw_hip": 0.0,
            "left_knee": 0.45, "left_ankle": 0.35,
            "right_pitch_hip": -0.2, "right_roll_hip": 0.5, "right_yaw_hip": 0.0,
            "right_knee": 0.45, "right_ankle": 0.35
        },
        "unstable_lean": {
            "left_pitch_hip": 0.8, "left_roll_hip": 0.0, "left_yaw_hip": 0.0,
            "left_knee": 0.2, "left_ankle": 0.1,
            "right_pitch_hip": 0.8, "right_roll_hip": 0.0, "right_yaw_hip": 0.0,
            "right_knee": 0.2, "right_ankle": 0.1
        }
    }
    
    results = {}
    
    for pose_name, joint_positions in test_poses.items():
        print(f"\nTesting pose: {pose_name}")
        
        # Apply the pose
        for joint_name, position in joint_positions.items():
            try:
                joint_index = m.actuator(joint_name).id
                data.ctrl[joint_index] = position
            except:
                print(f"  Warning: Could not set {joint_name}")
        
        # Step simulation to update physics and let robot settle
        for _ in range(100):  # More steps to settle completely
            mujoco.mj_step(m, data)
        
        # Calculate COM and stability
        com_pos = com_calculator.calculate_com()
        com_info = com_calculator.get_com_info()
        pelvis_pos = Utils.get_body_position_global(m, data, "Pelvis")
        
        results[pose_name] = {
            "com_position": com_pos,
            "pelvis_position": pelvis_pos,
            "is_stable": com_info["is_stable"],
            "stability_margin": com_info["stability_margin"],
            "num_contacts": com_info["num_contacts"]
        }
        
        print(f"  COM: ({com_pos[0]:.3f}, {com_pos[1]:.3f}, {com_pos[2]:.3f})")
        print(f"  Pelvis: ({pelvis_pos[0]:.3f}, {pelvis_pos[1]:.3f}, {pelvis_pos[2]:.3f})")
        print(f"  Stable: {com_info['is_stable']}")
        print(f"  Contacts: {com_info['num_contacts']}")
        print(f"  Margin: {com_info['stability_margin']:.3f}m")
    
    # Validation checks
    print("\n=== VALIDATION RESULTS ===")
    
    success = True
    
    # Check that we can calculate COM for all poses
    for pose_name, result in results.items():
        com_pos = result["com_position"]
        if np.all(com_pos == 0):
            print(f"❌ {pose_name}: COM calculation failed (all zeros)")
            success = False
        else:
            print(f"✅ {pose_name}: COM calculated successfully")
    
    # Check that wide stance is more stable than normal standing
    if results["wide_stance"]["stability_margin"] > results["standing"]["stability_margin"]:
        print("✅ Wide stance shows higher stability margin than standing")
    else:
        print("⚠️  Wide stance should be more stable than normal standing")
    
    # Check that we detect some contacts
    total_contacts = sum(result["num_contacts"] for result in results.values())
    if total_contacts > 0:
        print(f"✅ Contact detection working ({total_contacts} total contacts detected)")
    else:
        print("❌ No contacts detected - check contact detection")
        success = False
    
    # Check that COM positions are reasonable (not too far from robot)
    reasonable_com = True
    for pose_name, result in results.items():
        com_pos = result["com_position"]
        pelvis_pos = result["pelvis_position"]
        distance = np.linalg.norm(com_pos - pelvis_pos)
        if distance > 2.0:  # More than 2 meters seems unreasonable
            print(f"❌ {pose_name}: COM too far from pelvis ({distance:.3f}m)")
            reasonable_com = False
            success = False
    
    if reasonable_com:
        print("✅ All COM positions are reasonable")
    
    print(f"\n{'✅ ALL TESTS PASSED' if success else '❌ SOME TESTS FAILED'}")
    
    return success

def test_gait_transitions():
    """Test that gait transitions work smoothly."""
    
    print("\n=== GAIT TRANSITION VALIDATION ===")
    
    m = mujoco.MjModel.from_xml_path("./bars.xml")
    data = mujoco.MjData(m)
    
    com_calculator = CenterOfMassCalculator(m, data)
    gait_generator = QuasiStaticGaitGenerator(m, data, com_calculator)
    
    # Start walking
    gait_generator.start_walking(0.0)
    
    # Test smooth transitions
    positions_over_time = []
    
    for step in range(100):  # 10 seconds at 0.1s intervals
        sim_time = step * 0.1
        
        # Get joint positions
        joint_positions = gait_generator.get_current_pose(sim_time)
        
        # Store for smoothness check
        left_hip_pitch = joint_positions.get("left_pitch_hip", 0.0)
        positions_over_time.append(left_hip_pitch)
        
        # Apply positions
        for joint_name, position in joint_positions.items():
            try:
                joint_index = m.actuator(joint_name).id
                data.ctrl[joint_index] = position
            except:
                pass
        
        # Step simulation
        mujoco.mj_step(m, data)
    
    # Check for smoothness (no huge jumps)
    max_change = 0.0
    for i in range(1, len(positions_over_time)):
        change = abs(positions_over_time[i] - positions_over_time[i-1])
        max_change = max(max_change, change)
    
    print(f"Maximum position change between steps: {max_change:.4f} rad")
    
    if max_change < 0.5:  # Less than ~30 degrees per 0.1s
        print("✅ Gait transitions are smooth")
        return True
    else:
        print("❌ Gait transitions too abrupt")
        return False

if __name__ == "__main__":
    try:
        test1_passed = test_stability_calculation()
        test2_passed = test_gait_transitions()
        
        if test1_passed and test2_passed:
            print("\n🎉 ALL VALIDATION TESTS PASSED!")
            exit(0)
        else:
            print("\n❌ SOME VALIDATION TESTS FAILED")
            exit(1)
            
    except Exception as e:
        print(f"Error during validation: {e}")
        import traceback
        traceback.print_exc()
        exit(1)