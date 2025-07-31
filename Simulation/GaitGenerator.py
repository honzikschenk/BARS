import numpy as np
import json
import math
from typing import Dict, List, Tuple, Optional
from CenterOfMass import CenterOfMassCalculator


class GaitPhase:
    """
    Represents a single phase in a gait cycle.
    """
    
    def __init__(self, name: str, joint_positions: Dict[str, float], 
                 duration: float, support_foot: str = "both"):
        """
        Initialize a gait phase.
        
        Args:
            name: Name of the gait phase
            joint_positions: Dictionary of joint names to positions
            duration: Duration of this phase in seconds
            support_foot: Which foot(s) are in contact ("left", "right", "both")
        """
        self.name = name
        self.joint_positions = joint_positions
        self.duration = duration
        self.support_foot = support_foot


class QuasiStaticGaitGenerator:
    """
    Generate quasi-static gaits for the bipedal robot.
    A quasi-static gait maintains static stability at all times.
    """
    
    def __init__(self, model, data, com_calculator: CenterOfMassCalculator):
        """
        Initialize the gait generator.
        
        Args:
            model: MuJoCo model
            data: MuJoCo data
            com_calculator: Center of mass calculator instance
        """
        self.model = model
        self.data = data
        self.com_calculator = com_calculator
        
        # Gait parameters (user-tunable)
        self.step_length = 0.15  # meters
        self.step_height = 0.05  # meters
        self.step_width = 0.2    # meters (lateral separation)
        self.phase_duration = 2.0  # seconds per phase
        self.shift_duration = 1.0  # seconds for weight shifting
        
        # Current gait state
        self.current_phase = 0
        self.phase_start_time = 0.0
        self.gait_phases = []
        self.is_walking = False
        
        # Joint names
        self.joint_names = [
            "left_pitch_hip", "left_roll_hip", "left_yaw_hip", "left_knee", "left_ankle",
            "right_pitch_hip", "right_roll_hip", "right_yaw_hip", "right_knee", "right_ankle"
        ]
        
        self._generate_default_gait()
    
    def _generate_default_gait(self):
        """
        Generate a default walking gait cycle.
        """
        # Base standing position
        standing_pos = {
            "left_pitch_hip": -0.2, "left_roll_hip": 0.0, "left_yaw_hip": 0.0,
            "left_knee": 0.45, "left_ankle": 0.35,
            "right_pitch_hip": -0.2, "right_roll_hip": 0.0, "right_yaw_hip": 0.0,
            "right_knee": 0.45, "right_ankle": 0.35
        }
        
        self.gait_phases = [
            # Phase 0: Standing (both feet on ground)
            GaitPhase("standing", standing_pos.copy(), self.phase_duration, "both"),
            
            # Phase 1: Shift weight to right foot
            GaitPhase("weight_right", self._generate_weight_shift_pose("right"), 
                     self.shift_duration, "right"),
            
            # Phase 2: Lift left foot and move forward
            GaitPhase("left_swing", self._generate_swing_pose("left"), 
                     self.phase_duration, "right"),
            
            # Phase 3: Place left foot down
            GaitPhase("left_stance", self._generate_stance_pose("left"), 
                     self.shift_duration, "both"),
            
            # Phase 4: Shift weight to left foot
            GaitPhase("weight_left", self._generate_weight_shift_pose("left"), 
                     self.shift_duration, "left"),
            
            # Phase 5: Lift right foot and move forward
            GaitPhase("right_swing", self._generate_swing_pose("right"), 
                     self.phase_duration, "left"),
            
            # Phase 6: Place right foot down (back to standing)
            GaitPhase("right_stance", self._generate_stance_pose("right"), 
                     self.shift_duration, "both")
        ]
    
    def _generate_weight_shift_pose(self, support_leg: str) -> Dict[str, float]:
        """
        Generate a pose that shifts weight to one leg.
        
        Args:
            support_leg: "left" or "right"
        
        Returns:
            Dictionary of joint positions
        """
        base_pos = {
            "left_pitch_hip": -0.2, "left_roll_hip": 0.0, "left_yaw_hip": 0.0,
            "left_knee": 0.45, "left_ankle": 0.35,
            "right_pitch_hip": -0.2, "right_roll_hip": 0.0, "right_yaw_hip": 0.0,
            "right_knee": 0.45, "right_ankle": 0.35
        }
        
        # Shift weight by adjusting hip roll
        if support_leg == "right":
            base_pos["left_roll_hip"] = -0.3  # Lean toward right leg
            base_pos["right_roll_hip"] = -0.1
        else:  # left
            base_pos["left_roll_hip"] = 0.1
            base_pos["right_roll_hip"] = 0.3   # Lean toward left leg
        
        return base_pos
    
    def _generate_swing_pose(self, swing_leg: str) -> Dict[str, float]:
        """
        Generate a pose for the swing phase (one leg lifted and moving forward).
        
        Args:
            swing_leg: "left" or "right"
        
        Returns:
            Dictionary of joint positions
        """
        base_pos = self._generate_weight_shift_pose("right" if swing_leg == "left" else "left")
        
        # Lift and move the swing leg
        if swing_leg == "left":
            base_pos["left_pitch_hip"] = 0.3   # Hip flexion to lift leg
            base_pos["left_knee"] = 0.8        # Bend knee more
            base_pos["left_ankle"] = 0.1       # Dorsiflex ankle
        else:  # right
            base_pos["right_pitch_hip"] = 0.3
            base_pos["right_knee"] = 0.8
            base_pos["right_ankle"] = 0.1
        
        return base_pos
    
    def _generate_stance_pose(self, landing_leg: str) -> Dict[str, float]:
        """
        Generate a pose for placing a foot down.
        
        Args:
            landing_leg: "left" or "right"
        
        Returns:
            Dictionary of joint positions
        """
        base_pos = {
            "left_pitch_hip": -0.1, "left_roll_hip": 0.0, "left_yaw_hip": 0.0,
            "left_knee": 0.45, "left_ankle": 0.35,
            "right_pitch_hip": -0.1, "right_roll_hip": 0.0, "right_yaw_hip": 0.0,
            "right_knee": 0.45, "right_ankle": 0.35
        }
        
        # Slight forward lean after step
        if landing_leg == "left":
            base_pos["left_pitch_hip"] = -0.3
        else:
            base_pos["right_pitch_hip"] = -0.3
        
        return base_pos
    
    def update_gait_parameters(self, **kwargs):
        """
        Update gait parameters and regenerate the gait.
        
        Args:
            **kwargs: Gait parameters to update (step_length, step_height, etc.)
        """
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)
        
        self._generate_default_gait()
    
    def start_walking(self, current_time: float):
        """
        Start the walking gait.
        
        Args:
            current_time: Current simulation time
        """
        self.is_walking = True
        self.current_phase = 0
        self.phase_start_time = current_time
    
    def stop_walking(self):
        """
        Stop the walking gait and return to standing.
        """
        self.is_walking = False
        self.current_phase = 0
    
    def get_current_pose(self, current_time: float) -> Dict[str, float]:
        """
        Get the current joint positions based on the gait phase and timing.
        
        Args:
            current_time: Current simulation time
        
        Returns:
            Dictionary of joint positions
        """
        if not self.is_walking or not self.gait_phases:
            # Return standing pose
            return self.gait_phases[0].joint_positions if self.gait_phases else {}
        
        # Check if we need to advance to the next phase
        time_in_phase = current_time - self.phase_start_time
        current_phase_obj = self.gait_phases[self.current_phase]
        
        if time_in_phase >= current_phase_obj.duration:
            # Advance to next phase
            self.current_phase = (self.current_phase + 1) % len(self.gait_phases)
            self.phase_start_time = current_time
            time_in_phase = 0.0
            current_phase_obj = self.gait_phases[self.current_phase]
        
        # Interpolate between current and next phase for smooth transitions
        next_phase_idx = (self.current_phase + 1) % len(self.gait_phases)
        next_phase_obj = self.gait_phases[next_phase_idx]
        
        # Use smooth interpolation (cosine interpolation for smoother motion)
        t = time_in_phase / current_phase_obj.duration
        smooth_t = 0.5 * (1 - math.cos(math.pi * t))  # Smooth S-curve
        
        interpolated_pose = {}
        for joint_name in self.joint_names:
            current_pos = current_phase_obj.joint_positions.get(joint_name, 0.0)
            next_pos = next_phase_obj.joint_positions.get(joint_name, 0.0)
            
            # Interpolate with smooth transition at the end of the phase
            if t > 0.8:  # Start blending with next phase in last 20% of current phase
                blend_t = (t - 0.8) / 0.2
                interpolated_pose[joint_name] = current_pos + blend_t * (next_pos - current_pos)
            else:
                interpolated_pose[joint_name] = current_pos
        
        return interpolated_pose
    
    def get_current_phase_info(self, current_time: float) -> Dict:
        """
        Get information about the current gait phase.
        
        Args:
            current_time: Current simulation time
        
        Returns:
            Dictionary with phase information
        """
        if not self.is_walking or not self.gait_phases:
            return {"phase": "stopped", "progress": 0.0, "support_foot": "both"}
        
        time_in_phase = current_time - self.phase_start_time
        current_phase_obj = self.gait_phases[self.current_phase]
        progress = min(time_in_phase / current_phase_obj.duration, 1.0)
        
        return {
            "phase": current_phase_obj.name,
            "phase_number": self.current_phase,
            "progress": progress,
            "support_foot": current_phase_obj.support_foot,
            "time_remaining": max(0, current_phase_obj.duration - time_in_phase)
        }
    
    def check_stability(self) -> bool:
        """
        Check if the current pose maintains static stability.
        
        Returns:
            True if stable, False otherwise
        """
        return self.com_calculator.is_statically_stable()
    
    def get_stability_info(self) -> Dict:
        """
        Get detailed stability information for the current pose.
        
        Returns:
            Dictionary with stability metrics
        """
        com_info = self.com_calculator.get_com_info()
        return {
            "is_stable": com_info["is_stable"],
            "stability_margin": com_info["stability_margin"],
            "com_position": com_info["position"],
            "num_contacts": com_info["num_contacts"]
        }
    
    def save_gait_to_file(self, filename: str):
        """
        Save the current gait to a JSON file.
        
        Args:
            filename: Name of the file to save to
        """
        gait_data = {
            "parameters": {
                "step_length": self.step_length,
                "step_height": self.step_height,
                "step_width": self.step_width,
                "phase_duration": self.phase_duration,
                "shift_duration": self.shift_duration
            },
            "phases": []
        }
        
        for phase in self.gait_phases:
            gait_data["phases"].append({
                "name": phase.name,
                "joint_positions": phase.joint_positions,
                "duration": phase.duration,
                "support_foot": phase.support_foot
            })
        
        with open(filename, 'w') as f:
            json.dump(gait_data, f, indent=2)
    
    def load_gait_from_file(self, filename: str):
        """
        Load a gait from a JSON file.
        
        Args:
            filename: Name of the file to load from
        """
        with open(filename, 'r') as f:
            gait_data = json.load(f)
        
        # Load parameters
        params = gait_data.get("parameters", {})
        for key, value in params.items():
            if hasattr(self, key):
                setattr(self, key, value)
        
        # Load phases
        self.gait_phases = []
        for phase_data in gait_data.get("phases", []):
            phase = GaitPhase(
                phase_data["name"],
                phase_data["joint_positions"],
                phase_data["duration"],
                phase_data.get("support_foot", "both")
            )
            self.gait_phases.append(phase)


class GaitLibrary:
    """
    A collection of predefined gaits for different scenarios.
    """
    
    @staticmethod
    def get_walking_gait_slow() -> Dict:
        """Get parameters for a slow, stable walking gait."""
        return {
            "step_length": 0.1,
            "step_height": 0.03,
            "step_width": 0.25,
            "phase_duration": 3.0,
            "shift_duration": 1.5
        }
    
    @staticmethod
    def get_walking_gait_normal() -> Dict:
        """Get parameters for a normal walking gait."""
        return {
            "step_length": 0.15,
            "step_height": 0.05,
            "step_width": 0.2,
            "phase_duration": 2.0,
            "shift_duration": 1.0
        }
    
    @staticmethod
    def get_walking_gait_fast() -> Dict:
        """Get parameters for a faster walking gait."""
        return {
            "step_length": 0.2,
            "step_height": 0.07,
            "step_width": 0.18,
            "phase_duration": 1.5,
            "shift_duration": 0.8
        }
    
    @staticmethod
    def get_wide_stance_gait() -> Dict:
        """Get parameters for a wide, very stable gait."""
        return {
            "step_length": 0.08,
            "step_height": 0.02,
            "step_width": 0.3,
            "phase_duration": 4.0,
            "shift_duration": 2.0
        }