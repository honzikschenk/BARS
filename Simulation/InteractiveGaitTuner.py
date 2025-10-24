import time
import json
from typing import Dict, Any, Optional
import mujoco
from GaitGenerator import QuasiStaticGaitGenerator, GaitLibrary
from CenterOfMass import CenterOfMassCalculator


class InteractiveGaitTuner:
    """
    Interactive interface for tuning gait parameters in real-time.
    """
    
    def __init__(self, model, data, gait_generator: QuasiStaticGaitGenerator, 
                 com_calculator: CenterOfMassCalculator):
        """
        Initialize the interactive gait tuner.
        
        Args:
            model: MuJoCo model
            data: MuJoCo data
            gait_generator: Gait generator instance
            com_calculator: Center of mass calculator
        """
        self.model = model
        self.data = data
        self.gait_generator = gait_generator
        self.com_calculator = com_calculator
        
        # Tuning parameters with their ranges
        self.tuning_params = {
            "step_length": {"value": 0.15, "min": 0.05, "max": 0.3, "step": 0.01},
            "step_height": {"value": 0.05, "min": 0.01, "max": 0.15, "step": 0.005},
            "step_width": {"value": 0.2, "min": 0.1, "max": 0.4, "step": 0.01},
            "phase_duration": {"value": 2.0, "min": 0.5, "max": 5.0, "step": 0.1},
            "shift_duration": {"value": 1.0, "min": 0.3, "max": 3.0, "step": 0.1}
        }
        
        # Interface state
        self.selected_param = 0
        self.param_names = list(self.tuning_params.keys())
        self.show_help = True
        self.show_stability_info = True
        self.show_com_info = True
        self.recording_mode = False
        
        # Statistics tracking
        self.stability_history = []
        self.max_history_length = 100
        
        # Load initial parameters from gait generator
        self._sync_params_from_gait_generator()
    
    def _sync_params_from_gait_generator(self):
        """Sync tuning parameters from the gait generator."""
        for param_name in self.param_names:
            if hasattr(self.gait_generator, param_name):
                self.tuning_params[param_name]["value"] = getattr(self.gait_generator, param_name)
    
    def handle_keyboard_input(self, key_pressed: str) -> bool:
        """
        Handle keyboard input for parameter tuning.
        
        Args:
            key_pressed: The key that was pressed
        
        Returns:
            True if the input was handled, False otherwise
        """
        if key_pressed == "h":
            self.show_help = not self.show_help
            return True
        
        elif key_pressed == "s":
            self.show_stability_info = not self.show_stability_info
            return True
        
        elif key_pressed == "c":
            self.show_com_info = not self.show_com_info
            return True
        
        elif key_pressed == "r":
            self.recording_mode = not self.recording_mode
            print(f"Recording mode: {'ON' if self.recording_mode else 'OFF'}")
            return True
        
        # Parameter selection (1-5 keys)
        elif key_pressed in "12345":
            idx = int(key_pressed) - 1
            if idx < len(self.param_names):
                self.selected_param = idx
                return True
        
        # Parameter adjustment
        elif key_pressed == "up" or key_pressed == "+":
            self._adjust_parameter(1)
            return True
        
        elif key_pressed == "down" or key_pressed == "-":
            self._adjust_parameter(-1)
            return True
        
        # Gait control
        elif key_pressed == "space":
            if self.gait_generator.is_walking:
                self.gait_generator.stop_walking()
                print("Gait stopped")
            else:
                self.gait_generator.start_walking(time.time())
                print("Gait started")
            return True
        
        # Preset gaits
        elif key_pressed == "q":
            self._load_preset_gait(GaitLibrary.get_walking_gait_slow())
            print("Loaded slow walking gait")
            return True
        
        elif key_pressed == "w":
            self._load_preset_gait(GaitLibrary.get_walking_gait_normal())
            print("Loaded normal walking gait")
            return True
        
        elif key_pressed == "e":
            self._load_preset_gait(GaitLibrary.get_walking_gait_fast())
            print("Loaded fast walking gait")
            return True
        
        elif key_pressed == "t":
            self._load_preset_gait(GaitLibrary.get_wide_stance_gait())
            print("Loaded wide stance gait")
            return True
        
        # Save/Load
        elif key_pressed == "ctrl+s":
            self._save_current_gait()
            return True
        
        elif key_pressed == "ctrl+l":
            self._load_saved_gait()
            return True
        
        return False
    
    def _adjust_parameter(self, direction: int):
        """
        Adjust the currently selected parameter.
        
        Args:
            direction: +1 to increase, -1 to decrease
        """
        param_name = self.param_names[self.selected_param]
        param_info = self.tuning_params[param_name]
        
        current_value = param_info["value"]
        step = param_info["step"]
        min_val = param_info["min"]
        max_val = param_info["max"]
        
        new_value = current_value + (direction * step)
        new_value = max(min_val, min(max_val, new_value))
        
        if new_value != current_value:
            param_info["value"] = new_value
            
            # Update the gait generator
            kwargs = {param_name: new_value}
            self.gait_generator.update_gait_parameters(**kwargs)
            
            print(f"Updated {param_name}: {new_value:.3f}")
    
    def _load_preset_gait(self, preset_params: Dict[str, float]):
        """
        Load a preset gait configuration.
        
        Args:
            preset_params: Dictionary of gait parameters
        """
        for param_name, value in preset_params.items():
            if param_name in self.tuning_params:
                self.tuning_params[param_name]["value"] = value
        
        self.gait_generator.update_gait_parameters(**preset_params)
        self._sync_params_from_gait_generator()
    
    def _save_current_gait(self):
        """Save the current gait parameters to a file."""
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"custom_gait_{timestamp}.json"
        self.gait_generator.save_gait_to_file(filename)
        print(f"Saved gait to {filename}")
    
    def _load_saved_gait(self):
        """Load a previously saved gait (placeholder for file selection)."""
        # In a real implementation, this would show a file dialog
        try:
            self.gait_generator.load_gait_from_file("custom_gait.json")
            self._sync_params_from_gait_generator()
            print("Loaded saved gait")
        except FileNotFoundError:
            print("No saved gait file found")
    
    def update_statistics(self, current_time: float):
        """
        Update stability statistics.
        
        Args:
            current_time: Current simulation time
        """
        stability_info = self.gait_generator.get_stability_info()
        
        # Add to history
        self.stability_history.append({
            "time": current_time,
            "is_stable": stability_info["is_stable"],
            "stability_margin": stability_info["stability_margin"]
        })
        
        # Trim history if too long
        if len(self.stability_history) > self.max_history_length:
            self.stability_history.pop(0)
    
    def get_stability_statistics(self) -> Dict[str, Any]:
        """
        Get stability statistics over recent history.
        
        Returns:
            Dictionary with stability statistics
        """
        if not self.stability_history:
            return {"stability_percentage": 0.0, "avg_margin": 0.0, "samples": 0}
        
        stable_count = sum(1 for entry in self.stability_history if entry["is_stable"])
        total_count = len(self.stability_history)
        
        margins = [entry["stability_margin"] for entry in self.stability_history 
                  if entry["stability_margin"] > 0]
        avg_margin = sum(margins) / len(margins) if margins else 0.0
        
        return {
            "stability_percentage": (stable_count / total_count) * 100,
            "avg_margin": avg_margin,
            "samples": total_count
        }
    
    def get_display_info(self, current_time: float) -> str:
        """
        Get formatted text for display in the viewer.
        
        Args:
            current_time: Current simulation time
        
        Returns:
            Formatted text string
        """
        lines = []
        
        if self.show_help:
            lines.extend([
                "=== GAIT TUNER CONTROLS ===",
                "H: Toggle this help",
                "S: Toggle stability info",
                "C: Toggle COM info", 
                "R: Toggle recording mode",
                "",
                "1-5: Select parameter to tune",
                "UP/DOWN or +/-: Adjust parameter",
                "SPACE: Start/Stop walking",
                "",
                "Presets:",
                "Q: Slow gait  W: Normal  E: Fast  T: Wide",
                "Ctrl+S: Save  Ctrl+L: Load",
                "=========================="
            ])
        
        # Current parameters
        lines.append("=== GAIT PARAMETERS ===")
        for i, param_name in enumerate(self.param_names):
            param_info = self.tuning_params[param_name]
            marker = ">" if i == self.selected_param else " "
            lines.append(f"{marker} {i+1}. {param_name}: {param_info['value']:.3f}")
        
        # Gait status
        lines.append("")
        phase_info = self.gait_generator.get_current_phase_info(current_time)
        gait_status = "WALKING" if self.gait_generator.is_walking else "STOPPED"
        lines.append(f"Gait Status: {gait_status}")
        if self.gait_generator.is_walking:
            lines.append(f"Phase: {phase_info['phase']} ({phase_info['progress']:.1%})")
            lines.append(f"Support: {phase_info['support_foot']}")
        
        if self.show_stability_info:
            lines.append("")
            lines.append("=== STABILITY INFO ===")
            stability_info = self.gait_generator.get_stability_info()
            lines.append(f"Stable: {'YES' if stability_info['is_stable'] else 'NO'}")
            lines.append(f"Margin: {stability_info['stability_margin']:.3f}m")
            lines.append(f"Contacts: {stability_info['num_contacts']}")
            
            # Statistics
            stats = self.get_stability_statistics()
            lines.append(f"Stability %: {stats['stability_percentage']:.1f}%")
        
        if self.show_com_info:
            lines.append("")
            lines.append("=== CENTER OF MASS ===")
            com_info = self.com_calculator.get_com_info()
            pos = com_info['position']
            vel = com_info['velocity']
            lines.append(f"Position: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
            lines.append(f"Velocity: ({vel[0]:.3f}, {vel[1]:.3f}, {vel[2]:.3f})")
        
        if self.recording_mode:
            lines.append("")
            lines.append("🔴 RECORDING MODE")
        
        return "\n".join(lines)
    
    def get_current_joint_positions(self, current_time: float) -> Dict[str, float]:
        """
        Get the current target joint positions from the gait generator.
        
        Args:
            current_time: Current simulation time
        
        Returns:
            Dictionary of joint positions
        """
        return self.gait_generator.get_current_pose(current_time)
    
    def apply_joint_positions(self, joint_positions: Dict[str, float]):
        """
        Apply joint positions to the MuJoCo model.
        
        Args:
            joint_positions: Dictionary of joint names to positions
        """
        for joint_name, position in joint_positions.items():
            try:
                joint_index = self.model.actuator(joint_name).id
                self.data.ctrl[joint_index] = position
            except Exception as e:
                print(f"Warning: Could not set position for joint {joint_name}: {e}")
    
    def update(self, current_time: float):
        """
        Update the gait tuner (call this every simulation step).
        
        Args:
            current_time: Current simulation time
        """
        # Update statistics
        self.update_statistics(current_time)
        
        # Get and apply current joint positions
        if self.gait_generator.is_walking:
            joint_positions = self.get_current_joint_positions(current_time)
            self.apply_joint_positions(joint_positions)
    
    def reset_robot_position(self):
        """Reset the robot to standing position."""
        standing_positions = {
            "left_pitch_hip": -0.2, "left_roll_hip": 0.0, "left_yaw_hip": 0.0,
            "left_knee": 0.45, "left_ankle": 0.35,
            "right_pitch_hip": -0.2, "right_roll_hip": 0.0, "right_yaw_hip": 0.0,
            "right_knee": 0.45, "right_ankle": 0.35
        }
        self.apply_joint_positions(standing_positions)
        
        # Reset simulation data
        mujoco.mj_resetData(self.model, self.data)
        self.apply_joint_positions(standing_positions)
        
        # Stop walking
        self.gait_generator.stop_walking()


class KeyboardHandler:
    """
    Simple keyboard input handler for the interactive interface.
    """
    
    def __init__(self):
        self.key_state = {}
        self.last_key_time = {}
        self.key_repeat_delay = 0.1  # seconds
    
    def is_key_pressed(self, key: str, current_time: float) -> bool:
        """
        Check if a key is pressed (with repeat delay).
        
        Args:
            key: Key to check
            current_time: Current time
        
        Returns:
            True if key should be considered pressed
        """
        # This is a simplified implementation
        # In a real application, you'd integrate with the viewer's key callback
        return False
    
    def update_key_state(self, key: str, is_pressed: bool, current_time: float):
        """
        Update the state of a key.
        
        Args:
            key: Key name
            is_pressed: Whether the key is currently pressed
            current_time: Current time
        """
        self.key_state[key] = is_pressed
        if is_pressed:
            self.last_key_time[key] = current_time