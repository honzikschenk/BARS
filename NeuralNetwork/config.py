"""
Configuration and utilities for omnidirectional gait learning
=============================================================

This module provides configuration classes and utility functions
for the reinforcement learning framework.
"""

import numpy as np
from typing import Dict, List, Any, Optional
from dataclasses import dataclass, asdict
import json
from pathlib import Path


@dataclass
class EnvironmentConfig:
    """Configuration for the bipedal gait environment."""
    dt: float = 0.01
    max_episode_steps: int = 1000
    target_velocity: float = 0.5
    stability_threshold: float = 0.1
    height_threshold: float = 0.3
    tilt_threshold: float = 0.5
    
    # Reward weights
    velocity_weight: float = 1.0
    stability_weight: float = 1.0
    posture_weight: float = 1.0
    energy_weight: float = 0.5
    balance_weight: float = 0.3


@dataclass
class RobotConfig:
    """Configuration for the BARS robot parameters."""
    # Physical parameters
    leg_length: float = 0.4  # meters
    foot_width: float = 0.1
    foot_length: float = 0.15
    body_height: float = 0.3
    body_width: float = 0.2
    
    # Joint limits (radians)
    hip_pitch_range: tuple = (-1.5, 1.5)
    hip_roll_range: tuple = (-0.8, 0.8)
    hip_yaw_range: tuple = (-0.5, 0.5)
    knee_range: tuple = (-2.0, 0.0)
    ankle_range: tuple = (-0.8, 0.8)
    
    # Joint names mapping to BARS robot
    joint_names: List[str] = None
    
    def __post_init__(self):
        if self.joint_names is None:
            self.joint_names = [
                "left_pitch_hip", "left_roll_hip", "left_yaw_hip",
                "left_knee", "left_ankle",
                "right_pitch_hip", "right_roll_hip", "right_yaw_hip", 
                "right_knee", "right_ankle"
            ]


class ConfigManager:
    """Manages configuration loading and saving."""
    
    def __init__(self, config_dir: str = "configs"):
        """Initialize configuration manager."""
        self.config_dir = Path(config_dir)
        self.config_dir.mkdir(exist_ok=True)
    
    def save_config(self, config: Any, name: str) -> str:
        """Save configuration to file."""
        filepath = self.config_dir / f"{name}.json"
        
        config_dict = asdict(config) if hasattr(config, '__dataclass_fields__') else config
        
        with open(filepath, 'w') as f:
            json.dump(config_dict, f, indent=2)
        
        return str(filepath)
    
    def load_environment_config(self, name: str) -> EnvironmentConfig:
        """Load environment configuration from file."""
        filepath = self.config_dir / f"{name}.json"
        
        with open(filepath, 'r') as f:
            config_dict = json.load(f)
        
        return EnvironmentConfig(**config_dict)
    
    def load_robot_config(self, name: str) -> RobotConfig:
        """Load robot configuration from file."""
        filepath = self.config_dir / f"{name}.json"
        
        with open(filepath, 'r') as f:
            config_dict = json.load(f)
        
        return RobotConfig(**config_dict)


def get_default_configs() -> Dict[str, Any]:
    """Get default configuration objects for quick setup."""
    return {
        'environment': EnvironmentConfig(),
        'robot': RobotConfig(),
        'network': {
            'hidden_layers': [64, 32],
            'activation': 'tanh',
            'learning_rate': 0.001,
            'momentum': 0.9,
            'weight_decay': 0.01
        },
        'training': {
            'max_episodes': 1000,
            'trajectories_per_episode': 5,
            'evaluation_frequency': 50,
            'save_frequency': 100,
            'target_reward': 15.0,
            'early_stopping_patience': 200
        }
    }


def create_default_config_files(config_dir: str = "configs") -> None:
    """Create default configuration files."""
    manager = ConfigManager(config_dir)
    defaults = get_default_configs()
    
    # Save each configuration
    for name, config in defaults.items():
        if name in ['environment', 'robot']:
            # These are dataclass objects
            if name == 'environment':
                config_obj = EnvironmentConfig()
            else:
                config_obj = RobotConfig()
            manager.save_config(config_obj, f"default_{name}")
        else:
            # These are dictionaries
            manager.save_config(config, f"default_{name}")
    
    print(f"Default configuration files created in {config_dir}/")


def validate_action(action: np.ndarray, robot_config: RobotConfig) -> np.ndarray:
    """
    Validate and clip actions to safe joint ranges.
    
    Args:
        action: Raw action vector
        robot_config: Robot configuration with joint limits
        
    Returns:
        Clipped action vector
    """
    if len(action) != len(robot_config.joint_names):
        raise ValueError(f"Action dimension {len(action)} doesn't match "
                        f"number of joints {len(robot_config.joint_names)}")
    
    # Define joint limits for each joint type
    joint_limits = {
        'hip_pitch': robot_config.hip_pitch_range,
        'hip_roll': robot_config.hip_roll_range,
        'hip_yaw': robot_config.hip_yaw_range,
        'knee': robot_config.knee_range,
        'ankle': robot_config.ankle_range
    }
    
    clipped_action = action.copy()
    
    for i, joint_name in enumerate(robot_config.joint_names):
        # Determine joint type
        joint_type = None
        for jtype in joint_limits.keys():
            if jtype in joint_name:
                joint_type = jtype
                break
        
        if joint_type:
            min_val, max_val = joint_limits[joint_type]
            clipped_action[i] = np.clip(action[i], min_val, max_val)
    
    return clipped_action


def calculate_stability_margin(center_of_mass: np.ndarray, 
                             left_foot_pos: np.ndarray,
                             right_foot_pos: np.ndarray,
                             foot_width: float = 0.1,
                             foot_length: float = 0.15) -> float:
    """
    Calculate stability margin based on center of mass and support polygon.
    
    Args:
        center_of_mass: 3D position of center of mass
        left_foot_pos: 3D position of left foot
        right_foot_pos: 3D position of right foot
        foot_width: Width of each foot
        foot_length: Length of each foot
        
    Returns:
        Distance to stability boundary (positive = stable)
    """
    # Project to ground plane (x-y)
    com_2d = center_of_mass[:2]
    
    # Define support polygon vertices (simplified as rectangle between feet)
    left_corners = [
        left_foot_pos[:2] + np.array([-foot_length/2, -foot_width/2]),
        left_foot_pos[:2] + np.array([foot_length/2, -foot_width/2]),
        left_foot_pos[:2] + np.array([foot_length/2, foot_width/2]),
        left_foot_pos[:2] + np.array([-foot_length/2, foot_width/2])
    ]
    
    right_corners = [
        right_foot_pos[:2] + np.array([-foot_length/2, -foot_width/2]),
        right_foot_pos[:2] + np.array([foot_length/2, -foot_width/2]),
        right_foot_pos[:2] + np.array([foot_length/2, foot_width/2]),
        right_foot_pos[:2] + np.array([-foot_length/2, foot_width/2])
    ]
    
    # Combine into support polygon
    polygon = left_corners + right_corners
    
    # Calculate minimum distance to polygon edges
    min_distance = float('inf')
    
    for i in range(len(polygon)):
        p1 = polygon[i]
        p2 = polygon[(i + 1) % len(polygon)]
        
        # Distance from point to line segment
        dist = point_to_line_distance(com_2d, p1, p2)
        min_distance = min(min_distance, dist)
    
    # Check if point is inside polygon (simplified)
    inside = point_in_polygon(com_2d, polygon)
    
    return min_distance if inside else -min_distance


def point_to_line_distance(point: np.ndarray, line_start: np.ndarray, line_end: np.ndarray) -> float:
    """Calculate distance from point to line segment."""
    line_vec = line_end - line_start
    point_vec = point - line_start
    
    line_len = np.linalg.norm(line_vec)
    if line_len == 0:
        return np.linalg.norm(point_vec)
    
    # Project point onto line
    t = max(0, min(1, np.dot(point_vec, line_vec) / (line_len ** 2)))
    projection = line_start + t * line_vec
    
    return np.linalg.norm(point - projection)


def point_in_polygon(point: np.ndarray, polygon: List[np.ndarray]) -> bool:
    """Check if point is inside polygon using ray casting algorithm."""
    x, y = point
    n = len(polygon)
    inside = False
    
    p1x, p1y = polygon[0]
    for i in range(1, n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    
    return inside


def normalize_observation(observation: np.ndarray, 
                         obs_mean: Optional[np.ndarray] = None,
                         obs_std: Optional[np.ndarray] = None) -> np.ndarray:
    """
    Normalize observation vector for better training stability.
    
    Args:
        observation: Raw observation vector
        obs_mean: Mean values for normalization (computed if None)
        obs_std: Standard deviation for normalization (computed if None)
        
    Returns:
        Normalized observation vector
    """
    if obs_mean is None:
        obs_mean = np.zeros_like(observation)
    if obs_std is None:
        obs_std = np.ones_like(observation)
    
    # Avoid division by zero
    obs_std = np.maximum(obs_std, 1e-8)
    
    return (observation - obs_mean) / obs_std


def create_gait_pattern(direction: str, phase: float, step_height: float = 0.05) -> Dict[str, float]:
    """
    Generate basic gait pattern for different movement directions.
    
    Args:
        direction: Movement direction ('forward', 'backward', 'left', 'right', etc.)
        phase: Gait phase [0, 1]
        step_height: Maximum foot lift height
        
    Returns:
        Dictionary of joint angles for this gait phase
    """
    # Basic walking pattern parameters
    hip_swing = 0.3  # Hip swing amplitude
    knee_bend = 0.5  # Knee bend amplitude
    
    # Determine which leg is in swing phase
    left_swing = 0.0 <= phase < 0.5
    
    # Base joint angles
    joint_angles = {
        "left_pitch_hip": 0.0,
        "left_roll_hip": 0.0,
        "left_yaw_hip": 0.0,
        "left_knee": 0.0,
        "left_ankle": 0.0,
        "right_pitch_hip": 0.0,
        "right_roll_hip": 0.0,
        "right_yaw_hip": 0.0,
        "right_knee": 0.0,
        "right_ankle": 0.0
    }
    
    # Modify based on direction and gait phase
    if direction in ['forward', 'backward']:
        sign = 1 if direction == 'forward' else -1
        
        if left_swing:
            joint_angles["left_pitch_hip"] = sign * hip_swing * np.sin(2 * np.pi * phase)
            joint_angles["left_knee"] = knee_bend * np.sin(2 * np.pi * phase)
            joint_angles["right_pitch_hip"] = -sign * hip_swing * 0.5
        else:
            joint_angles["right_pitch_hip"] = sign * hip_swing * np.sin(2 * np.pi * (phase - 0.5))
            joint_angles["right_knee"] = knee_bend * np.sin(2 * np.pi * (phase - 0.5))
            joint_angles["left_pitch_hip"] = -sign * hip_swing * 0.5
    
    elif direction in ['left', 'right']:
        sign = 1 if direction == 'left' else -1
        
        if left_swing:
            joint_angles["left_roll_hip"] = sign * hip_swing * np.sin(2 * np.pi * phase)
            joint_angles["left_knee"] = knee_bend * np.sin(2 * np.pi * phase)
        else:
            joint_angles["right_roll_hip"] = sign * hip_swing * np.sin(2 * np.pi * (phase - 0.5))
            joint_angles["right_knee"] = knee_bend * np.sin(2 * np.pi * (phase - 0.5))
    
    return joint_angles


if __name__ == "__main__":
    # Create default configuration files when run directly
    create_default_config_files()
    print("Configuration utilities initialized.")