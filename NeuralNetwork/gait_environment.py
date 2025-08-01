"""
Bipedal Robot Gait Learning Environment
=======================================

This module defines the environment interface for training omnidirectional gaits
on the BARS bipedal robot. It provides state representation, action spaces,
and reward functions specific to stable walking in all directions.
"""

import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
from enum import Enum


class GaitDirection(Enum):
    """Enumeration of omnidirectional movement types."""
    FORWARD = "forward"
    BACKWARD = "backward"
    LEFT_STRAFE = "left_strafe"
    RIGHT_STRAFE = "right_strafe"
    ROTATE_LEFT = "rotate_left"
    ROTATE_RIGHT = "rotate_right"
    STOP = "stop"


@dataclass
class RobotState:
    """
    Represents the current state of the bipedal robot.
    
    Attributes:
        position: [x, y, z] position in world coordinates
        orientation: [roll, pitch, yaw] orientation in radians
        linear_velocity: [vx, vy, vz] linear velocity
        angular_velocity: [wx, wy, wz] angular velocity
        joint_positions: Joint angles for all degrees of freedom
        joint_velocities: Joint angular velocities
        contact_forces: Ground contact forces for each foot
        center_of_mass: Center of mass position relative to support polygon
        stability_margin: Distance to stability boundary
    """
    position: np.ndarray
    orientation: np.ndarray
    linear_velocity: np.ndarray
    angular_velocity: np.ndarray
    joint_positions: np.ndarray
    joint_velocities: np.ndarray
    contact_forces: np.ndarray
    center_of_mass: np.ndarray
    stability_margin: float


class BipedalGaitEnvironment:
    """
    Reinforcement learning environment for training omnidirectional gaits.
    
    This environment simulates the BARS bipedal robot and provides interfaces
    for training reinforcement learning agents to walk in all directions.
    """
    
    def __init__(self, 
                 dt: float = 0.01,
                 max_episode_steps: int = 1000,
                 target_velocity: float = 0.5,
                 stability_threshold: float = 0.1):
        """
        Initialize the gait learning environment.
        
        Args:
            dt: Simulation timestep
            max_episode_steps: Maximum steps per episode
            target_velocity: Target walking velocity (m/s)
            stability_threshold: Minimum stability margin for reward
        """
        self.dt = dt
        self.max_episode_steps = max_episode_steps
        self.target_velocity = target_velocity
        self.stability_threshold = stability_threshold
        
        # Joint configuration - matches BARS robot design
        self.joint_names = [
            "left_pitch_hip", "left_roll_hip", "left_yaw_hip",
            "left_knee", "left_ankle",
            "right_pitch_hip", "right_roll_hip", "right_yaw_hip", 
            "right_knee", "right_ankle"
        ]
        
        # Action and observation spaces
        self.num_joints = len(self.joint_names)
        self.action_dim = self.num_joints  # Joint angle targets
        self.observation_dim = self._get_observation_dim()
        
        # Episode tracking
        self.current_step = 0
        self.episode_reward = 0.0
        self.robot_state: Optional[RobotState] = None
        
        # Target direction for current episode
        self.target_direction = GaitDirection.FORWARD
        self.target_velocity_vector = np.array([0.5, 0.0, 0.0])  # Forward by default
        
    def _get_observation_dim(self) -> int:
        """Calculate total observation dimension."""
        # Position(3) + orientation(3) + linear_vel(3) + angular_vel(3) + 
        # joint_pos(10) + joint_vel(10) + contact_forces(2) + com(3) + stability(1)
        return 3 + 3 + 3 + 3 + self.num_joints + self.num_joints + 2 + 3 + 1
    
    def reset(self, target_direction: Optional[GaitDirection] = None) -> np.ndarray:
        """
        Reset the environment for a new episode.
        
        Args:
            target_direction: Desired movement direction for this episode
            
        Returns:
            Initial observation vector
        """
        self.current_step = 0
        self.episode_reward = 0.0
        
        # Set target direction and velocity
        if target_direction is not None:
            self.target_direction = target_direction
        else:
            # Randomly sample direction for training variety
            self.target_direction = np.random.choice(list(GaitDirection))
        
        self.target_velocity_vector = self._direction_to_velocity(self.target_direction)
        
        # Initialize robot state
        self.robot_state = RobotState(
            position=np.array([0.0, 0.0, 0.7]),  # Standing height
            orientation=np.array([0.0, 0.0, 0.0]),  # Upright
            linear_velocity=np.zeros(3),
            angular_velocity=np.zeros(3),
            joint_positions=np.zeros(self.num_joints),  # Neutral stance
            joint_velocities=np.zeros(self.num_joints),
            contact_forces=np.array([400.0, 400.0]),  # Both feet on ground
            center_of_mass=np.array([0.0, 0.0, 0.0]),  # Centered
            stability_margin=0.5  # Stable initial position
        )
        
        return self._get_observation()
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict]:
        """
        Execute one environment step.
        
        Args:
            action: Joint angle targets for all degrees of freedom
            
        Returns:
            observation: Next state observation
            reward: Reward for this step
            done: Whether episode is complete
            info: Additional information dictionary
        """
        # Clip actions to safe ranges
        action = np.clip(action, -1.5, 1.5)  # Reasonable joint limits
        
        # Simulate robot dynamics (placeholder - would interface with MuJoCo)
        self._simulate_step(action)
        
        # Calculate reward
        reward = self._calculate_reward(action)
        self.episode_reward += reward
        
        # Check termination conditions
        done = self._is_episode_done()
        
        # Increment step counter
        self.current_step += 1
        
        # Prepare info dictionary
        info = {
            'target_direction': self.target_direction.value,
            'target_velocity': self.target_velocity_vector.tolist(),
            'actual_velocity': self.robot_state.linear_velocity.tolist(),
            'stability_margin': self.robot_state.stability_margin,
            'episode_reward': self.episode_reward,
            'step': self.current_step
        }
        
        return self._get_observation(), reward, done, info
    
    def _direction_to_velocity(self, direction: GaitDirection) -> np.ndarray:
        """Convert movement direction to target velocity vector."""
        velocity_map = {
            GaitDirection.FORWARD: np.array([self.target_velocity, 0.0, 0.0]),
            GaitDirection.BACKWARD: np.array([-self.target_velocity, 0.0, 0.0]),
            GaitDirection.LEFT_STRAFE: np.array([0.0, self.target_velocity, 0.0]),
            GaitDirection.RIGHT_STRAFE: np.array([0.0, -self.target_velocity, 0.0]),
            GaitDirection.ROTATE_LEFT: np.array([0.0, 0.0, 0.5]),  # Angular velocity
            GaitDirection.ROTATE_RIGHT: np.array([0.0, 0.0, -0.5]),
            GaitDirection.STOP: np.array([0.0, 0.0, 0.0])
        }
        return velocity_map[direction]
    
    def _simulate_step(self, action: np.ndarray) -> None:
        """
        Simulate one step of robot dynamics.
        
        This is a placeholder implementation. In a full system, this would
        interface with MuJoCo or another physics simulator.
        """
        if self.robot_state is None:
            return
            
        # Simple dynamics model for demonstration
        # Update joint positions (simplified)
        self.robot_state.joint_positions = action
        
        # Simple velocity integration based on actions
        forward_component = np.mean(action[3::5])  # Average knee bending
        lateral_component = np.mean(action[1::5])  # Average hip roll
        
        # Update velocities with simple mapping
        self.robot_state.linear_velocity[0] = forward_component * 0.5
        self.robot_state.linear_velocity[1] = lateral_component * 0.3
        
        # Update position
        self.robot_state.position += self.robot_state.linear_velocity * self.dt
        
        # Simple stability calculation
        self.robot_state.stability_margin = max(0.0, 0.5 - np.abs(forward_component))
    
    def _get_observation(self) -> np.ndarray:
        """Get current observation vector."""
        if self.robot_state is None:
            return np.zeros(self.observation_dim)
            
        obs = np.concatenate([
            self.robot_state.position,
            self.robot_state.orientation,
            self.robot_state.linear_velocity,
            self.robot_state.angular_velocity,
            self.robot_state.joint_positions,
            self.robot_state.joint_velocities,
            self.robot_state.contact_forces,
            self.robot_state.center_of_mass,
            [self.robot_state.stability_margin]
        ])
        
        return obs
    
    def _calculate_reward(self, action: np.ndarray) -> float:
        """
        Calculate reward for omnidirectional gait learning.
        
        Reward components:
        - Velocity tracking: Match target velocity in desired direction
        - Stability: Maintain upright posture and balance
        - Energy efficiency: Minimize unnecessary movements
        - Smoothness: Encourage smooth gait patterns
        """
        if self.robot_state is None:
            return 0.0
            
        reward = 0.0
        
        # 1. Velocity tracking reward
        velocity_error = np.linalg.norm(
            self.robot_state.linear_velocity[:2] - self.target_velocity_vector[:2]
        )
        velocity_reward = np.exp(-2.0 * velocity_error)
        reward += velocity_reward
        
        # 2. Stability reward
        stability_reward = np.clip(self.robot_state.stability_margin / self.stability_threshold, 0.0, 1.0)
        reward += stability_reward
        
        # 3. Upright posture reward
        tilt_penalty = np.abs(self.robot_state.orientation[0]) + np.abs(self.robot_state.orientation[1])
        posture_reward = np.exp(-5.0 * tilt_penalty)
        reward += posture_reward
        
        # 4. Energy efficiency reward (penalize large joint velocities)
        energy_penalty = np.mean(np.square(self.robot_state.joint_velocities))
        energy_reward = np.exp(-0.1 * energy_penalty)
        reward += 0.5 * energy_reward
        
        # 5. Contact force balance (both feet should share load)
        contact_balance = 1.0 - abs(self.robot_state.contact_forces[0] - self.robot_state.contact_forces[1]) / 800.0
        reward += 0.3 * max(0.0, contact_balance)
        
        return reward
    
    def _is_episode_done(self) -> bool:
        """Check if episode should terminate."""
        if self.robot_state is None:
            return True
            
        # Episode ends if:
        # 1. Maximum steps reached
        if self.current_step >= self.max_episode_steps:
            return True
            
        # 2. Robot falls (height too low)
        if self.robot_state.position[2] < 0.3:
            return True
            
        # 3. Robot becomes too unstable
        if self.robot_state.stability_margin < 0.0:
            return True
            
        # 4. Robot tilts too much
        if np.abs(self.robot_state.orientation[0]) > 0.5 or np.abs(self.robot_state.orientation[1]) > 0.5:
            return True
            
        return False
    
    def render(self, mode: str = 'human') -> None:
        """Render the environment (placeholder for visualization)."""
        if mode == 'human' and self.robot_state is not None:
            print(f"Step: {self.current_step}")
            print(f"Target: {self.target_direction.value}")
            print(f"Position: {self.robot_state.position}")
            print(f"Velocity: {self.robot_state.linear_velocity}")
            print(f"Stability: {self.robot_state.stability_margin:.3f}")
            print(f"Reward: {self.episode_reward:.3f}")
            print("-" * 40)