import json
import random
from typing import List, Dict, Any
import numpy as np
import Utils
import mujoco  # type: ignore

# Constants
MAX_RANGE = 1.5708  # 90 degrees in radians
LEARNING_RATE = 0.1  # Learning rate for policy updates
MOMENTUM = 0.9  # Momentum for smoothing updates

# Joint names for the robot
joint_names = [
    "left_pitch_hip",
    "left_roll_hip",
    "left_yaw_hip",
    "left_knee",
    "left_ankle",
    "right_pitch_hip",
    "right_roll_hip",
    "right_yaw_hip",
    "right_knee",
    "right_ankle"
]


class Policy:
    """
    A policy to represent a trained model.
    
    Attributes:
        weights (List[float]): The weights of the policy.
        biases (List[float]): The biases of the policy.
        velocity_weights (List[float]): Velocity for weights (used for momentum).
        velocity_biases (List[float]): Velocity for biases (used for momentum).
    """
    def __init__(self, weights: List[float] = [0.0] * len(joint_names), biases: List[float] = [0.0] * len(joint_names)):
        self.weights = weights
        self.biases = biases
        self.velocity_weights = [0.0] * len(weights)
        self.velocity_biases = [0.0] * len(biases)

    def __init__(self, filename: str):
        """
        Initialize the policy from a file.
        
        Args:
            filename: The name of the file containing the policy data.
        """
        with open(filename, 'r') as f:
            data = json.load(f)
        
        self.weights = data["weights"]
        self.biases = data["biases"]

    def to_dict(self) -> Dict[str, Any]:
        """
        Convert the policy to a dictionary.
        
        Returns:
            A dictionary representation of the policy.
        """
        return {
            "weights": self.weights,
            "biases": self.biases
        }

    @staticmethod
    def from_dict(data: Dict[str, Any]) -> 'Policy':
        """
        Create a policy from a dictionary.
        
        Args:
            data: A dictionary containing the policy data.
        
        Returns:
            A Policy object.
        """
        return Policy(weights=data["weights"], biases=data["biases"])

    def reset_policy(self):
        """
        Reset the policy to its initial state.
        """
        weights = [random.uniform(-MAX_RANGE, MAX_RANGE) for _ in range(len(joint_names))]
        biases = [random.uniform(-MAX_RANGE, MAX_RANGE) for _ in range(len(joint_names))]
        
        self.weights = weights
        self.biases = biases


    def get_action(self, model, data, exploration_rate: float = 0.3) -> Dict[str, float]:
        """
        Get the action based on the policy with added exploration.
        
        Args:
            model: The Mujoco model.
            data: The Mujoco data.
            policy: The policy to use for generating the action.
            exploration_rate: The probability of taking a random action (exploration).
        
        Returns:
            A dictionary of joint positions as the action.
        """
        joint_positions = {}
        for joint_name in joint_names:
            # Get the current joint position
            current_position = Utils.get_joint_position(model, data, joint_name)
            
            # Apply the policy (weights and biases)
            index = joint_names.index(joint_name)
            action = current_position + self.biases[index]
            
            # Add exploration (randomness)
            if random.random() < exploration_rate:
                action += random.uniform(-0.5, 0.5)  # Add small random noise
            
            # Clamp the position to the valid range
            joint_positions[joint_name] = max(-MAX_RANGE, min(MAX_RANGE, action))

        return joint_positions
    
    def train_policy(self, reward: float):
        print("trained")


def save_policy(policy: Dict[str, Any]):
    """
    Save the policy to a file.
    
    Args:
        policy: The policy to save.
    """
    with open('policy.json', 'w') as f:
        json.dump(policy, f)


def load_policy(filename: str = 'policy.json') -> Policy:
    """
    Load the policy from a file.
    
    Returns:
        The loaded policy.
    """
    with open(str, 'r') as f:
        data = json.load(f)
    return Policy.from_dict(data)


def apply_action(model, data, action: Dict[str, float]):
    """
    Apply an action to the Mujoco model.
    
    Args:
        model: The Mujoco model.
        data: The Mujoco data.
        action: The action to apply.
    """
    for joint_name, position in action.items():
        Utils.set_joint_position(model, data, joint_name, position)


def get_reward(model, data, time_spent: float) -> float:
    """
    Get the reward for the current state of the Mujoco model.

    The reward is based on the y-coordinate of the pelvis position. The more negative the y-coordinate, the higher the reward.
    The reward is also calculated based on time spent in the simulation. More time spent in the simulation results in a higher reward.
    
    Args:
        model: The Mujoco model.
        data: The Mujoco data.
        time_spent: The time spent in the simulation.
    
    Returns:
        The reward for the current state.
    """
    pelvis_position = Utils.get_body_position_global(model, data, 'Pelvis')

    reward = -pelvis_position[1] + time_spent * 0.5

    return reward

class NeuralNetwork:
    """
    A neural network class for policy approximation.
    """
    
