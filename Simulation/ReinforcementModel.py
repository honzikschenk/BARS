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
    A class to represent a policy.
    
    Attributes:
        weights (List[float]): The weights of the policy.
        biases (List[float]): The biases of the policy.
        velocity_weights (List[float]): Velocity for weights (used for momentum).
        velocity_biases (List[float]): Velocity for biases (used for momentum).
    """
    def __init__(self, weights: List[float], biases: List[float]):
        self.weights = weights
        self.biases = biases
        self.velocity_weights = [0.0] * len(weights)  # Initialize velocities for momentum
        self.velocity_biases = [0.0] * len(biases)

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

def reset_policy() -> Policy:
    """
    Reset the policy to its initial state.
    
    Returns:
        The reset policy.
    """
    weights = [random.uniform(-MAX_RANGE, MAX_RANGE) for _ in range(len(joint_names))]
    biases = [random.uniform(-MAX_RANGE, MAX_RANGE) for _ in range(len(joint_names))]
    return Policy(weights=weights, biases=biases)


def get_action(model, data, policy: Policy, exploration_rate: float = 0.3) -> Dict[str, float]:
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
        action = current_position + policy.biases[index]
        
        # Add exploration (randomness)
        if random.random() < exploration_rate:
            action += random.uniform(-0.5, 0.5)  # Add small random noise
        
        # Clamp the position to the valid range
        joint_positions[joint_name] = max(-MAX_RANGE, min(MAX_RANGE, action))

    return joint_positions


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


def train_policy(policy: Policy, reward: float) -> Policy:
    """
    Train the policy based on the reward using learning rate and momentum.
    
    Args:
        policy: The policy to train.
        reward: The reward to use for training.
    
    Returns:
        The trained policy.
    """
    for i in range(len(policy.weights)):
        # Calculate gradients based on reward and current policy output
        gradient_weight = reward * policy.weights[i]
        gradient_bias = reward * policy.biases[i]

        # Update velocities using momentum
        policy.velocity_weights[i] = MOMENTUM * policy.velocity_weights[i] + LEARNING_RATE * gradient_weight
        policy.velocity_biases[i] = MOMENTUM * policy.velocity_biases[i] + LEARNING_RATE * gradient_bias

        # Update weights and biases using velocities
        policy.weights[i] += policy.velocity_weights[i]
        policy.biases[i] += policy.velocity_biases[i]

    # Ensure weights and biases are within the valid range
    policy.weights = [max(-MAX_RANGE, min(MAX_RANGE, w)) for w in policy.weights]
    policy.biases = [max(-MAX_RANGE, min(MAX_RANGE, b)) for b in policy.biases]

    # Save the updated policy
    save_policy(policy.to_dict())

    return policy


def save_policy(policy: Dict[str, Any]):
    """
    Save the policy to a file.
    
    Args:
        policy: The policy to save.
    """
    with open('policy.json', 'w') as f:
        json.dump(policy, f)


def load_policy() -> Policy:
    """
    Load the policy from a file.
    
    Returns:
        The loaded policy.
    """
    with open('policy.json', 'r') as f:
        data = json.load(f)
    return Policy.from_dict(data)
