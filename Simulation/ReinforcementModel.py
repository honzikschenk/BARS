# Reward is get_body_position_global(m, data, 'Pelvis')[1] as negative as possible
# Penalty is resetting the simulation if the pelvis is too low
# Actions are the joint positions
# 

import json
import random
from typing import List, Dict, Any
import numpy as np

import Utils

import mujoco # type: ignore

# Constants
MAX_RANGE = 1.5708 # 90 degrees in radians

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
    """
    def __init__(self, weights: List[float], biases: List[float]):
        self.weights = weights
        self.biases = biases

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



class Action:
    """
    A class to represent an action.
    
    Attributes:
        joint_positions (Dict[str, float]): The joint positions for the action, keyed by joint name.
    """
    joint_positions: Dict[str, float] = {}

    def __init__(self, joint_positions: Dict[str, float]):
        for joint_name, position in joint_positions.items():
            self.joint_positions[joint_name] = position

    def to_dict(self) -> Dict[str, Any]:
        """
        Convert the action to a dictionary.
        
        Returns:
            A dictionary representation of the action.
        """
        return {
            "joint_positions": self.joint_positions
        }

    @staticmethod
    def from_dict(data: Dict[str, Any]) -> 'Action':
        """
        Create an action from a dictionary.
        
        Args:
            data: A dictionary containing the action data.
        
        Returns:
            An Action object.
        """
        return Action(joint_positions=data["joint_positions"])



def load_policy():
    """
    Load the policy from a file.
    
    Returns:
        The loaded policy.
    """
    # Load the policy from a file
    with open('policy.json', 'r') as f:
        policy = json.load(f)
    
    return policy

def save_policy(policy):
    """
    Save the policy to a file.
    
    Args:
        policy: The policy to save.
    """
    # Save the policy to a file
    with open('policy.json', 'w') as f:
        json.dump(policy, f)

def return_apply_action(policy):
    return policy

def train_policy(model, data, policy: Policy, reward: float):
    """
    Train the policy based on the model and data.
    
    Args:
        model: The Mujoco model.
        data: The Mujoco data.
        policy: The policy to train.
    
    Returns:
        The trained policy.
    """
    return policy

def reset_policy():
    """
    Reset the policy to its initial state.
    
    Returns:
        The reset policy.
    """
    # Reset the policy to its initial state
    # This is a placeholder for actual reset logic
    # For now, we will just return a new policy with random weights and biases
    weights = [random.uniform(-MAX_RANGE, MAX_RANGE) for _ in range(len(joint_names))]
    biases = [random.uniform(-MAX_RANGE, MAX_RANGE) for _ in range(len(joint_names))]
    return Policy(weights=weights, biases=biases)

def get_action(model, data, policy: Policy):
    """
    Get the action based on the policy.
    
    Args:
        model: The Mujoco model.
        data: The Mujoco data.
        policy: The policy to use for generating the action.
    
    Returns:
        The generated action.
    """
    # Get the joint positions from the policy
    joint_positions = {}

    for joint_name, weight in zip(joint_names, policy.weights):
        joint_positions[joint_name] = Utils.get_joint_position(model, data, joint_name) + weight

    action = Action(joint_positions)

    return action

def get_random_action():
    """
    Generate a random action.
    
    Returns:
        A random action.
    """
    joint_positions = {}
    for joint_name in joint_names:
        joint_positions[joint_name] = random.uniform(-MAX_RANGE, MAX_RANGE)

    action = Action(joint_positions)
    return action

def apply_action(model, data, action: Action):
    """
    Apply an action to the Mujoco model.
    
    Args:
        model: The Mujoco model.
        data: The Mujoco data.
        action: The action to apply.
    """
    # Apply the action to the Mujoco model
    for joint_name, position in action.joint_positions.items():
        Utils.set_joint_position(model, data, joint_name, position)
    
def get_reward(model, data, time_spent: float):
    """
    Get the reward for the current state of the Mujoco model.

    The reward is based on the y-coordinate of the pelvis position. The more negative the y-coordinate, the higher the reward.
    The reward is also calculated based on time spent in the simulation. More time spent in the simulation results in a higher reward.
    
    Args:
        model: The Mujoco model.
        data: The Mujoco data.
    
    Returns:
        The reward for the current state.
    """
    # Get the global position of the pelvis
    pelvis_position = Utils.get_body_position_global(model, data, 'Pelvis')

    reward = -pelvis_position[1]
    reward += time_spent
    
    return reward
