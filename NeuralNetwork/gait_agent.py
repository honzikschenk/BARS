"""
Reinforcement Learning Agent for Omnidirectional Gait Learning
==============================================================

This module implements a policy-gradient based agent for learning stable
omnidirectional gaits on the BARS bipedal robot.
"""

import numpy as np
from typing import List, Dict, Tuple, Optional
import json
from dataclasses import dataclass, asdict
from gait_environment import BipedalGaitEnvironment, GaitDirection


@dataclass
class NetworkConfig:
    """Configuration for neural network architecture."""
    hidden_layers: List[int]
    activation: str = "tanh"
    learning_rate: float = 0.001
    momentum: float = 0.9
    weight_decay: float = 0.01


class SimplePolicy:
    """
    A simple neural network policy for gait control.
    
    Uses a feedforward network to map observations to joint angle commands.
    Designed specifically for omnidirectional bipedal locomotion.
    """
    
    def __init__(self, observation_dim: int, action_dim: int, config: NetworkConfig):
        """
        Initialize the policy network.
        
        Args:
            observation_dim: Size of observation vector
            action_dim: Size of action vector (number of joints)
            config: Network configuration parameters
        """
        self.observation_dim = observation_dim
        self.action_dim = action_dim
        self.config = config
        
        # Initialize network layers
        layer_sizes = [observation_dim] + config.hidden_layers + [action_dim]
        self.weights = []
        self.biases = []
        
        # Xavier initialization for weights
        for i in range(len(layer_sizes) - 1):
            in_size, out_size = layer_sizes[i], layer_sizes[i + 1]
            w = np.random.normal(0, np.sqrt(2.0 / in_size), (in_size, out_size))
            b = np.zeros(out_size)
            self.weights.append(w)
            self.biases.append(b)
        
        # Momentum terms for optimization
        self.weight_momentum = [np.zeros_like(w) for w in self.weights]
        self.bias_momentum = [np.zeros_like(b) for b in self.biases]
        
        # Training statistics
        self.training_step = 0
        self.episode_returns = []
        
    def forward(self, observation: np.ndarray) -> np.ndarray:
        """
        Forward pass through the network.
        
        Args:
            observation: Current state observation
            
        Returns:
            Action vector (joint angle targets)
        """
        x = observation.copy()
        
        # Forward pass through hidden layers
        for i in range(len(self.weights) - 1):
            x = np.dot(x, self.weights[i]) + self.biases[i]
            # Apply activation function
            if self.config.activation == "tanh":
                x = np.tanh(x)
            elif self.config.activation == "relu":
                x = np.maximum(0, x)
        
        # Output layer (no activation for continuous control)
        action = np.dot(x, self.weights[-1]) + self.biases[-1]
        
        # Scale to reasonable joint ranges
        action = np.tanh(action) * 1.0  # Scale to [-1, 1] radians
        
        return action
    
    def get_action(self, observation: np.ndarray, add_noise: bool = False) -> np.ndarray:
        """
        Get action for given observation, optionally with exploration noise.
        
        Args:
            observation: Current state
            add_noise: Whether to add exploration noise
            
        Returns:
            Action to execute
        """
        action = self.forward(observation)
        
        if add_noise:
            # Add Gaussian noise for exploration
            noise_scale = max(0.1, 1.0 - self.training_step / 10000.0)  # Decay noise
            noise = np.random.normal(0, noise_scale, action.shape)
            action += noise
            
        # Clip to safe ranges
        action = np.clip(action, -1.5, 1.5)
        
        return action
    
    def update_policy(self, trajectories: List[Dict]) -> Dict[str, float]:
        """
        Update policy using collected trajectory data.
        
        Args:
            trajectories: List of trajectory dictionaries containing
                         observations, actions, rewards, etc.
                         
        Returns:
            Training statistics dictionary
        """
        if not trajectories:
            return {}
        
        # Calculate returns and advantages
        all_returns = []
        all_observations = []
        all_actions = []
        
        for traj in trajectories:
            returns = self._calculate_returns(traj['rewards'])
            all_returns.extend(returns)
            all_observations.extend(traj['observations'])
            all_actions.extend(traj['actions'])
        
        all_returns = np.array(all_returns)
        all_observations = np.array(all_observations)
        all_actions = np.array(all_actions)
        
        # Normalize returns for stability
        returns_mean = np.mean(all_returns)
        returns_std = np.std(all_returns) + 1e-8
        normalized_returns = (all_returns - returns_mean) / returns_std
        
        # Compute policy gradients
        weight_grads = [np.zeros_like(w) for w in self.weights]
        bias_grads = [np.zeros_like(b) for b in self.biases]
        
        for obs, action, ret in zip(all_observations, all_actions, normalized_returns):
            # Forward pass to get predicted action
            pred_action = self.forward(obs)
            
            # Policy gradient (simplified for demonstration)
            action_error = action - pred_action
            grad_scale = ret * self.config.learning_rate
            
            # Backpropagate (simplified)
            self._accumulate_gradients(obs, action_error * grad_scale, weight_grads, bias_grads)
        
        # Apply gradients with momentum
        self._apply_gradients(weight_grads, bias_grads, len(all_returns))
        
        self.training_step += 1
        
        # Store episode returns for tracking
        episode_returns = [sum(traj['rewards']) for traj in trajectories]
        self.episode_returns.extend(episode_returns)
        
        return {
            'mean_return': returns_mean,
            'std_return': returns_std,
            'mean_episode_return': np.mean(episode_returns),
            'training_step': self.training_step
        }
    
    def _calculate_returns(self, rewards: List[float], gamma: float = 0.99) -> List[float]:
        """Calculate discounted returns from rewards."""
        returns = []
        running_return = 0.0
        
        for reward in reversed(rewards):
            running_return = reward + gamma * running_return
            returns.append(running_return)
        
        return list(reversed(returns))
    
    def _accumulate_gradients(self, observation: np.ndarray, error: np.ndarray,
                            weight_grads: List[np.ndarray], bias_grads: List[np.ndarray]) -> None:
        """Accumulate gradients for one sample (simplified backprop)."""
        # This is a simplified gradient calculation
        # In practice, you'd want proper automatic differentiation
        
        # For demonstration, use finite differences approximation
        epsilon = 1e-6
        
        for layer in range(len(self.weights)):
            # Weight gradients
            for i in range(self.weights[layer].shape[0]):
                for j in range(self.weights[layer].shape[1]):
                    # Finite difference approximation
                    self.weights[layer][i, j] += epsilon
                    action_plus = self.forward(observation)
                    self.weights[layer][i, j] -= 2 * epsilon
                    action_minus = self.forward(observation)
                    self.weights[layer][i, j] += epsilon  # Restore
                    
                    grad = np.sum((action_plus - action_minus) * error) / (2 * epsilon)
                    weight_grads[layer][i, j] += grad
            
            # Bias gradients (simplified)
            for j in range(len(self.biases[layer])):
                self.biases[layer][j] += epsilon
                action_plus = self.forward(observation)
                self.biases[layer][j] -= 2 * epsilon
                action_minus = self.forward(observation)
                self.biases[layer][j] += epsilon  # Restore
                
                grad = np.sum((action_plus - action_minus) * error) / (2 * epsilon)
                bias_grads[layer][j] += grad
    
    def _apply_gradients(self, weight_grads: List[np.ndarray], bias_grads: List[np.ndarray], batch_size: int) -> None:
        """Apply accumulated gradients with momentum."""
        for i in range(len(self.weights)):
            # Average gradients over batch
            weight_grads[i] /= batch_size
            bias_grads[i] /= batch_size
            
            # Apply momentum
            self.weight_momentum[i] = (self.config.momentum * self.weight_momentum[i] + 
                                     self.config.learning_rate * weight_grads[i])
            self.bias_momentum[i] = (self.config.momentum * self.bias_momentum[i] + 
                                   self.config.learning_rate * bias_grads[i])
            
            # Update weights
            self.weights[i] += self.weight_momentum[i]
            self.biases[i] += self.bias_momentum[i]
            
            # Apply weight decay
            self.weights[i] *= (1.0 - self.config.weight_decay)
    
    def save(self, filepath: str) -> None:
        """Save policy to file."""
        policy_data = {
            'observation_dim': self.observation_dim,
            'action_dim': self.action_dim,
            'config': asdict(self.config),
            'weights': [w.tolist() for w in self.weights],
            'biases': [b.tolist() for b in self.biases],
            'training_step': self.training_step,
            'episode_returns': self.episode_returns[-100:]  # Save last 100 returns
        }
        
        with open(filepath, 'w') as f:
            json.dump(policy_data, f, indent=2)
    
    def load(self, filepath: str) -> None:
        """Load policy from file."""
        with open(filepath, 'r') as f:
            policy_data = json.load(f)
        
        self.observation_dim = policy_data['observation_dim']
        self.action_dim = policy_data['action_dim']
        self.config = NetworkConfig(**policy_data['config'])
        self.weights = [np.array(w) for w in policy_data['weights']]
        self.biases = [np.array(b) for b in policy_data['biases']]
        self.training_step = policy_data['training_step']
        self.episode_returns = policy_data['episode_returns']
        
        # Reinitialize momentum terms
        self.weight_momentum = [np.zeros_like(w) for w in self.weights]
        self.bias_momentum = [np.zeros_like(b) for b in self.biases]


class OmnidirectionalGaitAgent:
    """
    Complete agent for learning omnidirectional gaits.
    
    Manages the training process, policy updates, and evaluation.
    """
    
    def __init__(self, env: BipedalGaitEnvironment, config: Optional[NetworkConfig] = None):
        """
        Initialize the gait learning agent.
        
        Args:
            env: Bipedal gait environment
            config: Network configuration (uses default if None)
        """
        self.env = env
        
        if config is None:
            config = NetworkConfig(
                hidden_layers=[64, 32],
                activation="tanh",
                learning_rate=0.001,
                momentum=0.9,
                weight_decay=0.01
            )
        
        self.policy = SimplePolicy(env.observation_dim, env.action_dim, config)
        self.training_history = []
        
    def collect_trajectory(self, target_direction: Optional[GaitDirection] = None, 
                         max_steps: int = 1000, explore: bool = True) -> Dict:
        """
        Collect a complete trajectory from the environment.
        
        Args:
            target_direction: Desired movement direction
            max_steps: Maximum steps per trajectory
            explore: Whether to add exploration noise
            
        Returns:
            Trajectory dictionary with observations, actions, rewards, etc.
        """
        observations = []
        actions = []
        rewards = []
        
        obs = self.env.reset(target_direction)
        done = False
        step = 0
        
        while not done and step < max_steps:
            # Get action from policy
            action = self.policy.get_action(obs, add_noise=explore)
            
            # Store experience
            observations.append(obs.copy())
            actions.append(action.copy())
            
            # Execute action
            obs, reward, done, info = self.env.step(action)
            rewards.append(reward)
            
            step += 1
        
        return {
            'observations': observations,
            'actions': actions,
            'rewards': rewards,
            'length': len(rewards),
            'total_reward': sum(rewards),
            'target_direction': target_direction.value if target_direction else 'random'
        }
    
    def train_episode(self, num_trajectories: int = 5) -> Dict[str, float]:
        """
        Train for one episode using multiple trajectories.
        
        Args:
            num_trajectories: Number of trajectories to collect
            
        Returns:
            Training statistics
        """
        trajectories = []
        
        # Collect trajectories with different target directions
        directions = list(GaitDirection)
        for i in range(num_trajectories):
            target_dir = directions[i % len(directions)]
            traj = self.collect_trajectory(target_direction=target_dir, explore=True)
            trajectories.append(traj)
        
        # Update policy
        training_stats = self.policy.update_policy(trajectories)
        
        # Add trajectory statistics
        training_stats.update({
            'num_trajectories': len(trajectories),
            'mean_trajectory_length': np.mean([t['length'] for t in trajectories]),
            'mean_trajectory_reward': np.mean([t['total_reward'] for t in trajectories])
        })
        
        self.training_history.append(training_stats)
        
        return training_stats
    
    def evaluate(self, target_direction: GaitDirection, num_episodes: int = 5) -> Dict[str, float]:
        """
        Evaluate the current policy on a specific direction.
        
        Args:
            target_direction: Direction to evaluate
            num_episodes: Number of evaluation episodes
            
        Returns:
            Evaluation statistics
        """
        episode_rewards = []
        episode_lengths = []
        success_count = 0
        
        for _ in range(num_episodes):
            traj = self.collect_trajectory(target_direction=target_direction, explore=False)
            episode_rewards.append(traj['total_reward'])
            episode_lengths.append(traj['length'])
            
            # Count as success if episode completed without early termination
            if traj['length'] >= 800:  # Most of max episode length
                success_count += 1
        
        return {
            'target_direction': target_direction.value,
            'mean_reward': np.mean(episode_rewards),
            'std_reward': np.std(episode_rewards),
            'mean_length': np.mean(episode_lengths),
            'success_rate': success_count / num_episodes,
            'num_episodes': num_episodes
        }
    
    def save_agent(self, filepath: str) -> None:
        """Save the complete agent state."""
        self.policy.save(filepath)
        
        # Save additional agent data
        agent_data = {
            'training_history': self.training_history[-50:],  # Last 50 training steps
            'config': asdict(self.policy.config)
        }
        
        history_path = filepath.replace('.json', '_history.json')
        with open(history_path, 'w') as f:
            json.dump(agent_data, f, indent=2)
    
    def load_agent(self, filepath: str) -> None:
        """Load the complete agent state."""
        self.policy.load(filepath)
        
        # Load training history if available
        history_path = filepath.replace('.json', '_history.json')
        try:
            with open(history_path, 'r') as f:
                agent_data = json.load(f)
            self.training_history = agent_data.get('training_history', [])
        except FileNotFoundError:
            pass  # No history file, continue with empty history