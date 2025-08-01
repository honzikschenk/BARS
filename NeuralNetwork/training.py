"""
Training utilities and main training loop for omnidirectional gait learning
===========================================================================

This module provides training utilities and the main training loop for
learning stable omnidirectional gaits on the BARS bipedal robot.
"""

import numpy as np
import time
import json
from typing import Dict, List, Optional
from pathlib import Path

from gait_environment import BipedalGaitEnvironment, GaitDirection
from gait_agent import OmnidirectionalGaitAgent, NetworkConfig


class TrainingConfig:
    """Configuration parameters for training."""
    
    def __init__(self,
                 max_episodes: int = 1000,
                 trajectories_per_episode: int = 5,
                 evaluation_frequency: int = 50,
                 save_frequency: int = 100,
                 target_reward: float = 15.0,
                 early_stopping_patience: int = 200):
        """
        Initialize training configuration.
        
        Args:
            max_episodes: Maximum training episodes
            trajectories_per_episode: Number of trajectories per training episode
            evaluation_frequency: Episodes between evaluations
            save_frequency: Episodes between model saves
            target_reward: Target reward for early stopping
            early_stopping_patience: Episodes to wait for improvement
        """
        self.max_episodes = max_episodes
        self.trajectories_per_episode = trajectories_per_episode
        self.evaluation_frequency = evaluation_frequency
        self.save_frequency = save_frequency
        self.target_reward = target_reward
        self.early_stopping_patience = early_stopping_patience


class TrainingLogger:
    """Handles logging and tracking of training progress."""
    
    def __init__(self, log_dir: str = "training_logs"):
        """Initialize logger with output directory."""
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(exist_ok=True)
        
        self.training_log = []
        self.evaluation_log = []
        
    def log_training_step(self, episode: int, stats: Dict) -> None:
        """Log training step statistics."""
        log_entry = {
            'episode': episode,
            'timestamp': time.time(),
            **stats
        }
        self.training_log.append(log_entry)
        
        # Print progress
        if episode % 10 == 0:
            print(f"Episode {episode}: "
                  f"Reward: {stats.get('mean_trajectory_reward', 0):.2f}, "
                  f"Length: {stats.get('mean_trajectory_length', 0):.1f}")
    
    def log_evaluation(self, episode: int, evaluation_results: Dict) -> None:
        """Log evaluation results."""
        log_entry = {
            'episode': episode,
            'timestamp': time.time(),
            **evaluation_results
        }
        self.evaluation_log.append(log_entry)
        
        print(f"\n=== Evaluation at Episode {episode} ===")
        print(f"Direction: {evaluation_results['target_direction']}")
        print(f"Mean Reward: {evaluation_results['mean_reward']:.2f}")
        print(f"Success Rate: {evaluation_results['success_rate']:.2%}")
        print(f"Mean Length: {evaluation_results['mean_length']:.1f}")
        print("=" * 40)
    
    def save_logs(self) -> None:
        """Save logs to files."""
        training_file = self.log_dir / "training_log.json"
        evaluation_file = self.log_dir / "evaluation_log.json"
        
        with open(training_file, 'w') as f:
            json.dump(self.training_log, f, indent=2)
            
        with open(evaluation_file, 'w') as f:
            json.dump(self.evaluation_log, f, indent=2)


def train_omnidirectional_gait(training_config: TrainingConfig,
                             network_config: Optional[NetworkConfig] = None,
                             save_dir: str = "models",
                             log_dir: str = "training_logs") -> OmnidirectionalGaitAgent:
    """
    Main training function for omnidirectional gait learning.
    
    Args:
        training_config: Training parameters
        network_config: Neural network configuration
        save_dir: Directory to save trained models
        log_dir: Directory to save training logs
        
    Returns:
        Trained agent
    """
    print("=" * 60)
    print("BARS Omnidirectional Gait Learning")
    print("=" * 60)
    
    # Create directories
    save_path = Path(save_dir)
    save_path.mkdir(exist_ok=True)
    
    # Initialize environment and agent
    env = BipedalGaitEnvironment()
    agent = OmnidirectionalGaitAgent(env, network_config)
    logger = TrainingLogger(log_dir)
    
    print(f"Environment initialized:")
    print(f"  Observation dimension: {env.observation_dim}")
    print(f"  Action dimension: {env.action_dim}")
    print(f"  Joint names: {env.joint_names}")
    print()
    
    # Training loop
    best_reward = -float('inf')
    episodes_without_improvement = 0
    start_time = time.time()
    
    for episode in range(training_config.max_episodes):
        # Training step
        training_stats = agent.train_episode(training_config.trajectories_per_episode)
        logger.log_training_step(episode, training_stats)
        
        # Check for improvement
        current_reward = training_stats.get('mean_trajectory_reward', -float('inf'))
        if current_reward > best_reward:
            best_reward = current_reward
            episodes_without_improvement = 0
            
            # Save best model
            best_model_path = save_path / "best_model.json"
            agent.save_agent(str(best_model_path))
        else:
            episodes_without_improvement += 1
        
        # Evaluation
        if episode % training_config.evaluation_frequency == 0:
            # Evaluate on a few different directions
            directions_to_test = [GaitDirection.FORWARD, GaitDirection.LEFT_STRAFE, GaitDirection.ROTATE_LEFT]
            
            for direction in directions_to_test:
                eval_results = agent.evaluate(direction, num_episodes=3)
                logger.log_evaluation(episode, eval_results)
        
        # Save periodic checkpoint
        if episode % training_config.save_frequency == 0:
            checkpoint_path = save_path / f"checkpoint_episode_{episode}.json"
            agent.save_agent(str(checkpoint_path))
        
        # Early stopping check
        if current_reward >= training_config.target_reward:
            print(f"\nTarget reward {training_config.target_reward} reached! Stopping training.")
            break
            
        if episodes_without_improvement >= training_config.early_stopping_patience:
            print(f"\nNo improvement for {training_config.early_stopping_patience} episodes. Stopping training.")
            break
    
    # Final evaluation on all directions
    print("\n" + "=" * 60)
    print("Final Evaluation")
    print("=" * 60)
    
    final_results = {}
    for direction in GaitDirection:
        eval_results = agent.evaluate(direction, num_episodes=5)
        final_results[direction.value] = eval_results
        logger.log_evaluation(-1, eval_results)  # -1 indicates final evaluation
    
    # Save final model and logs
    final_model_path = save_path / "final_model.json"
    agent.save_agent(str(final_model_path))
    logger.save_logs()
    
    # Training summary
    elapsed_time = time.time() - start_time
    print(f"\nTraining completed in {elapsed_time:.1f} seconds")
    print(f"Best reward achieved: {best_reward:.2f}")
    print(f"Final model saved to: {final_model_path}")
    
    return agent


def demo_training():
    """
    Demonstration of the training process with default parameters.
    
    This function can be called to run a short training demo.
    """
    print("Running omnidirectional gait training demo...")
    
    # Configure for quick demo
    training_config = TrainingConfig(
        max_episodes=100,
        trajectories_per_episode=3,
        evaluation_frequency=20,
        save_frequency=50,
        target_reward=10.0,
        early_stopping_patience=50
    )
    
    network_config = NetworkConfig(
        hidden_layers=[32, 16],
        activation="tanh",
        learning_rate=0.01,
        momentum=0.9,
        weight_decay=0.001
    )
    
    # Run training
    agent = train_omnidirectional_gait(
        training_config=training_config,
        network_config=network_config,
        save_dir="demo_models",
        log_dir="demo_logs"
    )
    
    return agent


def evaluate_trained_model(model_path: str, num_episodes: int = 10):
    """
    Evaluate a trained model on all movement directions.
    
    Args:
        model_path: Path to saved model
        num_episodes: Number of episodes per direction
    """
    print(f"Evaluating model: {model_path}")
    
    # Initialize environment and agent
    env = BipedalGaitEnvironment()
    agent = OmnidirectionalGaitAgent(env)
    
    # Load trained model
    agent.load_agent(model_path)
    
    # Evaluate on all directions
    print("\nEvaluation Results:")
    print("-" * 50)
    
    for direction in GaitDirection:
        results = agent.evaluate(direction, num_episodes=num_episodes)
        print(f"{direction.value:12}: "
              f"Reward: {results['mean_reward']:6.2f} ± {results['std_reward']:5.2f}, "
              f"Success: {results['success_rate']:5.1%}")


if __name__ == "__main__":
    # Run demo training when script is executed directly
    demo_training()