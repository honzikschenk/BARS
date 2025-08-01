#!/usr/bin/env python3
"""
Omnidirectional Gait Learning Framework for BARS
=================================================

This script demonstrates the beginning reinforcement learning framework
for learning omnidirectional gaits on the BARS bipedal robot.

Usage:
    python gait_learning_demo.py [--train] [--evaluate MODEL_PATH] [--demo]
"""

import argparse
import sys
from pathlib import Path

# Import our gait learning framework
from gait_environment import BipedalGaitEnvironment, GaitDirection
from gait_agent import OmnidirectionalGaitAgent, NetworkConfig
from training import TrainingConfig, train_omnidirectional_gait, evaluate_trained_model
from config import get_default_configs, create_default_config_files


def run_demo():
    """Run a quick demonstration of the framework."""
    print("=" * 60)
    print("BARS Omnidirectional Gait Learning - Demo")
    print("=" * 60)
    
    # Initialize environment
    print("1. Initializing environment...")
    env = BipedalGaitEnvironment()
    print(f"   ✓ Environment created with {env.observation_dim} observations, {env.action_dim} actions")
    
    # Initialize agent
    print("2. Initializing agent...")
    config = NetworkConfig(hidden_layers=[32, 16], learning_rate=0.01)
    agent = OmnidirectionalGaitAgent(env, config)
    print("   ✓ Agent created with neural network policy")
    
    # Test different movement directions
    print("3. Testing movement directions...")
    directions = [GaitDirection.FORWARD, GaitDirection.LEFT_STRAFE, GaitDirection.ROTATE_LEFT]
    
    for direction in directions:
        print(f"   Testing {direction.value}...")
        trajectory = agent.collect_trajectory(direction, max_steps=50, explore=False)
        print(f"   ✓ {direction.value}: {trajectory['length']} steps, "
              f"reward: {trajectory['total_reward']:.2f}")
    
    # Demonstrate training step
    print("4. Running training step...")
    training_stats = agent.train_episode(num_trajectories=3)
    print(f"   ✓ Training completed: mean reward = {training_stats['mean_trajectory_reward']:.2f}")
    
    print("\n✅ Demo completed successfully!")
    print("The framework is ready for full training.")
    

def run_training():
    """Run full training process."""
    print("Starting full training process...")
    
    # Training configuration
    training_config = TrainingConfig(
        max_episodes=200,
        trajectories_per_episode=5,
        evaluation_frequency=25,
        save_frequency=50,
        target_reward=12.0,
        early_stopping_patience=100
    )
    
    network_config = NetworkConfig(
        hidden_layers=[64, 32],
        activation="tanh",
        learning_rate=0.003,
        momentum=0.9,
        weight_decay=0.01
    )
    
    # Run training
    agent = train_omnidirectional_gait(
        training_config=training_config,
        network_config=network_config,
        save_dir="trained_models",
        log_dir="training_logs"
    )
    
    print("Training completed!")
    return agent


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description="BARS Omnidirectional Gait Learning")
    parser.add_argument("--demo", action="store_true", 
                       help="Run quick demonstration")
    parser.add_argument("--train", action="store_true",
                       help="Run full training process")  
    parser.add_argument("--evaluate", type=str, metavar="MODEL_PATH",
                       help="Evaluate a trained model")
    parser.add_argument("--config", action="store_true",
                       help="Create default configuration files")
    
    args = parser.parse_args()
    
    if args.config:
        create_default_config_files()
        print("Default configuration files created.")
        return
    
    if args.demo:
        run_demo()
        return
    
    if args.train:
        run_training()
        return
    
    if args.evaluate:
        if not Path(args.evaluate).exists():
            print(f"Error: Model file {args.evaluate} not found")
            sys.exit(1)
        evaluate_trained_model(args.evaluate)
        return
    
    # Default: run demo
    print("No specific action specified. Running demo...")
    print("Use --help for available options.")
    print()
    run_demo()


if __name__ == "__main__":
    main()