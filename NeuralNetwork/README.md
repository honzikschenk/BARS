# Neural Network - Omnidirectional Gait Learning

This folder contains a beginning reinforcement learning framework for learning omnidirectional gaits on the BARS bipedal robot.

## Overview

The framework provides:
- **Environment Interface**: Simulation environment for bipedal robot gait learning
- **Policy Agent**: Neural network-based agent for learning walking policies
- **Training Infrastructure**: Complete training loop with evaluation and logging
- **Omnidirectional Support**: Training for all movement directions (forward, backward, left, right, rotation)

## Key Features

- **Omnidirectional Movement**: Support for 7 movement types:
  - Forward walking
  - Backward walking  
  - Left/right strafing
  - Left/right rotation
  - Stationary balance

- **Bipedal Robot Interface**: Designed specifically for BARS robot with 10 DOF (5 per leg)

- **Reward Function**: Multi-component reward encouraging:
  - Velocity tracking in desired direction
  - Stability and balance maintenance
  - Upright posture
  - Energy efficiency
  - Smooth gait patterns

- **Training Framework**: Complete RL training pipeline with:
  - Policy gradient learning
  - Momentum-based optimization
  - Evaluation on all movement directions
  - Model checkpointing and logging

## Files

### Core Framework
- `gait_environment.py` - Bipedal robot environment for RL training
- `gait_agent.py` - Neural network agent for policy learning
- `training.py` - Training utilities and main training loop
- `config.py` - Configuration management and utilities

### Demo and Examples
- `gait_learning_demo.py` - Main demonstration script
- `main.py` - Original Gymnasium example (preserved)

## Quick Start

### 1. Run Demo
```bash
cd NeuralNetwork
python gait_learning_demo.py --demo
```

This will:
- Initialize the gait learning environment
- Create a neural network agent
- Test movement in different directions
- Run a training step
- Show framework capabilities

### 2. Create Configuration Files
```bash
python gait_learning_demo.py --config
```

### 3. Run Training
```bash
python gait_learning_demo.py --train
```

### 4. Evaluate Trained Model
```bash
python gait_learning_demo.py --evaluate trained_models/best_model.json
```

## Framework Architecture

### Environment (`BipedalGaitEnvironment`)
- **State Representation**: Robot pose, velocities, joint states, stability metrics
- **Action Space**: Target joint angles for 10 DOF
- **Reward Function**: Multi-objective reward for stable omnidirectional movement
- **Episode Management**: Configurable episode length and termination conditions

### Agent (`OmnidirectionalGaitAgent`) 
- **Policy Network**: Feedforward neural network mapping observations to actions
- **Training Algorithm**: Policy gradient with momentum optimization
- **Exploration**: Gaussian noise with decay for training exploration
- **Evaluation**: Systematic evaluation on all movement directions

### Training (`training.py`)
- **Multi-trajectory Training**: Collect multiple trajectories per training step
- **Direction Sampling**: Automatic sampling of different movement directions
- **Progress Tracking**: Logging and evaluation during training
- **Model Management**: Automatic saving of best models and checkpoints

## Configuration

The framework uses several configuration classes:

- `EnvironmentConfig`: Environment parameters (timestep, episode length, reward weights)
- `RobotConfig`: Robot-specific parameters (joint limits, physical dimensions) 
- `NetworkConfig`: Neural network architecture and training parameters
- `TrainingConfig`: Training process parameters (episodes, evaluation frequency)

## Example Usage

```python
from gait_environment import BipedalGaitEnvironment, GaitDirection
from gait_agent import OmnidirectionalGaitAgent, NetworkConfig

# Create environment
env = BipedalGaitEnvironment()

# Create agent with custom network
config = NetworkConfig(hidden_layers=[64, 32], learning_rate=0.001)
agent = OmnidirectionalGaitAgent(env, config)

# Train for one episode
stats = agent.train_episode(num_trajectories=5)

# Evaluate on forward walking
results = agent.evaluate(GaitDirection.FORWARD, num_episodes=10)
print(f"Forward walking success rate: {results['success_rate']:.1%}")
```

## Integration with BARS Robot

This framework is designed to integrate with the broader BARS system:

- **Joint Names**: Matches BARS robot joint configuration
- **Action Scaling**: Joint angle targets in appropriate ranges for servo motors
- **State Representation**: Compatible with robot sensor data and state estimation
- **Simulation Bridge**: Ready for integration with MuJoCo simulation in `/Simulation`

## Future Extensions

The framework provides a foundation for:
- Integration with MuJoCo physics simulation
- Real robot deployment via ROS interface
- Advanced RL algorithms (PPO, SAC, etc.)
- Terrain adaptation and disturbance rejection
- Multi-modal gaits (walking, running, climbing)

## Dependencies

The framework is designed to work with standard Python libraries:
- NumPy for numerical computations
- JSON for configuration and model serialization
- Pathlib for file management

Optional dependencies for full functionality:
- MuJoCo for physics simulation
- Matplotlib for training visualization
- TensorBoard for advanced logging

## Notes

This is a **beginning** reinforcement learning framework as requested. It provides:
- Complete structure for omnidirectional gait learning
- Working implementation ready for testing
- Clean interfaces for extension and integration
- Focus specifically on the BARS bipedal robot use case

The framework is designed to be minimal yet complete, providing a solid foundation for developing more advanced gait learning capabilities.