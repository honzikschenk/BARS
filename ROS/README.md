# BARS ROS2 Framework

This directory contains the ROS2 implementation for the BARS (Bipedal Autonomous Robotics System) project.

## Overview

The ROS2 framework provides a standardized communication layer between the various components of the BARS system, including:

- Motor control and servo management
- Sensor data processing
- State management integration
- Simulation environment interface
- Neural network integration

## Architecture

The ROS2 implementation follows a modular design with the following packages:

- **bars_bringup**: Launch files and system integration
- **bars_msgs**: Custom message and service definitions
- **bars_control**: Motor control and servo driver interfaces
- **bars_simulation**: Integration with MuJoCo simulation environment
- **bars_state_manager**: Interface with the existing C++ StateManager
- **bars_perception**: Sensor data processing (future expansion)

## Development Environment

This ROS2 workspace is designed to work with:
- ROS2 Humble Hawksbill
- Multi-architecture Docker containers (x86_64, arm64)
- Cross-platform development (Linux, macOS, Windows)

## Quick Start

### Using Docker (Recommended)

```bash
# Build the development container
docker-compose build

# Start the development environment
docker-compose up -d

# Enter the container
docker-compose exec bars_dev bash

# Build the ROS2 workspace
colcon build

# Source the workspace
source install/setup.bash
```

### Native Installation

```bash
# Install ROS2 Humble (Ubuntu 22.04)
sudo apt update
sudo apt install ros-humble-desktop

# Build the workspace
cd ROS
colcon build

# Source the workspace
source install/setup.bash
```

## Integration with Existing BARS Components

The ROS2 framework is designed to integrate seamlessly with existing BARS components:

- **Arduino Code**: Communication via serial/USB bridge to ROS2 topics
- **MuJoCo Simulation**: Wrapped as ROS2 nodes for physics simulation
- **StateManager**: Exposed as ROS2 services for state transitions
- **Neural Networks**: Integration for inference and training data