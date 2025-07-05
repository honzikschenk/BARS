# BARS - Bipedal Autonomous Robotics System

> [!IMPORTANT]
> This project is a work in progress (WIP). Features, documentation, and designs are subject to change as development continues.

A low-cost, open-source bipedal robot designed for makers, researchers, and robotics enthusiasts.

For more info, watch this [YouTube video](https://youtube.com/shorts/22_nTzXpSXs?feature=share).

## Overview

BARS (Bipedal Autonomous Robotics System) is an affordable alternative to expensive commercial bipedal robots. BARS aims to make bipedal robotics accessible through 3D printing, common materials, and open-source software.

## Key Features

- **Affordable**: Built with 3D printed parts and accessible materials
- **Modular**: Easy to assemble, modify, and repair
- **Dual-Brain Architecture**: Arduino for real-time control, Jetson for high-level decision making

## Hardware Architecture

The robot uses a distributed computing approach:

### Control System

- **Arduino/PlatformIO**: Real-time motor control and sensor processing
- **NVIDIA Jetson**: High-level planning, SLAM, and neural network inference

### Physical Design

- **Frame**: 3D printed chassis designed in OnShape
- **Metal Parts**: Custom machined components for joints and connections
- **Actuators**: Servo motors
- **Sensors**: WIP - Coming soon

## Project Structure

### `/arduino-code/`

PlatformIO-based firmware for the Arduino microcontroller. Handles real-time motor control, low-level sensor reading, and communication with the Jetson computer.

### `/Simulation/`

MuJoCo-based physics simulation environment for testing and development. Includes:

- Physics simulation
- WIP Reinforcement learning training (`ReinforcementModel.py`)
- 3D CAD models in STL format for all robot parts

### `/NeuralNetwork/`

Work in Progress. More into to come soon.

### `/InverseKinematics/`

Work in Progress. Plans are to use mathematical algorithms for robot movement:

- Body-to-foot inverse kinematics calculations
- Gait planning algorithms
- Coordinate transformation utilities

### `/StateManager/`

C++ library for managing robot behavioral states:

- Easy-to-use state machine implementation
- Included state transition logic

### `/human-interaction/`

Work in Progress. Plans are to:

- Command robot with LLM prompts
- Have a [debug app](https://github.com/honzikschenk/RoboScope) for real-time monitoring and manual control

## Quick Start

### Prerequisites

- Python 3.8+ with pip
- Arduino IDE or PlatformIO
- MuJoCo physics engine
- C++ compiler

### Installation

1. **Clone the repository**

   ```bash
   git clone https://github.com/honzikschenk/BARS.git
   cd BARS
   ```

2. **Set up Python environment**

   ```bash
   python -m venv .venv
   source .venv/bin/activate  # On Windows: .venv\Scripts\activate
   pip install -r requirements.txt # To come soon (when needed)
   ```

3. **Test the simulation**

   ```bash
   cd Simulation
   mjpython main.py
   ```

4. **Build StateManager**

   ```bash
   cd StateManager
   g++ -std=c++11 -o StateManagerTest Test.cpp StateManager.cpp && ./StateManagerTest
   ```

### Running the Project

- **Simulation**: `cd Simulation && mjpython main.py`
- **Arduino**: Use PlatformIO to upload `arduino-code/` to your microcontroller

## Development Status

- [x] Initial robot design
- [x] Physics simulation environment
- [x] Arduino control framework
- [x] State management system
- [ ] Basic gait control
- [ ] Neural network training
- [ ] Hardware integration
- [ ] Complete robot assembly
- [ ] Autonomous walking

*BARS is under active development. Star this repo to follow progress!*
