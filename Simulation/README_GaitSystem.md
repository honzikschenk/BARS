# Gait Generation System

This directory contains a comprehensive gait generation system for the BARS bipedal robot with the following key features:

## Overview

The system provides:
- **Quasi-static gait generation** with configurable parameters
- **Real-time center of mass calculation** and visualization
- **Interactive parameter tuning** for gait optimization
- **Stability analysis** and monitoring
- **Multiple preset gaits** for different scenarios

## Files

### Core Components
- `GaitGenerator.py` - Main gait generation with quasi-static walking algorithms
- `CenterOfMass.py` - Center of mass calculation and stability analysis
- `InteractiveGaitTuner.py` - User-friendly interface for real-time gait tuning
- `main.py` - Enhanced simulation with integrated gait system
- `Utils.py` - Utility functions for robot control

### Configuration
- `bars.xml` - Original MuJoCo robot model
- `bars_enhanced.xml` - Enhanced model with COM visualization (auto-generated)
- `positions.json` - Extended set of predefined robot poses

### Demo and Testing
- `demo_gait_system.py` - Headless demo showing all features
- `validate_gait_system.py` - Validation tests for system components

## Usage

### Interactive Mode (with display)
```bash
python main.py
```

### Demo Mode (headless)
```bash
python demo_gait_system.py
```

### Validation Tests
```bash
python validate_gait_system.py
```

## Interactive Controls

When running `main.py` with a display:

### Parameter Selection
- **Keys 1-5**: Select gait parameter to tune
  1. Step length
  2. Step height  
  3. Step width
  4. Phase duration
  5. Shift duration

### Parameter Adjustment
- **Up/Down arrows** or **+/-**: Adjust selected parameter
- Real-time parameter updates with immediate visual feedback

### Gait Control
- **Space**: Start/stop walking gait
- **R**: Toggle recording mode

### Preset Gaits
- **Q**: Load slow, stable gait
- **W**: Load normal walking gait
- **E**: Load fast walking gait  
- **T**: Load wide stance gait

### Display Options
- **H**: Toggle help display
- **S**: Toggle stability information
- **C**: Toggle center of mass information

### Save/Load
- **Ctrl+S**: Save current gait parameters
- **Ctrl+L**: Load saved gait

## Gait Parameters

### Adjustable Parameters
- **Step Length** (0.05-0.3m): Forward distance per step
- **Step Height** (0.01-0.15m): Foot lift height during swing
- **Step Width** (0.1-0.4m): Lateral foot separation
- **Phase Duration** (0.5-5.0s): Time per gait phase
- **Shift Duration** (0.3-3.0s): Time for weight shifting

### Preset Configurations
- **Slow**: Conservative, highly stable gait
- **Normal**: Balanced speed and stability
- **Fast**: Quicker gait with shorter phases
- **Wide**: Extra-wide stance for maximum stability

## System Features

### Gait Generation
- 7-phase walking cycle with smooth transitions
- Cosine interpolation for natural joint motion
- Configurable timing and spatial parameters
- Automatic phase progression and looping

### Stability Analysis
- Real-time center of mass calculation
- Support polygon detection from foot contacts
- Stability margin computation
- Statistical tracking over time

### Visualization
- Center of mass marker in 3D space
- Contact point visualization
- Real-time parameter display
- Gait phase indicators

### Robustness
- Automatic fall detection and recovery
- Parameter validation and bounds checking
- Smooth parameter transitions
- Save/restore functionality

## Technical Details

### Quasi-Static Gait Approach
The system generates gaits that maintain static stability at all times by:
1. Ensuring the center of mass projection stays within the support polygon
2. Using gradual weight shifts between legs
3. Lifting feet only when the other leg can support full body weight
4. Planning smooth trajectories with stability margins

### Center of Mass Calculation
COM is computed by:
1. Iterating through all robot body segments
2. Weighting each segment by its mass
3. Excluding environmental objects (floor, etc.)
4. Real-time updates during motion

### Stability Metrics
- **Static Stability**: COM projection within support polygon
- **Stability Margin**: Distance from COM to support polygon edge
- **Contact Detection**: Foot-ground contact identification
- **Dynamic Statistics**: Stability percentage over time

## Dependencies

- Python 3.6+
- MuJoCo physics engine
- NumPy for numerical computations
- JSON for configuration storage

## Getting Started

1. Ensure MuJoCo is installed: `pip install mujoco numpy`
2. Run the demo: `python demo_gait_system.py`
3. Try interactive mode: `python main.py` (requires display)
4. Experiment with different parameters and presets
5. Save your favorite gait configurations

## Troubleshooting

### No Display Available
Use `demo_gait_system.py` for headless operation and testing.

### Robot Falls Immediately
- Try the "Wide" preset (T key) for maximum stability
- Reduce step length and increase phase duration
- Check that the robot starts in a valid standing pose

### Poor Stability
- Increase step width for wider base of support
- Reduce step length for smaller disturbances
- Extend phase durations for more gradual movements
- Use the "Slow" preset as a stable starting point

### Parameter Adjustments Not Visible
- Ensure gait is started (Space key)
- Check that the correct parameter is selected (1-5 keys)
- Verify parameter is within valid range (shown in display)

## Future Enhancements

Potential improvements include:
- Dynamic gait generation with momentum utilization
- Terrain adaptation and obstacle avoidance
- Machine learning integration for gait optimization
- Multi-objective optimization (speed vs. stability vs. energy)
- Real-time footstep planning
- Integration with higher-level navigation systems