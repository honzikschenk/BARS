# BARS ROS2 Examples

This directory contains example code demonstrating how to use the BARS ROS2 framework.

## Examples

### Arduino Integration

**`arduino_ros2_bridge.ino`**

Arduino sketch that demonstrates integration with the ROS2 framework:

- JSON-based communication protocol
- Servo command parsing and execution
- Status reporting back to ROS2
- Emergency stop functionality
- Compatible with existing Arduino servo libraries

**Dependencies:**
- ArduinoJson library
- Servo library (built-in)

**Usage:**
1. Install ArduinoJson library in Arduino IDE or PlatformIO
2. Upload sketch to your Arduino
3. Run the ROS2 Arduino bridge node
4. The Arduino will communicate with ROS2 through serial

### ROS2 Control Demo

**`control_demo.py`**

Python script demonstrating robot control through ROS2:

- Joint angle control
- Walking motion patterns
- Status monitoring
- Service calls for state management

**Usage:**
```bash
# In ROS2 environment
cd /workspace
source install/setup.bash
python3 examples/control_demo.py
```

**Features demonstrated:**
- Publishing joint angle commands
- Subscribing to servo states
- Using robot state services
- Implementing motion patterns
- Error handling and logging

## Integration Workflow

1. **Arduino Setup**: Upload the `arduino_ros2_bridge.ino` to your Arduino
2. **ROS2 Launch**: Start the BARS control system:
   ```bash
   ros2 launch bars_bringup bars.launch.py serial_port:=/dev/ttyUSB0
   ```
3. **Run Demo**: Execute the control demo:
   ```bash
   python3 examples/control_demo.py
   ```
4. **Monitor**: View topics and services:
   ```bash
   ros2 topic list
   ros2 topic echo /servo_states
   ros2 service list
   ```

## Message Flow

```
control_demo.py -> /joint_angles_cmd -> servo_controller_node
                                           |
                                           v
                     /servo_commands -> arduino_bridge_node
                                           |
                                           v
                                      Serial/USB -> Arduino
                                           |
                                           v
                     /servo_states <- arduino_bridge_node
                           |
                           v
                    control_demo.py (feedback)
```

## Customization

### Adding New Joints

1. Update servo configuration in `config/servo_config.yaml`
2. Modify `JointAngles.msg` to include new joints
3. Update the servo controller mapping
4. Rebuild the workspace: `colcon build`

### Custom Motion Patterns

Create new motion functions in `control_demo.py` or create your own control node:

```python
def custom_motion(self, joint_msg, t):
    # Your custom joint angle calculations
    joint_msg.left_hip_pitch = math.sin(t)
    # ... more joint assignments
```

### Adding Sensors

1. Create new message types in `bars_msgs`
2. Update Arduino code to read and report sensor data
3. Create ROS2 nodes to process sensor data
4. Integrate with existing control loops

## Troubleshooting

**Arduino not responding:**
- Check serial port permissions: `sudo chmod 666 /dev/ttyUSB0`
- Verify baud rate matches (115200)
- Check Arduino IDE serial monitor for debugging

**ROS2 messages not flowing:**
- Verify all nodes are running: `ros2 node list`
- Check topic connections: `ros2 topic info /servo_commands`
- Monitor for errors: `ros2 log level set /arduino_bridge DEBUG`

**Servo movements erratic:**
- Check power supply to servos
- Verify servo configuration in YAML file
- Reduce movement speed in safety limits