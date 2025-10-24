#!/usr/bin/env python3
"""
Arduino Bridge Node for BARS Robot

This node provides communication between ROS2 and the Arduino microcontroller,
translating between ROS2 messages and serial communication protocols.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from bars_msgs.msg import ServoCommand, ServoState, RobotState
import serial
import json
import threading
import time


class ArduinoBridgeNode(Node):
    """Bridge between ROS2 and Arduino for servo control and status reporting."""
    
    def __init__(self):
        super().__init__('arduino_bridge')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('publish_rate', 50.0)
        
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers
        self.servo_state_pub = self.create_publisher(
            ServoState, 'servo_states', qos_profile)
        self.robot_status_pub = self.create_publisher(
            String, 'arduino_status', qos_profile)
        
        # Subscribers
        self.servo_cmd_sub = self.create_subscription(
            ServoCommand, 'servo_commands', self.servo_command_callback, qos_profile)
        
        # Serial connection
        self.serial_conn = None
        self.serial_lock = threading.Lock()
        self.connect_serial()
        
        # Status publishing timer
        self.status_timer = self.create_timer(
            1.0 / self.publish_rate, self.publish_status)
        
        self.get_logger().info(f'Arduino bridge initialized on {self.serial_port}')
    
    def connect_serial(self):
        """Establish serial connection to Arduino."""
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.timeout
            )
            self.get_logger().info(f'Connected to Arduino on {self.serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            self.serial_conn = None
    
    def servo_command_callback(self, msg):
        """Handle incoming servo commands."""
        if not self.serial_conn or not self.serial_conn.is_open:
            self.get_logger().warn('No serial connection available')
            return
        
        try:
            # Create command message for Arduino
            cmd = {
                'type': 'servo_cmd',
                'servo_id': msg.servo_id,
                'angle': msg.target_angle,
                'speed': msg.speed,
                'enable': msg.enable
            }
            
            with self.serial_lock:
                command_str = json.dumps(cmd) + '\n'
                self.serial_conn.write(command_str.encode())
                
        except Exception as e:
            self.get_logger().error(f'Error sending servo command: {e}')
    
    def publish_status(self):
        """Read and publish status information from Arduino."""
        if not self.serial_conn or not self.serial_conn.is_open:
            return
        
        try:
            with self.serial_lock:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode().strip()
                    if line:
                        try:
                            data = json.loads(line)
                            self.process_arduino_data(data)
                        except json.JSONDecodeError:
                            # Non-JSON status message
                            status_msg = String()
                            status_msg.data = line
                            self.robot_status_pub.publish(status_msg)
                            
        except Exception as e:
            self.get_logger().error(f'Error reading from Arduino: {e}')
    
    def process_arduino_data(self, data):
        """Process structured data from Arduino."""
        if data.get('type') == 'servo_state':
            servo_state = ServoState()
            servo_state.servo_id = data.get('servo_id', 0)
            servo_state.current_angle = data.get('angle', 0.0)
            servo_state.current_speed = data.get('speed', 0.0)
            servo_state.load = data.get('load', 0.0)
            servo_state.is_moving = data.get('moving', False)
            servo_state.error = data.get('error', False)
            servo_state.temperature = data.get('temp', 0)
            
            self.servo_state_pub.publish(servo_state)
        
        elif data.get('type') == 'status':
            status_msg = String()
            status_msg.data = json.dumps(data)
            self.robot_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()