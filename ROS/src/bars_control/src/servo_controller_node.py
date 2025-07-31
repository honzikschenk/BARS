#!/usr/bin/env python3
"""
Servo Controller Node for BARS Robot

This node provides high-level servo control, including joint angle conversion,
trajectory planning, and safety monitoring.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from bars_msgs.msg import ServoCommand, ServoState, JointAngles
from bars_msgs.srv import SetRobotState, GetRobotStatus
import numpy as np
import yaml
import os


class ServoControllerNode(Node):
    """High-level servo control and joint management."""
    
    def __init__(self):
        super().__init__('servo_controller')
        
        # Parameters
        self.declare_parameter('config_file', 'servo_config.yaml')
        self.declare_parameter('safety_enabled', True)
        self.declare_parameter('max_angle_change_rate', 1.0)  # rad/s
        
        # QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers
        self.servo_cmd_pub = self.create_publisher(
            ServoCommand, 'servo_commands', qos_profile)
        
        # Subscribers
        self.joint_angles_sub = self.create_subscription(
            JointAngles, 'joint_angles_cmd', self.joint_angles_callback, qos_profile)
        self.servo_state_sub = self.create_subscription(
            ServoState, 'servo_states', self.servo_state_callback, qos_profile)
        
        # Services
        self.set_state_srv = self.create_service(
            SetRobotState, 'set_robot_state', self.set_robot_state_callback)
        self.get_status_srv = self.create_service(
            GetRobotStatus, 'get_robot_status', self.get_robot_status_callback)
        
        # Internal state
        self.servo_config = {}
        self.current_servo_states = {}
        self.target_joint_angles = {}
        self.current_joint_angles = {}
        self.robot_state = "idle"
        self.safety_enabled = self.get_parameter('safety_enabled').value
        self.max_angle_change_rate = self.get_parameter('max_angle_change_rate').value
        
        # Load configuration
        self.load_servo_config()
        
        # Control timer
        self.control_timer = self.create_timer(0.02, self.control_loop)  # 50Hz
        
        self.get_logger().info('Servo controller initialized')
    
    def load_servo_config(self):
        """Load servo configuration from file."""
        config_file = self.get_parameter('config_file').value
        
        # Default configuration if file doesn't exist
        default_config = {
            'servos': {
                0: {'joint': 'left_hip_pitch', 'offset': 0.0, 'direction': 1},
                1: {'joint': 'left_hip_roll', 'offset': 0.0, 'direction': 1},
                2: {'joint': 'left_hip_yaw', 'offset': 0.0, 'direction': 1},
                3: {'joint': 'left_knee_pitch', 'offset': 0.0, 'direction': 1},
                4: {'joint': 'left_ankle_pitch', 'offset': 0.0, 'direction': 1},
                5: {'joint': 'left_ankle_roll', 'offset': 0.0, 'direction': 1},
                6: {'joint': 'right_hip_pitch', 'offset': 0.0, 'direction': -1},
                7: {'joint': 'right_hip_roll', 'offset': 0.0, 'direction': -1},
                8: {'joint': 'right_hip_yaw', 'offset': 0.0, 'direction': -1},
                9: {'joint': 'right_knee_pitch', 'offset': 0.0, 'direction': -1},
                10: {'joint': 'right_ankle_pitch', 'offset': 0.0, 'direction': -1},
                11: {'joint': 'right_ankle_roll', 'offset': 0.0, 'direction': -1},
            },
            'limits': {
                'hip_pitch': [-0.5, 0.5],
                'hip_roll': [-0.3, 0.3],
                'hip_yaw': [-0.2, 0.2],
                'knee_pitch': [0.0, 1.5],
                'ankle_pitch': [-0.3, 0.3],
                'ankle_roll': [-0.2, 0.2],
            }
        }
        
        try:
            # Try to load config file
            config_path = os.path.join(
                self.get_package_share_directory('bars_control'), 'config', config_file)
            with open(config_path, 'r') as f:
                self.servo_config = yaml.safe_load(f)
        except FileNotFoundError:
            self.get_logger().warn(f'Config file {config_file} not found, using defaults')
            self.servo_config = default_config
        except Exception as e:
            self.get_logger().error(f'Error loading config: {e}, using defaults')
            self.servo_config = default_config
    
    def joint_angles_callback(self, msg):
        """Handle incoming joint angle commands."""
        joint_angles = {
            'left_hip_pitch': msg.left_hip_pitch,
            'left_hip_roll': msg.left_hip_roll,
            'left_hip_yaw': msg.left_hip_yaw,
            'left_knee_pitch': msg.left_knee_pitch,
            'left_ankle_pitch': msg.left_ankle_pitch,
            'left_ankle_roll': msg.left_ankle_roll,
            'right_hip_pitch': msg.right_hip_pitch,
            'right_hip_roll': msg.right_hip_roll,
            'right_hip_yaw': msg.right_hip_yaw,
            'right_knee_pitch': msg.right_knee_pitch,
            'right_ankle_pitch': msg.right_ankle_pitch,
            'right_ankle_roll': msg.right_ankle_roll,
        }
        
        # Apply safety checks
        if self.safety_enabled:
            joint_angles = self.apply_safety_limits(joint_angles)
        
        self.target_joint_angles = joint_angles
    
    def servo_state_callback(self, msg):
        """Handle servo state updates."""
        self.current_servo_states[msg.servo_id] = msg
        
        # Update current joint angles
        servo_config = self.servo_config.get('servos', {}).get(msg.servo_id)
        if servo_config:
            joint_name = servo_config['joint']
            offset = servo_config.get('offset', 0.0)
            direction = servo_config.get('direction', 1)
            self.current_joint_angles[joint_name] = (msg.current_angle - offset) * direction
    
    def apply_safety_limits(self, joint_angles):
        """Apply safety limits to joint angles."""
        limited_angles = joint_angles.copy()
        limits = self.servo_config.get('limits', {})
        
        for joint_name, angle in joint_angles.items():
            joint_type = joint_name.split('_')[-1]  # Extract joint type (pitch, roll, yaw)
            if joint_type in limits:
                min_limit, max_limit = limits[joint_type]
                limited_angles[joint_name] = np.clip(angle, min_limit, max_limit)
                
                if angle != limited_angles[joint_name]:
                    self.get_logger().warn(f'Joint {joint_name} limited from {angle} to {limited_angles[joint_name]}')
        
        return limited_angles
    
    def control_loop(self):
        """Main control loop for servo commands."""
        if not self.target_joint_angles:
            return
        
        # Convert joint angles to servo commands
        for servo_id, servo_config in self.servo_config.get('servos', {}).items():
            joint_name = servo_config['joint']
            if joint_name in self.target_joint_angles:
                offset = servo_config.get('offset', 0.0)
                direction = servo_config.get('direction', 1)
                target_angle = self.target_joint_angles[joint_name]
                
                # Convert to servo angle
                servo_angle = target_angle * direction + offset
                
                # Create servo command
                cmd = ServoCommand()
                cmd.servo_id = servo_id
                cmd.target_angle = servo_angle
                cmd.speed = 0.5  # Default speed
                cmd.enable = True
                
                self.servo_cmd_pub.publish(cmd)
    
    def set_robot_state_callback(self, request, response):
        """Service callback for setting robot state."""
        old_state = self.robot_state
        self.robot_state = request.target_state
        
        response.success = True
        response.current_state = self.robot_state
        response.message = f'State changed from {old_state} to {self.robot_state}'
        
        self.get_logger().info(response.message)
        return response
    
    def get_robot_status_callback(self, request, response):
        """Service callback for getting robot status."""
        # This would be implemented with actual robot state
        response.success = True
        response.message = f'Robot in state: {self.robot_state}'
        
        if request.include_servo_details:
            response.servo_states = list(self.current_servo_states.values())
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ServoControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()