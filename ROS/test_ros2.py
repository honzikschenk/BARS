#!/usr/bin/env python3
"""
Simple test script to verify ROS2 setup
"""

import rclpy
from rclpy.node import Node
from bars_msgs.msg import JointAngles, ServoCommand
from std_msgs.msg import String
import time


class TestNode(Node):
    """Simple test node for verifying ROS2 functionality."""
    
    def __init__(self):
        super().__init__('test_node')
        
        # Publishers for testing message types
        self.joint_pub = self.create_publisher(JointAngles, 'test_joint_angles', 10)
        self.servo_pub = self.create_publisher(ServoCommand, 'test_servo_command', 10)
        self.status_pub = self.create_publisher(String, 'test_status', 10)
        
        # Timer for publishing test messages
        self.timer = self.create_timer(1.0, self.publish_test_messages)
        
        self.counter = 0
        self.get_logger().info('Test node started')
    
    def publish_test_messages(self):
        """Publish test messages to verify message types work."""
        
        # Test joint angles message
        joint_msg = JointAngles()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.left_hip_pitch = 0.1 * self.counter
        joint_msg.right_hip_pitch = -0.1 * self.counter
        self.joint_pub.publish(joint_msg)
        
        # Test servo command message
        servo_msg = ServoCommand()
        servo_msg.servo_id = 0
        servo_msg.target_angle = 0.1 * self.counter
        servo_msg.speed = 0.5
        servo_msg.enable = True
        self.servo_pub.publish(servo_msg)
        
        # Test status message
        status_msg = String()
        status_msg.data = f'Test message {self.counter}'
        self.status_pub.publish(status_msg)
        
        self.get_logger().info(f'Published test messages #{self.counter}')
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()