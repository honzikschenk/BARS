#!/usr/bin/env python3
"""
BARS Robot ROS2 Usage Example

This example demonstrates how to control the BARS robot using the ROS2 framework.
It shows basic joint control, status monitoring, and integration with the system.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from bars_msgs.msg import JointAngles, ServoState, RobotState
from bars_msgs.srv import SetRobotState, GetRobotStatus
from std_msgs.msg import String
import math
import time


class BarsControlExample(Node):
    """Example node demonstrating BARS robot control."""
    
    def __init__(self):
        super().__init__('bars_control_example')
        
        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers
        self.joint_cmd_pub = self.create_publisher(
            JointAngles, 'joint_angles_cmd', qos_profile)
        
        # Subscribers
        self.servo_state_sub = self.create_subscription(
            ServoState, 'servo_states', self.servo_state_callback, qos_profile)
        self.status_sub = self.create_subscription(
            String, 'arduino_status', self.status_callback, qos_profile)
        
        # Service clients
        self.set_state_client = self.create_client(SetRobotState, 'set_robot_state')
        self.get_status_client = self.create_client(GetRobotStatus, 'get_robot_status')
        
        # Internal state
        self.servo_states = {}
        self.demo_phase = 0
        self.start_time = time.time()
        
        # Demo timer
        self.demo_timer = self.create_timer(0.1, self.demo_loop)  # 10Hz
        
        self.get_logger().info('BARS control example started')
        
        # Set robot to active state
        self.set_robot_state('active')
    
    def servo_state_callback(self, msg):
        """Handle servo state updates."""
        self.servo_states[msg.servo_id] = msg
        if msg.servo_id == 0:  # Log only first servo to avoid spam
            self.get_logger().debug(f'Servo {msg.servo_id}: {msg.current_angle:.3f} rad')
    
    def status_callback(self, msg):
        """Handle status updates from Arduino."""
        self.get_logger().info(f'Arduino status: {msg.data}')
    
    def set_robot_state(self, state):
        """Set the robot's state using the service."""
        if not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Set state service not available')
            return
        
        request = SetRobotState.Request()
        request.target_state = state
        request.force_transition = False
        
        future = self.set_state_client.call_async(request)
        future.add_done_callback(self.set_state_callback)
    
    def set_state_callback(self, future):
        """Handle set state service response."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'State set successfully: {response.message}')
            else:
                self.get_logger().warn(f'Failed to set state: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
    
    def demo_loop(self):
        """Main demo loop with different movement patterns."""
        elapsed_time = time.time() - self.start_time
        
        # Create joint angles message
        joint_msg = JointAngles()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Demo phases
        if elapsed_time < 5.0:
            # Phase 1: Neutral position
            self.demo_phase = 1
            self.set_neutral_position(joint_msg)
            
        elif elapsed_time < 10.0:
            # Phase 2: Simple leg movement
            self.demo_phase = 2
            self.demo_leg_movement(joint_msg, elapsed_time - 5.0)
            
        elif elapsed_time < 15.0:
            # Phase 3: Walking-like motion
            self.demo_phase = 3
            self.demo_walking_motion(joint_msg, elapsed_time - 10.0)
            
        else:
            # Phase 4: Return to neutral and restart
            self.demo_phase = 4
            self.set_neutral_position(joint_msg)
            if elapsed_time > 17.0:
                self.start_time = time.time()  # Restart demo
        
        # Publish joint commands
        self.joint_cmd_pub.publish(joint_msg)
        
        # Log demo phase changes
        if hasattr(self, 'last_phase') and self.last_phase != self.demo_phase:
            phase_names = {1: 'Neutral', 2: 'Leg Movement', 3: 'Walking Motion', 4: 'Return to Neutral'}
            self.get_logger().info(f'Demo phase: {phase_names.get(self.demo_phase, "Unknown")}')
        self.last_phase = self.demo_phase
    
    def set_neutral_position(self, joint_msg):
        """Set all joints to neutral/standing position."""
        joint_msg.left_hip_pitch = 0.0
        joint_msg.left_hip_roll = 0.0
        joint_msg.left_hip_yaw = 0.0
        joint_msg.left_knee_pitch = 0.2  # Slight bend
        joint_msg.left_ankle_pitch = -0.1
        joint_msg.left_ankle_roll = 0.0
        
        joint_msg.right_hip_pitch = 0.0
        joint_msg.right_hip_roll = 0.0
        joint_msg.right_hip_yaw = 0.0
        joint_msg.right_knee_pitch = 0.2
        joint_msg.right_ankle_pitch = -0.1
        joint_msg.right_ankle_roll = 0.0
    
    def demo_leg_movement(self, joint_msg, t):
        """Demonstrate simple leg lifting motion."""
        # Left leg stays neutral
        joint_msg.left_hip_pitch = 0.0
        joint_msg.left_hip_roll = 0.0
        joint_msg.left_hip_yaw = 0.0
        joint_msg.left_knee_pitch = 0.2
        joint_msg.left_ankle_pitch = -0.1
        joint_msg.left_ankle_roll = 0.0
        
        # Right leg lifts and lowers
        lift_amount = 0.3 * math.sin(2 * math.pi * t / 3.0)  # 3-second cycle
        joint_msg.right_hip_pitch = lift_amount
        joint_msg.right_hip_roll = 0.0
        joint_msg.right_hip_yaw = 0.0
        joint_msg.right_knee_pitch = 0.2 + max(0, lift_amount)
        joint_msg.right_ankle_pitch = -0.1
        joint_msg.right_ankle_roll = 0.0
    
    def demo_walking_motion(self, joint_msg, t):
        """Demonstrate alternating walking-like motion."""
        # Walking cycle frequency
        cycle_freq = 0.5  # 0.5 Hz = 2-second cycle
        phase = 2 * math.pi * cycle_freq * t
        
        # Left leg motion
        left_lift = 0.2 * max(0, math.sin(phase))
        joint_msg.left_hip_pitch = left_lift
        joint_msg.left_hip_roll = 0.0
        joint_msg.left_hip_yaw = 0.0
        joint_msg.left_knee_pitch = 0.2 + left_lift
        joint_msg.left_ankle_pitch = -0.1 - left_lift * 0.5
        joint_msg.left_ankle_roll = 0.0
        
        # Right leg motion (opposite phase)
        right_lift = 0.2 * max(0, math.sin(phase + math.pi))
        joint_msg.right_hip_pitch = right_lift
        joint_msg.right_hip_roll = 0.0
        joint_msg.right_hip_yaw = 0.0
        joint_msg.right_knee_pitch = 0.2 + right_lift
        joint_msg.right_ankle_pitch = -0.1 - right_lift * 0.5
        joint_msg.right_ankle_roll = 0.0


def main(args=None):
    rclpy.init(args=args)
    
    node = BarsControlExample()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Demo interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()