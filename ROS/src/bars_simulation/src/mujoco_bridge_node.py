#!/usr/bin/env python3
"""
MuJoCo Bridge Node for BARS Robot

This node provides integration between the existing MuJoCo simulation
and the ROS2 framework, allowing control and monitoring through ROS topics.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from bars_msgs.msg import JointAngles, ServoState
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import JointState
import sys
import os

# Add the Simulation directory to Python path for importing existing code
simulation_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'Simulation')
if os.path.exists(simulation_path):
    sys.path.insert(0, simulation_path)


class MuJoCoBridgeNode(Node):
    """Bridge between MuJoCo simulation and ROS2."""
    
    def __init__(self):
        super().__init__('mujoco_bridge')
        
        # Parameters
        self.declare_parameter('simulation_rate', 50.0)  # Hz
        self.declare_parameter('model_path', 'bars.xml')
        
        self.simulation_rate = self.get_parameter('simulation_rate').value
        self.model_path = self.get_parameter('model_path').value
        
        # QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState, 'joint_states', qos_profile)
        self.pose_pub = self.create_publisher(
            Pose, 'robot_pose', qos_profile)
        
        # Subscribers
        self.joint_cmd_sub = self.create_subscription(
            JointAngles, 'joint_angles_cmd', self.joint_command_callback, qos_profile)
        
        # Initialize simulation (placeholder for actual MuJoCo integration)
        self.simulation_initialized = False
        self.joint_positions = {}
        self.initialize_simulation()
        
        # Simulation timer
        self.sim_timer = self.create_timer(
            1.0 / self.simulation_rate, self.simulation_step)
        
        self.get_logger().info('MuJoCo bridge initialized')
    
    def initialize_simulation(self):
        """Initialize MuJoCo simulation."""
        try:
            # This is a placeholder for actual MuJoCo initialization
            # In the future, this would load the existing simulation code
            self.get_logger().info(f'Initializing simulation with model: {self.model_path}')
            
            # Initialize joint positions to zero
            joint_names = [
                'left_hip_pitch', 'left_hip_roll', 'left_hip_yaw',
                'left_knee_pitch', 'left_ankle_pitch', 'left_ankle_roll',
                'right_hip_pitch', 'right_hip_roll', 'right_hip_yaw',
                'right_knee_pitch', 'right_ankle_pitch', 'right_ankle_roll'
            ]
            
            for joint in joint_names:
                self.joint_positions[joint] = 0.0
            
            self.simulation_initialized = True
            self.get_logger().info('Simulation initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize simulation: {e}')
            self.simulation_initialized = False
    
    def joint_command_callback(self, msg):
        """Handle incoming joint angle commands."""
        if not self.simulation_initialized:
            return
        
        # Update target joint positions
        self.joint_positions.update({
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
        })
        
        self.get_logger().debug('Updated joint targets from ROS command')
    
    def simulation_step(self):
        """Execute one simulation step and publish state."""
        if not self.simulation_initialized:
            return
        
        # This is where the actual MuJoCo simulation step would occur
        # For now, we'll just publish the current joint states
        
        # Publish joint states
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = list(self.joint_positions.keys())
        joint_state.position = list(self.joint_positions.values())
        joint_state.velocity = [0.0] * len(joint_state.name)  # Placeholder
        joint_state.effort = [0.0] * len(joint_state.name)    # Placeholder
        
        self.joint_state_pub.publish(joint_state)
        
        # Publish robot pose (placeholder)
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.3  # Approximate standing height
        pose.orientation.w = 1.0  # Identity quaternion
        
        self.pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = MuJoCoBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()