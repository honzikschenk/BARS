#!/usr/bin/env python3
"""
Launch file for BARS control system
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('bars_control')
    
    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for Arduino communication'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial communication'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='servo_config.yaml',
        description='Servo configuration file'
    )
    
    # Nodes
    arduino_bridge_node = Node(
        package='bars_control',
        executable='arduino_bridge_node.py',
        name='arduino_bridge',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'timeout': 1.0,
            'publish_rate': 50.0
        }],
        output='screen'
    )
    
    servo_controller_node = Node(
        package='bars_control',
        executable='servo_controller_node.py',
        name='servo_controller',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'safety_enabled': True,
            'max_angle_change_rate': 1.0
        }],
        output='screen'
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        config_file_arg,
        arduino_bridge_node,
        servo_controller_node,
    ])