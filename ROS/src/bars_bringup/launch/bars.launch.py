#!/usr/bin/env python3
"""
Main launch file for BARS robot system
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation mode'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for Arduino communication'
    )
    
    # Control system launch
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bars_control'),
                'launch',
                'control.launch.py'
            ])
        ]),
        launch_arguments={
            'serial_port': LaunchConfiguration('serial_port')
        }.items()
    )
    
    # Robot state publisher (will be implemented when URDF is available)
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     parameters=[{'robot_description': robot_description}],
    #     output='screen'
    # )
    
    return LaunchDescription([
        use_sim_arg,
        serial_port_arg,
        control_launch,
        # robot_state_publisher,  # Uncomment when URDF is available
    ])