#!/usr/bin/env python3
"""
Flight Computer Launch File
============================
Launches MAVROS bridge and navigation stack for flight computer component.
Provides interface between ROS2 navigation system and ArduPilot flight controller.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    mavros_fcu_url_arg = DeclareLaunchArgument(
        'mavros_fcu_url',
        default_value='udp://flight-controller:14550@',
        description='MAVROS FCU connection URL'
    )

    # MAVROS launch
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mavros'),
                'launch',
                'apm.launch'
            ])
        ]),
        launch_arguments={
            'fcu_url': LaunchConfiguration('mavros_fcu_url'),
            'gcs_url': '',
            'tgt_system': '1',
            'tgt_component': '1',
            'log_output': 'screen',
            'fcu_protocol': 'v2.0',
            'respawn_mavros': 'false',
        }.items()
    )

    # MAVROS Bridge Node - converts between ROS2 cmd_vel and MAVLink
    mavros_bridge_node = Node(
        package='mavros_bridge',
        executable='nav_to_mavlink.py',
        name='mavros_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # Robot Localization EKF Node
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('rover_navigation'),
                'config',
                'ekf.yaml'
            ]),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        mavros_fcu_url_arg,
        mavros_launch,
        mavros_bridge_node,
        robot_localization_node,
    ])