#!/usr/bin/env python3
"""
Gazebo Simulation Launch File
=============================
Launches pure Gazebo physics simulation with rover model and sensor plugins.
Provides realistic vehicle dynamics and sensor data for flight computer.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # Launch arguments
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('rover_gazebo'),
            'worlds',
            'rover_world.world'
        ]),
        description='Path to Gazebo world file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    gps_lat_arg = DeclareLaunchArgument(
        'gps_lat',
        default_value='37.7749',
        description='GPS latitude for coordinate system reference'
    )
    
    gps_lon_arg = DeclareLaunchArgument(
        'gps_lon',
        default_value='-122.4194',
        description='GPS longitude for coordinate system reference'
    )

    # Gazebo server (headless physics simulation)
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world_file'),
            'verbose': 'false',
            'physics': 'ode',
            'extra_gazebo_args': '--ros-args'
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': open(
                PathJoinSubstitution([
                    FindPackageShare('rover_description'),
                    'urdf',
                    'rover.urdf.xacro'
                ]).perform(None)
            ).read()
        }]
    )

    # Spawn rover model in Gazebo
    spawn_rover = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_rover',
        output='screen',
        arguments=[
            '-entity', 'rover',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.1',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('rover_control'),
                'config',
                'controllers.yaml'
            ]),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # Joint State Broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        output='screen',
        arguments=['joint_state_broadcaster'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Differential Drive Controller
    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='diff_drive_controller_spawner',
        output='screen',
        arguments=['diff_drive_controller'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    return LaunchDescription([
        world_file_arg,
        use_sim_time_arg,
        gps_lat_arg,
        gps_lon_arg,
        gazebo_server,
        robot_state_publisher,
        spawn_rover,
        controller_manager,
        joint_state_broadcaster,
        diff_drive_controller,
    ])