#!/usr/bin/env python3
"""
Step 1 of the navigation workflow: load the static map.

Starts:
  * map_server              (nav2_map_server)  -> publishes /map (nav_msgs/OccupancyGrid)
  * lifecycle_manager_map   (nav2_lifecycle_manager) -> configures + activates map_server

map_server is a *lifecycle* node: it does nothing until it is configured and
activated, which is what the lifecycle manager does for us (autostart:=true).

Usage:
  ros2 launch testbed_navigation map_loader.launch.py
  ros2 launch testbed_navigation map_loader.launch.py map:=/abs/path/to/other_map.yaml
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('map_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    declare_args = [
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution(
                [FindPackageShare('testbed_bringup'), 'maps', 'testbed_world.yaml']),
            description='Full path to the map YAML file to load'),
        DeclareLaunchArgument(
            'map_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('testbed_navigation'), 'config', 'map_server_params.yaml']),
            description='map_server parameter file'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo /clock'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Configure + activate map_server automatically'),
        DeclareLaunchArgument(
            'log_level', default_value='info', description='ROS log level'),
    ]

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        # later entries override earlier ones: the launch arguments win over the YAML file
        parameters=[params_file,
                    {'yaml_filename': map_yaml,
                     'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', log_level],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': autostart,
                     'node_names': ['map_server']}],
        arguments=['--ros-args', '--log-level', log_level],
    )

    return LaunchDescription(declare_args + [map_server, lifecycle_manager])
