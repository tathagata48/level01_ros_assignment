#!/usr/bin/env python3
"""
Step 2 of the navigation workflow: localize the robot on the map with AMCL.

Starts:
  * amcl                             (nav2_amcl) -> publishes map->odom TF, /amcl_pose, /particle_cloud
  * lifecycle_manager_localization   -> configures + activates amcl

AMCL needs: the /map topic (map_loader.launch.py), the /scan topic and the
odom->base_footprint TF (both from Gazebo). By default the map is NOT started
here so that each step can be run and debugged on its own; pass
`with_map:=true` to start map_loader.launch.py as well.

Usage:
  ros2 launch testbed_navigation localization.launch.py
  ros2 launch testbed_navigation localization.launch.py with_map:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('testbed_navigation')

    params_file = LaunchConfiguration('amcl_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    with_map = LaunchConfiguration('with_map')
    log_level = LaunchConfiguration('log_level')

    declare_args = [
        DeclareLaunchArgument(
            'amcl_params_file',
            default_value=PathJoinSubstitution([pkg_share, 'config', 'amcl_params.yaml']),
            description='AMCL parameter file'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use the Gazebo /clock'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Configure + activate amcl automatically'),
        DeclareLaunchArgument('with_map', default_value='false',
                              description='Also start map_loader.launch.py'),
        DeclareLaunchArgument('log_level', default_value='info', description='ROS log level'),
    ]

    map_loader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'map_loader.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time,
                          'autostart': autostart,
                          'log_level': log_level}.items(),
        condition=IfCondition(with_map),
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', log_level],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': autostart,
                     'node_names': ['amcl']}],
        arguments=['--ros-args', '--log-level', log_level],
    )

    return LaunchDescription(declare_args + [map_loader, amcl, lifecycle_manager])
