#!/usr/bin/env python3
"""
Convenience launch: the whole navigation workflow in one command.

  map_loader.launch.py  ->  localization.launch.py  ->  navigation.launch.py  (+ RViz)

Optionally also starts the Gazebo simulation (testbed_bringup/testbed_full_bringup.launch.py)
with its own RViz window disabled, so a single terminal runs everything.

Usage:
  # simulation already running in another terminal:
  ros2 launch testbed_navigation testbed_navigation.launch.py
  # everything, including Gazebo:
  ros2 launch testbed_navigation testbed_navigation.launch.py with_sim:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('testbed_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    rviz = LaunchConfiguration('rviz')
    with_sim = LaunchConfiguration('with_sim')

    def include(launch_file, args):
        # Included launch files share ONE launch-configuration scope with their parent.
        # scoped=True keeps the arguments declared by one include from leaking into the next.
        return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, 'launch', launch_file])),
            launch_arguments=args.items())])

    sim = GroupAction(
        scoped=True,
        condition=IfCondition(with_sim),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('testbed_bringup'), 'launch', 'testbed_full_bringup.launch.py'])),
            launch_arguments={'use_rviz': 'false'}.items())])

    def nav_stack():
        return [
            include('map_loader.launch.py', {'map': map_yaml, 'use_sim_time': use_sim_time}),
            include('localization.launch.py', {'use_sim_time': use_sim_time}),
            include('navigation.launch.py', {'use_sim_time': use_sim_time}),
            Node(package='rviz2', executable='rviz2', name='rviz2_navigation', output='log',
                 condition=IfCondition(rviz),
                 arguments=['-d', PathJoinSubstitution([pkg_share, 'rviz', 'testbed_navigation.rviz'])],
                 parameters=[{'use_sim_time': use_sim_time}]),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution(
                [FindPackageShare('testbed_bringup'), 'maps', 'testbed_world.yaml'])),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('with_sim', default_value='false',
                              description='Also start Gazebo + robot (testbed_full_bringup)'),
        sim,
        # when the simulation is started here, give Gazebo time to spawn the robot first
        TimerAction(period=8.0, actions=nav_stack(), condition=IfCondition(with_sim)),
        GroupAction(actions=nav_stack(), condition=UnlessCondition(with_sim)),
    ])
