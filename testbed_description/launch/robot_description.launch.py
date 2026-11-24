#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # FIX: robot_state_publisher must run on the simulation clock (/clock) like every
    # other node, otherwise its TF stamps are wall-clock time and Nav2 TF lookups fail.
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("testbed_description"),
            "urdf",
            "testbed.xacro",
            ]),
    ])

    # FIX: force the xacro output to be treated as a string (not parsed as YAML)
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo /clock (true) or wall clock (false)'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}]),
    ])
