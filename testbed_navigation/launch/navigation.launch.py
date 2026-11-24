#!/usr/bin/env python3
"""
Step 3 of the navigation workflow: the Nav2 navigation servers.

Starts (each one is a lifecycle node, parameters in config/nav2_params.yaml):
  * planner_server     global planner (NavFn)             -> action  /compute_path_to_pose
  * controller_server  local planner  (DWB)               -> action  /follow_path
  * behavior_server    recoveries (spin/backup/wait/...)  -> actions /spin, /backup, ...
  * bt_navigator       runs the behavior tree             -> action  /navigate_to_pose
  * waypoint_follower  multi-goal missions                -> action  /follow_waypoints
  * velocity_smoother  rate/accel limits on cmd_vel
  * collision_monitor  last safety layer (optional, use_collision_monitor:=false to skip)
  * lifecycle_manager_navigation brings all of the above up in order

Velocity topics:
  controller/behavior --cmd_vel_nav--> velocity_smoother --cmd_vel_smoothed--> collision_monitor --cmd_vel--> robot
  (without the collision monitor the smoother publishes straight to cmd_vel)

Prerequisites: map_loader.launch.py and localization.launch.py are running
(the global costmap needs /map and the map->odom TF from AMCL).

Usage:
  ros2 launch testbed_navigation navigation.launch.py
  ros2 launch testbed_navigation navigation.launch.py use_collision_monitor:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration('nav2_params_file')
    bt_xml = LaunchConfiguration('bt_xml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    use_cm = LaunchConfiguration('use_collision_monitor').perform(context).lower() in ('true', '1', 'yes')

    common_params = [params_file, {'use_sim_time': use_sim_time}]
    ros_args = ['--ros-args', '--log-level', log_level]
    smoothed_topic = 'cmd_vel_smoothed' if use_cm else 'cmd_vel'

    nodes = [
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             output='screen', parameters=common_params, arguments=ros_args,
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=common_params, arguments=ros_args),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             output='screen', parameters=common_params, arguments=ros_args,
             # recoveries also go through the smoother + collision monitor
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', arguments=ros_args,
             parameters=common_params + [{'default_nav_to_pose_bt_xml': bt_xml}]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen', parameters=common_params,
             arguments=ros_args),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen', parameters=common_params,
             arguments=ros_args,
             remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', smoothed_topic)]),
    ]
    lifecycle_nodes = ['controller_server', 'planner_server', 'behavior_server',
                       'bt_navigator', 'waypoint_follower', 'velocity_smoother']

    if use_cm:
        nodes.append(
            Node(package='nav2_collision_monitor', executable='collision_monitor',
                 name='collision_monitor', output='screen', parameters=common_params,
                 arguments=ros_args))
        lifecycle_nodes.append('collision_monitor')

    nodes.append(
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen', arguments=ros_args,
             parameters=[{'use_sim_time': use_sim_time,
                          'autostart': autostart,
                          'node_names': lifecycle_nodes}]))
    return nodes


def generate_launch_description():
    pkg_share = FindPackageShare('testbed_navigation')
    return LaunchDescription([
        # make node output appear line-by-line in the terminal
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml']),
            description='Nav2 navigation parameter file'),
        DeclareLaunchArgument(
            'bt_xml',
            default_value=PathJoinSubstitution(
                [pkg_share, 'behavior_trees', 'navigate_to_pose_testbed.xml']),
            description='Behavior tree used for NavigateToPose'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use the Gazebo /clock'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Configure + activate the servers automatically'),
        DeclareLaunchArgument('use_collision_monitor', default_value='true',
                              description='Insert nav2_collision_monitor before /cmd_vel'),
        DeclareLaunchArgument('log_level', default_value='info', description='ROS log level'),
        OpaqueFunction(function=launch_setup),
    ])
