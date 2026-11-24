#!/usr/bin/env python3
"""
Send navigation goals to Nav2's bt_navigator through the NavigateToPose action.

This is exactly what RViz's "Nav2 Goal" button does, written out so the goal
flow is visible:  client -> /navigate_to_pose (action) -> bt_navigator -> behavior tree
-> /compute_path_to_pose (planner_server) + /follow_path (controller_server).

Examples
  # single goal: x y yaw(deg) in the map frame
  ros2 run testbed_navigation send_goals.py --goal 6.0 7.5 0
  # visit every room of the test world and come back
  ros2 run testbed_navigation send_goals.py --tour
  # (re)set AMCL's initial pose first, then go
  ros2 run testbed_navigation send_goals.py --initial-pose 0 5 0 --goal -6.5 4 90
"""
import argparse
import math
import sys
import time

import rclpy
import rclpy.time
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformException, TransformListener

# A tour through every room of testbed_playground.world (map frame, metres / degrees).
# Each point was checked to have > 1.7 m clearance on the static map.
TOUR = [
    ('A upper-right room', 6.0, 7.5, 0.0),
    ('B upper-left room', -6.5, 4.0, 180.0),
    ('C lower-left room (via left door)', -5.5, -6.0, -90.0),
    ('D lower-middle room (via wall gap)', 1.0, -5.0, 0.0),
    ('E lower-right room (between diagonal walls)', 8.0, -5.5, 0.0),
    ('back to start', 0.0, 5.0, 0.0),
]

STATUS = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
          GoalStatus.STATUS_CANCELED: 'CANCELED'}


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class GoalSender(Node):
    def __init__(self):
        super().__init__('testbed_goal_sender',
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        # The robot's localized pose = TF map -> base_footprint (map->odom from AMCL, odom->base
        # from odometry). /amcl_pose is NOT used for this: it is only republished after the robot
        # has moved update_min_d / update_min_a, so it can lag behind the real pose.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._last_print = 0.0

    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (t.transform.translation.x, t.transform.translation.y, yaw)

    def set_initial_pose(self, x, y, yaw_deg):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        q = yaw_to_quaternion(math.radians(yaw_deg))
        (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = q
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25 ** 2
        msg.pose.covariance[35] = math.radians(15) ** 2
        for _ in range(3):                      # make sure AMCL gets it
            self.initial_pose_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().info(f'initial pose set to ({x:.2f}, {y:.2f}, {yaw_deg:.0f} deg)')

    def go_to(self, name, x, y, yaw_deg, timeout_s=300.0):
        if not self.client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('/navigate_to_pose action server not available - is navigation.launch.py running?')
            return False
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        q = yaw_to_quaternion(math.radians(yaw_deg))
        (goal.pose.pose.orientation.x, goal.pose.pose.orientation.y,
         goal.pose.pose.orientation.z, goal.pose.pose.orientation.w) = q

        self.get_logger().info(f'--> goal "{name}": ({x:.2f}, {y:.2f}, {yaw_deg:.0f} deg)')
        t0 = time.monotonic()
        send_future = self.client.send_goal_async(goal, feedback_callback=self._feedback)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if not handle.accepted:
            self.get_logger().error('goal rejected')
            return False
        result_future = handle.get_result_async()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - t0 > timeout_s:
                self.get_logger().warn('timeout - cancelling goal')
                handle.cancel_goal_async()
                return False
        status = result_future.result().status
        wall = time.monotonic() - t0
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)
        err = ''
        pose = self.robot_pose()
        if pose:
            ax, ay, ayaw = pose
            dyaw = math.degrees(math.atan2(math.sin(ayaw - math.radians(yaw_deg)),
                                           math.cos(ayaw - math.radians(yaw_deg))))
            err = f' | final pose (TF map->base_footprint) ({ax:.2f}, {ay:.2f}), position error {math.hypot(ax - x, ay - y):.2f} m, yaw error {dyaw:.1f} deg'
        self.get_logger().info(f'<-- "{name}": {STATUS.get(status, status)} after {wall:.1f} s (wall){err}')
        return status == GoalStatus.STATUS_SUCCEEDED

    def _feedback(self, fb):
        now = time.monotonic()
        if now - self._last_print > 5.0:
            f = fb.feedback
            self.get_logger().info(
                f'    distance remaining {f.distance_remaining:.2f} m, recoveries {f.number_of_recoveries}, '
                f'nav time {f.navigation_time.sec} s')
            self._last_print = now


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--goal', nargs=3, type=float, metavar=('X', 'Y', 'YAW_DEG'))
    parser.add_argument('--tour', action='store_true', help='visit every room of the test world')
    parser.add_argument('--initial-pose', nargs=3, type=float, metavar=('X', 'Y', 'YAW_DEG'))
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    if not (args.goal or args.tour or args.initial_pose):
        parser.error('give --goal, --tour and/or --initial-pose')

    rclpy.init()
    node = GoalSender()
    ok = True
    try:
        if args.initial_pose:
            node.set_initial_pose(*args.initial_pose)
        goals = []
        if args.goal:
            goals.append(('goal', *args.goal))
        if args.tour:
            goals += TOUR
        results = [(g[0], node.go_to(*g)) for g in goals]
        if results:
            node.get_logger().info('summary: ' + ', '.join(f'{n}={"OK" if r else "FAIL"}' for n, r in results))
            ok = all(r for _, r in results)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
