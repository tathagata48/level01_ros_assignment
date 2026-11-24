#!/usr/bin/env python3
"""Open-loop drive test: publish a constant cmd_vel (v, w) for T seconds of sim time and
compare what the robot actually did (from /odom) with what was commanded.

usage (simulation running, NO navigation stack publishing cmd_vel):
  python3 tools/drive_test.py 0.3 0.0 4.0     # straight: expect ~1.2 m
  python3 tools/drive_test.py 0.0 0.8 3.0     # spin in place: expect ~137 deg and ~0 m travel"""
import sys, math, rclpy, rclpy.parameter
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
v, w, T = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
rclpy.init()
n = Node('drive_test', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
pub = n.create_publisher(Twist, '/cmd_vel', 10)
state = {}
def cb(m):
    q = m.pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    state['p'] = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)
    state['tw'] = (m.twist.twist.linear.x, m.twist.twist.angular.z)
n.create_subscription(Odometry, '/odom', cb, 10)
while 'p' not in state: rclpy.spin_once(n, timeout_sec=0.1)
while n.get_clock().now().nanoseconds == 0: rclpy.spin_once(n, timeout_sec=0.1)
p0 = state['p']; t0 = n.get_clock().now().nanoseconds * 1e-9
tw = Twist(); tw.linear.x = v; tw.angular.z = w
samples = []
while n.get_clock().now().nanoseconds * 1e-9 - t0 < T:
    pub.publish(tw); rclpy.spin_once(n, timeout_sec=0.05)
    samples.append(state['tw'])
pub.publish(Twist())
for _ in range(10): rclpy.spin_once(n, timeout_sec=0.05); pub.publish(Twist())
p1 = state['p']
dist = math.hypot(p1[0]-p0[0], p1[1]-p0[1])
dyaw = math.atan2(math.sin(p1[2]-p0[2]), math.cos(p1[2]-p0[2]))
late = samples[len(samples)//2:]
print(f"cmd v={v} w={w} for {T}s | start {tuple(round(a,3) for a in p0)} end {tuple(round(a,3) for a in p1)}")
print(f"  travelled {dist:.3f} m (ideal {abs(v)*T:.3f}), yaw change {math.degrees(dyaw):.1f} deg (ideal {math.degrees(w*T):.1f})")
print(f"  mean measured v={sum(s[0] for s in late)/len(late):.3f}  w={sum(s[1] for s in late)/len(late):.3f} (2nd half)")
rclpy.shutdown()
