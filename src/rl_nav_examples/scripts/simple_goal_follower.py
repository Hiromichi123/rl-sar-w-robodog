#!/usr/bin/env python3
"""Minimal closed-loop goal follower for rl_real_JXG.

No obstacle avoidance is performed. The node converts a cloud goal and a SLAM
pose estimate into /rl_real/cmd_vel commands.
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class SimpleGoalFollower(Node):
    def __init__(self) -> None:
        super().__init__('rl_simple_goal_follower')

        self.declare_parameter('goal_topic', '/cloud/goal_pose')
        self.declare_parameter('goal_point_topic', '/cloud/goal_point')
        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('slam_pose_topic', '/slam/pose')
        self.declare_parameter('rl_pose2d_topic', '/rl_real/pose2d')
        self.declare_parameter('cmd_vel_topic', '/rl_real/cmd_vel')
        self.declare_parameter('command_topic', '/rl_real/command')
        self.declare_parameter('status_topic', '/rl_nav/status')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('goal_tolerance_m', 0.20)
        self.declare_parameter('yaw_tolerance_rad', 0.15)
        self.declare_parameter('max_vx', 0.25)
        self.declare_parameter('max_vy', 0.18)
        self.declare_parameter('max_yaw_rate', 0.45)
        self.declare_parameter('kp_xy', 0.55)
        self.declare_parameter('kp_yaw', 0.75)
        self.declare_parameter('send_nav_on_on_goal', True)
        self.declare_parameter('send_locomotion_on_goal', False)
        self.declare_parameter('send_nav_off_on_reach', False)
        self.declare_parameter('pose_timeout_s', 1.0)

        self.goal: Optional[Tuple[float, float, float]] = None
        self.pose: Optional[Tuple[float, float, float]] = None
        self.last_pose_time = self.get_clock().now()
        self.active = False

        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.command_pub = self.create_publisher(String, self.get_parameter('command_topic').value, 10)
        self.status_pub = self.create_publisher(String, self.get_parameter('status_topic').value, 10)

        self.create_subscription(PoseStamped, self.get_parameter('goal_topic').value, self.on_goal_pose, 10)
        self.create_subscription(PointStamped, self.get_parameter('goal_point_topic').value, self.on_goal_point, 10)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.on_odom, 10)
        self.create_subscription(PoseStamped, self.get_parameter('slam_pose_topic').value, self.on_slam_pose, 10)
        self.create_subscription(Float32MultiArray, self.get_parameter('rl_pose2d_topic').value, self.on_rl_pose2d, 10)

        period = 1.0 / float(self.get_parameter('control_rate_hz').value)
        self.create_timer(period, self.control_step)
        self.publish_status('ready')

    def publish_command(self, text: str) -> None:
        msg = String()
        msg.data = text
        for _ in range(3):
            self.command_pub.publish(msg)

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def on_goal_pose(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.set_goal(msg.pose.position.x, msg.pose.position.y, yaw)

    def on_goal_point(self, msg: PointStamped) -> None:
        yaw = self.pose[2] if self.pose is not None else 0.0
        self.set_goal(msg.point.x, msg.point.y, yaw)

    def set_goal(self, x: float, y: float, yaw: float) -> None:
        self.goal = (float(x), float(y), float(yaw))
        self.active = True
        if bool(self.get_parameter('send_nav_on_on_goal').value):
            self.publish_command('nav_on')
        if bool(self.get_parameter('send_locomotion_on_goal').value):
            self.publish_command('locomotion')
        self.publish_status(f'goal accepted x={x:.3f} y={y:.3f} yaw={yaw:.3f}')

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.update_pose(p.x, p.y, yaw_from_quat(q.x, q.y, q.z, q.w))

    def on_slam_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        self.update_pose(p.x, p.y, yaw_from_quat(q.x, q.y, q.z, q.w))

    def on_rl_pose2d(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 4:
            self.update_pose(float(msg.data[0]), float(msg.data[1]), float(msg.data[3]))

    def update_pose(self, x: float, y: float, yaw: float) -> None:
        self.pose = (float(x), float(y), float(yaw))
        self.last_pose_time = self.get_clock().now()

    def stop_motion(self) -> None:
        self.cmd_pub.publish(Twist())
        self.publish_command('vel_stop')

    def control_step(self) -> None:
        if not self.active or self.goal is None:
            return
        if self.pose is None:
            self.stop_motion()
            self.publish_status('waiting for pose')
            return

        age = (self.get_clock().now() - self.last_pose_time).nanoseconds * 1e-9
        if age > float(self.get_parameter('pose_timeout_s').value):
            self.stop_motion()
            self.publish_status(f'pose timeout {age:.2f}s')
            return

        x, y, yaw = self.pose
        gx, gy, gyaw = self.goal
        dx = gx - x
        dy = gy - y
        dist = math.hypot(dx, dy)
        yaw_err = wrap_pi(gyaw - yaw)

        if dist <= float(self.get_parameter('goal_tolerance_m').value) and abs(yaw_err) <= float(self.get_parameter('yaw_tolerance_rad').value):
            self.active = False
            self.stop_motion()
            if bool(self.get_parameter('send_nav_off_on_reach').value):
                self.publish_command('nav_off')
            self.publish_status(f'goal reached x={x:.3f} y={y:.3f} yaw={yaw:.3f}')
            return

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        err_body_x = cos_yaw * dx + sin_yaw * dy
        err_body_y = -sin_yaw * dx + cos_yaw * dy

        cmd = Twist()
        cmd.linear.x = clamp(float(self.get_parameter('kp_xy').value) * err_body_x, float(self.get_parameter('max_vx').value))
        cmd.linear.y = clamp(float(self.get_parameter('kp_xy').value) * err_body_y, float(self.get_parameter('max_vy').value))
        cmd.angular.z = clamp(float(self.get_parameter('kp_yaw').value) * yaw_err, float(self.get_parameter('max_yaw_rate').value))
        self.cmd_pub.publish(cmd)


def main() -> None:
    rclpy.init()
    node = SimpleGoalFollower()
    try:
        rclpy.spin(node)
    finally:
        node.stop_motion()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
