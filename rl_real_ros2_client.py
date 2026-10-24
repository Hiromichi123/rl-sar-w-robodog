#!/usr/bin/env python3
"""Small ROS2 command client for rl_real_JXG.

Examples:
  python3 rl_real_ros2_client.py stand
  python3 rl_real_ros2_client.py down
  python3 rl_real_ros2_client.py vel 0.2 0.0 0.0
  python3 rl_real_ros2_client.py stop
"""
import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

COMMAND_TOPIC = "/rl_real/command"
CMD_VEL_TOPIC = "/rl_real/cmd_vel"
IMU_STREAM_TOPIC = "/rl_real/imu_stream_enable"


def _publish_once(msg_type, topic, msg, count=3, interval=0.08):
    rclpy.init(args=None)
    node = rclpy.create_node("rl_real_ros2_client")
    pub = node.create_publisher(msg_type, topic, 10)
    deadline = time.time() + 0.5
    while time.time() < deadline and pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.05)
    for _ in range(count):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(interval)
    node.destroy_node()
    rclpy.shutdown()


def command(name: str):
    msg = String()
    msg.data = name
    _publish_once(String, COMMAND_TOPIC, msg)


def velocity(x: float, y: float, yaw: float):
    msg = Twist()
    msg.linear.x = float(x)
    msg.linear.y = float(y)
    msg.angular.z = float(yaw)
    _publish_once(Twist, CMD_VEL_TOPIC, msg)


def stop():
    velocity(0.0, 0.0, 0.0)
    command("vel_stop")


def stand():
    command("getup")


def down():
    command("getdown")


def locomotion():
    command("locomotion")


def passive():
    command("passive")


def set_zero_all():
    command("set_zero_all")


def imu_stream(enable: bool):
    msg = Bool()
    msg.data = bool(enable)
    _publish_once(Bool, IMU_STREAM_TOPIC, msg)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Publish ROS2 commands to rl_real_JXG")
    sub = parser.add_subparsers(dest="cmd", required=True)
    for name in ["stand", "down", "locomotion", "passive", "stop", "set_zero_all", "status", "nav_on", "nav_off"]:
        sub.add_parser(name)
    vel_p = sub.add_parser("vel")
    vel_p.add_argument("x", type=float)
    vel_p.add_argument("y", type=float)
    vel_p.add_argument("yaw", type=float)
    imu_p = sub.add_parser("imu")
    imu_p.add_argument("state", choices=["on", "off"])
    raw_p = sub.add_parser("raw")
    raw_p.add_argument("command")
    args = parser.parse_args(argv)

    if args.cmd == "vel":
        velocity(args.x, args.y, args.yaw)
    elif args.cmd == "imu":
        imu_stream(args.state == "on")
    elif args.cmd == "raw":
        command(args.command)
    elif args.cmd == "stop":
        stop()
    else:
        command(args.cmd)


if __name__ == "__main__":
    main(sys.argv[1:])
