#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""rl_real_JXG ROS2 控制脚本: 站立 / 趴下 / 强化学习行走 / 速度运动 / 状态与校验。

依赖机器人端 rl_real_JXG 服务(ROS2 话题接口):
  发布:
    /rl_real/command   (std_msgs/String)  getup/getdown/locomotion/passive/...
    /rl_real/cmd_vel   (geometry_msgs/Twist)
    /rl_real/check_stand      (std_msgs/String)  趴姿近零位检查(站立前检查)
    /rl_real/check_standing   (std_msgs/String)  站立期望值校验
  订阅:
    /rl_real/posture                 (~8Hz JSON: posture/fsm/settled/...)
    /rl_real/feedback                (文本)
    /rl_real/check_stand_result
    /rl_real/check_standing_result

姿态(posture)取值:
  lying       趴下(Passive)
  standing    站立(SafeHold 接管 / GetUp 完成)
  locomotion  强化学习行走中
  transition  动作插值/模型加载中
  unknown     未就绪

注意:
  1. 机器人端 cmd_vel 有 0.5s 超时保护, 本脚本以 20Hz 持续发布当前速度目标;
  2. 行走前必须处于 locomotion 姿态(RL 模型接管), 可加 --auto 自动 站立→强化学习;
  3. 首次实跑请扶机/架机, 确认姿态切换与预期一致后再无人看护运行。

示例:
  python3 rl_real_ros2_teleop.py status
  python3 rl_real_ros2_teleop.py check                 # 站立前趴姿检查
  python3 rl_real_ros2_teleop.py stand
  python3 rl_real_ros2_teleop.py check-standing        # 站立期望值校验
  python3 rl_real_ros2_teleop.py down
  python3 rl_real_ros2_teleop.py rl
  python3 rl_real_ros2_teleop.py move --x 0.3 --yaw 0.0 --duration 2.0 --auto
  python3 rl_real_ros2_teleop.py stop
  python3 rl_real_ros2_teleop.py interactive
"""

import argparse
import json
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

COMMAND_TOPIC = "/rl_real/command"
CMD_VEL_TOPIC = "/rl_real/cmd_vel"
POSTURE_TOPIC = "/rl_real/posture"
FEEDBACK_TOPIC = "/rl_real/feedback"
CHECK_STAND_TOPIC = "/rl_real/check_stand"
CHECK_STAND_RESULT_TOPIC = "/rl_real/check_stand_result"
CHECK_STANDING_TOPIC = "/rl_real/check_standing"
CHECK_STANDING_RESULT_TOPIC = "/rl_real/check_standing_result"

VEL_PUBLISH_HZ = 20.0
# 姿态切换超时(秒): getup 约 3s, getdown 约 2s, locomotion 含模型加载给足余量
DEFAULT_STAND_TIMEOUT = 10.0
DEFAULT_DOWN_TIMEOUT = 10.0
DEFAULT_RL_TIMEOUT = 12.0


class RobotTeleop(Node):
    def __init__(self, verbose: bool = True):
        super().__init__("rl_real_teleop")
        self.verbose = verbose
        self._lock = threading.Lock()

        # 发布器
        self.cmd_pub = self.create_publisher(String, COMMAND_TOPIC, 10)
        self.vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.check_stand_pub = self.create_publisher(String, CHECK_STAND_TOPIC, 10)
        self.check_standing_pub = self.create_publisher(String, CHECK_STANDING_TOPIC, 10)

        # 状态缓存
        self._posture = {"posture": "unknown", "fsm": "", "settled": False}
        self._last_feedback = ""
        self._check_stand_result = None
        self._check_standing_result = None
        self._check_stand_evt = threading.Event()
        self._check_standing_evt = threading.Event()

        # 订阅器
        self.create_subscription(String, POSTURE_TOPIC, self._on_posture, 10)
        self.create_subscription(String, FEEDBACK_TOPIC, self._on_feedback, 10)
        self.create_subscription(String, CHECK_STAND_RESULT_TOPIC, self._on_check_stand, 10)
        self.create_subscription(String, CHECK_STANDING_RESULT_TOPIC, self._on_check_standing, 10)

        # 当前速度目标(20Hz 持续发布, 规避机器人端 0.5s cmd_vel 超时);
        # 全零状态只维持 0.6s(覆盖一个超时窗口)后静默, 避免服务端对每帧回
        # "cmd_vel accepted" 造成反馈刷屏
        self._vel_target = (0.0, 0.0, 0.0)
        self._zero_since = None
        self._vel_timer = self.create_timer(1.0 / VEL_PUBLISH_HZ, self._publish_vel)

    # ---------- 话题回调 ----------
    def _on_posture(self, msg: String):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        with self._lock:
            self._posture = data
        if self.verbose:
            print(f"[posture] {msg.data}")

    def _on_feedback(self, msg: String):
        with self._lock:
            self._last_feedback = msg.data
        if self.verbose:
            print(f"[feedback] {msg.data}")

    def _on_check_stand(self, msg: String):
        try:
            self._check_stand_result = json.loads(msg.data)
        except (ValueError, TypeError):
            self._check_stand_result = {"raw": msg.data}
        self._check_stand_evt.set()
        if self.verbose:
            print(f"[check_stand_result] {msg.data}")

    def _on_check_standing(self, msg: String):
        try:
            self._check_standing_result = json.loads(msg.data)
        except (ValueError, TypeError):
            self._check_standing_result = {"raw": msg.data}
        self._check_standing_evt.set()
        if self.verbose:
            print(f"[check_standing_result] {msg.data}")

    # ---------- 基础接口 ----------
    def _publish_vel(self):
        x, y, yaw = self._vel_target
        now = time.time()
        is_zero = (x == 0.0 and y == 0.0 and yaw == 0.0)
        if is_zero:
            if self._zero_since is None:
                self._zero_since = now  # 刚归零: 续发 0.6s 零速, 确保机器人锁停
            if now - self._zero_since > 0.6:
                return  # 已静止超过一个超时窗口, 停止发布
        else:
            self._zero_since = None
        msg = Twist()
        msg.linear.x = float(x)
        msg.linear.y = float(y)
        msg.angular.z = float(yaw)
        self.vel_pub.publish(msg)

    def set_velocity(self, x: float, y: float, yaw: float):
        self._vel_target = (float(x), float(y), float(yaw))

    def send_command(self, command: str):
        """发送行为命令(连发 3 帧抗丢包, 与机器人端去重窗口兼容)。"""
        msg = String()
        msg.data = command
        for _ in range(3):
            self.cmd_pub.publish(msg)
            time.sleep(0.05)

    @property
    def posture(self) -> str:
        with self._lock:
            return str(self._posture.get("posture", "unknown"))

    @property
    def posture_full(self) -> dict:
        with self._lock:
            return dict(self._posture)

    @property
    def last_feedback(self) -> str:
        with self._lock:
            return self._last_feedback

    def wait_posture(self, targets, timeout: float) -> bool:
        """阻塞等待姿态进入 targets(集合)。transition 视为未到位继续等待。"""
        targets = set(targets)
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.posture in targets:
                return True
            time.sleep(0.1)
        return False

    # ---------- 高层行为 ----------
    def stand(self, timeout: float = DEFAULT_STAND_TIMEOUT) -> bool:
        if self.posture == "standing":
            print("[stand] 已处于站立姿态")
            return True
        print("[stand] 发送 getup ...")
        self.send_command("getup")
        ok = self.wait_posture({"standing"}, timeout)
        if ok:
            print("[stand] 站立完成")
        else:
            print(f"[stand] 超时未站立(当前={self.posture}): {self.last_feedback}")
        return ok

    def getdown(self, timeout: float = DEFAULT_DOWN_TIMEOUT) -> bool:
        if self.posture == "lying":
            print("[down] 已处于趴下姿态")
            return True
        print("[down] 发送 getdown ...")
        self.send_command("getdown")
        ok = self.wait_posture({"lying"}, timeout)
        if ok:
            print("[down] 趴下完成")
        else:
            print(f"[down] 超时未趴下(当前={self.posture}): {self.last_feedback}")
        return ok

    def locomotion(self, timeout: float = DEFAULT_RL_TIMEOUT) -> bool:
        if self.posture == "locomotion":
            print("[rl] 已处于强化学习行走态")
            return True
        print("[rl] 发送 locomotion (加载 ONNX 策略) ...")
        self.send_command("locomotion")
        ok = self.wait_posture({"locomotion"}, timeout)
        if ok:
            print("[rl] 强化学习模型已接管")
        else:
            print(f"[rl] 超时未进入 locomotion(当前={self.posture}): {self.last_feedback}")
        return ok

    def stop(self):
        """速度归零并锁定(保持当前姿态, 不下蹲)。"""
        self.set_velocity(0.0, 0.0, 0.0)
        self.send_command("vel_stop")
        print("[stop] 速度已归零")

    def passive(self):
        """急停卸载(仅 locomotion/standing 安全设计见机器人端 FSM)。"""
        self.set_velocity(0.0, 0.0, 0.0)
        self.send_command("passive")
        print("[passive] 已发送 passive 卸载命令")

    def check_stand(self, timeout: float = 3.0):
        """趴姿近零位检查(站立前检查), 返回结果 dict。"""
        self._check_stand_evt.clear()
        self._check_stand_result = None
        msg = String()
        msg.data = "check"
        for _ in range(3):
            self.check_stand_pub.publish(msg)
            time.sleep(0.05)
        self._check_stand_evt.wait(timeout)
        return self._check_stand_result

    def check_standing(self, timeout: float = 3.0):
        """站立期望值校验(default_dof_pos 邻域 + 静止 + IMU), 返回结果 dict。"""
        self._check_standing_evt.clear()
        self._check_standing_result = None
        msg = String()
        msg.data = "check"
        for _ in range(3):
            self.check_standing_pub.publish(msg)
            time.sleep(0.05)
        self._check_standing_evt.wait(timeout)
        return self._check_standing_result

    def move(self, x: float, y: float, yaw: float, duration: float,
             auto: bool = False, vx_max: float = 1.0, vyw_max: float = 1.0) -> bool:
        """以指定速度运动 duration 秒后自动停。

        auto=True 时自动完成 趴下→站立→强化学习 的前置切换。
        """
        # 软限幅, 避免首次测试速度过大
        x = max(-vx_max, min(vx_max, x))
        y = max(-vx_max, min(vx_max, y))
        yaw = max(-vyw_max, min(vyw_max, yaw))

        if self.posture != "locomotion":
            if not auto:
                print(f"[move] 当前姿态={self.posture}, 非 locomotion。"
                      f"请先执行: stand 然后 rl, 或使用 --auto")
                return False
            print("[move] --auto: 自动进入强化学习态 ...")
            if self.posture == "lying":
                if not self.stand():
                    return False
            if not self.locomotion():
                return False

        print(f"[move] vx={x:.2f} vy={y:.2f} yaw={yaw:.2f}, 持续 {duration:.1f}s")
        self.set_velocity(x, y, yaw)
        try:
            time.sleep(duration)
        finally:
            self.set_velocity(0.0, 0.0, 0.0)
        print("[move] 结束, 速度归零")
        return True


def spin_background(node: Node):
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    return thread


def cmd_status(robot: RobotTeleop, _args) -> int:
    robot.send_command("status")
    time.sleep(0.6)
    print(json.dumps(robot.posture_full, ensure_ascii=False, indent=2))
    return 0


def cmd_check(robot: RobotTeleop, _args) -> int:
    res = robot.check_stand()
    if res is None:
        print("未收到 check_stand_result", file=sys.stderr)
        return 2
    print(json.dumps(res, ensure_ascii=False, indent=2))
    return 0 if res.get("ready") else 1


def cmd_check_standing(robot: RobotTeleop, _args) -> int:
    res = robot.check_standing()
    if res is None:
        print("未收到 check_standing_result", file=sys.stderr)
        return 2
    print(json.dumps(res, ensure_ascii=False, indent=2))
    return 0 if res.get("at_stand") else 1


def cmd_stand(robot: RobotTeleop, args) -> int:
    return 0 if robot.stand(args.timeout) else 1


def cmd_down(robot: RobotTeleop, args) -> int:
    return 0 if robot.getdown(args.timeout) else 1


def cmd_locomotion(robot: RobotTeleop, args) -> int:
    return 0 if robot.locomotion(args.timeout) else 1


def cmd_stop(robot: RobotTeleop, _args) -> int:
    robot.stop()
    return 0


def cmd_passive(robot: RobotTeleop, _args) -> int:
    robot.passive()
    return 0


def cmd_move(robot: RobotTeleop, args) -> int:
    return 0 if robot.move(args.x, args.y, args.yaw, args.duration,
                           auto=args.auto, vx_max=args.vx_max, vyw_max=args.yaw_max) else 1


def cmd_interactive(robot: RobotTeleop, _args) -> int:
    menu = (
        "\n==== rl_real 交互控制 ====\n"
        "  s  站立(getup)          d  趴下(getdown)\n"
        "  l  进入强化学习(locomotion)\n"
        "  m  移动(输入 vx vy yaw 秒数, 自动持续发布)\n"
        "  t  速度归零停止         p  passive 卸载\n"
        "  c  站立前趴姿检查       v  站立期望值校验\n"
        "  h  查询当前姿态         q  退出\n"
        "=========================="
    )
    print(menu)
    while True:
        try:
            key = input("> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if key in ("q", "quit", "exit"):
            break
        elif key == "s":
            robot.stand()
        elif key == "d":
            robot.getdown()
        elif key == "l":
            robot.locomotion()
        elif key == "t":
            robot.stop()
        elif key == "p":
            robot.passive()
        elif key == "c":
            res = robot.check_stand()
            print(json.dumps(res, ensure_ascii=False) if res else "无响应")
        elif key == "v":
            res = robot.check_standing()
            print(json.dumps(res, ensure_ascii=False) if res else "无响应")
        elif key == "h":
            print(json.dumps(robot.posture_full, ensure_ascii=False))
        elif key == "m":
            if robot.posture != "locomotion":
                ans = input("当前不在 locomotion, 是否自动 站立→强化学习? [y/N] ").strip().lower()
                if ans != "y":
                    continue
                if robot.posture == "lying" and not robot.stand():
                    continue
                if not robot.locomotion():
                    continue
            try:
                vals = input("输入 vx vy yaw duration(空格分隔, 例: 0.3 0 0 2): ").split()
                x, y, yaw, dur = map(float, vals[:4])
            except (ValueError, IndexError):
                print("输入无效, 示例: 0.3 0 0 2")
                continue
            robot.move(x, y, yaw, dur, auto=False)
        elif key == "":
            pass
        else:
            print("未知指令, 请参考菜单")
            print(menu)
    robot.set_velocity(0.0, 0.0, 0.0)
    time.sleep(0.7)  # 确保零速续发满一个 cmd_vel 超时窗口
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="rl_real_JXG ROS2 控制: 站立/趴下/强化学习/运动/状态查询")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("stand", help="站立(getup), 等待姿态=standing")
    p.add_argument("--timeout", type=float, default=DEFAULT_STAND_TIMEOUT)
    p.set_defaults(func=cmd_stand)

    p = sub.add_parser("down", help="趴下(getdown), 等待姿态=lying")
    p.add_argument("--timeout", type=float, default=DEFAULT_DOWN_TIMEOUT)
    p.set_defaults(func=cmd_down)

    p = sub.add_parser("rl", help="进入强化学习行走态(locomotion)")
    p.add_argument("--timeout", type=float, default=DEFAULT_RL_TIMEOUT)
    p.set_defaults(func=cmd_locomotion)

    p = sub.add_parser("move", help="在 locomotion 下以给定速度运动指定时长")
    p.add_argument("--x", type=float, default=0.3, help="前后速度 m/s")
    p.add_argument("--y", type=float, default=0.0, help="侧移速度 m/s")
    p.add_argument("--yaw", type=float, default=0.0, help="转向角速度 rad/s")
    p.add_argument("--duration", type=float, default=2.0, help="运动时长(秒)")
    p.add_argument("--auto", action="store_true", help="自动 站立→强化学习 前置切换")
    p.add_argument("--vx-max", type=float, default=1.0, dest="vx_max")
    p.add_argument("--yaw-max", type=float, default=1.0, dest="yaw_max")
    p.set_defaults(func=cmd_move)

    p = sub.add_parser("stop", help="速度归零(保持当前姿态)")
    p.set_defaults(func=cmd_stop)

    p = sub.add_parser("passive", help="发送 passive 卸载")
    p.set_defaults(func=cmd_passive)

    p = sub.add_parser("status", help="查询当前姿态与 FSM 状态")
    p.set_defaults(func=cmd_status)

    p = sub.add_parser("check", help="站立前检查: 趴姿近零位+IMU(check_stand)")
    p.set_defaults(func=cmd_check)

    p = sub.add_parser("check-standing", help="站立期望值校验: 关节在 default_dof_pos 邻域")
    p.set_defaults(func=cmd_check_standing)

    p = sub.add_parser("interactive", help="交互式菜单控制")
    p.set_defaults(func=cmd_interactive)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    rclpy.init(args=None)
    robot = RobotTeleop(verbose=True)
    spin_background(robot)

    # 等姿态话题首帧(服务端 ~8Hz 广播), 最多 1.5s
    deadline = time.time() + 1.5
    while time.time() < deadline and robot.posture == "unknown":
        time.sleep(0.1)

    try:
        rc = args.func(robot, args)
    finally:
        robot.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
