#!/usr/bin/env python3
"""Rotate/scale Insight9 compressed camera image and republish.

订阅相机原始压缩图 -> 解码 -> 旋转(可选) -> 缩放(可选) -> 重新 JPEG 压缩 -> 发布。
目的：1088x1920 原始大图经 rosbridge 传输 + 前端渲染都很吃力，先在本机缩到小图，
前端订阅缩放后的话题，帧率体验明显更好。
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

_ROTATE_MAP = {
    90: cv2.ROTATE_90_CLOCKWISE,
    180: cv2.ROTATE_180,
    270: cv2.ROTATE_90_COUNTERCLOCKWISE,
}

_SCALE_PRESETS = {
    "full": 1.0,
    "half": 0.5,
    "quarter": 0.25,
}


def _parse_scale(value: str) -> float:
    v = value.strip().lower()
    if v in _SCALE_PRESETS:
        return _SCALE_PRESETS[v]
    return float(v)


class CameraRotateNode(Node):
    def __init__(self, input_topic, output_topic, angle, scale, quality, fps_limit):
        super().__init__("camera_rotate")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self._sub = self.create_subscription(
            CompressedImage, input_topic, self._on_image, qos)
        self._pub = self.create_publisher(CompressedImage, output_topic, qos)

        self._angle = angle
        self._scale = scale
        self._quality = quality
        self._min_interval = (1.0 / fps_limit) if fps_limit and fps_limit > 0 else 0.0
        self._last_pub = 0.0

        self.get_logger().info(
            f"{input_topic} -> {output_topic}  angle={angle} scale={scale} "
            f"quality={quality} fps_limit={fps_limit or 'none'}")

    def _on_image(self, msg: CompressedImage):
        if cv2 is None:
            self.get_logger().error("cv2 not available, cannot process image", throttle_duration_sec=5.0)
            return

        now = time.time()
        if self._min_interval and (now - self._last_pub) < self._min_interval:
            return

        try:
            arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is None:
                self.get_logger().warning("imdecode returned None, drop frame", throttle_duration_sec=5.0)
                return

            if self._angle:
                img = cv2.rotate(img, _ROTATE_MAP[self._angle])

            if self._scale and self._scale != 1.0:
                img = cv2.resize(
                    img, None, fx=self._scale, fy=self._scale,
                    interpolation=cv2.INTER_AREA)

            ok, enc = cv2.imencode(
                ".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, int(self._quality)])
            if not ok:
                self.get_logger().warning("cv2.imencode failed, drop frame", throttle_duration_sec=5.0)
                return

            out = CompressedImage()
            out.header = msg.header
            out.format = "jpeg"
            out.data = enc.tobytes()
            self._pub.publish(out)
            self._last_pub = now
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"decode/rotate failed, drop frame: {exc}",
                                      throttle_duration_sec=5.0)


def main():
    parser = argparse.ArgumentParser(description="Rotate Insight9 compressed camera image")
    parser.add_argument("--input", default="/camera/camera/color/image_rect_raw/compressed")
    parser.add_argument("--output", default="/camera/camera/color/image_rect_raw/rotated/compressed")
    parser.add_argument("--angle", type=int, default=90, choices=[0, 90, 180, 270])
    parser.add_argument("--scale", default="half", help="half/full/quarter 或数字缩放因子")
    parser.add_argument("--quality", type=int, default=80)
    parser.add_argument("--fps", type=float, default=0.0, help="输出限流帧率，0 表示不限")
    args = parser.parse_args()

    rclpy.init()
    node = CameraRotateNode(
        args.input, args.output, args.angle,
        _parse_scale(args.scale), args.quality, args.fps)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
