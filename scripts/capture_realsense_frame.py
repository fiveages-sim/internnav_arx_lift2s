#!/usr/bin/env python3
"""Grab one RealSense RGB-D pair from ROS 2 and save to .npz.

Use system Python (ROS Jazzy), NOT the internnav conda env:

  source /opt/ros/jazzy/setup.bash
  source ~/ARX5_ROBOT_1-main/realsense/install/setup.bash
  # Zenoh router + camera launch must already be running
  /usr/bin/python3 ~/InternNav/scripts/capture_realsense_frame.py --out /tmp/internnav_obs.npz
"""

from __future__ import annotations

import argparse
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def img_to_numpy(msg: Image) -> np.ndarray:
    h, w = msg.height, msg.width
    if msg.encoding in ('rgb8', 'bgr8'):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        if msg.encoding == 'bgr8':
            arr = arr[:, :, ::-1].copy()
        return arr
    if msg.encoding in ('16UC1', 'mono16'):
        return np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
    if msg.encoding == '32FC1':
        return np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
    raise RuntimeError(f'Unsupported encoding: {msg.encoding}')


class OnceGrabber(Node):
    def __init__(self, color_topic: str, depth_topic: str):
        super().__init__('internnav_frame_grabber')
        self.rgb = None
        self.depth = None
        self.create_subscription(Image, color_topic, self._on_color, qos_profile_sensor_data)
        self.create_subscription(Image, depth_topic, self._on_depth, qos_profile_sensor_data)

    def _on_color(self, msg: Image) -> None:
        if self.rgb is None:
            self.rgb = img_to_numpy(msg)
            self.get_logger().info(f'Got RGB {self.rgb.shape} enc={msg.encoding}')

    def _on_depth(self, msg: Image) -> None:
        if self.depth is None:
            depth = img_to_numpy(msg)
            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) / 1000.0
            self.depth = depth
            self.get_logger().info(f'Got depth {self.depth.shape} dtype={self.depth.dtype}')


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', default='/tmp/internnav_obs.npz')
    parser.add_argument('--color-topic', default='/camera_head/color/image_raw')
    parser.add_argument('--depth-topic', default='/camera_head/depth/image_raw')
    parser.add_argument('--timeout', type=float, default=30.0)
    parser.add_argument(
        '--allow-missing-depth',
        action='store_true',
        help='If depth never arrives, fill with 1.5m (DualVLN is RGB-primary)',
    )
    args = parser.parse_args()

    rclpy.init()
    node = OnceGrabber(args.color_topic, args.depth_topic)
    deadline = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
    try:
        while rclpy.ok():
            need_rgb = node.rgb is None
            need_depth = node.depth is None and not args.allow_missing_depth
            if not need_rgb and not need_depth:
                break
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.get_clock().now().nanoseconds > deadline:
                missing = []
                if node.rgb is None:
                    missing.append('RGB')
                if node.depth is None and not args.allow_missing_depth:
                    missing.append('depth')
                if missing:
                    print(
                        f'Timeout waiting for {", ".join(missing)}. '
                        'Check USB is 3.x (not 2.1), camera launch logs, and:\n'
                        '  ros2 topic hz /camera_head/color/image_raw\n'
                        '  ros2 topic hz /camera_head/depth/image_raw',
                        file=sys.stderr,
                    )
                    return 1
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if node.depth is None:
        h, w = node.rgb.shape[:2]
        node.depth = np.ones((h, w), dtype=np.float32) * 1.5
        print('Warning: no depth frame; using constant 1.5 m depth')

    np.savez_compressed(args.out, rgb=node.rgb, depth=node.depth)
    print(f'Saved {args.out} rgb={node.rgb.shape} depth={node.depth.shape}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
