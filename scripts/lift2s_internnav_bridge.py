#!/usr/bin/env python3
"""Lift2S ↔ InternNav DualVLN bridge (camera → HTTP agent → /cmd_vel).

Run with system Python + ROS (NOT conda), while:
  - Sim-06: start_server.py on :8087
  - Robot: rmw_zenohd + RealSense + lift2s quick_start

Default is DRY-RUN (prints actions only). Motion requires --enable-motion.

  source /opt/ros/jazzy/setup.bash
  source ~/ARX5_ROBOT_1-main/realsense/install/setup.bash
  source ~/lift2s-ws/install/setup.bash   # if needed for env

  # Dry-run (safe):
  /usr/bin/python3 ~/InternNav/scripts/lift2s_internnav_bridge.py \
    --host 192.168.110.35 \
    --instruction "go forward a little bit then stop" \
    --steps 5

  # Motion (e-stop ready, clear floor):
  /usr/bin/python3 ~/InternNav/scripts/lift2s_internnav_bridge.py \
    --host 192.168.110.35 \
    --instruction "go forward a little bit then stop" \
    --steps 5 \
    --enable-motion
"""

from __future__ import annotations

import argparse
import base64
import pickle
import sys
import time

import numpy as np
import requests
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

# Habitat-style discrete actions used by DualVLN / InternNav realworld client
ACTION_STOP = 0
ACTION_FORWARD = 1
ACTION_TURN_LEFT = 2
ACTION_TURN_RIGHT = 3
ACTION_LOOK_DOWN = 5


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


def action_name(code: int) -> str:
    return {
        ACTION_STOP: 'STOP',
        ACTION_FORWARD: 'FORWARD',
        ACTION_TURN_LEFT: 'TURN_LEFT',
        ACTION_TURN_RIGHT: 'TURN_RIGHT',
        ACTION_LOOK_DOWN: 'LOOK_DOWN',
        -1: 'STOP(-1)',
    }.get(code, f'UNKNOWN({code})')


class ThinAgentClient:
    """Minimal Agent Server client (no internnav import — works on system Python)."""

    def __init__(self, host: str, port: int, model_path: str, width: int, height: int):
        self.base_url = f'http://{host}:{port}'
        session = requests.Session()
        session.trust_env = False  # ignore http_proxy for LAN
        self.session = session

        agent_config = {
            'server_host': host,
            'server_port': port,
            'model_name': 'internvla_n1',
            'ckpt_path': '',
            'model_settings': {
                'policy_name': 'InternVLAN1_Policy',
                'state_encoder': None,
                'env_num': 1,
                'sim_num': 1,
                'model_path': model_path,
                'camera_intrinsic': [
                    [396.23, 0.0, 323.22],
                    [0.0, 395.82, 246.21],
                    [0.0, 0.0, 1.0],
                ],
                'width': width,
                'height': height,
                'hfov': 79,
                'resize_w': 384,
                'resize_h': 384,
                'max_new_tokens': 1024,
                'num_frames': 32,
                'num_history': 8,
                'num_future_steps': 4,
                'device': 'cuda:0',
                'predict_step_nums': 32,
                'continuous_traj': True,
            },
        }
        print(f'Connecting to {self.base_url} ...')
        r = self.session.post(
            f'{self.base_url}/agent/init',
            json={'agent_config': agent_config},
            headers={'Content-Type': 'application/json'},
            timeout=600,
        )
        r.raise_for_status()
        self.agent_name = r.json()['agent_name']
        print(f'Agent init OK: {self.agent_name}')

    def step(self, rgb: np.ndarray, depth: np.ndarray, instruction: str):
        obs = [{'rgb': rgb, 'depth': depth, 'instruction': instruction}]
        payload = {
            'observation': base64.b64encode(pickle.dumps(obs)).decode('utf-8'),
        }
        r = self.session.post(
            f'{self.base_url}/agent/{self.agent_name}/step',
            json=payload,
            headers={'Content-Type': 'application/json'},
            timeout=600,
        )
        r.raise_for_status()
        return r.json()['action']


class CameraBuffer(Node):
    def __init__(self, color_topic: str, depth_topic: str, cmd_vel_topic: str):
        super().__init__('lift2s_internnav_bridge')
        self.rgb = None
        self.depth = None
        self.create_subscription(Image, color_topic, self._on_color, qos_profile_sensor_data)
        self.create_subscription(Image, depth_topic, self._on_depth, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

    def _on_color(self, msg: Image) -> None:
        self.rgb = img_to_numpy(msg)

    def _on_depth(self, msg: Image) -> None:
        depth = img_to_numpy(msg)
        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) / 1000.0
        self.depth = depth

    def wait_for_frames(self, timeout: float = 15.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.rgb is not None and self.depth is not None:
                return True
        return False

    def publish_twist(self, vx: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def stop(self) -> None:
        self.publish_twist(0.0, 0.0)


def parse_first_action(raw) -> int:
    """Parse AgentServer step result into one discrete action code."""
    if isinstance(raw, list) and raw:
        item = raw[0]
        if isinstance(item, dict) and 'action' in item:
            acts = item['action']
            return int(acts[0] if isinstance(acts, list) else acts)
        if isinstance(item, (list, tuple)):
            return int(item[0])
        return int(item)
    raise ValueError(f'Unexpected action payload: {raw!r}')


def twist_for_action(code: int, v_lin: float, v_ang: float) -> tuple[float, float]:
    if code == ACTION_FORWARD:
        return v_lin, 0.0
    if code == ACTION_TURN_LEFT:
        return 0.0, v_ang
    if code == ACTION_TURN_RIGHT:
        return 0.0, -v_ang
    # 0, -1, 5, unknown → no base motion
    return 0.0, 0.0


def main() -> int:
    parser = argparse.ArgumentParser(description='Lift2S InternNav DualVLN bridge')
    parser.add_argument('--host', default='192.168.110.35')
    parser.add_argument('--port', type=int, default=8087)
    parser.add_argument('--model-path', default='checkpoints/InternVLA-N1-DualVLN')
    parser.add_argument('--instruction', default='go forward a little bit then stop')
    parser.add_argument('--color-topic', default='/camera_head/color/image_raw')
    parser.add_argument('--depth-topic', default='/camera_head/depth/image_raw')
    parser.add_argument('--cmd-vel-topic', default='/cmd_vel')
    parser.add_argument('--steps', type=int, default=5, help='Number of inference steps')
    parser.add_argument('--period', type=float, default=2.0, help='Seconds between steps')
    parser.add_argument('--pulse', type=float, default=0.8, help='Seconds to apply each motion')
    parser.add_argument('--v-lin', type=float, default=0.08, help='Forward speed (m/s)')
    parser.add_argument('--v-ang', type=float, default=0.25, help='Yaw rate (rad/s)')
    parser.add_argument(
        '--enable-motion',
        action='store_true',
        help='Actually publish /cmd_vel (default: dry-run print only)',
    )
    args = parser.parse_args()

    if args.enable_motion:
        print('*** MOTION ENABLED — keep e-stop ready; Ctrl+C stops ***')
        print('Note: WBC/VR may also publish /cmd_vel; keep arms idle / HOLD if needed.')
    else:
        print('Dry-run mode (no /cmd_vel). Add --enable-motion to drive.')

    rclpy.init()
    cam = CameraBuffer(args.color_topic, args.depth_topic, args.cmd_vel_topic)
    try:
        print('Waiting for camera frames...')
        if not cam.wait_for_frames(20.0):
            print('No RGB/depth. Is RealSense + Zenoh running?', file=sys.stderr)
            return 1

        h, w = cam.rgb.shape[:2]
        agent = ThinAgentClient(args.host, args.port, args.model_path, w, h)

        for i in range(args.steps):
            # refresh latest frames
            t0 = time.time()
            while time.time() - t0 < 0.5 and rclpy.ok():
                rclpy.spin_once(cam, timeout_sec=0.05)

            rgb = cam.rgb.copy()
            depth = cam.depth.copy()
            print(f'\n=== step {i + 1}/{args.steps} ===')
            print(f'instruction={args.instruction!r} rgb={rgb.shape}')
            raw = agent.step(rgb, depth, args.instruction)
            code = parse_first_action(raw)
            print(f'action raw={raw} → {code} ({action_name(code)})')

            vx, wz = twist_for_action(code, args.v_lin, args.v_ang)
            if args.enable_motion and (vx != 0.0 or wz != 0.0):
                end = time.time() + args.pulse
                while time.time() < end and rclpy.ok():
                    cam.publish_twist(vx, wz)
                    rclpy.spin_once(cam, timeout_sec=0.05)
                    time.sleep(0.05)
                cam.stop()
            elif args.enable_motion:
                cam.stop()
                print('No base motion for this action.')

            time.sleep(max(0.0, args.period - args.pulse))

        cam.stop()
        print('\nBridge finished.')
        return 0
    except KeyboardInterrupt:
        print('\nInterrupted — stopping.')
        cam.stop()
        return 130
    finally:
        cam.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
