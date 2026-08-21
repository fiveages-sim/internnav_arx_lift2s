#!/usr/bin/env python3
"""Smoke-test InternNav AgentServer with a fake or real RGB-D observation.

Run on Sim-06 while start_server.py is listening on port 8087:

  conda activate internnav
  cd ~/InternNav
  python scripts/smoke_test_fake_obs.py --host localhost

Or from the Lift2S robot (server on Sim-06):

  python scripts/smoke_test_fake_obs.py --host 192.168.110.35
  python scripts/smoke_test_fake_obs.py --host 192.168.110.35 --npz /tmp/internnav_obs.npz
"""

from __future__ import annotations

import argparse

import numpy as np

from internnav.configs.agent import AgentCfg
# Import client only — avoid loading AgentServer / full agent deps on the robot.
from internnav.utils.comm_utils.client import AgentClient


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='localhost', help='Sim-06 LAN IP, or localhost if on Sim-06')
    parser.add_argument('--port', type=int, default=8087)
    parser.add_argument(
        '--model-path',
        default='checkpoints/InternVLA-N1-DualVLN',
        help='Path relative to InternNav root on the SERVER machine',
    )
    parser.add_argument(
        '--instruction',
        default='go forward a little bit then stop',
        help='Language command for DualVLN',
    )
    parser.add_argument(
        '--npz',
        default='',
        help='Optional .npz from capture_realsense_frame.py (keys: rgb, depth)',
    )
    args = parser.parse_args()

    if args.npz:
        data = np.load(args.npz)
        rgb = data['rgb']
        depth = data['depth'].astype(np.float32)
        h, w = int(rgb.shape[0]), int(rgb.shape[1])
        print(f'Loaded real frame from {args.npz}: rgb={rgb.shape} depth={depth.shape}')
    else:
        h, w = 480, 640
        rgb = np.zeros((h, w, 3), dtype=np.uint8)
        rgb[:, :, 1] = 180
        depth = np.ones((h, w), dtype=np.float32) * 1.5

    cfg = AgentCfg(
        server_host=args.host,
        server_port=args.port,
        model_name='internvla_n1',
        ckpt_path='',
        model_settings={
            'policy_name': 'InternVLAN1_Policy',
            'state_encoder': None,
            'env_num': 1,
            'sim_num': 1,
            'model_path': args.model_path,
            'camera_intrinsic': [
                [396.23, 0.0, 323.22],
                [0.0, 395.82, 246.21],
                [0.0, 0.0, 1.0],
            ],
            'width': w,
            'height': h,
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
    )

    print(f'Connecting to http://{args.host}:{args.port} ...')
    agent = AgentClient(cfg)
    print('Agent init OK:', agent)

    obs = [
        {
            'rgb': rgb,
            'depth': depth,
            'instruction': args.instruction,
        }
    ]
    print('Calling agent.step with fake observation (may take tens of seconds)...')
    action = agent.step(obs)
    print('Action returned:', action)
    print('Smoke test finished.')


if __name__ == '__main__':
    main()
