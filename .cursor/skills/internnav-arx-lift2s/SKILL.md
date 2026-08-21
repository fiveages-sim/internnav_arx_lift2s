---
name: internnav-arx-lift2s
description: >-
  Implements and debugs InternNav InternVLA-N1 DualVLN on ARX Lift2S with remote
  inference on Sim-06 (HTTP Agent Service :8087), Zenoh ROS 2 camera/chassis on
  the robot, RealSense D405, lift2s-ws /cmd_vel bridge, proxy/NO_PROXY LAN fixes,
  and smoke/motion tests. Use when the user mentions InternNav, DualVLN,
  InternVLA-N1, Lift2S navigation, Sim-06 model server, lift2s_internnav_bridge,
  AgentClient, or bringing up language navigation on ARX Lift2S.
---

# InternNav on ARX Lift2S

## Architecture (do not violate)

```
Robot (Lift2S): RealSense → ROS2/Zenoh → bridge → /cmd_vel
                      │
                      └── HTTP POST :8087 ──► Sim-06: start_server.py + DualVLN GPU
```

| Rule | Detail |
|------|--------|
| Inference host | **Sim-06 only** (`start_server.py --host 0.0.0.0 --port 8087`) |
| Robot GPU | Do **not** run DualVLN server on 8 GB laptop GPU |
| Zenoh | Robot only (`rmw_zenohd`). **No** Zenoh on Sim-06 for this design |
| Robot↔server | **HTTP**, not ROS |
| Camera ownership | ROS launch **xor** `realsense-viewer`, never both |
| Motion safety | Bridge defaults to dry-run; `--enable-motion` only with e-stop |

Full narrative guide: `~/文档/Documentations/InternNav_ARX_Lift2S_Implementation_Guide.md`

## Custom scripts (preserve; not upstream)

| Path | Use |
|------|-----|
| `~/InternNav/scripts/smoke_test_fake_obs.py` | HTTP smoke (fake or `--npz`) |
| `~/InternNav/scripts/capture_realsense_frame.py` | One RGB-D frame → `/tmp/internnav_obs.npz` (**system** Python) |
| `~/InternNav/scripts/lift2s_internnav_bridge.py` | Live cam → DualVLN → optional `/cmd_vel` (**system** Python) |
| `~/InternNav/internnav/utils/__init__.py` | Lazy `AgentServer` so robot can import client without `quaternion` |

If missing after `git clean`/re-clone, recreate from conversation artifacts or the implementation guide—do not tell users upstream InternNav includes Lift2S support.

Import client as:

```python
from internnav.utils.comm_utils.client import AgentClient
```

## Agent workflow checklist

Copy and track:

```
- [ ] Sim-06: DualVLN checkpoint under checkpoints/InternVLA-N1-DualVLN
- [ ] Sim-06: start_server.py listening; curl openapi → 200
- [ ] Robot: NO_PROXY / unset http_proxy for LAN
- [ ] Robot: rmw_zenohd
- [ ] Robot: RealSense USB 3.x + ros2 launch; topic hz ~30
- [ ] Robot: smoke fake then --npz real frame
- [ ] Robot: lift2s-ws quick_start; /cmd_vel present
- [ ] Dry-run bridge → then --enable-motion
```

## Terminal layout

| Where | Command role |
|-------|----------------|
| Sim-06 | `conda activate internnav && cd ~/InternNav && python scripts/eval/start_server.py --host 0.0.0.0 --port 8087` |
| Robot T1 | `source /opt/ros/jazzy/setup.bash && ros2 run rmw_zenoh_cpp rmw_zenohd` |
| Robot T2 | RealSense `rs_launch.py` (serial with leading `_`) |
| Robot T3 | `~/lift2s-ws/./quick_start.sh` |
| Robot T4 | Bridge / smoke / `rqt_image_view` |

Order: **server → Zenoh → camera → chassis → client**.

## LAN / proxy

```bash
export NO_PROXY="192.168.110.35,192.168.110.0/24,192.168.111.0/24,localhost,127.0.0.1"
export no_proxy="$NO_PROXY"
unset http_proxy HTTP_PROXY https_proxy HTTPS_PROXY
curl -sS --noproxy '*' -w "\nhttp_code=%{http_code}\n" --connect-timeout 3 \
  http://192.168.110.35:8087/openapi.json | head
```

Replace IPs with site values. HTTP **502** on LAN ⇒ proxy hijack, not FastAPI.

## Discrete actions

| Code | Meaning | `/cmd_vel` |
|------|---------|------------|
| 0 / -1 | STOP | zeros |
| 1 | FORWARD | `+linear.x` |
| 2 | TURN_LEFT | `+angular.z` |
| 3 | TURN_RIGHT | `-angular.z` |
| 5 | LOOK_DOWN | usually no base |

DualVLN is VLN-style; literal `"turn right"` may still yield FORWARD/STOP. Scale motion with `--v-lin`, `--v-ang`, `--pulse`. Verify chassis with raw `ros2 topic pub` on `/cmd_vel` if needed.

## Python environments

| Task | Interpreter |
|------|-------------|
| `start_server.py`, `smoke_test_fake_obs.py` | `conda activate internnav` |
| `capture_realsense_frame.py`, `lift2s_internnav_bridge.py` | `/usr/bin/python3` after sourcing ROS (rclpy ≠ conda 3.10) |

## When helping the user

1. Confirm architecture (HTTP to Sim-06; Zenoh local).
2. Prefer existing scripts over inventing Go2-only InternNav realworld clients.
3. Dry-run before `--enable-motion`.
4. For “see camera”: `rqt_image_view` on `/camera_head/color/image_raw` while ROS owns device.
5. Object goals need target **large and centered** in RGB; D405 is short-range.
6. Do not recommend Zenoh on Sim-06 or DualVLN server on the robot.

## Additional resources

- Command cookbook and troubleshooting: [reference.md](reference.md)
- Long-form human guide: `~/文档/Documentations/InternNav_ARX_Lift2S_Implementation_Guide.md`
