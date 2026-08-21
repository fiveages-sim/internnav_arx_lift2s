# InternNav ↔ ARX Lift2S

Integration scripts and docs to run InternNav DualVLN on ARX Lift2S.

- Robot: ROS 2 Jazzy + Zenoh + RealSense D405 + `open-deploy-ws` (`arx-lift2s`)
- Inference: InternNav Agent Server on a GPU PC (e.g. Sim-06), HTTP `:8087`

This repository is **not** a full InternNav fork. Clone upstream InternNav for the model server:

https://github.com/InternRobotics/InternNav

Robot workspace (chassis / control):

https://github.com/fiveages-sim/open-deploy-ws (branch `arx-lift2s`)

## Contents

| Path | Description |
|------|-------------|
| `scripts/smoke_test_fake_obs.py` | HTTP smoke test (fake or `.npz` obs) |
| `scripts/capture_realsense_frame.py` | Capture one RGB-D frame from ROS |
| `scripts/lift2s_internnav_bridge.py` | Camera → DualVLN → optional `/cmd_vel` |
| `docs/` | Implementation + architecture guides |
| `patches/internnav_utils_init.py` | Lazy `AgentClient` import helper for robot |

## Quick start

See `docs/InternNav_ARX_Lift2S_Implementation_Guide.md`.
