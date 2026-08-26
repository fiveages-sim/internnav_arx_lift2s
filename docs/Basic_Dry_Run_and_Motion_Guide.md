# Basic Dry-Run and Motion Guide — Sim-06 ↔ ARX Lift2S

Step-by-step command script from **linking Sim-06 to Lift2S** through **dry-run** and **motion** with InternNav DualVLN.

**Prerequisites:** Installation complete ([Installation_Guide.md](./Installation_Guide.md)).  
**Architecture:** [InternNav_ARX_Lift2S_Architecture.md](./InternNav_ARX_Lift2S_Architecture.md).

| Machine | Example IP | SSH / login |
|---------|------------|-------------|
| **Sim-06** (model server) | `192.168.110.35` | `fiveages@Sim-06` |
| **Lift2S** (robot) | `192.168.111.209` | `arx@192.168.111.209` |

Replace IPs if your LAN differs. Confirm Sim-06: `ip -4 addr show enp4s0`.

**Camera:** Intel RealSense D405 (example serial `323622271380`).  
**Bridge script:** `~/internnav_arx_lift2s/scripts/lift2s_internnav_bridge.py`

---

## Startup order (always follow this)

```text
Sim-06 model server  →  Lift2S Zenoh  →  RealSense  →  [optional rqt]  →  dry-run
                                                              ↓
                                                    quick_start (chassis)  →  motion
```

| Phase | Needs chassis (`quick_start.sh`)? |
|-------|-----------------------------------|
| Dry-run | **No** (prints actions only) |
| Motion (`--enable-motion`) | **Yes** (publishes `/cmd_vel`) |

Keep an **e-stop** ready before motion. Arms **HOLD** / idle during base tests.

---

## 0. Camera lift (before navigation)

The D405 on Lift2S points **downward**. For language navigation, raise **AC_ONE** lift (hybrid mode via `quick_start.sh`) so the camera sees more scene (walls, objects), not only floor.

Before dry-run or motion, confirm the goal object is visible in `rqt` (Section 4).

---

## 1. Sim-06 — DualVLN model server

**Device:** Sim-06  
**Terminal:** `Sim-06-A` (leave open)

```bash
conda activate internnav
cd ~/InternNav
python scripts/eval/start_server.py --host 0.0.0.0 --port 8087
```

**Important**

- Use port **8087** for Lift2S (bridge default).
- Do **not** pass `--config scripts/eval/configs/h1_internvla_n1_async_cfg.py` unless you also change the bridge to port **8023** — that H1 eval config overrides the port to 8023.

**Expect**

```text
INFO: Uvicorn running on http://0.0.0.0:8087
```

**Health check** (Sim-06 or any LAN machine):

```bash
curl -sS --noproxy '*' -w "\nhttp_code=%{http_code}\n" --connect-timeout 3 \
  http://192.168.110.35:8087/openapi.json | head -c 80
```

Expect `http_code=200`. A `GET /` returning **404** is normal.

First `POST /agent/init` from the robot loads the ~16 GB checkpoint (can take 1–2 minutes).

---

## 2. Lift2S — SSH and Zenoh

**Device:** Lift2S  
**Terminal:** `Lift2S-T1` (leave open)

```bash
ssh arx@192.168.111.209
source /opt/ros/jazzy/setup.bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

**Why:** ROS 2 on this robot uses Zenoh RMW; camera and bridge need the router first.

---

## 3. Lift2S — RealSense D405

**Device:** Lift2S  
**Terminal:** `Lift2S-T2` (leave open)

```bash
cd ~/ARX5_ROBOT_1-main/realsense
source /opt/ros/jazzy/setup.bash
source ./install/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=/ \
  camera_name:=camera_head \
  serial_no:=_323622271380 \
  depth_module.color_profile:=640x480x30 \
  depth_module.depth_profile:=640x480x30
```

Replace `serial_no` with your D405 serial if different.

**Expect:** `USB type: 3.x` and `RealSense Node Is Up!`  
Do **not** run `realsense-viewer` while this node is running.

**Optional — verify frame rate** (new tab on Lift2S):

```bash
source /opt/ros/jazzy/setup.bash
source ~/ARX5_ROBOT_1-main/realsense/install/setup.bash
ros2 topic hz /camera_head/color/image_raw
```

Expect ~30 Hz. Ctrl+C to stop.

---

## 4. Lift2S — View camera (`rqt_image_view`)

**Device:** Lift2S  
**Terminal:** `Lift2S-T3` (optional; close when done viewing)

**Why:** See exactly what DualVLN receives as RGB before running the bridge.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ARX5_ROBOT_1-main/realsense/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

In the dropdown, select:

```text
/camera_head/color/image_raw
```

Optional depth check:

```text
/camera_head/depth/image_raw
```

**GUI over SSH:** If the window does not appear, SSH from a machine with an X server:

```bash
ssh -X arx@192.168.111.209
```

Then run the `rqt` commands above.

---

## 5. Lift2S — Dry-run (no base motion)

**Device:** Lift2S  
**Terminal:** `Lift2S-T4`  
**Requires:** Sim-06-A, Lift2S-T1, Lift2S-T2 running. **Does not** require `quick_start.sh`.

**Why:** Confirms HTTP link, camera RGB-D, and model actions **before** moving the base.

```bash
export NO_PROXY="192.168.110.35,localhost,127.0.0.1"
export no_proxy="$NO_PROXY"
unset http_proxy HTTP_PROXY https_proxy HTTPS_PROXY ALL_PROXY

source /opt/ros/jazzy/setup.bash
source ~/ARX5_ROBOT_1-main/realsense/install/setup.bash

/usr/bin/python3 ~/internnav_arx_lift2s/scripts/lift2s_internnav_bridge.py \
  --host 192.168.110.35 \
  --instruction "walk forward and find the black dustbin on the floor" \
  --steps 10
```

**Expect on Lift2S**

```text
Dry-run mode (no /cmd_vel). Add --enable-motion to drive.
Agent init OK: internvla_n1
action raw=... → 1 (FORWARD)
```

**Expect on Sim-06-A:** `POST /agent/init` (201) and `POST /agent/internvla_n1/step` (200).

**Notes**

- Dry-run **does not move** the robot; consecutive steps may show different turns (`TURN_LEFT` vs `TURN_RIGHT`) because internal agent history advances without motion. **Motion run** is the real test.
- Good dry-run: mix of `FORWARD`, `TURN_LEFT`, `TURN_RIGHT` (not only one action or long `STOP` streaks).

---

## 6. Lift2S — Chassis (`quick_start.sh`)

**Device:** Lift2S  
**Terminal:** `Lift2S-T5` (leave open during motion)

**Why:** Motion publishes `/cmd_vel`; the chassis stack must be listening.

```bash
cd ~/lift2s-ws
./quick_start.sh
```

### Menu choices (validated)

| Step | Prompt (summary) | Choose |
|------|------------------|--------|
| 1 | 请选择操作 | **`2` 启动 (Launch)** |
| 2 | 请选择启动项 | **`3` 整机 (Lift / Lift2S / X7S)** |
| 3 | 请选择整机机型 | **`2` Lift2S** |
| 4 | 请选择 Lift2S 控制方式 | **`2` 全身控制 (Full Body)** |
| 5 | 请选择运行模式 | **`1` 真机启动 (Real Hardware)** |
| 6 | 请选择真机升降控制模式 | **`2` hybrid** (Enter) |

```text
Lift2S 全身控制 + 真机，臂=full_control，升降=hybrid
```

After launch, verify (new tab):

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep -E 'cmd_vel|arx_lift/odom'
```

Keep arms **HOLD** so WBC does not fight `/cmd_vel`.

---

## 7. Lift2S — Motion run

**Device:** Lift2S  
**Terminal:** `Lift2S-T4` (reuse bridge terminal)  
**Requires:** Sim-06-A, Lift2S-T1, T2, **T5 (`quick_start`)**

**Why:** Closed-loop navigation — DualVLN actions drive the base.

Clear floor space. E-stop ready. Goal object visible in camera (use `rqt` if unsure).

```bash
export NO_PROXY="192.168.110.35,localhost,127.0.0.1"
export no_proxy="$NO_PROXY"
unset http_proxy HTTP_PROXY https_proxy HTTPS_PROXY ALL_PROXY

source /opt/ros/jazzy/setup.bash
source ~/ARX5_ROBOT_1-main/realsense/install/setup.bash
source ~/lift2s-ws/install/setup.bash

/usr/bin/python3 ~/internnav_arx_lift2s/scripts/lift2s_internnav_bridge.py \
  --host 192.168.110.35 \
  --instruction "walk forward and find the black dustbin on the floor" \
  --steps 80 \
  --enable-motion \
  --v-lin 0.20 \
  --v-ang 0.4 \
  --pulse 2.0 \
  --period 3.0
```

### Validated baseline (black dustbin demo)

| Parameter | Value | Notes |
|-----------|-------|--------|
| Instruction | `walk forward and find the black dustbin on the floor` | VLN-style goal |
| `--steps` | `80` | Use `50` for short tests; `80` if robot stops early |
| `--v-lin` | `0.20` | Forward speed (m/s) |
| `--v-ang` | `0.4` | Turn rate (rad/s) |
| `--pulse` | `2.0` | Seconds per motion command |
| `--period` | `3.0` | Seconds between inference steps |

**Expect:** `*** MOTION ENABLED ***`, base turns and forward pulses; often ends with many `STOP (0)` when near the goal (~0.5 m).

**Action codes**

| Code | Meaning |
|------|---------|
| `0` | STOP |
| `1` | FORWARD |
| `2` | TURN_LEFT |
| `3` | TURN_RIGHT |
| `5` | LOOK_DOWN (no base motion in bridge) |
| `-1` | STOP / no-op |

---

## 8. Stop the base after motion

**Device:** Lift2S

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

Or Ctrl+C the bridge (it also sends stop on exit).

---

## 9. Shutdown order

| Order | Terminal | Action |
|-------|----------|--------|
| 1 | Lift2S bridge | Ctrl+C |
| 2 | Lift2S | Zero `/cmd_vel` (command above) |
| 3 | Lift2S-T5 | Ctrl+C `quick_start` |
| 4 | Lift2S-T3 | Ctrl+C `rqt` (if open) |
| 5 | Lift2S-T2 | Ctrl+C RealSense |
| 6 | Lift2S-T1 | Ctrl+C Zenoh |
| 7 | Sim-06-A | Ctrl+C model server |

---

## 10. Terminal map (quick reference)

| ID | Device | Service |
|----|--------|---------|
| Sim-06-A | Sim-06 | `start_server.py --host 0.0.0.0 --port 8087` |
| Lift2S-T1 | Lift2S | `rmw_zenohd` |
| Lift2S-T2 | Lift2S | RealSense launch |
| Lift2S-T3 | Lift2S | `rqt_image_view` (optional) |
| Lift2S-T4 | Lift2S | `lift2s_internnav_bridge.py` |
| Lift2S-T5 | Lift2S | `quick_start.sh` (motion only) |

---

## 11. Troubleshooting

| Symptom | Check |
|---------|--------|
| `Connection refused` to `:8087` | Sim-06-A running? Not bound to `:8023`? |
| `No RGB/depth` | T1 Zenoh + T2 RealSense running? |
| `Agent init` hangs | Wait 1–2 min for first model load on Sim-06 |
| Base does not move | T5 `quick_start` running? Used `--enable-motion`? |
| Only spins, never reaches goal | Raise AC_ONE lift; place goal in camera view; increase `--steps` |
| Long `STOP` at end | Normal near goal — model ends episode |
| `GET /` 404 on server | Harmless — use `/openapi.json` or `/agent/init` |

---

## Related docs

- [Quick_Start.md](./Quick_Start.md) — shorter reference
- [Installation_Guide.md](./Installation_Guide.md) — first-time setup
- [InternNav_ARX_Lift2S_Architecture.md](./InternNav_ARX_Lift2S_Architecture.md) — how the pipeline works
