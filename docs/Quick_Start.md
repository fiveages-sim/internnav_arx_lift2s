# Quick Start — InternNav DualVLN on ARX Lift2S

Assumes installation is complete (InternNav + DualVLN on Sim-06, ROS / RealSense / `lift2s-ws` on the robot).  
Replace `192.168.110.35` with your Sim-06 LAN IP if different. Camera serial example: `323622271380`.

**Startup order:** Sim-06 server → Zenoh → RealSense → chassis (`quick_start.sh`) → bridge / `rqt`.

Keep an e-stop ready before `--enable-motion`.

---

## Sim-06 — model server

```bash
conda activate internnav
cd ~/InternNav
python scripts/eval/start_server.py --host 0.0.0.0 --port 8087
```

Leave this terminal running.

Optional check (any machine on the LAN):

```bash
curl -sS --noproxy '*' -w "\nhttp_code=%{http_code}\n" --connect-timeout 3 \
  http://192.168.110.35:8087/openapi.json | head
```

Expect `http_code=200`.

---

## Robot — terminal layout

### T1 — Zenoh router

```bash
source /opt/ros/jazzy/setup.bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

### T2 — RealSense D405

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

Confirm log: `USB type: 3.x` and `RealSense Node Is Up!`  
Do **not** run `realsense-viewer` at the same time.

### T3 — Chassis (`lift2s-ws` / `./quick_start.sh`)

```bash
cd ~/lift2s-ws
./quick_start.sh
```

#### Menu choices used for InternNav bring-up

Use these selections (validated config: **Lift2S full body + real hardware + hybrid lift**):

| Step | Prompt (summary) | Choose |
|------|------------------|--------|
| 1 | 请选择操作 | **`2` 启动 (Launch)** |
| 2 | 请选择启动项 | **`3` 整机 (Lift / Lift2S / X7S)** *(if “最近启动” entries appear first, pick **整机** by its listed number)* |
| 3 | 请选择整机机型 | **`2` Lift2S** |
| 4 | 请选择 Lift2S 控制方式 | **`2` 全身控制 (Full Body)** |
| 5 | 请选择运行模式 | **`1` 真机启动 (Real Hardware)** |
| 6 | 请选择真机升降控制模式 | **`2` hybrid** *(default / Enter)* |

Equivalent recorded description:

```text
Lift2S 全身控制 + 真机，臂=full_control，升降=hybrid
```

Launch file: `ocs2_arm_controller full_body.launch.py` with `robot:=arx_lift2s`, `xacro_lift_motor_mode:=hybrid`.

After launch, you should see `/cmd_vel` and `/arx_lift/odom` in `ros2 topic list`.

> **Note:** A prior run may also list **分体控制 (Split Body)** as a recent option. Prefer **全身控制** for the DualVLN `/cmd_vel` tests described here. Keep arms idle / HOLD so WBC/VR do not fight `/cmd_vel`.

### T4 — View RealSense RGB (`rqt`)

```bash
source /opt/ros/jazzy/setup.bash
source ~/ARX5_ROBOT_1-main/realsense/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

In the dropdown, select:

```text
/camera_head/color/image_raw
```

### T5 — DualVLN bridge (dry-run, then motion)

Bypass HTTP proxy for LAN if needed:

```bash
export NO_PROXY="192.168.110.35,localhost,127.0.0.1"
export no_proxy="$NO_PROXY"
unset http_proxy HTTP_PROXY
```

**Dry-run** (no `/cmd_vel`):

```bash
source /opt/ros/jazzy/setup.bash
source ~/ARX5_ROBOT_1-main/realsense/install/setup.bash

/usr/bin/python3 ~/internnav_arx_lift2s/scripts/lift2s_internnav_bridge.py \
  --host 192.168.110.35 \
  --instruction "go forward a little bit then stop" \
  --steps 5
```

**Motion** (e-stop ready):

```bash
/usr/bin/python3 ~/internnav_arx_lift2s/scripts/lift2s_internnav_bridge.py \
  --host 192.168.110.35 \
  --instruction "go forward a little bit then stop" \
  --steps 5 \
  --enable-motion \
  --v-lin 0.15 \
  --v-ang 0.5 \
  --pulse 1.5 \
  --period 2.5
```

If scripts live under `~/InternNav/scripts/` instead, substitute that path.

---

## Optional one-shot smoke (no chassis)

```bash
source /opt/ros/jazzy/setup.bash
source ~/ARX5_ROBOT_1-main/realsense/install/setup.bash
/usr/bin/python3 ~/internnav_arx_lift2s/scripts/capture_realsense_frame.py \
  --out /tmp/internnav_obs.npz

conda activate internnav
cd ~/InternNav
python ~/internnav_arx_lift2s/scripts/smoke_test_fake_obs.py \
  --host 192.168.110.35 \
  --npz /tmp/internnav_obs.npz \
  --instruction "go forward a little bit then stop"
```

---

## Stop

- Bridge / `rqt`: `Ctrl+C`
- Stop base: `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"`
- Then stop RealSense, Zenoh, `quick_start` launch, and Sim-06 server as needed
