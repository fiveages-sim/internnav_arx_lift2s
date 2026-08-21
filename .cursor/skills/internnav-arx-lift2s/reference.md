# InternNav × Lift2S — command reference

Example IPs (replace locally): Sim-06 `192.168.110.35`, robot `192.168.111.209`. Camera serial example: `323622271380` → `serial_no:=_323622271380`.

## Sim-06 — env and server

```bash
conda activate internnav
cd ~/InternNav
# Checkpoint: checkpoints/InternVLA-N1-DualVLN
python scripts/eval/start_server.py --host 0.0.0.0 --port 8087
```

Health:

```bash
curl -sS --noproxy '*' -w "\nhttp_code=%{http_code}\n" --connect-timeout 3 \
  http://127.0.0.1:8087/openapi.json | head
```

`flash_attn` must import on Sim-06. Habitat missing is OK for Agent Service only.

## Robot — Zenoh

```bash
source /opt/ros/jazzy/setup.bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

## Robot — RealSense

```bash
lsusb | grep -i 0b5b
lsusb -t | grep -E '5000M|10000M|Video'   # need USB3, not only 480M
rs-enumerate-devices -s

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

Verify:

```bash
ros2 topic hz /camera_head/color/image_raw
ros2 run rqt_image_view rqt_image_view   # pick /camera_head/color/image_raw
```

## Robot — HTTP smoke

```bash
conda activate internnav
cd ~/InternNav
export NO_PROXY="192.168.110.35,localhost,127.0.0.1"; export no_proxy="$NO_PROXY"
unset http_proxy HTTP_PROXY

python scripts/smoke_test_fake_obs.py --host 192.168.110.35

source /opt/ros/jazzy/setup.bash
source ~/ARX5_ROBOT_1-main/realsense/install/setup.bash
/usr/bin/python3 ~/InternNav/scripts/capture_realsense_frame.py --out /tmp/internnav_obs.npz

python scripts/smoke_test_fake_obs.py --host 192.168.110.35 \
  --npz /tmp/internnav_obs.npz \
  --instruction "go forward a little bit then stop"
```

## Robot — chassis

```bash
cd ~/lift2s-ws && ./quick_start.sh
ros2 topic list | grep -E 'cmd_vel|arx_lift/odom|camera_head'
```

Manual turn (e-stop ready):

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.4}}"
# Ctrl+C then:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

## Robot — bridge

Dry-run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ARX5_ROBOT_1-main/realsense/install/setup.bash
/usr/bin/python3 ~/InternNav/scripts/lift2s_internnav_bridge.py \
  --host 192.168.110.35 \
  --instruction "go forward a little bit then stop" \
  --steps 5
```

Motion:

```bash
/usr/bin/python3 ~/InternNav/scripts/lift2s_internnav_bridge.py \
  --host 192.168.110.35 \
  --instruction "go to the black tumbler then stop" \
  --steps 8 \
  --enable-motion \
  --v-lin 0.12 \
  --v-ang 0.4 \
  --pulse 1.2 \
  --period 2.5
```

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `http_code=502` / empty curl | `--noproxy '*'`, `NO_PROXY`, unset proxy env |
| Zenoh router warning on camera | Start `rmw_zenohd` before camera |
| No camera topics | Launch down / USB unplugged / Viewer holding device |
| Frames timeout | USB 3 port; quit Viewer |
| Viewer blank in IDE terminal | NoMachine + `DISPLAY=:0` or desktop terminal |
| `quaternion` ImportError | Lazy utils init; import `comm_utils.client` only |
| Smoke script missing | Restore custom scripts (not in upstream) |
| Always STOP | Instruction/view mismatch; center object in `rqt` |
| Small motion | Raise `--v-lin` / `--v-ang` / `--pulse` |
| cmd_vel no motion | `chassis_mode=1`; stop VR/WBC fighting `/cmd_vel` |

## Install notes (high level)

- Conda env `internnav`, Python 3.10, on robot and Sim-06.
- Sim-06: Torch CUDA build matching GPU; DualVLN checkpoint; working `flash_attn`.
- Robot: thin client deps; do not host DualVLN.
- Proxy only for clone/pip/HF; bypass for LAN Agent Service.
