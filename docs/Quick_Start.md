# 快速开始 — 在 ARX Lift2S 上运行 InternNav DualVLN

假定安装已完成（Sim-06 上的 InternNav + DualVLN，机器人上的 ROS / RealSense / `lift2s-ws`）。  
若不同，请将 `192.168.110.35` 替换为你的 Sim-06 局域网 IP。相机序列号示例：`323622271380`。

**启动顺序：** Sim-06 服务 → Zenoh → RealSense → 底盘（`quick_start.sh`）→ bridge / `rqt`。

使用 `--enable-motion` 前请准备好急停。

---

## Sim-06 — 模型服务

```bash
conda activate internnav
cd ~/InternNav
python scripts/eval/start_server.py --host 0.0.0.0 --port 8087
```

保持该终端运行。

可选检查（局域网内任意机器）：

```bash
curl -sS --noproxy '*' -w "\nhttp_code=%{http_code}\n" --connect-timeout 3 \
  http://192.168.110.35:8087/openapi.json | head
```

期望得到 `http_code=200`。

---

## 机器人 — 终端布局

### T1 — Zenoh 路由

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

确认日志中有：`USB type: 3.x` 以及 `RealSense Node Is Up!`  
**不要**同时运行 `realsense-viewer`。

### T3 — 底盘（`lift2s-ws` / `./quick_start.sh`）

```bash
cd ~/lift2s-ws
./quick_start.sh
```

#### InternNav 启动所用菜单选项

使用以下选择（已验证配置：**Lift2S 整机 + 真机 + hybrid 升降**）：

| 步骤 | 提示（摘要） | 选择 |
|------|------------------|--------|
| 1 | 请选择操作 | **`2` 启动 (Launch)** |
| 2 | 请选择启动项 | **`3` 整机 (Lift / Lift2S / X7S)** *（若“最近启动”条目排在前面，按列表中的编号选择**整机**）* |
| 3 | 请选择整机机型 | **`2` Lift2S** |
| 4 | 请选择 Lift2S 控制方式 | **`2` 全身控制 (Full Body)** |
| 5 | 请选择运行模式 | **`1` 真机启动 (Real Hardware)** |
| 6 | 请选择真机升降控制模式 | **`2` hybrid** *（默认 / 回车）* |

等价记录描述：

```text
Lift2S 全身控制 + 真机，臂=full_control，升降=hybrid
```

启动文件：`ocs2_arm_controller full_body.launch.py`，参数 `robot:=arx_lift2s`，`xacro_lift_motor_mode:=hybrid`。

启动后，在 `ros2 topic list` 中应能看到 `/cmd_vel` 与 `/arx_lift/odom`。

> **注意：** 先前运行也可能在“最近启动”中列出**分体控制 (Split Body)**。本文所述 DualVLN `/cmd_vel` 测试请优先使用**全身控制**。保持机械臂空闲 / HOLD，避免 WBC/VR 与 `/cmd_vel` 争用。

### T4 — 查看 RealSense RGB（`rqt`）

```bash
source /opt/ros/jazzy/setup.bash
source ~/ARX5_ROBOT_1-main/realsense/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

在下拉菜单中选择：

```text
/camera_head/color/image_raw
```

### T5 — DualVLN bridge（先干跑，再运动）

如需绕过局域网 HTTP 代理：

```bash
export NO_PROXY="192.168.110.35,localhost,127.0.0.1"
export no_proxy="$NO_PROXY"
unset http_proxy HTTP_PROXY
```

**干跑**（不发布 `/cmd_vel`）：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ARX5_ROBOT_1-main/realsense/install/setup.bash

/usr/bin/python3 ~/internnav_arx_lift2s/scripts/lift2s_internnav_bridge.py \
  --host 192.168.110.35 \
  --instruction "go forward a little bit then stop" \
  --steps 5
```

**运动**（急停就绪）：

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

若脚本位于 `~/InternNav/scripts/`，请改用该路径。

---

## 可选一次性冒烟（无需底盘）

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

## 停止

- Bridge / `rqt`：`Ctrl+C`
- 停止底盘：`ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"`
- 然后按需停止 RealSense、Zenoh、`quick_start` 启动项以及 Sim-06 服务
