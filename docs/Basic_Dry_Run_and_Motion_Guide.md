# 基础干跑与运动指南 — Sim-06 ↔ ARX Lift2S

从**连接 Sim-06 与 Lift2S** 到使用 InternNav DualVLN 进行**干跑**与**运动**的分步指令脚本。

**前置条件：** 安装已完成（[Installation_Guide.md](./Installation_Guide.md)）。  
**架构：** [InternNav_ARX_Lift2S_Architecture.md](./InternNav_ARX_Lift2S_Architecture.md)。

| 机器 | 示例 IP | SSH / 登录 |
|---------|------------|-------------|
| **Sim-06**（模型服务） | `192.168.110.35` | `fiveages@Sim-06` |
| **Lift2S**（机器人） | `192.168.111.209` | `arx@192.168.111.209` |

若局域网不同请替换 IP。确认 Sim-06：`ip -4 addr show enp4s0`。

**相机：** Intel RealSense D405（序列号示例 `323622271380`）。  
**Bridge 脚本：** `~/internnav_arx_lift2s/scripts/lift2s_internnav_bridge.py`

---

## 启动顺序（务必遵守）

```text
Sim-06 model server  →  Lift2S Zenoh  →  RealSense  →  [optional rqt]  →  dry-run
                                                              ↓
                                                    quick_start (chassis)  →  motion
```

| 阶段 | 是否需要底盘（`quick_start.sh`）？ |
|-------|-----------------------------------|
| 干跑 | **否**（仅打印动作） |
| 运动（`--enable-motion`） | **是**（发布 `/cmd_vel`） |

运动前请准备好**急停**。底盘测试期间机械臂保持 **HOLD** / 空闲。

---

## 0. 相机升降（导航前）

Lift2S 上的 D405 朝向**下方**。做语言导航时，通过 `quick_start.sh` 的 hybrid 模式抬高 **AC_ONE** 升降，使相机看到更多场景（墙面、物体），而非只有地面。

干跑或运动前，在 `rqt` 中确认目标物体可见（第 4 节）。

---

## 1. Sim-06 — DualVLN 模型服务

**设备：** Sim-06  
**终端：** `Sim-06-A`（保持打开）

```bash
conda activate internnav
cd ~/InternNav
python scripts/eval/start_server.py --host 0.0.0.0 --port 8087
```

**重要**

- Lift2S 使用端口 **8087**（bridge 默认）。
- **不要**传入 `--config scripts/eval/configs/h1_internvla_n1_async_cfg.py`，除非同时将 bridge 改为端口 **8023** — 该 H1 评估配置会把端口覆盖为 8023。

**期望**

```text
INFO: Uvicorn running on http://0.0.0.0:8087
```

**健康检查**（Sim-06 或局域网内任意机器）：

```bash
curl -sS --noproxy '*' -w "\nhttp_code=%{http_code}\n" --connect-timeout 3 \
  http://192.168.110.35:8087/openapi.json | head -c 80
```

期望 `http_code=200`。`GET /` 返回 **404** 属正常。

机器人首次 `POST /agent/init` 会加载约 16 GB 的 checkpoint（可能需 1–2 分钟）。

---

## 2. Lift2S — SSH 与 Zenoh

**设备：** Lift2S  
**终端：** `Lift2S-T1`（保持打开）

```bash
ssh arx@192.168.111.209
source /opt/ros/jazzy/setup.bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

**原因：** 本机 ROS 2 使用 Zenoh RMW；相机与 bridge 需先有路由。

---

## 3. Lift2S — RealSense D405

**设备：** Lift2S  
**终端：** `Lift2S-T2`（保持打开）

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

若不同，请将 `serial_no` 替换为你的 D405 序列号。

**期望：** `USB type: 3.x` 与 `RealSense Node Is Up!`  
该节点运行时**不要**再运行 `realsense-viewer`。

**可选 — 验证帧率**（Lift2S 新开标签页）：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ARX5_ROBOT_1-main/realsense/install/setup.bash
ros2 topic hz /camera_head/color/image_raw
```

期望约 30 Hz。Ctrl+C 停止。

---

## 4. Lift2S — 查看相机（`rqt_image_view`）

**设备：** Lift2S  
**终端：** `Lift2S-T3`（可选；查看完毕可关闭）

**原因：** 在运行 bridge 前，确认 DualVLN 实际收到的 RGB。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ARX5_ROBOT_1-main/realsense/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

在下拉菜单中选择：

```text
/camera_head/color/image_raw
```

可选深度检查：

```text
/camera_head/depth/image_raw
```

**经 SSH 使用 GUI：** 若窗口未出现，请从带有 X server 的机器 SSH：

```bash
ssh -X arx@192.168.111.209
```

然后运行上述 `rqt` 命令。

---

## 5. Lift2S — 干跑（无底盘运动）

**设备：** Lift2S  
**终端：** `Lift2S-T4`  
**需要：** Sim-06-A、Lift2S-T1、Lift2S-T2 已运行。**不需要** `quick_start.sh`。

**原因：** 在移动底盘**之前**确认 HTTP 链路、相机 RGB-D 与模型动作。

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

**Lift2S 上期望**

```text
Dry-run mode (no /cmd_vel). Add --enable-motion to drive.
Agent init OK: internvla_n1
action raw=... → 1 (FORWARD)
```

**Sim-06-A 上期望：** `POST /agent/init`（201）与 `POST /agent/internvla_n1/step`（200）。

**说明**

- 干跑**不会移动**机器人；连续步可能出现不同转向（`TURN_LEFT` 与 `TURN_RIGHT`），因为内部 agent 历史在无运动时仍会推进。**运动跑**才是真正测试。
- 良好干跑：`FORWARD`、`TURN_LEFT`、`TURN_RIGHT` 混合出现（而非单一动作或长时间 `STOP`）。

---

## 6. Lift2S — 底盘（`quick_start.sh`）

**设备：** Lift2S  
**终端：** `Lift2S-T5`（运动期间保持打开）

**原因：** 运动会发布 `/cmd_vel`；底盘栈必须在监听。

```bash
cd ~/lift2s-ws
./quick_start.sh
```

### 菜单选择（已验证）

| 步骤 | 提示（摘要） | 选择 |
|------|------------------|--------|
| 1 | 请选择操作 | **`2` 启动 (Launch)** |
| 2 | 请选择启动项 | **`3` 整机 (Lift / Lift2S / X7S)** |
| 3 | 请选择整机机型 | **`2` Lift2S** |
| 4 | 请选择 Lift2S 控制方式 | **`2` 全身控制 (Full Body)** |
| 5 | 请选择运行模式 | **`1` 真机启动 (Real Hardware)** |
| 6 | 请选择真机升降控制模式 | **`2` hybrid**（回车） |

```text
Lift2S 全身控制 + 真机，臂=full_control，升降=hybrid
```

启动后验证（新标签页）：

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep -E 'cmd_vel|arx_lift/odom'
```

保持机械臂 **HOLD**，避免 WBC 与 `/cmd_vel` 争用。

---

## 7. Lift2S — 运动跑

**设备：** Lift2S  
**终端：** `Lift2S-T4`（复用 bridge 终端）  
**需要：** Sim-06-A、Lift2S-T1、T2、**T5（`quick_start`）**

**原因：** 闭环导航 — DualVLN 动作驱动底盘。

清理地面空间。急停就绪。目标物体在相机视野内可见（不确定时用 `rqt`）。

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

### 已验证基线（黑色垃圾桶演示）

| 参数 | 值 | 说明 |
|-----------|-------|--------|
| Instruction | `walk forward and find the black dustbin on the floor` | VLN 风格目标 |
| `--steps` | `80` | 短测用 `50`；若机器人过早停止用 `80` |
| `--v-lin` | `0.20` | 前进速度（m/s） |
| `--v-ang` | `0.4` | 转向速率（rad/s） |
| `--pulse` | `2.0` | 每条运动指令持续秒数 |
| `--period` | `3.0` | 推理步间隔秒数 |

**期望：** `*** MOTION ENABLED ***`，底盘出现转向与前进脉冲；接近目标（约 0.5 m）时往往以大量 `STOP (0)` 结束。

**动作编码**

| 编码 | 含义 |
|------|---------|
| `0` | STOP |
| `1` | FORWARD |
| `2` | TURN_LEFT |
| `3` | TURN_RIGHT |
| `5` | LOOK_DOWN（bridge 中无底盘运动） |
| `-1` | STOP / 空操作 |

---

## 8. 运动后停止底盘

**设备：** Lift2S

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

或对 bridge 按 Ctrl+C（退出时也会发送停止）。

---

## 9. 关机顺序

| 顺序 | 终端 | 操作 |
|-------|----------|--------|
| 1 | Lift2S bridge | Ctrl+C |
| 2 | Lift2S | 清零 `/cmd_vel`（上节命令） |
| 3 | Lift2S-T5 | Ctrl+C `quick_start` |
| 4 | Lift2S-T3 | Ctrl+C `rqt`（若开着） |
| 5 | Lift2S-T2 | Ctrl+C RealSense |
| 6 | Lift2S-T1 | Ctrl+C Zenoh |
| 7 | Sim-06-A | Ctrl+C 模型服务 |

---

## 10. 终端对照表（速查）

| ID | 设备 | 服务 |
|----|--------|---------|
| Sim-06-A | Sim-06 | `start_server.py --host 0.0.0.0 --port 8087` |
| Lift2S-T1 | Lift2S | `rmw_zenohd` |
| Lift2S-T2 | Lift2S | RealSense launch |
| Lift2S-T3 | Lift2S | `rqt_image_view`（可选） |
| Lift2S-T4 | Lift2S | `lift2s_internnav_bridge.py` |
| Lift2S-T5 | Lift2S | `quick_start.sh`（仅运动） |

---

## 11. 故障排查

| 现象 | 检查 |
|---------|--------|
| 连 `:8087` 出现 `Connection refused` | Sim-06-A 在跑？是否绑在 `:8023`？ |
| `No RGB/depth` | T1 Zenoh + T2 RealSense 在跑？ |
| `Agent init` 卡住 | 等待 Sim-06 首次模型加载 1–2 分钟 |
| 底盘不动 | T5 `quick_start` 在跑？是否用了 `--enable-motion`？ |
| 只原地转、到不了目标 | 抬高 AC_ONE 升降；把目标放进相机视野；增大 `--steps` |
| 结束时长 `STOP` | 接近目标时正常 — 模型结束 episode |
| 服务端 `GET /` 404 | 无害 — 使用 `/openapi.json` 或 `/agent/init` |

---

## 相关文档

- [Quick_Start.md](./Quick_Start.md) — 更短的参考
- [Installation_Guide.md](./Installation_Guide.md) — 首次安装
- [InternNav_ARX_Lift2S_Architecture.md](./InternNav_ARX_Lift2S_Architecture.md) — 流水线如何工作
