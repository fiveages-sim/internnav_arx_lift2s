# 在 ARX Lift2S 上部署 InternNav（InternVLA-N1 DualVLN）

**文档目的。** 分步说明如何在 **ARX Lift2S** 机器人上部署 [InternNav](https://github.com/OpenRobotLab/InternNav) 语言条件导航：重推理在远程 GPU 工作站（**zy**）上完成，感知与底盘控制在机器人端完成。

**实验室规定。** DualVLN 在 **zy** 上使用 **[uv](https://docs.astral.sh/uv/)**，**不要**再用 conda。

**读者对象。** 在 Lift2S 上调试 DualVLN 的工程师（ROS 2 Jazzy，Ubuntu 24.04）。

**本指南范围。** 从仓库克隆与依赖安装，到模型服务、相机启动、HTTP 客户端冒烟测试，以及闭环 `/cmd_vel` bridge。**不**涵盖机械臂抓取，也不涵盖量产导航安全认证。

**硬件参考（已验证配置）。**

| 角色 | 机器 | 说明 |
|------|---------|--------|
| 机器人 | ARX Lift2S 主机（`arx`） | ROS 2 Jazzy、Zenoh RMW、Intel RealSense D405、底盘经 `lift2s-ws` |
| 模型服务 | zy（`zy GPU host`） | NVIDIA RTX 5090 级 GPU、DualVLN checkpoint |
| 机器人 GPU | RTX 5070 Laptop（8 GB） | DualVLN **显存不足**；请勿在此运行模型服务 |

**典型局域网地址（请按现场替换）。**

| 主机 | 示例 IP |
|------|------------|
| zy | `192.168.110.35` |
| Lift2S 机器人 | `192.168.111.209` |

---

## 1. 流水线如何工作

### 1.1 架构概览

InternNav 的 DualVLN agent 接收 RGB（及 depth）观测与自然语言指令，返回 **离散导航动作**（Habitat 风格）。官方真机演示面向 Unitree Go2 等平台。Lift2S **不是**即插即用目标：保留 InternNav 的 **Agent Server** 做推理，并增加一层 **薄机器人客户端**，将动作映射到 ROS 2 `/cmd_vel`。

```
┌─────────────────────────────────────┐         HTTP :8087          ┌─────────────────────────────────────┐
│  ARX Lift2S (robot)                 │ ──────────────────────────► │  zy (model server)               │
│                                     │                             │                                     │
│  RealSense D405                     │     POST /agent/init        │  uv .venv (InternNav)               │
│       │                             │     POST /agent/.../step    │  start_server.py                    │
│       ▼                             │     (pickle+base64 obs)     │  DualVLN weights on GPU             │
│  realsense2_camera (ROS 2)          │ ◄────────────────────────── │  returns discrete actions           │
│       │                             │         JSON action         │                                     │
│       ▼                             │                             └─────────────────────────────────────┘
│  Zenoh router (rmw_zenohd)          │
│       │  local ROS graph only       │
│       ▼                             │
│  lift2s_internnav_bridge.py         │
│       │                             │
│       ▼                             │
│  /cmd_vel → Lift2S chassis HI       │
└─────────────────────────────────────┘
```

### 1.2 哪些走 ROS 2，哪些走 HTTP

| 路径 | 传输 | 用途 |
|------|-----------|---------|
| 相机 → bridge | **ROS 2**（Zenoh RMW），仅在**机器人本机** | `/camera_head/...` 下的图像话题 |
| 底盘里程计 / `/cmd_vel` | **ROS 2**，仅在**机器人本机** | 驱动底盘 |
| 机器人 ↔ DualVLN | **HTTP** 到 zy `:8087` | 加载 agent + 执行 `step` |

**重要。** 该设计下 **不要** 在 zy 上运行 Zenoh 路由。zy 不加入机器人的 ROS 图。推理是远程 HTTP，类似微服务。

### 1.3 离散动作

DualVLN 返回 Habitat 风格编码（与 InternNav 真机客户端约定一致）：

| 编码 | 含义 | 典型 `/cmd_vel` 映射 |
|------|---------|----------------------------|
| `0` | STOP | 全零 |
| `1` | FORWARD | `linear.x > 0` |
| `2` | TURN_LEFT | `angular.z > 0` |
| `3` | TURN_RIGHT | `angular.z < 0` |
| `5` | LOOK_DOWN（特殊） | 通常无底盘运动 |
| `-1` | 视为停止 / 无运动 | 全零 |

诸如 `"turn right"` 的语言指令**不保证**产出动作 `3`；DualVLN 面向 VLN 风格目标训练，并非字面遥操解析器。启用运动前务必在干跑模式下核实行动作。

### 1.4 Lift2S 上使用的自定义文件

上游 InternNav 不附带 Lift2S bridge。本 bring-up 增加以下文件（若重置仓库，请在 git 外保留副本）：

| 文件 | 作用 |
|------|------|
| `InternNav/scripts/smoke_test_fake_obs.py` | HTTP 冒烟测试（假观测或 `.npz` 观测） |
| `InternNav/scripts/capture_realsense_frame.py` | 经 ROS 一次性 RGB-D 捕获 → `/tmp/internnav_obs.npz` |
| `InternNav/scripts/lift2s_internnav_bridge.py` | 实时相机 → DualVLN → 可选 `/cmd_vel` |
| `InternNav/internnav/utils/__init__.py` | 懒加载 `AgentServer`，使机器人客户端无需导入完整服务端依赖 |

---

## 2. 前置条件

### 2.1 机器人

- Ubuntu 24.04，ROS 2 **Jazzy**
- `RMW_IMPLEMENTATION=rmw_zenoh_cpp`（通常写在 `~/.bashrc`）
- 可用的 `lift2s-ws`（`./quick_start.sh` 能启动底盘）
- Intel RealSense D405（或同类），**USB 3.x** 连接
- RealSense ROS 封装工作空间，例如 `~/ARX5_ROBOT_1-main/realsense`
- 网络可达 zy 的 TCP 端口 **8087**

### 2.2 zy

- 具备足够显存运行 DualVLN 的 CUDA GPU（文档常提到约 24 GB 级；已验证 RTX 5090）
- [uv](https://docs.astral.sh/uv/) (lab policy: **not** conda)
- 足够磁盘空间存放 InternNav + `checkpoints/InternVLA-N1-DualVLN`

### 2.3 网络 / 代理说明（国内 / FlyingBird）

- 代理（例如 FlyingBird `http://127.0.0.1:7892`）仅用于 **git clone**、**pip** 与 **Hugging Face** 下载。
- 访问 zy 的 **局域网** 流量须绕过代理，否则可能出现 HTTP **502**：

```bash
export NO_PROXY="192.168.110.35,192.168.110.0/24,192.168.111.0/24,localhost,127.0.0.1"
export no_proxy="$NO_PROXY"
unset http_proxy HTTP_PROXY https_proxy HTTPS_PROXY
```

一次性 curl 绕过：

```bash
curl -sS --noproxy '*' -w "\nhttp_code=%{http_code}\n" --connect-timeout 3 \
  http://192.168.110.35:8087/openapi.json | head
```

权重已在本地后，日常局域网操作可关闭 VPN。

---

## 3. 克隆 InternNav

在**机器人与 zy 两端**都执行（或克隆一次再同步）。机器人端带代理示例：

```bash
# Enable proxy only for this clone session if required at your site
export http_proxy=http://127.0.0.1:7892
export https_proxy=http://127.0.0.1:7892

cd ~
git clone https://github.com/OpenRobotLab/InternNav.git
cd InternNav
```

**原因。** 获取 Agent Server、DualVLN 策略代码与评估工具。

另见现场说明：`~/文档/Documentations/FlyingBird_GitHub_Clone_Guide.md`（若存在）。

---

## 4. Python 环境与依赖

### 4.1 创建 uv 环境

在每台将运行 InternNav Python 代码的机器上：

```bash
# 实验室规定：用 uv，不用 conda
cd ~/SocialNav/InternNav   # 或 ~/InternNav
uv venv .venv --python 3.10
source .venv/bin/activate
```

**原因。** InternNav 面向 Python 3.10；隔离依赖可避免与系统 ROS Python 3.12 冲突。

### 4.2 安装 PyTorch（匹配 GPU / CUDA）

**zy（已验证示例栈）：** CUDA 12.8 wheel，例如 Torch 2.7.x + cu128。

**机器人（若安装 Torch）：** 优先选择支持笔记本 GPU 计算能力的构建（例如 RTX 5070 / `sm_120` 可能需要较新的 cu128 构建）。机器人仍**不应**承载 DualVLN 推理。

示例模式（按当前 PyTorch 索引调整）：

```bash
source ~/SocialNav/InternNav/.venv/bin/activate
uv pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu128
```

**原因。** DualVLN 需要在**服务端**有可用的 CUDA Torch。错误的 CUDA/架构构建会在 import 或首次 CUDA 算子处失败。

### 4.3 安装 InternNav 与 Python 依赖

```bash
source ~/SocialNav/InternNav/.venv/bin/activate
cd ~/InternNav
uv pip install -e ".[model]" --no-build-isolation
# Install remaining requirements per InternNav README / requirements/*
```

在 **zy** 上还需确保 DualVLN 可用的 `flash_attn`（从源码编译需要 `nvcc`，或安装匹配 Torch/CUDA/cxx11 ABI 的**预编译 wheel**）。验证：

```bash
python -c "import flash_attn; print(flash_attn.__version__)"
```

在**机器人薄客户端**上，主要需要 `AgentClient` / 冒烟脚本所用包（`numpy`、`requests`、InternNav 配置）。若按如下方式导入，则完整 agent 服务端依赖（如 `quaternion`、Habitat）**不是**必需的：

```python
from internnav.utils.comm_utils.client import AgentClient
```

并保持 `internnav/utils/__init__.py` 对 `AgentServer` 懒加载。

### 4.4 下载 DualVLN checkpoint（zy）

将权重放在服务端 InternNav 根目录下，例如：

```text
~/SocialNav/InternNav/checkpoints/InternVLA-N1-DualVLN/
~/SocialNav/InternNav/checkpoints/InternVLA-N1-DualVLN-SocialGen-10k-3ep-20261115/
```

SocialGen 微调目录须含 `chat_template.json`（缺失则从基座 DualVLN 复制）。用 bridge/smoke 的 `--model-path` 传入相对路径；5090 上切换模型时重启 `start_server.py`。

（按发布的多分片 `safetensors` 布局）。可选的 Depth-Anything 权重用于其他 RGB-D / System-1 路径；本 bring-up 的 DualVLN RGB 导航可不依赖它们。

**原因。** 客户端调用 `/agent/init` 时，Agent Server 会按**服务端**机器上 InternNav 工程内的相对路径加载 `model_path`。

---

## 5. 在 zy 上启动模型服务

### 5.1 启动 Agent Service

**设备：仅 zy。**

```bash
source ~/SocialNav/InternNav/.venv/bin/activate
cd ~/SocialNav/InternNav
python scripts/eval/start_server.py --host 0.0.0.0 --port 8087
```

| 参数 | 含义 |
|----------|---------|
| `--host 0.0.0.0` | 监听所有网卡，供机器人经局域网连接 |
| `--port 8087` | Agent Service 默认端口 |

期望日志包含 Uvicorn 在 `http://0.0.0.0:8087` 上启动。若不做 Habitat 评估，Habitat 警告（`No module named 'habitat'`）可接受。

**不要**在 Lift2S 的 8 GB GPU 主机上为 DualVLN 运行 `start_server.py`。

### 5.2 验证 OpenAPI 接口

在 zy 或机器人上：

```bash
curl -sS --noproxy '*' -w "\nhttp_code=%{http_code}\n" --connect-timeout 3 \
  http://192.168.110.35:8087/openapi.json | head
```

**期望。** JSON 以 `"openapi":"3.1.0"` 开头，包含路径 `/agent/init`、`/agent/{agent_name}/step`、`/agent/{agent_name}/reset`，且 `http_code=200`。

**原因。** 在开展任何机器人客户端工作前，确认 FastAPI Agent Service 可达。

---

## 6. 机器人感知栈（ROS 2 + RealSense）

本节所有命令在 **Lift2S 机器人**上执行。

### 6.1 Zenoh 路由

```bash
source /opt/ros/jazzy/setup.bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

**原因。** 当 `RMW_IMPLEMENTATION=rmw_zenoh_cpp` 时，ROS 2 节点通过 Zenoh 路由发现彼此。没有它，话题可能为空或对端无法连接。保持该终端打开。**HTTP 推理不需要**在 zy 上跑此进程。

### 6.2 RealSense 相机（USB 3.x）

确认设备与 USB 速率：

```bash
lsusb | grep -i 0b5b
lsusb -t | grep -E '5000M|10000M|Video'
rs-enumerate-devices -s
```

优先 **5000M+**（USB 3）。USB 2.1（`480M`）常导致帧超时与缺失 RGB。

启动（序列号示例 `323622271380` — 请换成你的；注意 `serial_no` 前导 `_`）：

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

**期望。** 日志显示 `Device USB type: 3.x` 与 `RealSense Node Is Up!`。

验证话题：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ARX5_ROBOT_1-main/realsense/install/setup.bash
ros2 topic list | grep camera_head
ros2 topic hz /camera_head/color/image_raw
```

**期望。** 健康时 color 约 30 Hz。

### 6.3 查看相机（勿与 ROS 节点争用）

| 方法 | 何时使用 |
|--------|------|
| `ros2 run rqt_image_view rqt_image_view` → `/camera_head/color/image_raw` | ROS RealSense 正在运行时（与 DualVLN 所见相同图像） |
| `realsense-viewer` | 仅在**停止** ROS launch 之后；启用 **Stereo Module**，优先 **2D** 视图 |

同一时刻只能有一个进程占用 D405。

---

## 7. HTTP 客户端冒烟测试（无底盘运动）

**设备：机器人**（zy 服务须已启动）。按 §2.3 保留 `NO_PROXY` / 取消 `http_proxy`。

### 7.1 假观测（可选；在 zy 上用 `--host localhost` 也有效）

```bash
source ~/SocialNav/InternNav/.venv/bin/activate
cd ~/InternNav
python scripts/smoke_test_fake_obs.py --host 192.168.110.35
```

**原因。** 在不依赖 ROS 的情况下验证 `/agent/init` + `/agent/.../step` 以及 DualVLN 加载。

**期望。** `Agent init OK` 与 `Action returned: [{'action': [...], 'ideal_flag': ...}]`。

### 7.2 真实相机帧

用**系统** Python（ROS Jazzy）捕获，不要用 InternNav 的 uv 虚拟环境 — `rclpy` 面向系统 Python 3.12 构建：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ARX5_ROBOT_1-main/realsense/install/setup.bash
/usr/bin/python3 ~/InternNav/scripts/capture_realsense_frame.py --out /tmp/internnav_obs.npz
```

然后发送到 zy：

```bash
source ~/SocialNav/InternNav/.venv/bin/activate
cd ~/InternNav
python scripts/smoke_test_fake_obs.py --host 192.168.110.35 \
  --npz /tmp/internnav_obs.npz \
  --instruction "go forward a little bit then stop"
```

**原因。** 用真实 RGB-D 验证 `AgentClient` 所用观测编码路径（JSON 上的 pickle + base64）。

---

## 8. 底盘启动

**设备：机器人。**

```bash
cd ~/lift2s-ws
./quick_start.sh
# follow on-screen menus for your real-robot configuration
```

确认话题存在（连同相机话题）：

```bash
ros2 topic list | grep -E 'cmd_vel|arx_lift/odom|camera_head'
```

**期望。** 至少有 `/cmd_vel` 与 `/arx_lift/odom`，以及 `/camera_head/color/image_raw`。

**原因。** 当底盘模式允许速度指令时（通常 `chassis_mode=1`），Lift2S 硬件接口会消费 `/cmd_vel`。WBC/VR 也可能发布 `/cmd_vel`；DualVLN 驾驶测试期间请保持机械臂空闲 / HOLD。

人工冒烟检查（急停就绪）：

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.4}}"
# Ctrl+C, then stop:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

---

## 9. 闭环 bridge（相机 → DualVLN → `/cmd_vel`）

**设备：机器人。** 保持 zy 服务、Zenoh、RealSense 与 `lift2s-ws` 运行。

bridge 脚本使用**系统 Python**（`rclpy` + `requests`），经 HTTP 与 zy 通信（`trust_env=False`，使局域网不被代理）。

### 9.1 干跑（必须先做）

```bash
source /opt/ros/jazzy/setup.bash
source ~/ARX5_ROBOT_1-main/realsense/install/setup.bash

/usr/bin/python3 ~/InternNav/scripts/lift2s_internnav_bridge.py \
  --host 192.168.110.35 \
  --instruction "go forward a little bit then stop" \
  --steps 5
```

**原因。** 打印离散动作，不发布 `/cmd_vel`。

### 9.2 启用运动

清理场地；操作员手持急停：

```bash
/usr/bin/python3 ~/InternNav/scripts/lift2s_internnav_bridge.py \
  --host 192.168.110.35 \
  --instruction "go forward a little bit then stop" \
  --steps 5 \
  --enable-motion \
  --v-lin 0.15 \
  --v-ang 0.5 \
  --pulse 1.5 \
  --period 2.5
```

| 标志 | 作用 |
|------|------|
| `--enable-motion` | 实际发布 `/cmd_vel` |
| `--v-lin` / `--v-ang` | 线速度 / 角速度比例 |
| `--pulse` | 每个动作施加的时长 |
| `--period` | 推理步之间的间隔 |

**Bring-up 成功标准。** 实时 RGB-D → 远程 DualVLN → 底盘运动与打印动作一致（例如 FORWARD 脉冲）。

### 9.3 语言 / 物体目标（预期）

当物体在 `/camera_head/color/image_raw` 中**较大且居中**时（用 `rqt_image_view` 监视），DualVLN 可尝试诸如 `"go to the black tumbler then stop"` 的目标。它**不是**可靠的“原地旋转直到找到 X”控制器，也**不会**抓取物体。D405 为短距相机；房间尺度搜索能力有限。

---

## 10. 推荐终端布局

| 终端 | 机器 | 命令 / 角色 |
|----------|---------|----------------|
| 1 | zy | `start_server.py --host 0.0.0.0 --port 8087` |
| 2 | 机器人 | `rmw_zenohd` |
| 3 | 机器人 | RealSense `ros2 launch ...` |
| 4 | 机器人 | `lift2s-ws` `./quick_start.sh` |
| 5 | 机器人 | Bridge / 冒烟测试 / `rqt_image_view` |

启动顺序：**zy 服务** → **Zenoh** → **相机** → **底盘** → **bridge**。

---

## 11. 故障排查

| 现象 | 可能原因 | 处理 |
|---------|--------------|------------|
| `curl` 为空或 `http_code=502` | HTTP 代理拦截局域网 | `--noproxy '*'`、`NO_PROXY`、取消 `http_proxy` |
| `Unable to connect to a Zenoh router` | 相机在 `rmw_zenohd` 之前启动 | 先启 Zenoh；重启相机 |
| 无 `/camera_head` 话题 | 相机节点未起或 domain 不对 | 检查 launch、`ROS_DOMAIN_ID`、Zenoh |
| 帧超时 / 无 RGB | USB 2.1 | 换 USB 3 口；确认 `5000M` |
| Cursor 中 `realsense-viewer` 黑屏 | 无 `DISPLAY` | 用 NoMachine/桌面；`DISPLAY=:0` |
| Viewer 与 ROS 冲突 | 两个进程争用 D405 | 同一时刻仅一个 |
| 机器人上 `ModuleNotFoundError: quaternion` | 导入了完整 `AgentServer` 栈 | 使用 `comm_utils.client` + 懒加载 `utils/__init__.py` |
| 冒烟脚本 `No such file` | `git clean` / 重新克隆删掉了自定义脚本 | 从本指南备份恢复脚本 |
| DualVLN 总是 `STOP` / 忽略 “turn right” | 策略 / 指令不匹配 | 干跑；简化目标；对准相机；用手动 `/cmd_vel` 指向 |
| 运动过小 | bridge 默认偏保守 | 增大 `--v-lin`、`--v-ang`、`--pulse` |
| 已发布 `/cmd_vel` 但无运动 | 底盘模式 / WBC 占用 | 确认 `chassis_mode=1`，空闲 VR/WBC |

---

## 12. 安全与防护

- 设置 `--enable-motion` 时，始终保证硬件急停在手边。
- 从干跑与低速开始。
- DualVLN 是研究软件；不要将输出当作无碰撞导航。
- 在实验室局域网暴露 `0.0.0.0:8087` 便于调试；未做认证与防火墙控制时，**不要**暴露到公网。

---

## 13. 本 bring-up 已达成内容（清单）

- [x] 已安装 InternNav；DualVLN checkpoint 在 zy 上  
- [x] Agent Service 监听 `:8087`  
- [x] 机器人可达 OpenAPI（`http_code=200`）  
- [x] Lift2S 上 Zenoh + RealSense RGB-D  
- [x] 机器人端假观测与真实帧 HTTP `step`  
- [x] `lift2s-ws` 话题含 `/cmd_vel`  
- [x] 干跑与启用运动的 bridge  

**尚未纳入范围：** 持续直到 STOP 的策略、基于地图的导航、机械臂抓取放置、量产避障。

---

## 14. 参考：通信时序

1. Bridge（或冒烟客户端）`POST /agent/init`，携带 `AgentCfg` / `model_settings`（含**服务端**文件系统上的 `model_path`、`device: cuda:0`、相机内参、图像尺寸）。
2. 服务端将 DualVLN 载入 GPU 显存（zy 日志可见分片加载），返回 `agent_name`（例如 `internvla_n1`）。
3. 客户端构建观测字典 `{rgb, depth, instruction}`，pickle 后 base64 编码，`POST /agent/{agent_name}/step`。
4. 服务端运行 DualVLN，返回 JSON 动作。
5. Bridge 将第一个离散编码映射为 `/cmd_vel` 上的短时 `geometry_msgs/Twist` 脉冲（若启用 `--enable-motion`）。
6. 在启用时，Lift2S HI 将 `/cmd_vel` 转为底盘指令。

该设计下 ROS 2 **从不**跨越机器人↔zy 边界；跨越边界的只有 HTTP。

---

## 15. 文档历史

| 项 | 值 |
|------|--------|
| 平台 | ARX Lift2S + InternNav DualVLN |
| ROS | Jazzy + `rmw_zenoh_cpp` |
| 相机 | Intel RealSense D405 @ 640×480 |
| 推理主机 | 远程 zy Agent Service `:8087` |
| 指南状态 | 已通过闭环 `/cmd_vel` 运动测试验证 |

实验室环境变化时，维护者应更新示例 IP、序列号以及 Torch/CUDA 锁定版本。
