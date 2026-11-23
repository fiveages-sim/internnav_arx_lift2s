# InternNav 在 ARX Lift2S 机器人上如何工作

**读者对象。** 需要理解感知、ROS 2 / Zenoh、InternNav 模型变体，以及 Lift2S 专用粘合代码的工程师。

**相关文档。**

- Bring-up 命令：[`InternNav_ARX_Lift2S_Implementation_Guide.md`](./InternNav_ARX_Lift2S_Implementation_Guide.md)
- 上游：[InternRobotics/InternNav](https://github.com/InternRobotics/InternNav)
- 论文：[InternVLA-N1 technical report](https://internrobotics.github.io/internvla-n1.github.io/)、[DualVLN (arXiv:2512.08186)](https://arxiv.org/abs/2512.08186)

**已验证布局。** Lift2S 机载 PC（`arx`）运行相机 + 底盘 ROS；DualVLN 推理在 **zy** 上经 HTTP `:8087` 完成。

---

## 1. RealSense D405 如何工作、如何接到机器人

### 1.1 D405 是什么

Intel RealSense **D405** 是短距立体深度相机。它提供：

- **彩色（RGB）** — 通常为 RGB8 帧（例如 640×480 @ 30 Hz）
- **深度** — 通常为毫米为单位的 16 位深度（Z16）
- 可选红外流（DualVLN RGB 路径不需要）

物理上经 **USB** 接到 Lift2S 主机 PC。可靠的 RGB+depth 流需要 **USB 3.x**（内核常报告 `5000M`）。USB 2.1（`480M`）经常导致帧超时。

在本机器人上设备大致显示为：

```text
Bus 004 Device xxx: ID 8086:0b5b Intel RealSense Depth Camera 405
Serial (example): 323622271380
```

机器人上的软件栈：

1. **librealsense** — 与 USB 设备通信的底层 SDK
2. **`realsense2_camera`** ROS 2 节点 — 封装 librealsense 并发布 ROS 话题
3. 可选 **`realsense-viewer`** — 仅 GUI；**不得**与 ROS 节点同时运行（同一时刻一个进程占用相机）

机器人上的 `~/librealsense` 是 SDK 树；ROS 封装通常位于如 `~/ARX5_ROBOT_1-main/realsense` 的工作空间。底盘 bring-up 在 `~/lift2s-ws`（`quick_start.sh`）。

### 1.2 连接路径（硬件 → ROS 话题）

```
D405 (USB 3)
   │  librealsense
   ▼
realsense2_camera_node  (ROS 2 process)
   │  publishes sensor_msgs/Image, CameraInfo, …
   ▼
ROS 2 middleware = rmw_zenoh_cpp
   │  via local Zenoh router (rmw_zenohd)
   ▼
Other ROS 2 nodes on the same robot
   (rqt_image_view, lift2s_internnav_bridge, chassis stack, …)
```

启动后的典型话题（名称可能因 launch 参数使用 `camera_head`）：

| 话题 | 类型 | 内容 |
|-------|------|---------|
| `/camera_head/color/image_raw` | `sensor_msgs/msg/Image` | RGB 像素 |
| `/camera_head/depth/image_raw` | `sensor_msgs/msg/Image` | 深度 |
| `/camera_head/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 内参 |

### 1.3 ROS 2 节点如何通信（以及 Zenoh 的位置）

ROS 2 是 **发布/订阅** 系统。节点不会像 REST API 那样互相调用。发布者在**命名话题**上写入消息；任意数量具有匹配类型与兼容 QoS 的订阅者可以接收。

在本 Lift2S 配置中：

| 组件 | 作用 |
|-------|------|
| `RMW_IMPLEMENTATION=rmw_zenoh_cpp` | 告诉 ROS 2 底层使用 Zenoh 作为传输 |
| `ros2 run rmw_zenoh_cpp rmw_zenohd` | 机器人上的 Zenoh **路由**；对端通过它发现彼此 |
| `ROS_DOMAIN_ID` | 需要互相可见的节点必须一致（常为 `0`） |

**关于 Lift2S 上 InternNav 的重要澄清：**

- Zenoh 用于**机载** ROS 流量（相机 ↔ bridge ↔ 底盘）。
- zy 上的 DualVLN **不**加入该 Zenoh 图。
- 机器人 ↔ zy 使用 **HTTP**，而非 ROS。

因此：“D405 经 ROS 2 Zenoh 通信”在**机器人内部**是正确的。这**不是** zy 上 DualVLN 接收图像的方式。

### 1.4 机器人如何“理解”/接收 RGB 话题

没有任何魔法会自动“理解”RGB。接收节点会：

1. 以类型 `sensor_msgs/msg/Image` **订阅** `/camera_head/color/image_raw`。
2. 中间件将每条已发布消息投递到**回调**。
3. 回调读取 `msg.height`、`msg.width`、`msg.encoding` 与 `msg.data`（原始字节）。
4. 应用代码将字节重塑为 NumPy 数组（例如对 `rgb8` 为 `H×W×3` `uint8`）。

示例（概念性，与我们的 bridge 一致）：

```python
# Subscriber callback receives sensor_msgs/Image
rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
```

`rqt_image_view` 同样为显示解码。我们的 `lift2s_internnav_bridge.py` 则用于构建发给 DualVLN 的观测。

**谁发布？** 仅 RealSense ROS 节点（在 launch 期间）。  
**谁接收？** 任意本机订阅者（bridge、`rqt`、调试工具）。底盘**不需要** RGB 才能驱动车轮；只有导航客户端需要。

---

## 2. InternNav 模型：VLA / VLN — 有多少种，DualVLN 是什么

### 2.1 InternNav 是平台，不是“3 个 VLA + 3 个 VLN”

[InternNav](https://github.com/InternRobotics/InternNav) 是面向导航基础模型的**开放平台**：数据集、训练、评估（Habitat / Isaac）、基线与真机辅助工具。

README 大致将支持方法分为：

| 族 | 示例 | 角色 |
|--------|----------|------|
| 经典 / 基线 **VLN** agent | Seq2Seq、CMA、RDP、StreamVLN（即将） | 视觉-语言**导航**基线 |
| 局部 / 几何 / diffusion 导航器 | DD-PPO、iPlanner、GNM、ViNT、NoMad、**NavDP** | 常视为 **System 1** 风格局部策略 |
| **InternVLA-N1** 双系统配置 | Decoupled S2+S1、Dual System w/ NavDP*、**DualVLN** | 主要**导航 VLA** 产品线 |

因此，说“InternNav 正好有 3 个 VLA 和 3 个 VLN”是**不正确**的。更准确地说：

- 有**许多** VLN / 导航基线。
- **InternVLA-N1** 是旗舰**面向导航的 VLA** 模型族。
- 你列出的三条是 **InternVLA-N1 双系统设计的三种部署 / 训练配置**，并非三个无关 VLA 再加三个无关 VLN。

### 2.2 三条 InternVLA-N1 线路分别指什么

InternVLA-N1 采用**双系统**思路（见 InternVLA-N1 报告 / DualVLN 论文）：

| 组件 | 典型职责 |
|-----------|-------------|
| **System 2** | 较慢、语言条件的“我该去哪？” / 由视觉 + 指令得到的潜在规划 |
| **System 1** | 较快的局部执行 / 底层运动（例如 NavDP 风格），常为 RGB-D |

README 中的三条：

1. **InternVLA-N1 (System 2) + Decoupled System 1**  
   System 2 与单独训练 / 单独运行的 System 1 配对（例如 NavDP 作为局部策略），不一定作为单一 DualVLN 包联合微调。

2. **InternVLA-N1 (Dual System) w/ NavDP\***  
   双系统，其中 System 1 为 **NavDP\***（与 System 2 联合调优）。观测常为 **RGB-D**。

3. **InternVLA-N1 (Dual System) DualVLN** ← **我们在 Lift2S 上运行的**  
   双系统 DualVLN 变体；在基准上有较强的 **RGB** VLN 表现。这是在 zy 上提供的 checkpoint。

**在 Lift2S 上我们仅使用 DualVLN**（远程 Agent Service）。不会同时运行全部三套栈。

### 2.3 InternNav 总体上如何“工作”

高层流程（仿真或真机）：

1. 构建**观测**：图像（± depth）、若有则位姿，以及**语言指令**。
2. 调用 **agent** 的 `step(observation)`。
3. 接收**动作**（离散 Habitat 风格编码和/或连续轨迹，取决于配置）。
4. **环境**（Habitat 仿真，或真机封装）执行这些动作。

官方真机演示常封装 Unitree Go2。Lift2S 需要**自定义封装**：我们的 HTTP bridge + `/cmd_vel` 映射。

---

## 3. InternNav 是否接收 D405 RGB 话题？是否发布 `/cmd_vel`？

### 3.1 *本* Lift2S 部署的简短回答

| 问题 | 回答 |
|----------|--------|
| 上游 DualVLN 是否订阅 `/camera_head/color/image_raw`？ | **否。** |
| zy 上的 DualVLN 是否看到 ROS 话题？ | **否。** 它只看到 HTTP 请求体。 |
| RGB 帧如何到达 DualVLN？ | 机器人 bridge 在 ROS 中**订阅** → 打包 NumPy → **HTTP** `POST /agent/.../step`。 |
| InternNav / DualVLN 是否发布 `/cmd_vel`？ | **否。** |
| 谁发布 `/cmd_vel`？ | 我们的 **`lift2s_internnav_bridge.py`**（在 `--enable-motion` 时），将离散动作映射为 `geometry_msgs/Twist`。 |

### 3.2 Lift2S 端到端数据路径

```
D405 USB
  → realsense2_camera (pub RGB/Depth)
  → Zenoh ROS graph (robot-local)
  → lift2s_internnav_bridge (sub Image)
  → HTTP JSON (pickle+base64 observation) → zy Agent Server
  → DualVLN GPU inference
  → HTTP JSON action codes
  → bridge maps 0/1/2/3 → Twist
  → /cmd_vel (ROS, Zenoh)
  → Lift2S chassis hardware interface
```

### 3.3 运动如何决定（前进 / 左转 / 右转 / 停止）

DualVLN **不会**输出像 “forward” 这样的英文词。它输出 **离散动作 ID**（Habitat 风格），例如：

| 编码 | 含义 | Bridge → `/cmd_vel`（典型） |
|------|---------|--------------------------------|
| `0` | STOP | 全零 |
| `1` | FORWARD | `linear.x > 0` |
| `2` | TURN_LEFT | `angular.z > 0` |
| `3` | TURN_RIGHT | `angular.z < 0` |
| `5` | LOOK_DOWN（特殊） | 通常无底盘运动 |
| `-1` | 视为空操作 / 停止 | 全零 |

我们 bridge 使用的离散集合中**没有原生“后退”**。后退需要不同动作空间或自定义映射。

**DualVLN 如何选择编码：** 以近期 RGB（若提供则含 depth）、指令文本，以及内部双系统状态 / 历史为条件的神经推理。它是学成的 VLN 策略，不是硬编码的“若瓶子在左则转弯”规则。诸如 `"turn right"` 的字面指令仍可能因图像与训练分布产出 FORWARD 或 STOP。

当设置 `--enable-motion` 时，bridge 再对非停止动作施加短时**速度脉冲**（`--v-lin`、`--v-ang`、`--pulse`）。

---

## 4. 为 Lift2S 创建的自定义脚本（路径与角色）

上游 InternNav **不**附带 ARX Lift2S 驱动。这些文件在 bring-up 期间添加。**请备份**；`git clean` / 重新克隆可能删掉它们。

| 路径 | 用途 | 为何需要 |
|------|---------|------------|
| `~/InternNav/scripts/smoke_test_fake_obs.py` | HTTP 冒烟：在 zy 上初始化 DualVLN agent，并用合成 RGB-D 帧或 `--npz` 真实捕获调用 `step` | 在**无**底盘运动时证明局域网 + Agent Service + checkpoint 加载 |
| `~/InternNav/scripts/capture_realsense_frame.py` | ROS 订阅者（系统 Python），等待一帧 color + depth 并写入 `/tmp/internnav_obs.npz` | 将 ROS/`rclpy`（系统 Python 3.12）与 uv InternNav 客户端分离；为冒烟测试提供真实像素 |
| `~/InternNav/scripts/lift2s_internnav_bridge.py` | 实时循环：订阅相机 → HTTP DualVLN → 打印动作；可选 `--enable-motion` 发布 `/cmd_vel` | Lift2S 的**闭环**导航粘合层；DualVLN 自身从不说 ROS |
| `~/InternNav/internnav/utils/__init__.py`（懒加载 `AgentServer`） | 立即导出 `AgentClient`；仅在请求时加载 `AgentServer` | 让机器人导入薄 HTTP 客户端，而无需拉取 Habitat / `quaternion` / 完整服务端依赖 |

### 4.1 使用哪个解释器

| 脚本 | 解释器 |
|--------|-------------|
| `smoke_test_fake_obs.py` | `source ~/SocialNav/InternNav/.venv/bin/activate` |
| `capture_realsense_frame.py` | `/usr/bin/python3` + 已 source 的 ROS |
| `lift2s_internnav_bridge.py` | `/usr/bin/python3` + 已 source 的 ROS |
| `scripts/eval/start_server.py` | **仅 zy** 上的 `uv` 虚拟环境 |

### 4.2 各脚本的最小心智模型

**冒烟测试** — “zy 能否回答 `/agent/init` 与 `/agent/.../step`？”

**捕获** — “ROS 能否把 D405 的一对 RGB-D 送入 NumPy 文件？”

**Bridge** — “我们能否反复把实时相机帧变成 DualVLN 动作，并（可选）驱动 `/cmd_vel`？”

**懒加载 utils init** — “机器人能否在不安装完整仿真/服务端栈的情况下运行客户端？”

---

## 5. 对应你的终端 / 工作空间笔记

从机器人主机布局（`~/lift2s-ws`、`~/librealsense`、`~/InternNav`）：

| 路径 | 在本故事中的角色 |
|------|--------------------|
| `~/librealsense` | D405 的 SDK 源码 / 安装 |
| RealSense ROS 工作空间（例如 `ARX5_ROBOT_1-main/realsense` 下） | 已构建的 `realsense2_camera` 封装 |
| `~/lift2s-ws` | 底盘 / 全身栈；提供 `/cmd_vel` 消费者（`quick_start.sh`） |
| `~/InternNav` | 客户端脚本 +（在 zy 上）DualVLN 服务端 + 权重 |

`git status` 显示 `lift2s-ws/src` 下**子模块**脏污与 DualVLN 推理无关；只表示这些嵌套仓库检出了新提交。InternNav↔相机通信不经过那些子模块提交。

---

## 6. FAQ 摘要

**D405 如何连接？** USB 3 → librealsense → `realsense2_camera` → ROS 话题。

**节点如何通信？** 经 ROS 2 发布/订阅，使用 Zenoh RMW 与机器人上的本地 `rmw_zenohd` 路由。

**机器人如何接收 RGB？** 任意**订阅**彩色图像话题的节点会收到 `sensor_msgs/Image` 回调，并将字节解码为图像数组。

**DualVLN 是否订阅该话题？** 否。**Bridge** 订阅，再经 HTTP 发送数组。

**InternNav 是否发布 `/cmd_vel`？** 上游 DualVLN 不发布。启用运动时由我们的 **bridge** 发布。

**三条 InternVLA-N1 是否是“3 个 VLA + 3 个 VLN”？** 否。它们是三种 **InternVLA-N1 双系统配置**；InternNav 还附带许多其他 VLN/System-1 基线。我们仅使用 **DualVLN**。

**前进/左转/右转/停止如何选定？** DualVLN 神经策略 → 离散动作 ID → bridge 速度脉冲。我们的映射中无原生后退。

---

## 7. 文档历史

| 项 | 值 |
|------|--------|
| 主题 | ARX Lift2S 上 InternNav DualVLN 的概念架构 |
| 感知 | Intel RealSense D405 + ROS 2 Jazzy + Zenoh |
| 推理 | zy 上的远程 Agent Service（`:8087`） |
| 控制 | 自定义 bridge → `/cmd_vel` |
| 上游 | [InternRobotics/InternNav](https://github.com/InternRobotics/InternNav) |
