# InternNav ↔ ARX Lift2S

在 ARX Lift2S 上运行 InternNav DualVLN 的集成脚本与文档。

- 机器人端：ROS 2 Jazzy + Zenoh + RealSense D405 + `open-deploy-ws`（`arx-lift2s`）
- 推理端：GPU 主机上的 InternNav Agent Server（例如 zy），HTTP `:8087`

本仓库**不是**完整的 InternNav fork。模型服务请克隆上游 InternNav：

https://github.com/InternRobotics/InternNav

机器人工作空间（底盘 / 控制）：

https://github.com/fiveages-sim/open-deploy-ws（分支 `arx-lift2s`）

## 内容

| 路径 | 说明 |
|------|-------------|
| `scripts/smoke_test_fake_obs.py` | HTTP 冒烟测试（假观测或 `.npz` 观测） |
| `scripts/capture_realsense_frame.py` | 从 ROS 捕获一帧 RGB-D |
| `scripts/lift2s_internnav_bridge.py` | 相机 → DualVLN → 可选 `/cmd_vel` |
| `docs/` | 实现与架构指南 |
| `docs/Basic_Dry_Run_and_Motion_Guide.md` | zy ↔ Lift2S 干跑 + 运动指令脚本 |
| `patches/internnav_utils_init.py` | 机器人端懒加载 `AgentClient` 的辅助补丁 |

## 快速开始

参见 `docs/Quick_Start.md`，或完整指令脚本 `docs/Basic_Dry_Run_and_Motion_Guide.md`。

## Cursor skill

供处理本集成的 agent 使用的项目 skill：

`.cursor/skills/internnav-arx-lift2s/`

如需在本仓库外使用，可复制到 `~/.cursor/skills/`。

## 演示

[Lift2S DualVLN demo (mp4)](docs/media/InternNav_ARX_LIFT2S.mp4)
