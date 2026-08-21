# Implementing InternNav (InternVLA-N1 DualVLN) on ARX Lift2S

**Document purpose.** Step-by-step guide for deploying [InternNav](https://github.com/OpenRobotLab/InternNav) language-conditioned navigation on an **ARX Lift2S** robot, with heavy inference on a remote GPU workstation (**Sim-06**) and sensing / chassis control on the robot.

**Audience.** Engineers bringing up DualVLN on Lift2S (ROS 2 Jazzy, Ubuntu 24.04).

**Scope of this guide.** From repository clone and dependency setup through model server, camera bring-up, HTTP client smoke tests, and a closed-loop `/cmd_vel` bridge. It does **not** cover arm grasping or production navigation safety certification.

**Hardware reference (validated setup).**

| Role | Machine | Notes |
|------|---------|--------|
| Robot | ARX Lift2S host (`arx`) | ROS 2 Jazzy, Zenoh RMW, Intel RealSense D405, chassis via `lift2s-ws` |
| Model server | Sim-06 (`fiveages@Sim-06`) | NVIDIA RTX 5090-class GPU, DualVLN checkpoint |
| Robot GPU | RTX 5070 Laptop (8 GB) | **Insufficient** for DualVLN; do not run the model server here |

**Typical LAN addresses (replace with your site values).**

| Host | Example IP |
|------|------------|
| Sim-06 | `192.168.110.35` |
| Lift2S robot | `192.168.111.209` |

---

## 1. How the pipeline works

### 1.1 Architecture overview

InternNav’s DualVLN agent expects RGB (and depth) observations plus a natural-language instruction, and returns **discrete navigation actions** (Habitat-style). Official real-world demos target platforms such as Unitree Go2. Lift2S is **not** a drop-in target: you keep InternNav’s **Agent Server** for inference and add a **thin robot client** that maps actions to ROS 2 `/cmd_vel`.

```
┌─────────────────────────────────────┐         HTTP :8087          ┌─────────────────────────────────────┐
│  ARX Lift2S (robot)                 │ ──────────────────────────► │  Sim-06 (model server)               │
│                                     │                             │                                     │
│  RealSense D405                     │     POST /agent/init        │  conda env: internnav               │
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

### 1.2 What uses ROS 2 vs what uses HTTP

| Path | Transport | Purpose |
|------|-----------|---------|
| Camera → bridge | **ROS 2** (Zenoh RMW) on the **robot only** | Image topics under `/camera_head/...` |
| Chassis odometry / `/cmd_vel` | **ROS 2** on the **robot only** | Drive the base |
| Robot ↔ DualVLN | **HTTP** to Sim-06 `:8087` | Load agent + run `step` |

**Important.** You do **not** run a Zenoh router on Sim-06 for this design. Sim-06 does not join the robot’s ROS graph. Inference is remote HTTP, analogous to a microservice.

### 1.3 Discrete actions

DualVLN returns Habitat-style codes (as used by InternNav’s real-world client conventions):

| Code | Meaning | Typical `/cmd_vel` mapping |
|------|---------|----------------------------|
| `0` | STOP | zeros |
| `1` | FORWARD | `linear.x > 0` |
| `2` | TURN_LEFT | `angular.z > 0` |
| `3` | TURN_RIGHT | `angular.z < 0` |
| `5` | LOOK_DOWN (special) | usually no base motion |
| `-1` | treat as stop / no motion | zeros |

Language instructions such as `"turn right"` are **not** guaranteed to produce action `3`; DualVLN is trained for VLN-style goals, not as a literal teleop parser. Always verify actions in dry-run mode before enabling motion.

### 1.4 Custom files used on Lift2S

Upstream InternNav does not ship a Lift2S bridge. This bring-up adds (keep copies outside git if you reset the repo):

| File | Role |
|------|------|
| `InternNav/scripts/smoke_test_fake_obs.py` | HTTP smoke test (fake or `.npz` observation) |
| `InternNav/scripts/capture_realsense_frame.py` | One-shot RGB-D capture via ROS → `/tmp/internnav_obs.npz` |
| `InternNav/scripts/lift2s_internnav_bridge.py` | Live camera → DualVLN → optional `/cmd_vel` |
| `InternNav/internnav/utils/__init__.py` | Lazy-load `AgentServer` so the robot client need not import full server deps |

---

## 2. Prerequisites

### 2.1 Robot

- Ubuntu 24.04, ROS 2 **Jazzy**
- `RMW_IMPLEMENTATION=rmw_zenoh_cpp` (typically in `~/.bashrc`)
- Working `lift2s-ws` (`./quick_start.sh` can start chassis)
- Intel RealSense D405 (or similar) with **USB 3.x** connection
- RealSense ROS wrapper workspace, e.g. `~/ARX5_ROBOT_1-main/realsense`
- Network reachability to Sim-06 on TCP port **8087**

### 2.2 Sim-06

- CUDA-capable GPU with enough VRAM for DualVLN (docs often cite ~24 GB class; RTX 5090 validated)
- Conda / Miniconda
- Disk space for InternNav + `checkpoints/InternVLA-N1-DualVLN`

### 2.3 Network / proxy notes (China / FlyingBird)

- Use a proxy (e.g. FlyingBird `http://127.0.0.1:7892`) for **git clone**, **pip**, and **Hugging Face** downloads only.
- For **LAN** traffic to Sim-06, bypass the proxy or you may see HTTP **502**:

```bash
export NO_PROXY="192.168.110.35,192.168.110.0/24,192.168.111.0/24,localhost,127.0.0.1"
export no_proxy="$NO_PROXY"
unset http_proxy HTTP_PROXY https_proxy HTTPS_PROXY
```

One-shot curl bypass:

```bash
curl -sS --noproxy '*' -w "\nhttp_code=%{http_code}\n" --connect-timeout 3 \
  http://192.168.110.35:8087/openapi.json | head
```

VPN may stay off during day-to-day LAN operation once weights are local.

---

## 3. Clone InternNav

Run on **both** robot and Sim-06 (or clone once and sync). Example on the robot with proxy:

```bash
# Enable proxy only for this clone session if required at your site
export http_proxy=http://127.0.0.1:7892
export https_proxy=http://127.0.0.1:7892

cd ~
git clone https://github.com/OpenRobotLab/InternNav.git
cd InternNav
```

**Why.** Obtains the Agent Server, DualVLN policy code, and evaluation utilities.

See also site-specific notes: `~/文档/Documentations/FlyingBird_GitHub_Clone_Guide.md` (if present).

---

## 4. Python environment and dependencies

### 4.1 Create the conda environment

On each machine that will run InternNav Python code:

```bash
conda create -n internnav python=3.10 -y
conda activate internnav
cd ~/InternNav
```

**Why.** InternNav targets Python 3.10; isolating deps avoids conflicts with system ROS Python 3.12.

### 4.2 Install PyTorch (match GPU / CUDA)

**Sim-06 (example validated stack):** CUDA 12.8 wheels such as Torch 2.7.x + cu128.

**Robot (if installing Torch at all):** prefer a build that supports the laptop GPU compute capability (e.g. RTX 5070 / `sm_120` may need newer cu128 builds). The robot still should **not** host DualVLN inference.

Example pattern (adjust to current PyTorch index):

```bash
conda activate internnav
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu128
```

**Why.** DualVLN requires a working CUDA Torch on the **server**. Wrong CUDA/arch builds fail at import or at first CUDA op.

### 4.3 Install InternNav and Python deps

```bash
conda activate internnav
cd ~/InternNav
pip install -e .
# Install remaining requirements per InternNav README / requirements/*
```

On **Sim-06**, also ensure `flash_attn` works for DualVLN (compile from source needs `nvcc`, or install a matching **prebuilt wheel** for your Torch/CUDA/cxx11 ABI). Verify:

```bash
python -c "import flash_attn; print(flash_attn.__version__)"
```

On the **robot thin client**, you mainly need packages used by `AgentClient` / smoke scripts (`numpy`, `requests`, InternNav configs). Full agent server imports (e.g. `quaternion`, Habitat) are **not** required if you import:

```python
from internnav.utils.comm_utils.client import AgentClient
```

and keep `internnav/utils/__init__.py` lazy-loading `AgentServer`.

### 4.4 Download DualVLN checkpoint (Sim-06)

Place weights under the server’s InternNav root, for example:

```text
~/InternNav/checkpoints/InternVLA-N1-DualVLN/
```

(multi-shard `safetensors` layout as released). Optional Depth-Anything weights are for other RGB-D / System-1 paths; DualVLN RGB navigation can run without them for this bring-up.

**Why.** The Agent Server loads `model_path` relative to the InternNav project on the **server** machine when a client calls `/agent/init`.

---

## 5. Model server on Sim-06

### 5.1 Start the Agent Service

**Device: Sim-06 only.**

```bash
conda activate internnav
cd ~/InternNav
python scripts/eval/start_server.py --host 0.0.0.0 --port 8087
```

| Argument | Meaning |
|----------|---------|
| `--host 0.0.0.0` | Listen on all interfaces so the robot can connect over LAN |
| `--port 8087` | Default Agent Service port |

Expected log lines include Uvicorn startup on `http://0.0.0.0:8087`. A Habitat warning (`No module named 'habitat'`) is acceptable if you are not running Habitat eval.

**Do not** run `start_server.py` on the Lift2S 8 GB GPU host for DualVLN.

### 5.2 Verify the OpenAPI surface

On Sim-06 or the robot:

```bash
curl -sS --noproxy '*' -w "\nhttp_code=%{http_code}\n" --connect-timeout 3 \
  http://192.168.110.35:8087/openapi.json | head
```

**Expected.** JSON starting with `"openapi":"3.1.0"` and paths `/agent/init`, `/agent/{agent_name}/step`, `/agent/{agent_name}/reset`, with `http_code=200`.

**Why.** Confirms the FastAPI Agent Service is reachable before any robot client work.

---

## 6. Robot sensing stack (ROS 2 + RealSense)

All commands in this section run on the **Lift2S robot**.

### 6.1 Zenoh router

```bash
source /opt/ros/jazzy/setup.bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

**Why.** With `RMW_IMPLEMENTATION=rmw_zenoh_cpp`, ROS 2 nodes discover each other through a Zenoh router. Without it, topics may be empty or peers fail to connect. Keep this terminal open. **Sim-06 does not need this** for HTTP inference.

### 6.2 RealSense camera (USB 3.x)

Confirm device and USB speed:

```bash
lsusb | grep -i 0b5b
lsusb -t | grep -E '5000M|10000M|Video'
rs-enumerate-devices -s
```

Prefer **5000M+** (USB 3). USB 2.1 (`480M`) often causes frame timeouts and missing RGB.

Launch (example serial `323622271380` — replace with yours; note the leading `_` in `serial_no`):

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

**Expected.** Log shows `Device USB type: 3.x` and `RealSense Node Is Up!`.

Verify topics:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ARX5_ROBOT_1-main/realsense/install/setup.bash
ros2 topic list | grep camera_head
ros2 topic hz /camera_head/color/image_raw
```

**Expected.** ~30 Hz on color when healthy.

### 6.3 Viewing the camera (do not fight the ROS node)

| Method | When |
|--------|------|
| `ros2 run rqt_image_view rqt_image_view` → `/camera_head/color/image_raw` | While ROS RealSense is running (same images DualVLN sees) |
| `realsense-viewer` | Only after **stopping** ROS launch; enable **Stereo Module**, prefer **2D** view |

Only one process may own the D405 at a time.

---

## 7. HTTP client smoke tests (no chassis motion)

**Device: robot** (Sim-06 server must be up). Preserve `NO_PROXY` / unset `http_proxy` as in §2.3.

### 7.1 Fake observation (optional, also valid on Sim-06 with `--host localhost`)

```bash
conda activate internnav
cd ~/InternNav
python scripts/smoke_test_fake_obs.py --host 192.168.110.35
```

**Why.** Validates `/agent/init` + `/agent/.../step` and DualVLN load without ROS.

**Expected.** `Agent init OK` and `Action returned: [{'action': [...], 'ideal_flag': ...}]`.

### 7.2 Real camera frame

Capture with **system** Python (ROS Jazzy), not conda — `rclpy` is built for system Python 3.12:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ARX5_ROBOT_1-main/realsense/install/setup.bash
/usr/bin/python3 ~/InternNav/scripts/capture_realsense_frame.py --out /tmp/internnav_obs.npz
```

Then send to Sim-06:

```bash
conda activate internnav
cd ~/InternNav
python scripts/smoke_test_fake_obs.py --host 192.168.110.35 \
  --npz /tmp/internnav_obs.npz \
  --instruction "go forward a little bit then stop"
```

**Why.** Proves the observation encoding path used by `AgentClient` (pickle + base64 over JSON) with real RGB-D.

---

## 8. Chassis bring-up

**Device: robot.**

```bash
cd ~/lift2s-ws
./quick_start.sh
# follow on-screen menus for your real-robot configuration
```

Confirm topics exist (together with camera topics):

```bash
ros2 topic list | grep -E 'cmd_vel|arx_lift/odom|camera_head'
```

**Expected.** At least `/cmd_vel` and `/arx_lift/odom`, plus `/camera_head/color/image_raw`.

**Why.** Lift2S hardware interface consumes `/cmd_vel` when chassis mode allows velocity commands (typically `chassis_mode=1`). WBC/VR may also publish `/cmd_vel`; keep arms idle / HOLD during DualVLN driving tests.

Manual sanity check (e-stop ready):

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.4}}"
# Ctrl+C, then stop:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

---

## 9. Closed-loop bridge (camera → DualVLN → `/cmd_vel`)

**Device: robot.** Keep Sim-06 server, Zenoh, RealSense, and `lift2s-ws` running.

The bridge script uses **system Python** (`rclpy` + `requests`) and talks to Sim-06 over HTTP (`trust_env=False` so LAN is not proxied).

### 9.1 Dry-run (required first)

```bash
source /opt/ros/jazzy/setup.bash
source ~/ARX5_ROBOT_1-main/realsense/install/setup.bash

/usr/bin/python3 ~/InternNav/scripts/lift2s_internnav_bridge.py \
  --host 192.168.110.35 \
  --instruction "go forward a little bit then stop" \
  --steps 5
```

**Why.** Prints discrete actions without publishing `/cmd_vel`.

### 9.2 Enable motion

Clear space; operator on e-stop:

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

| Flag | Role |
|------|------|
| `--enable-motion` | Actually publish `/cmd_vel` |
| `--v-lin` / `--v-ang` | Linear / angular speed scales |
| `--pulse` | Duration each action is applied |
| `--period` | Spacing between inference steps |

**Success criterion for bring-up.** Live RGB-D → remote DualVLN → base moves consistently with printed actions (e.g. FORWARD pulses).

### 9.3 Language / object goals (expectations)

DualVLN can attempt goals such as `"go to the black tumbler then stop"` when the object is **large and centered** in `/camera_head/color/image_raw` (monitor with `rqt_image_view`). It is **not** a reliable “spin until you find X” controller, and it does **not** grasp objects. D405 is short-range; room-scale search is limited.

---

## 10. Recommended terminal layout

| Terminal | Machine | Command / role |
|----------|---------|----------------|
| 1 | Sim-06 | `start_server.py --host 0.0.0.0 --port 8087` |
| 2 | Robot | `rmw_zenohd` |
| 3 | Robot | RealSense `ros2 launch ...` |
| 4 | Robot | `lift2s-ws` `./quick_start.sh` |
| 5 | Robot | Bridge / smoke tests / `rqt_image_view` |

Startup order: **Sim-06 server** → **Zenoh** → **camera** → **chassis** → **bridge**.

---

## 11. Troubleshooting

| Symptom | Likely cause | Mitigation |
|---------|--------------|------------|
| `curl` empty or `http_code=502` | HTTP proxy intercepting LAN | `--noproxy '*'`, `NO_PROXY`, unset `http_proxy` |
| `Unable to connect to a Zenoh router` | Camera started before `rmw_zenohd` | Start Zenoh first; restart camera |
| No `/camera_head` topics | Camera node down or wrong domain | Check launch, `ROS_DOMAIN_ID`, Zenoh |
| Frame timeouts / no RGB | USB 2.1 | Move to USB 3 port; confirm `5000M` |
| `realsense-viewer` blank in Cursor | No `DISPLAY` | Use NoMachine/desktop; `DISPLAY=:0` |
| Viewer vs ROS conflict | Two owners of D405 | Only one at a time |
| `ModuleNotFoundError: quaternion` on robot | Importing full `AgentServer` stack | Use `comm_utils.client` + lazy `utils/__init__.py` |
| Smoke script `No such file` | Custom scripts removed by `git clean` / re-clone | Restore scripts from this guide’s backup |
| DualVLN always `STOP` / ignores “turn right” | Policy / instruction mismatch | Dry-run; simplify goals; aim camera; use manual `/cmd_vel` to point |
| Motion too small | Conservative bridge defaults | Increase `--v-lin`, `--v-ang`, `--pulse` |
| `/cmd_vel` published but no motion | Chassis mode / WBC ownership | Confirm `chassis_mode=1`, idle VR/WBC |

---

## 12. Security and safety

- Always keep a hardware e-stop within reach when `--enable-motion` is set.
- Start with dry-run and low speeds.
- DualVLN is research software; do not treat outputs as collision-free navigation.
- Exposing `0.0.0.0:8087` on a lab LAN is convenient; do not expose it to the public Internet without authentication and firewall controls.

---

## 13. What this bring-up achieved (checklist)

- [x] InternNav installed; DualVLN checkpoint on Sim-06  
- [x] Agent Service listening on `:8087`  
- [x] Robot reaches OpenAPI (`http_code=200`)  
- [x] Zenoh + RealSense RGB-D on Lift2S  
- [x] Fake and real-frame HTTP `step` from robot  
- [x] `lift2s-ws` topics including `/cmd_vel`  
- [x] Dry-run and motion-enabled bridge  

**Not in scope yet:** continuous until-STOP policies, map-based navigation, arm pick-and-place, production obstacle avoidance.

---

## 14. Reference: communication sequence

1. Bridge (or smoke client) `POST /agent/init` with `AgentCfg` / `model_settings` (including `model_path` on the **server** filesystem, `device: cuda:0`, camera intrinsics, image size).
2. Server loads DualVLN into GPU memory (shard load visible in Sim-06 logs) and returns `agent_name` (e.g. `internvla_n1`).
3. Client builds observation dict `{rgb, depth, instruction}`, pickles and base64-encodes it, `POST /agent/{agent_name}/step`.
4. Server runs DualVLN and returns JSON actions.
5. Bridge maps the first discrete code to a short `geometry_msgs/Twist` pulse on `/cmd_vel` (if `--enable-motion`).
6. Lift2S HI converts `/cmd_vel` to chassis commands when enabled.

ROS 2 never crosses the robot↔Sim-06 boundary in this design; only HTTP does.

---

## 15. Document history

| Item | Value |
|------|--------|
| Platform | ARX Lift2S + InternNav DualVLN |
| ROS | Jazzy + `rmw_zenoh_cpp` |
| Camera | Intel RealSense D405 @ 640×480 |
| Inference host | Remote Sim-06 Agent Service `:8087` |
| Guide status | Validated through closed-loop `/cmd_vel` motion tests |

Maintainers should update example IPs, serial numbers, and Torch/CUDA pin versions when the lab environment changes.
