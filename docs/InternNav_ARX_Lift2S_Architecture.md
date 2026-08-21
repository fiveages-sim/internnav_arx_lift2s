# How InternNav Works on the ARX Lift2S Robot

**Audience.** Engineers who need a conceptual understanding of sensing, ROS 2 / Zenoh, InternNav model variants, and the Lift2S-specific glue code.

**Related docs.**

- Bring-up commands: [`InternNav_ARX_Lift2S_Implementation_Guide.md`](./InternNav_ARX_Lift2S_Implementation_Guide.md)
- Upstream: [InternRobotics/InternNav](https://github.com/InternRobotics/InternNav)
- Papers: [InternVLA-N1 technical report](https://internrobotics.github.io/internvla-n1.github.io/), [DualVLN (arXiv:2512.08186)](https://arxiv.org/abs/2512.08186)

**Validated layout.** Lift2S onboard PC (`arx`) runs camera + chassis ROS; DualVLN inference runs on **Sim-06** over HTTP `:8087`.

---

## 1. How the RealSense D405 works and how it connects to the robot

### 1.1 What the D405 is

The Intel RealSense **D405** is a short-range stereo depth camera. It exposes:

- **Color (RGB)** — typically RGB8 frames (e.g. 640×480 @ 30 Hz)
- **Depth** — typically 16-bit depth in millimeters (Z16)
- Optional infrared streams (not required for the DualVLN RGB path)

Physically it connects to the Lift2S host PC over **USB**. For reliable RGB+depth streaming you need **USB 3.x** (kernel often reports `5000M`). USB 2.1 (`480M`) frequently causes frame timeouts.

On this robot the device appears as something like:

```text
Bus 004 Device xxx: ID 8086:0b5b Intel RealSense Depth Camera 405
Serial (example): 323622271380
```

Software stack on the robot:

1. **librealsense** — low-level SDK talking to the USB device
2. **`realsense2_camera`** ROS 2 node — wraps librealsense and publishes ROS topics
3. Optional **`realsense-viewer`** — GUI only; **must not** run at the same time as the ROS node (one process owns the camera)

`~/librealsense` on the robot is the SDK tree; the ROS wrapper is typically under a workspace such as `~/ARX5_ROBOT_1-main/realsense`. Chassis bring-up lives in `~/lift2s-ws` (`quick_start.sh`).

### 1.2 Connection path (hardware → ROS topics)

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

Typical topics after launch (names may use `camera_head` depending on launch args):

| Topic | Type | Content |
|-------|------|---------|
| `/camera_head/color/image_raw` | `sensor_msgs/msg/Image` | RGB pixels |
| `/camera_head/depth/image_raw` | `sensor_msgs/msg/Image` | Depth |
| `/camera_head/color/camera_info` | `sensor_msgs/msg/CameraInfo` | Intrinsics |

### 1.3 How ROS 2 nodes communicate (and where Zenoh fits)

ROS 2 is a **pub/sub** system. Nodes do not call each other like a REST API. A publisher writes messages on a **named topic**; any number of subscribers with a matching type and compatible QoS can receive them.

On this Lift2S setup:

| Piece | Role |
|-------|------|
| `RMW_IMPLEMENTATION=rmw_zenoh_cpp` | Tells ROS 2 to use Zenoh as the transport under the hood |
| `ros2 run rmw_zenoh_cpp rmw_zenohd` | Zenoh **router** on the robot; peers discover each other through it |
| `ROS_DOMAIN_ID` | Must match across nodes that should see each other (often `0`) |

**Important clarification for InternNav on Lift2S:**

- Zenoh is used for **on-robot** ROS traffic (camera ↔ bridge ↔ chassis).
- DualVLN on Sim-06 does **not** join this Zenoh graph.
- Robot ↔ Sim-06 uses **HTTP**, not ROS.

So: “D405 communicates through ROS 2 Zenoh” is correct **inside the robot**. It is **not** how DualVLN on Sim-06 receives images.

### 1.4 How the robot “understands” / receives the RGB topic

Nothing magical “understands” RGB by itself. A receiving node:

1. Creates a **subscription** to `/camera_head/color/image_raw` with type `sensor_msgs/msg/Image`.
2. The middleware delivers each published message to a **callback**.
3. The callback reads `msg.height`, `msg.width`, `msg.encoding`, and `msg.data` (raw bytes).
4. Application code reshapes bytes into a NumPy array (e.g. `H×W×3` `uint8` for `rgb8`).

Example (conceptual, as in our bridge):

```python
# Subscriber callback receives sensor_msgs/Image
rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
```

`rqt_image_view` does the same for display. Our `lift2s_internnav_bridge.py` does it to build the observation sent to DualVLN.

**Who publishes?** Only the RealSense ROS node (while launched).  
**Who receives?** Any local subscriber (bridge, `rqt`, debug tools). The chassis does **not** need RGB to wheel the base; only the navigation client does.

---

## 2. InternNav models: VLA / VLN — how many, and what DualVLN is

### 2.1 InternNav is a platform, not “3 VLAs + 3 VLNs”

[InternNav](https://github.com/InternRobotics/InternNav) is an **open platform** for navigation foundation models: datasets, training, evaluation (Habitat / Isaac), baselines, and real-world helpers.

The README groups supported methods roughly as:

| Family | Examples | Role |
|--------|----------|------|
| Classical / baseline **VLN** agents | Seq2Seq, CMA, RDP, StreamVLN (coming) | Vision-Language **Navigation** baselines |
| Local / geometric / diffusion navigators | DD-PPO, iPlanner, GNM, ViNT, NoMad, **NavDP** | Often treated as **System 1**-style local policies |
| **InternVLA-N1** dual-system configs | Decoupled S2+S1, Dual System w/ NavDP*, **DualVLN** | Main **navigation VLA** line |

So it is **incorrect** to say “InternNav has exactly 3 VLAs and 3 VLNs.” More accurately:

- There are **many** VLN / navigation baselines.
- **InternVLA-N1** is the flagship **VLA-for-navigation** model family.
- The three bullets you listed are **three deployment / training configurations of InternVLA-N1’s dual-system design**, not three unrelated VLAs plus three unrelated VLNs.

### 2.2 What the three InternVLA-N1 lines mean

InternVLA-N1 uses a **dual-system** idea (see the InternVLA-N1 report / DualVLN paper):

| Component | Typical job |
|-----------|-------------|
| **System 2** | Slower, language-conditioned “where should I go?” / latent plan from vision + instruction |
| **System 1** | Faster local execution / low-level motion (e.g. NavDP-style), often RGB-D |

Your three README entries:

1. **InternVLA-N1 (System 2) + Decoupled System 1**  
   System 2 paired with a separately trained / separately run System 1 (e.g. NavDP as local policy), not necessarily jointly fine-tuned as one DualVLN package.

2. **InternVLA-N1 (Dual System) w/ NavDP\***  
   Dual system where System 1 is **NavDP\*** (jointly tuned with System 2). Observation often **RGB-D**.

3. **InternVLA-N1 (Dual System) DualVLN** ← **what we run on Lift2S**  
   Dual-system DualVLN variant; strong **RGB** VLN performance on benchmarks. This is the checkpoint served on Sim-06.

**On Lift2S we use DualVLN only** (remote Agent Service). We are not running all three stacks at once.

### 2.3 How InternNav “works” in general

At a high level (sim or real):

1. Build an **observation**: images (± depth), pose if available, and a **language instruction**.
2. Call the **agent** `step(observation)`.
3. Receive **actions** (discrete Habitat-style codes and/or continuous trajectories, depending on config).
4. The **environment** (Habitat sim, or a real robot wrapper) executes those actions.

Official real-world demos often wrap Unitree Go2. Lift2S needs a **custom wrapper**: our HTTP bridge + `/cmd_vel` mapping.

---

## 3. Does InternNav receive D405 RGB topics? Does it publish `/cmd_vel`?

### 3.1 Short answers for *this* Lift2S deployment

| Question | Answer |
|----------|--------|
| Does upstream DualVLN subscribe to `/camera_head/color/image_raw`? | **No.** |
| Does DualVLN on Sim-06 see ROS topics? | **No.** It only sees HTTP request bodies. |
| How do RGB frames reach DualVLN? | Robot bridge **subscribes** in ROS → packs NumPy → **HTTP** `POST /agent/.../step`. |
| Does InternNav / DualVLN publish `/cmd_vel`? | **No.** |
| Who publishes `/cmd_vel`? | Our **`lift2s_internnav_bridge.py`** (when `--enable-motion`), mapping discrete actions to `geometry_msgs/Twist`. |

### 3.2 End-to-end data path on Lift2S

```
D405 USB
  → realsense2_camera (pub RGB/Depth)
  → Zenoh ROS graph (robot-local)
  → lift2s_internnav_bridge (sub Image)
  → HTTP JSON (pickle+base64 observation) → Sim-06 Agent Server
  → DualVLN GPU inference
  → HTTP JSON action codes
  → bridge maps 0/1/2/3 → Twist
  → /cmd_vel (ROS, Zenoh)
  → Lift2S chassis hardware interface
```

### 3.3 How motion is decided (forward / left / right / stop)

DualVLN does **not** output English words like “forward”. It outputs **discrete action IDs** (Habitat-style), for example:

| Code | Meaning | Bridge → `/cmd_vel` (typical) |
|------|---------|--------------------------------|
| `0` | STOP | all zeros |
| `1` | FORWARD | `linear.x > 0` |
| `2` | TURN_LEFT | `angular.z > 0` |
| `3` | TURN_RIGHT | `angular.z < 0` |
| `5` | LOOK_DOWN (special) | usually no base motion |
| `-1` | treat as no-op / stop | zeros |

**There is no native “backward”** in this discrete set used by our bridge. Backward would require a different action space or a custom mapping.

**How DualVLN chooses the code:** neural inference conditioned on recent RGB (and depth if provided), instruction text, and internal dual-system state / history. It is a learned VLN policy, not a hard-coded “if bottle on left then turn” rule. Literal instructions like `"turn right"` may still yield FORWARD or STOP depending on the image and training distribution.

The bridge then applies a short **velocity pulse** (`--v-lin`, `--v-ang`, `--pulse`) for non-stop actions when `--enable-motion` is set.

---

## 4. Custom scripts created for Lift2S (paths and roles)

Upstream InternNav does **not** ship an ARX Lift2S driver. These files were added during bring-up. **Back them up**; `git clean` / re-clone can delete them.

| Path | Purpose | Why needed |
|------|---------|------------|
| `~/InternNav/scripts/smoke_test_fake_obs.py` | HTTP smoke test: init DualVLN agent on Sim-06 and call `step` with a synthetic RGB-D frame, or with `--npz` from a real capture | Proves LAN + Agent Service + checkpoint load **without** chassis motion |
| `~/InternNav/scripts/capture_realsense_frame.py` | ROS subscriber (system Python) that waits for one color + depth message and writes `/tmp/internnav_obs.npz` | Separates ROS/`rclpy` (system Python 3.12) from conda InternNav client; supplies real pixels for smoke tests |
| `~/InternNav/scripts/lift2s_internnav_bridge.py` | Live loop: subscribe camera → HTTP DualVLN → print actions; optional `--enable-motion` publishes `/cmd_vel` | **Closed-loop** navigation glue for Lift2S; DualVLN never speaks ROS itself |
| `~/InternNav/internnav/utils/__init__.py` (lazy `AgentServer`) | Export `AgentClient` immediately; load `AgentServer` only when requested | Lets the robot import the thin HTTP client without pulling Habitat / `quaternion` / full server deps |

### 4.1 Which interpreter to use

| Script | Interpreter |
|--------|-------------|
| `smoke_test_fake_obs.py` | `conda activate internnav` |
| `capture_realsense_frame.py` | `/usr/bin/python3` + sourced ROS |
| `lift2s_internnav_bridge.py` | `/usr/bin/python3` + sourced ROS |
| `scripts/eval/start_server.py` | `conda` on **Sim-06 only** |

### 4.2 Minimal mental model of each script

**Smoke test** — “Can Sim-06 answer `/agent/init` and `/agent/.../step`?”

**Capture** — “Can ROS deliver one RGB-D pair from D405 into a NumPy file?”

**Bridge** — “Can we repeatedly turn live camera frames into DualVLN actions and (optionally) drive `/cmd_vel`?”

**Lazy utils init** — “Can the robot run the client without installing the full sim/server stack?”

---

## 5. Mapping your terminal / workspace notes

From the robot host layout (`~/lift2s-ws`, `~/librealsense`, `~/InternNav`):

| Path | Role in this story |
|------|--------------------|
| `~/librealsense` | SDK sources / install for D405 |
| RealSense ROS workspace (e.g. under `ARX5_ROBOT_1-main/realsense`) | Built `realsense2_camera` wrapper |
| `~/lift2s-ws` | Chassis / whole-body stack; provides `/cmd_vel` consumer (`quick_start.sh`) |
| `~/InternNav` | Client scripts + (on Sim-06) DualVLN server + weights |

`git status` showing dirty **submodules** under `lift2s-ws/src` is unrelated to DualVLN inference; it only means those nested repos have new commits checked out. InternNav↔camera communication does not go through those submodule commits.

---

## 6. FAQ summary

**How does D405 connect?** USB 3 → librealsense → `realsense2_camera` → ROS topics.

**How do nodes talk?** Pub/sub over ROS 2 with Zenoh RMW and a local `rmw_zenohd` router on the robot.

**How does the robot receive RGB?** Any node that **subscribes** to the color image topic gets `sensor_msgs/Image` callbacks and decodes bytes to an image array.

**Does DualVLN subscribe to that topic?** No. The **bridge** does, then sends arrays over HTTP.

**Does InternNav publish `/cmd_vel`?** Upstream DualVLN does not. Our **bridge** does when motion is enabled.

**Are the three InternVLA-N1 bullets “3 VLAs + 3 VLNs”?** No. They are three **InternVLA-N1 dual-system configurations**; InternNav also ships many other VLN/System-1 baselines. We use **DualVLN** only.

**How is forward/left/right/stop chosen?** DualVLN neural policy → discrete action ID → bridge velocity pulse. No native backward in our mapping.

---

## 7. Document history

| Item | Value |
|------|--------|
| Topic | Conceptual architecture of InternNav DualVLN on ARX Lift2S |
| Sensing | Intel RealSense D405 + ROS 2 Jazzy + Zenoh |
| Inference | Remote Agent Service on Sim-06 (`:8087`) |
| Control | Custom bridge → `/cmd_vel` |
| Upstream | [InternRobotics/InternNav](https://github.com/InternRobotics/InternNav) |
