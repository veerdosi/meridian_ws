# Meridian WS

ROS 2 + MuJoCo workspace for UR5e compliance control and USB-C insertion task.

**Stack:** Ubuntu 24.04 (Docker), ROS 2 Jazzy, Python 3.12, MuJoCo 3.7+

---

## Workspace Structure

```
meridian_ws/
├── Dockerfile
├── entrypoint.sh
├── run_setup.sh
└── src/
    ├── meridian_description/          # URDF/MJCF assets (ament_cmake)
    │   ├── assets/
    │   │   ├── ur5e.urdf              # generated from xacro
    │   │   └── ur5e.xml              # augmented MJCF (F/T sensor, fingertip, table, USB-C target, actuators)
    │   ├── CMakeLists.txt
    │   └── package.xml
    ├── meridian_control/              # custom messages + compliance controller (ament_cmake)
    │   ├── meridian_control/
    │   │   ├── __init__.py
    │   │   └── compliance_controller.py   # 5-state force-compliant insertion controller
    │   ├── msg/
    │   │   ├── ExecutionOutcome.msg
    │   │   └── ComplianceState.msg
    │   ├── CMakeLists.txt
    │   └── package.xml
    ├── meridian_sensors/              # F/T sensor node (ament_python)
    │   ├── meridian_sensors/
    │   │   ├── __init__.py
    │   │   └── ft_sensor_node.py
    │   ├── node_scripts/
    │   │   └── ft_sensor_node         # libexec wrapper
    │   ├── setup.py
    │   └── package.xml
    ├── meridian_sim/                  # MuJoCo sim node + launch (ament_python)
    │   ├── meridian_sim/
    │   │   ├── __init__.py
    │   │   └── mujoco_sim_node.py
    │   ├── node_scripts/
    │   │   └── mujoco_sim_node        # libexec wrapper
    │   ├── launch/
    │   │   └── sim_sensors.launch.py
    │   ├── scripts/
    │   │   ├── convert_urdf_to_mjcf.py
    │   │   ├── augment_mjcf.py
    │   │   └── test_mjcf.py
    │   ├── setup.py
    │   └── package.xml
    └── meridian_annotation/           # VLM annotation pipeline (ament_python)
        ├── meridian_annotation/
        │   └── __init__.py
        ├── scripts/
        │   └── annotate_episode.py   # reads bag, renders Fz trace, calls VLM
        ├── setup.py
        └── package.xml
```

---

## Docker Setup

### Build the image

```bash
docker build -t meridian:latest /Users/veerdosi/Documents/code/github/meridian_ws
```

### Run the container

```bash
docker run -it --name meridian_dev --platform linux/arm64 \
  -v /Users/veerdosi/Documents/code/github/meridian_ws:/root/meridian_ws \
  -p 5900:5900 \
  -p 8765:8765 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  meridian:latest
```

The entrypoint automatically starts:
- `Xvfb :99` — virtual display (1920×1080)
- `x11vnc` — VNC server on port 5900, password: `meridian`

### Connect the VNC viewer (macOS)

Finder → Go → Connect to Server → `vnc://localhost:5900`  
Password: `meridian`

XQuartz is **not** needed.

---

## First-Time Setup (inside container)

```bash
cd ~/meridian_ws
bash run_setup.sh
```

This runs all 5 setup steps:

| Step | What it does |
|------|-------------|
| 1 | `colcon build` — builds all 4 ROS 2 packages |
| 2 | Generates `ur5e.urdf` from xacro |
| 3 | Converts URDF → MJCF via MuJoCo |
| 4 | Augments MJCF with F/T sensor, fingertip, table, USB-C target |
| 5 | Runs headless verification (6 joints, F/T readings) |

After setup, rebuild to pick up the new ROS 2 nodes:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/meridian_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Launch Files

### sim_sensors.launch.py — Full simulation pipeline

```bash
source ~/meridian_ws/install/setup.bash
ros2 launch meridian_sim sim_sensors.launch.py
```

Starts two nodes:
- `mujoco_sim_node` — MuJoCo sim at 1 kHz, publishes joint states + raw F/T
- `ft_sensor_node` — reads raw_sim, adds Gaussian noise, applies Butterworth LPF, republishes

---

## Demo Playbook

Exact commands in order — reproducible in under 2 minutes.

### 1. Start container

```bash
docker rm meridian_dev 2>/dev/null || true
docker run -it --name meridian_dev --platform linux/arm64 \
  -v /Users/veerdosi/Documents/code/github/meridian_ws:/root/meridian_ws \
  -p 5900:5900 \
  -p 8765:8765 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  meridian:latest
```

### 2. Build workspace (first time or after changes)

Inside container:
```bash
cd ~/meridian_ws && bash run_setup.sh
```

### 3. Launch full stack in demo mode

Inside container:
```bash
source ~/meridian_ws/install/setup.bash
ros2 launch meridian_sim sim_sensors.launch.py demo_mode:=true
```

### 4. Connect Foxglove Desktop

- Open Foxglove Desktop on Mac
- Click **Open connection** → **Rosbridge WebSocket**
- URL: `ws://localhost:8765`
- Click **Open**

### 5. Load layout

In Foxglove Desktop: **Layout** → **Import from file** → select
`/Users/veerdosi/Documents/code/github/meridian_ws/foxglove/meridian_demo.json`

### 6. Watch episodes run

The robot runs 10 episodes automatically. Episode 2 produces a scripted FAILURE (over_force).
Watch the state panel cycle through: `APPROACH → PRE_CONTACT → CONTACT_ACTIVE → SEATED/FAILURE`.

### 7. Run annotation on episode 2

After the batch completes, find the latest bag:
```bash
ls ~/meridian_ws/bags/
# e.g.: 20240101_120000
```

Run annotation:
```bash
source ~/meridian_ws/install/setup.bash
python3 ~/meridian_ws/src/meridian_annotation/scripts/annotate_episode.py \
  --bag ~/meridian_ws/bags/<YYYYMMDD_HHMMSS> \
  --episode 2
```

Without `ANTHROPIC_API_KEY` set, this saves `episode_2_ft_trace.png` in the current directory
and exits cleanly. Set the key to enable the VLM call:
```bash
export ANTHROPIC_API_KEY=sk-ant-...
python3 ~/meridian_ws/src/meridian_annotation/scripts/annotate_episode.py \
  --bag ~/meridian_ws/bags/<YYYYMMDD_HHMMSS> \
  --episode 2
```

---

## Foxglove Layout

The layout file `foxglove/meridian_demo.json` contains a 3-panel layout:
- **3D panel** (left, large) — robot model following `/joint_states`
- **Fz plot** (top right) — raw vs filtered insertion force
- **State text** (bottom right) — current compliance controller state

**To load:** Foxglove Desktop → Layout → Import from file → select `foxglove/meridian_demo.json`.

Additional panels (full wrench, outcome log) can be added manually from the Foxglove panel picker.

---

## Annotation Pipeline

`src/meridian_annotation/scripts/annotate_episode.py` reads a rosbag2 bag and annotates
a specific episode using the Anthropic VLM.

**Required env var:** `ANTHROPIC_API_KEY` — if absent, the script saves the plot and exits cleanly.

**Usage:**
```bash
source ~/meridian_ws/install/setup.bash
python3 src/meridian_annotation/scripts/annotate_episode.py \
  --bag ~/meridian_ws/bags/<YYYYMMDD_HHMMSS> \
  --episode <N>
```

**Outputs (in current working directory):**
- `episode_N_ft_trace.png` — Fz trace plot with red abort/seat line
- `episode_N_annotation.json` — VLM annotation JSON (only if API key is set)

## Replay

To replay any recorded bag into Foxglove Desktop:

```bash
source ~/meridian_ws/install/setup.bash
ros2 launch meridian_sim replay.launch.py bag:=~/meridian_ws/bags/20240101_120000
```

Then connect Foxglove Desktop to `ws://localhost:8765`. The bag plays back in real time with
clock synchronization (`--clock` flag).

---

## ROS 2 Nodes

### mujoco_sim_node (`meridian_sim`)

| Item | Detail |
|------|--------|
| Package | `meridian_sim` |
| Executable | `ros2 run meridian_sim mujoco_sim_node` |
| Sim rate | 1 kHz in background thread, publishes at 200 Hz |

**Parameters**

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `mjcf_path` | string | — (required) | Absolute path to `ur5e.xml` |

**Publications**

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | 200 Hz | Positions, velocities, efforts for all 6 UR5e joints |
| `/ft_sensor/raw_sim` | `geometry_msgs/WrenchStamped` | 200 Hz | Raw F/T from MuJoCo sensordata |

**Subscriptions**

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_commands` | `std_msgs/Float64MultiArray` | 6-element target positions written to `mjdata.ctrl` |

**Services**

| Service | Type | Description |
|---------|------|-------------|
| `~/reset` | `std_srvs/Empty` | Resets to a random near-target pose (fingertip ±2 mm XY from `target_site`) using Jacobian IK |

---

### ft_sensor_node (`meridian_sensors`)

| Item | Detail |
|------|--------|
| Package | `meridian_sensors` |
| Executable | `ros2 run meridian_sensors ft_sensor_node` |
| Callback rate | Driven by `/ft_sensor/raw_sim` subscription (up to 1 kHz) |

**Parameters**

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `use_sim` | bool | `true` | Subscribe to `/ft_sensor/raw_sim`; if false, logs a warning (hardware stub) |
| `add_noise` | bool | `false` | Apply Gaussian noise before filtering |
| `noise_std_force` | float | `0.1` | Force noise σ (N) |
| `noise_std_torque` | float | `0.01` | Torque noise σ (Nm) |
| `cutoff_hz` | float | `200.0` | Butterworth LPF cutoff frequency (Hz) |

**Publications**

| Topic | Type | Description |
|-------|------|-------------|
| `/ft_sensor/raw` | `geometry_msgs/WrenchStamped` | Post-noise, pre-filter wrench |
| `/ft_sensor/filtered` | `geometry_msgs/WrenchStamped` | 2nd-order Butterworth filtered wrench |

**Subscriptions**

| Topic | Type | Description |
|-------|------|-------------|
| `/ft_sensor/raw_sim` | `geometry_msgs/WrenchStamped` | MuJoCo raw F/T (when `use_sim:=true`) |

---

## Topics

| Topic | Type | Publisher | Rate | Notes |
|-------|------|-----------|------|-------|
| `/joint_states` | `sensor_msgs/JointState` | `mujoco_sim_node` | 200 Hz | 6 joints: positions, velocities, efforts |
| `/ft_sensor/raw_sim` | `geometry_msgs/WrenchStamped` | `mujoco_sim_node` | 200 Hz | MuJoCo sensordata, frame `ft_sensor_site` |
| `/ft_sensor/raw` | `geometry_msgs/WrenchStamped` | `ft_sensor_node` | 200 Hz | Noisy wrench (post-noise, pre-filter) |
| `/ft_sensor/filtered` | `geometry_msgs/WrenchStamped` | `ft_sensor_node` | 200 Hz | Butterworth LPF output |
| `/joint_commands` | `std_msgs/Float64MultiArray` | external | any | 6 position targets → `mjdata.ctrl[:6]` |

---

## Custom Messages (`meridian_control`)

Build with `ament_cmake` + `rosidl`. Import after sourcing the workspace:

```python
from meridian_control.msg import ExecutionOutcome, ComplianceState
```

### ExecutionOutcome.msg

```
bool success
string failure_mode
float64 peak_force
float64 insertion_distance
float64 duration_sec
string profile_id
```

### ComplianceState.msg

```
string state
string active_profile_id
float64 fz_filtered
float64 insertion_distance
```

---

## Viewing the Simulation

With the container running and VNC connected:

```bash
python3 src/meridian_sim/scripts/test_mjcf.py
```

The MuJoCo passive viewer opens on the VNC display for 5 seconds.

---

## MJCF Model Details (`ur5e.xml`)

| Element | Description |
|---------|-------------|
| 6 joints | shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3 |
| `ft_sensor_site` | Site on wrist_3_link at flange offset [0, 0, 0.0823] |
| `ft_force` / `ft_torque` | Force + torque sensors on `ft_sensor_site`; sensordata[0:3] / [3:6] |
| `fingertip` | Cylinder geom (r=0.01, l=0.02) at flange; collision enabled |
| `table` | Flat box, top face at z=0; collision enabled |
| `usbc_target` | 15×7×10 mm box at [0.4, 0, 0] with `target_site` |
| Mesh geoms | All 7 structural STL meshes have `contype="0" conaffinity="0"` (visual only, prevents self-collision that would lock joints) |
| 6 position actuators | `shoulder_pan_act` … `wrist_3_act`; kp=500 (pan/lift/elbow), kp=200 (wrists); forcerange matches UR5e spec |

---

## Status

- [x] Step 1 — Workspace scaffold + colcon build
- [x] Step 2 — UR5e URDF generation
- [x] Step 3 — URDF → MJCF conversion
- [x] Step 4 — MJCF augmentation (F/T sensor, fingertip, table, target)
- [x] Step 5 — Simulation verified (6 joints, non-zero F/T readings)
- [x] Step 6 — Custom messages (`ExecutionOutcome`, `ComplianceState` in `meridian_control`)
- [x] Step 7 — F/T sensor ROS 2 node (`ft_sensor_node` — noise, Butterworth LPF, 1 kHz)
- [x] Step 8 — MuJoCo sim node (`mujoco_sim_node` — 1 kHz sim thread, joint states, F/T, reset service)
- [x] Step 9 — Compliance controller (`compliance_controller` — 5-state machine, Jacobian IK, 7/10 SEATED in 10-episode batch)
- [x] Step 10 — Foxglove bridge, rosbag2 recording, demo mode, annotation pipeline

---

## Step 9 — Compliance Controller

### Overview

`compliance_controller` implements a force-compliant insertion state machine for the USB-C peg-in-hole task. It drives the UR5e fingertip from above the target, detects contact, and uses impedance control to seat the peg within force and displacement bounds.

### State Machine

```
IDLE → APPROACH → PRE_CONTACT → CONTACT_ACTIVE → SEATED / FAILURE → RETRACT → IDLE
```

| State | Action |
|-------|--------|
| `IDLE` | Waits for first sensor message, then transitions to APPROACH |
| `APPROACH` | Jacobian IK drives fingertip to `target_position` + `approach_height` |
| `PRE_CONTACT` | Descends at 10 mm/s; detects contact when `|fz| > contact_threshold_n` |
| `CONTACT_ACTIVE` | Impedance loop: XY position error → joint delta via Jacobian; Z force-controlled toward `target_insertion_force_n` |
| `SEATED` | Declared when `seating_force_min_n ≤ peak_fz ≤ seating_force_max_n` and `displacement ≥ seating_displacement_m` |
| `RETRACT` | Jacobian IK lifts fingertip back to approach height; calls `~/reset` for next episode |

Abort conditions checked in every state: `|fz| > abort_force_ceiling_n` or any torque component `> abort_torque_ceiling_nm`.

### Running

```bash
source ~/meridian_ws/install/setup.bash
ros2 launch meridian_sim sim_sensors.launch.py
```

The launch file starts `mujoco_sim_node`, `ft_sensor_node`, and `compliance_controller` together.

### Parameters (`compliance_params.yaml`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `target_position` | `[0.4, 0.0, 0.01]` | XYZ of the USB-C target (m) |
| `approach_height` | `0.15` | Z height to hover above target (m) |
| `contact_threshold_n` | `5.0` | Fz threshold to enter CONTACT_ACTIVE (N) |
| `seating_force_min_n` | `1.0` | Min Fz to declare SEATED (N) |
| `seating_force_max_n` | `20.0` | Max Fz before abort (N) |
| `seating_displacement_m` | `0.001` | Min Z displacement to declare SEATED (m) |
| `abort_force_ceiling_n` | `50.0` | \|Fz\| abort threshold (N) |
| `abort_torque_ceiling_nm` | `30.0` | Max torque component abort threshold (Nm) |
| `contact_timeout_sec` | `30.0` | Timeout in CONTACT_ACTIVE before FAILURE (s) |
| `kp_xy` | `50.0` | XY position gain for impedance control |
| `target_insertion_force_n` | `5.0` | Target Fz during insertion (N) |
| `force_gain` | `0.0001` | Z force error → Z velocity gain |
| `max_episodes` | `10` | Number of insertion attempts per batch |

### Topics and Services

| Topic / Service | Type | Direction | Description |
|-----------------|------|-----------|-------------|
| `/ft_sensor/filtered` | `WrenchStamped` | sub | Filtered F/T from `ft_sensor_node` |
| `/ft_sensor_site_pose` | `PoseStamped` | sub | Fingertip pose from `mujoco_sim_node` |
| `/jacobian` | `Float64MultiArray` | sub | 6×6 Jacobian from `mujoco_sim_node` |
| `/joint_states` | `JointState` | sub | Joint positions/velocities |
| `/joint_commands` | `Float64MultiArray` | pub | 6-DOF position commands |
| `/compliance_controller/state` | `ComplianceState` | pub | Current state + Fz + displacement |
| `/execution_outcome` | `ExecutionOutcome` | pub | Per-episode result (SEATED / FAILURE) |
| `mujoco_sim_node/reset` | `std_srvs/Empty` | client | Resets sim to randomised near-target pose |

### Sim Tuning Notes

- **Gravity disabled** (`<option gravity="0 0 0"/>` in `ur5e.xml`): pure-P actuators without gravity allows stable hovering; reactive contact forces are still physical.
- **Velocity damping added** to all actuators (`kv="50"` shoulder/elbow, `kv="20"` wrists): prevents oscillation during fast Jacobian IK motions.
- **Jacobian IK direction preservation**: joint deltas are scaled by `_scale_dq()` (uniform scaling) rather than per-element clip, which would distort the intended Cartesian direction.
- **Abort uses `|fz|` only**: MuJoCo's FT sensor includes structural constraint forces; large lateral forces from arm dynamics are filtered out by checking only the Z axis for abort.
