# magpie_control

Autonomous grasping stack for the UR5 + MAGPIE gripper. Integrates ROS2 nodes, SAM3 vision, Gemini DeliGrasp descriptors, and adaptive slip detection into a single pipeline that detects an object by name and grasps it.

---

## What's in this repo

| Component | What it does |
|---|---|
| `gripper_node` | ROS2 node — Dynamixel AX-12 finger control, force/position services, 10 Hz state publisher |
| `ur5_node` | ROS2 node — UR5 RTDE wrapper, MoveLinear / teach-mode services |
| `ft_sensor_node` | ROS2 node — ATI F/T sensor via `netft_rdt_driver` |
| `tactile_sensor_node` | ROS2 node — tactile fingertip sensor |
| `sam3_infer.py` | SAM3 socket server — open-vocabulary segmentation over a Unix socket (subprocess, no ROS) |
| `pointcloud_utils.py` | Depth → world-frame point cloud, PCA grasp-angle and object-height estimation |
| `test_halfrun.py` | Detect + describe only (no arm movement) |
| `test_fullrun.py` | Full grasp pipeline: detect → describe → move → grasp (one-shot force) |
| `test_fullrun_slip.py` | Full grasp with **adaptive slip detection** — retightens if contact force is insufficient |

---

## Key changes from the original repo

### 1. ROSification — everything is a node

The original repo was a plain Python library. We added:

- **`ur5_node`** wraps `rtde_control` / `rtde_receive` into a proper ROS2 node so the arm can be controlled by any pipeline script via services (`/ur5/move_linear`, `/ur5/disable_teach_mode`, …).
- **`gripper_node`** already existed; we extended it with `/gripper/set_force` and verified the `/gripper/state` diagnostic flow (`temperature: 0.0` = node not talking to hardware).
- Camera frames and TCP pose are published as ROS topics so scripts can snapshot them without managing connections directly.

### 2. SAM3 open-vocabulary segmentation

`sam3_infer.py` runs SAM3 (Segment Anything Model 3) in a **separate subprocess** via a Unix socket (`/tmp/sam3.sock`). The main pipeline sends a JPEG + object name and receives back bounding boxes, labels, confidence scores, and a pixel mask. Using a subprocess bridges the SAM3 / transformers environment from the ROS2 Python environment without version conflicts.

- Requires: HuggingFace access token (`HF_TOKEN` env var) and SAM3 model access approved on HuggingFace.
- Hardware: runs on RTX 2070 (float32 patch applied — bf16 not supported on RTX 20xx).
- Start the server before running any pipeline script:
  ```bash
  python3 scripts/sam3_infer.py
  ```

### 3. Gemini DeliGrasp descriptor

Before moving the arm, the pipeline calls the Gemini API (`gemini-2.5-flash`) with a structured prompt to produce grasp parameters for the target object:

| Parameter | Meaning |
|---|---|
| `mass_g` | Estimated object mass (g) |
| `mu` | Friction coefficient |
| `k` | Object stiffness (N/m) |
| `aperture_mm` | Pre-close finger gap (mm) |
| `initial_force` | Minimum holding force (N) — physics |
| `add_force` | Force increment per slip retry (N) |
| `closure_mm` | Extra closure past contact (mm) |

Set `GEMINI_API_KEY` before running. If no `--object` is given the pipeline asks Gemini to identify the object from the camera image automatically.

### 4. Adaptive slip detection (`test_fullrun_slip.py`)

One-shot force control (as in `test_fullrun.py`) works for known objects but can fail on slippery or unexpectedly heavy items. `test_fullrun_slip.py` adds a retry loop:

1. Set gripper force to `max(initial_force, 5.0)` N (the 5 N floor overcomes Dynamixel mechanism friction).
2. Close gripper with `/gripper/close` (drives to contact, not a fixed position).
3. Read `/gripper/state` after a 3.5 s settle time.
4. If `measured_force < initial_force × 0.4`, declare slip and increment force by `add_force`, then retry.
5. Repeat up to `--slip-retries` times (default: 3). Cap at 16 N.

The key design insight: **`initial_force` (the holding physics estimate) is decoupled from `CLOSE_FORCE_N` (the movement force)**. A feather-light object might need only 0.5 N to hold but needs ≥ 5 N just to get the fingers moving.

### 5. Auto grasp-offset from PCA

`--grasp-offset` controls how far the fingertips descend below the object surface centroid before closing. Previously this was set manually. Now:

- After the initial detection, the SAM3 mask is back-projected to a world-frame point cloud via `build_segmented_pcd()`.
- PCA decomposes the cloud into three axes: `extent_m = [major, minor, normal]`.
- `normal` (index 2) is the smallest axis — the object's vertical height.
- `auto_offset = clamp(height / 2, 0.005, 0.04)` positions fingertips at the mid-height of the object.
- Pass `--grasp-offset <m>` explicitly to override.

---

## Quick start — full adaptive grasp

```bash
# 1. Start ROS2 nodes (three terminals)
ros2 run magpie_control ur5_node
ros2 run magpie_control gripper_node
ros2 run realsense2_camera realsense2_camera_node  # or equivalent camera node

# 2. Start SAM3 server (separate terminal, GPU env)
python3 scripts/sam3_infer.py

# 3. Run the pipeline
export GEMINI_API_KEY=your_key_here
source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
python3 scripts/test_fullrun_slip.py --object "red block" --socket
```

### CLI flags

| Flag | Default | Meaning |
|---|---|---|
| `--object TEXT` | auto-detect | Object name sent to SAM3 and Gemini |
| `--socket [PATH]` | `/tmp/sam3.sock` | Use SAM3 socket server |
| `--grasp-offset M` | auto (PCA) | Fingertip descent below centroid (m) |
| `--approach-height M` | `0.10` | Height above object for approach pose (m) |
| `--min-force N` | `0.0` | Override minimum holding force (N) |
| `--slip-retries N` | `3` | Max adaptive retighten attempts |
| `--dry-run` | off | Detect + compute poses only, no movement |

### Dry run (safe to test any time)

```bash
python3 scripts/test_fullrun_slip.py --object "red block" --socket --dry-run
```

---

## Installation

```bash
git clone https://github.com/correlllab/magpie_control.git
cd magpie_control
pip install . --user
```

Build ROS2 packages:

```bash
cd ~/ws_ctrl
colcon build --packages-select magpie_msgs magpie_control
source install/setup.bash
```

---

## ROS2 Gripper Node: Run and Control

If you see:

```bash
ros2 run magpie_control gripper_node
No executable found
```

the package is usually not built/sourced in your ROS2 workspace yet (or you are in a different shell that is not sourced).

If you see:

```bash
ModuleNotFoundError: No module named 'magpie_msgs'
```

`magpie_msgs` was not available in the current shell environment. Build both packages and source the workspace-wide setup file (not only a single package local setup).

### 1. Build and source from workspace root

From your ROS2 workspace root (example: `~/ws_ctrl`):

```bash
cd ~/ws_ctrl
colcon build --packages-select magpie_msgs magpie_control
source install/setup.bash
```

Optional sanity check:

```bash
ros2 pkg executables magpie_control
```

You should see at least:

- `magpie_control gripper_node`
- `magpie_control ft_sensor_node`
- `magpie_control tactile_sensor_node`

### 2. Run the gripper node

```bash
ros2 run magpie_control gripper_node
```

You can override parameters at startup:

```bash
ros2 run magpie_control gripper_node --ros-args \
	-p auto_detect_port:=true \
	-p port:=/dev/ttyUSB0 \
	-p default_speed:=100 \
	-p default_torque:=200
```

### 3. Control the gripper with ROS services

Open gripper:

```bash
ros2 service call /gripper/open std_srvs/srv/Trigger "{}"
```

Close gripper:

```bash
ros2 service call /gripper/close std_srvs/srv/Trigger "{}"
```

Set aperture/position (millimeters):

```bash
ros2 service call /gripper/set_position magpie_msgs/srv/SetGripperPosition "{position: 40.0, speed: 0.5}"
```

Set force limit (N):

```bash
ros2 service call /gripper/set_force magpie_msgs/srv/SetGripperForce "{max_force: 8.0}"
```

Calibrate:

```bash
ros2 service call /gripper/calibrate std_srvs/srv/Trigger "{}"
```

Reset parameters (torque, speed, compliance, and open pose):

```bash
ros2 service call /gripper/reset_parameters std_srvs/srv/Trigger "{}"
```

Monitor state:

```bash
ros2 topic echo /gripper/state
```

### 4. Full Gripper Node API (Units and Interfaces)

All gripper aperture/position values are in **millimeters (mm)**.

#### Startup parameters

- `auto_detect_port` (bool, default: `true`): auto-discover Dynamixel serial device.
- `port` (string, default: `/dev/ttyUSB0`): explicit serial device when auto-detect is disabled.
- `use_eflesh` (bool, default: `false`): enable eflesh sensor initialization.
- `default_speed` (int, default: `100`): initial Dynamixel moving speed setting.
- `default_torque` (int, default: `200`): initial Dynamixel torque limit.

#### Published topic

- Topic: `/gripper/state`
- Type: `magpie_msgs/msg/GripperState`
- Rate: 10 Hz
- Fields:
	- `position` (mm)
	- `finger_positions` (mm, `[right, left]`)
	- `force` (N)
	- `temperature` (°C) — **`0.0` means the node has lost hardware contact; restart the node**
	- `is_moving` (bool)
	- `contact_detected` (bool)

#### Services

- `/gripper/open` (`std_srvs/srv/Trigger`)
- `/gripper/close` (`std_srvs/srv/Trigger`)
- `/gripper/calibrate` (`std_srvs/srv/Trigger`)
- `/gripper/reset_parameters` (`std_srvs/srv/Trigger`)
- `/gripper/set_force` (`magpie_msgs/srv/SetGripperForce`):
	- request: `max_force` (N)
- `/gripper/set_position` (`magpie_msgs/srv/SetGripperPosition`):
	- request: `position` (mm), `speed` in [0.0, 1.0]
	- response: `actual_position` (mm), `success`, `message`

#### Action

- `/gripper/deligrasp` (`magpie_msgs/action/DeliGrasp`)
- goal params (`magpie_msgs/msg/DeliGraspParams`):
	- `goal_aperture` (mm)
	- `initial_force` (N)
	- `additional_closure` (mm)
	- `additional_force` (N)
	- `complete_grasp` (bool)
- result:
	- `final_aperture` (mm)
	- `final_force` (N)
	- `force_log` (N samples)

Example action call:

```bash
ros2 action send_goal /gripper/deligrasp magpie_msgs/action/DeliGrasp \
"{params: {goal_aperture: 35.0, initial_force: 1.5, additional_closure: 1.0, additional_force: 0.2, complete_grasp: true}}"
```

### 5. Common ROS2 troubleshooting

- Make sure your ROS distro is sourced before workspace setup:

```bash
source /opt/ros/humble/setup.bash
source ~/ws_ctrl/install/setup.bash
```

- Avoid sourcing only one package setup (for example `install/magpie_control/local_setup.bash`) when running `gripper_node`; that can omit runtime dependencies such as `magpie_msgs`.

- Verify package visibility:

```bash
ros2 pkg list | grep magpie_control
```

- If executables are still missing, rebuild cleanly:

```bash
cd ~/ws_ctrl
rm -rf build/magpie_control install/magpie_control log
colcon build --packages-select magpie_control
source install/setup.bash
```

---

## Bring Up Fresh Dynamixel Motors

This repo includes a small CLI utility to find and configure new AX-12 motors.

1. Scan for motors on likely serial ports:

```bash
python -m magpie_control.dxl_setup scan --id-max 30
```

2. If two brand-new motors are attached at once (both default to ID 1), unplug one first.
Leave one motor as ID 1, then plug in the other motor by itself and change it to ID 2:

```bash
python -m magpie_control.dxl_setup set-id --port /dev/ttyACM0 --baud 1000000 --current-id 1 --new-id 2
```

3. Optional: set baudrate (e.g., to keep everything at 1,000,000):

```bash
python -m magpie_control.dxl_setup set-baud --port /dev/ttyACM0 --baud 57600 --id 2 --new-baud 1000000
```

4. Rescan and verify both IDs are visible at 1,000,000:

```bash
python -m magpie_control.dxl_setup scan --ports /dev/ttyACM0 --bauds 1000000 --id-max 10
```
