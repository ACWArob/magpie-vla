# AutoGrasp ROSification — Technical Procedure
**Platform:** Ubuntu 22.04, ROS 2 Humble  
**Robot:** UR5 CB3 at `192.168.0.4`  
**Gripper:** MAGPIE (AX12 Dynamixel motors) on `/dev/ttyACM0` via OpenRB-150  
**F/T Sensor:** ATI Mini45 + NetFT box at `192.168.0.6`  
**Camera:** Intel RealSense D405, USB, `/dev/video0`  
**Lab machine:** Ubuntu 22.04, `192.168.0.7`, ROS 2 Humble  
**Colcon workspace:** `~/ws_ctrl`  
**Source package:** `~/magpie_control`

---

## Background and Motivation

The original AutoGrasp stack was a single monolithic Python script that controlled all hardware directly:
- The UR5 arm via `rtde_control` / `rtde_receive` (Universal Robots RTDE library)
- The MAGPIE gripper via the `Dynamixel SDK` over a serial port
- The RealSense camera via Intel's `pyrealsense2` SDK
- The ATI Mini45 F/T sensor via raw UDP sockets

This architecture worked for single-script experiments but made the system difficult to extend, parallelize, or integrate with standard robotics tools like MoveIt, RViz, or learned policy frameworks. The goal of this ROSification effort is to decompose the stack into independent ROS 2 nodes that communicate over well-defined topics and services, enabling:

1. **Modularity** — any node can be replaced or upgraded independently
2. **Introspection** — `ros2 topic echo` and `ros2 topic hz` can monitor any data stream at any time
3. **Integration** — downstream nodes (learned policies, LLM planners, teleoperation) can subscribe to standard ROS topics without knowing hardware details
4. **Safety** — each node handles its own shutdown cleanly (e.g., gripper opens on exit)

---

## System Architecture

### ROS Node Graph

```
Intel RealSense D405 (USB)
        │
        ▼
realsense2_camera ──────────► /camera/camera/color/image_raw  (sensor_msgs/Image)
                  ──────────► /camera/camera/depth/image_rect_raw (sensor_msgs/Image)
                  ──────────► /camera/camera/color/camera_info (sensor_msgs/CameraInfo)

MAGPIE Gripper (/dev/ttyACM0)
        │
        ▼
gripper_node ──────────────► /gripper/state  (magpie_msgs/GripperState) @ 10 Hz
             ◄────────────── /gripper/open, /gripper/close  (std_srvs/Trigger)
             ◄────────────── /gripper/set_position  (magpie_msgs/SetGripperPosition)
             ◄────────────── /gripper/set_force  (magpie_msgs/SetGripperForce)
             ◄────────────── /gripper/deligrasp  (magpie_msgs/DeliGrasp) [action]

ATI Mini45 (192.168.0.6 UDP:49152)
        │
        ▼
ft_sensor_node ────────────► /ft_sensor/wrench  (geometry_msgs/WrenchStamped) @ 50 Hz
               ◄──────────── /ft_sensor/zero  (std_srvs/Trigger)

UR5 arm (192.168.0.4 TCP:30004 RTDE)
        │
        ▼
ur5_node ──────────────────► /arm/joint_states  (sensor_msgs/JointState) @ 10 Hz
         ──────────────────► /arm/tcp_pose  (geometry_msgs/PoseStamped) @ 10 Hz
         ◄────────────────── /arm/move_j  (magpie_msgs/MoveJoint)
         ◄────────────────── /arm/move_l  (magpie_msgs/MoveLinear)
         ◄────────────────── /arm/get_pose  (magpie_msgs/GetPose)
         ◄────────────────── /arm/set_speed  (magpie_msgs/SetSpeed)
         ◄────────────────── /arm/move_safe, /arm/stop  (std_srvs/Trigger)

deligrasp_node ◄────────────  /camera/camera/color/image_raw
               ◄────────────  /camera/camera/depth/image_rect_raw
               ◄────────────  /camera/camera/color/camera_info
               ◄────────────  /arm/tcp_pose
               ──── calls ──► /arm/move_l
               ──── calls ──► /gripper/open
               ──── calls ──► /gripper/deligrasp  [action]
               ──── calls ──► /arm/move_safe
               ◄────────────  /grasp/execute  (std_srvs/Trigger) [trigger]
```

### Package Dependencies

```
magpie_msgs        (custom ROS 2 message definitions — must build first)
    └── magpie_control  (this package — nodes, launch, config)
            ├── gripper_node.py     wraps magpie_control.gripper.Gripper
            ├── ft_sensor_node.py   wraps magpie_control.ft_sensor.OptoForce
            ├── tactile_sensor_node.py  stub for E-flesh sensors
            ├── ur5_node.py         wraps magpie_control.ur5.UR5_Interface
            └── deligrasp_node.py   orchestrates full grasp pipeline
```

### External Packages Used

| Package | Source | Role |
|---|---|---|
| `realsense2_camera` | `ros-humble-realsense2-camera` (apt) | RealSense D405 ROS driver |
| `magpie_perception` | github.com/correlllab/magpie_perception | Grounding DINO detection, SAM segmentation, point cloud processing |
| `rtde_control` / `rtde_receive` | pip, UR RTDE library | Direct UR5 communication |
| `dynamixel_sdk` | pip | AX12 motor serial protocol |
| `transformers` | pip (HuggingFace) | Grounding DINO model weights and inference |

---

## Step 1 — Code Review and Bug Fixes

Before running any hardware, the existing ROS node drafts (`gripper_node.py`, `ft_sensor_node.py`, `tactile_sensor_node.py`) were reviewed for correctness. Three critical bugs were found that would have caused silent failures or hangs on real hardware, plus a minor deprecation and a missing file.

### Bug 1 — Executor Deadlock in `calibrate_callback`

**File:** `gripper_node.py`, `calibrate_callback()`  
**What it was:**
```python
self.gripper.open_gripper()
rclpy.spin_once(self, timeout_sec=2.0)   # ← deadlock
self.gripper.close_gripper()
```

**Technical explanation:** ROS 2's executor (the object that manages the event loop) holds a mutex lock while dispatching a callback. `rclpy.spin_once()` attempts to acquire that same mutex in order to process the next event. When called from within a callback that the executor is already running, the mutex is already held — `spin_once` blocks waiting for it, the callback blocks waiting for `spin_once`, and neither ever proceeds. The process hangs indefinitely.

This is a fundamental constraint of single-threaded executors: you cannot re-enter the executor from within a callback it is executing.

**Fix:**
```python
self.gripper.open_gripper()
time.sleep(2.0)          # blocking sleep is fine — serial I/O is synchronous anyway
self.gripper.close_gripper()
```

### Bug 2 — Uncalibrated Force-to-Torque Conversion in `set_force_callback`

**File:** `gripper_node.py`, `set_force_callback()`  
**What it was:**
```python
torque = int(min(max(request.max_force * 10, 0), 1023))
self.gripper.set_torque(torque)
```

**Technical explanation:** The AX12 Dynamixel motors report and accept load in dimensionless 10-bit units (0–1023). The relationship between these units and contact force in Newtons is non-linear — it depends on the gripper's crank-linkage geometry, motor characteristics, and friction. Stephen Otto's thesis (ProQuest 2868478510, p17, Figure 14) derived an empirical piecewise polynomial calibration:

- For load < 100 bits: `N = 0.0025·load − 0.0000007·load²`  
- For load ≥ 100 bits: `N = −0.00001889·load² + 0.038399·load − 3.4073`

The `* 10` linear scaling in the original code bore no relationship to this calibration. At 2 N (a typical gentle grasp), the correct load value is approximately 180 bits; `2 * 10 = 20` bits would barely move the motor.

**Fix:** Call the existing calibrated method directly:
```python
self.gripper.set_force(request.max_force, finger='both')
```

### Bug 3 — Blocking Serial I/O Starves ROS Executor in `deligrasp_execute_callback`

**File:** `gripper_node.py`, `deligrasp_execute_callback()`  
**What it was:**
```python
async def deligrasp_execute_callback(self, goal_handle):
    ...
    final_aperture_mm, final_force_n, _, grasp_log = self.gripper.deligrasp(...)
```

**Technical explanation:** The callback is declared `async`, which means the ROS action server expects it to yield control back to the event loop periodically via `await`. However, `Gripper.deligrasp()` is a fully synchronous function containing hundreds of serial read/write calls and `time.sleep()` waits totalling several seconds. Calling it directly inside an `async` callback without `await asyncio.to_thread()` or equivalent does not yield to the event loop at all — it blocks the entire thread. During this time, no other ROS callbacks execute: the 10 Hz gripper state publisher stops, no other services can be called, and the ROS heartbeat goes silent.

**Fix:** The `Gripper` class already provides `deligrasp_async()` which correctly wraps all blocking calls with `asyncio.to_thread()`, allowing the event loop to remain responsive:
```python
final_aperture_mm, final_force_n, _, grasp_log = await self.gripper.deligrasp_async(...)
```

### Bug 4 — Deprecated Logger Method

**File:** `tactile_sensor_node.py`  
**What it was:** `self.get_logger().warn(...)`  
**Fix:** `self.get_logger().warning(...)` — deprecated since ROS 2 Foxy, removed in later distributions.

### Missing File — `config/gripper_config.yaml`

**What it was:** The launch file unconditionally passed `config/gripper_config.yaml` as the `parameters=` argument for every node, but the `config/` directory did not exist in the package.

**Why it matters:** In ROS 2, if a parameters file is specified and cannot be found, the node process exits immediately at startup with a file-not-found error. All four nodes in the launch file would have silently failed.

**Fix:** Created `config/gripper_config.yaml` containing all declared parameters for all nodes. This file is referenced by the launch file and installed to the package's share directory via `setup.py`.

---

## Step 2 — Network Verification

### Why This Step

RTDE (Real-Time Data Exchange) is the protocol used to communicate with the UR5 controller. It runs over TCP on port 30004. Before writing any ROS code for the arm, we verified the full communication path from the lab machine to the robot.

### Network Discovery Issue

The UR5 IP was initially stated as `192.168.0.6`. The lab machine's IP was found to be `192.168.0.7`, so a ping to `.6` succeeded but RTDE port 30004 was closed. The actual addresses are:

| Device | IP |
|---|---|
| Lab machine | 192.168.0.7 |
| UR5 arm controller | 192.168.0.4 |
| ATI Mini45 NetFT box | 192.168.0.6 |

The UR5 IP in `ur5.py` defaulted to `192.168.0.4` which was already correct — no code change was needed.

### RTDE Port Verification

```bash
# Confirm the arm is reachable
ping -c 4 192.168.0.4

# Confirm RTDE port is open (port 30004 = RTDE, 30001 = primary, 29999 = dashboard)
nc -zv 192.168.0.4 30004

# Confirm live data is readable
python3 -c "
import rtde_receive
r = rtde_receive.RTDEReceiveInterface('192.168.0.4')
print('Joint angles (rad):', r.getActualQ())
print('TCP pose:', r.getActualTCPPose())
print('Robot mode:', r.getRobotMode())   # 7 = running, 6 = power off
r.disconnect()
"
```

**Result:** Robot mode 7 (running), joint angles and TCP pose returned correctly.

### What RTDE Is

RTDE is a bidirectional interface built into UR controller software. It allows an external computer to read robot state (joint positions, TCP pose, forces, speeds) at up to 500 Hz and write control inputs (target positions, velocities, register values). Unlike the URScript interface (which uploads programs to the robot), RTDE is always available and does not require any setup on the teach pendant.

The `rtde_control` library wraps RTDE to provide higher-level motion commands (`moveJ`, `moveL`, `speedL`, etc.) which are translated into URScript programs uploaded to the controller at runtime.

---

## Step 3 — Intel RealSense D405 Camera

### Why the Official ROS Package

The `realsense_wrapper.py` in the existing codebase wraps Intel's `pyrealsense2` Python SDK directly. While functional, writing a custom ROS node around it would require:
- Manual Image message construction and publishing
- Manual CameraInfo message construction with intrinsic calibration data
- Custom timestamp handling
- Frame synchronisation between color and depth streams

The official `realsense2_camera` ROS 2 package (maintained by Intel) handles all of this correctly, including hardware-accelerated post-processing, depth-to-color alignment, and point cloud generation. Using it avoids reinventing solved problems.

### USB Speed Warning

The D405 was connected via a USB 2.1 port (480 Mbps). The camera is USB 3.1 Gen 1 (5 Gbps) capable. At USB 2.1 the camera is bandwidth-constrained and defaults to 10 FPS at 848×480. Connecting to a blue USB 3.0 port achieves 30 FPS. For DeliGrasp, 10 FPS is functional but limits how quickly the pipeline can respond to scene changes.

### Installation and Launch

```bash
sudo apt install ros-humble-realsense2-camera

# Verify OS detects the camera
lsusb | grep -i intel      # Intel RealSense D405, ID 8086:0b5b
ls /dev/video*             # /dev/video0 through /dev/video5

# Launch (uses default 848x480 @ 10 FPS)
ros2 launch realsense2_camera rs_launch.py

# Verify topics
ros2 topic hz /camera/camera/color/image_raw     # ~10 Hz
ros2 topic hz /camera/camera/depth/image_rect_raw  # ~10 Hz
```

### Topic Names

The package publishes under a double-namespaced path:
```
/camera/camera/color/image_raw          # RGB, sensor_msgs/Image, encoding bgr8
/camera/camera/depth/image_rect_raw     # Depth, sensor_msgs/Image, encoding 16UC1 (mm)
/camera/camera/color/camera_info        # Intrinsics, sensor_msgs/CameraInfo
```

The outer `camera/` is the ROS node namespace, the inner `camera/` is the sensor module name. All downstream subscribers must use these exact names or remap them.

---

## Step 4 — MAGPIE Gripper Node

### Hardware Overview

The MAGPIE gripper uses two Dynamixel AX12-A servo motors (IDs 1 and 2) driven by an OpenRB-150 Dynamixel controller board. The OpenRB-150 enumerates as a USB CDC-ACM device at `/dev/ttyACM0` and communicates at 1 Mbaud using the Dynamixel Protocol 1.0. The `Gripper` class in `gripper.py` provides all motor control logic including the DeliGrasp force-control algorithm.

### Permissions Fix

Linux restricts serial port access to members of the `dialout` group. The `user` account was not in this group by default, causing a `Permission denied` error when the node tried to open `/dev/ttyACM0`.

```bash
# Run as admin
sudo usermod -aG dialout user

# Activate without full logout (session only)
newgrp dialout

# Verify
groups   # should include: dialout user
```

A full logout and login is required to make the group membership permanent across all sessions.

### Custom Message Package (`magpie_msgs`)

`gripper_node.py` imports `magpie_msgs.srv.SetGripperPosition`, `SetGripperForce`, `magpie_msgs.msg.GripperState`, `DeliGraspParams`, and `magpie_msgs.action.DeliGrasp`. These are custom message definitions that must be built before `magpie_control`. In a colcon workspace, package build order is determined by dependency declarations in `package.xml`.

```bash
mkdir -p ~/ws_ctrl/src
cp -r ~/Downloads/May_14/magpie_msgs-main ~/ws_ctrl/src/magpie_msgs
cp -r ~/magpie_control ~/ws_ctrl/src/magpie_control

cd ~/ws_ctrl
source /opt/ros/humble/setup.bash

# Build magpie_msgs first — it generates Python/C++ code from .msg/.srv/.action files
colcon build --packages-select magpie_msgs
source install/setup.bash

# Now build magpie_control with symlink-install
# --symlink-install means Python source edits take effect without rebuilding
colcon build --packages-select magpie_control --symlink-install
source install/setup.bash
```

### What the Gripper Node Exposes

**Published topic:**
- `/gripper/state` (`magpie_msgs/GripperState`) at 10 Hz — contains total aperture (mm), mean force (N), temperature (°C), and individual finger positions

**Services:**
- `/gripper/open` — opens fully (safety default)
- `/gripper/close` — closes fully
- `/gripper/set_position` — move to target aperture in mm with optional speed scaling
- `/gripper/set_force` — set torque limit using calibrated N→load polynomial
- `/gripper/calibrate` — open, wait 2 s, close (resets motor reference)
- `/gripper/reset_parameters` — restore default speed, torque, compliance settings

**Action server:**
- `/gripper/deligrasp` (`magpie_msgs/DeliGrasp`) — executes the DeliGrasp force-control algorithm, providing feedback (phase, current aperture/force) and returning final aperture, force, and full force log

### DeliGrasp Algorithm

DeliGrasp (Delicate Grasp) is a force-controlled grasping algorithm designed for objects with unknown stiffness. It operates as follows:

1. Set torque limit to the initial force `fc` (converted to load bits via the empirical polynomial)
2. Move gripper to goal aperture `x` while recording motor load at each position tick
3. Check for slip: if neither finger's load exceeded `fc/2` during the approach, the object was not contacted
4. While slipping: decrease goal aperture by `dx` mm, increase force limit by `df` N, repeat
5. When contact is confirmed: optionally perform a final `dx` mm closure to secure the grasp

The stop criterion `fc/2` per finger (rather than `fc` total) reflects that each finger independently bears half the required contact force when both are symmetrically engaged.

### Verification

```bash
# Run node directly (auto-detect port)
ros2 run magpie_control gripper_node

# Or with explicit port if auto-detect fails
ros2 run magpie_control gripper_node \
  --ros-args -p auto_detect_port:=false -p port:=/dev/ttyACM0

# Check state (should show aperture ~103mm when open, temp ~33°C)
ros2 topic echo /gripper/state --once

# Test open/close
ros2 service call /gripper/open std_srvs/srv/Trigger {}
ros2 service call /gripper/close std_srvs/srv/Trigger {}

# Test force control (1.5 N — gentle grasp)
ros2 service call /gripper/set_force \
  magpie_msgs/srv/SetGripperForce "{max_force: 1.5}"
```

---

## Step 5 — ATI Mini45 F/T Sensor Node

### Hardware Overview

The ATI Mini45 is a 6-axis force/torque sensor mounted between the UR5 wrist flange and the MAGPIE gripper. It measures Fx, Fy, Fz (forces in N) and Tx, Ty, Tz (torques in N·m) at the tool attachment point.

The sensor connects to an ATI NetFT interface box which provides a 100 Mbps Ethernet connection. The NetFT box runs a UDP server on port 49152 that accepts datagram commands and streams 36-byte measurement packets.

### Protocol

The `OptoForce` class in `ft_sensor.py` implements the ATI NetFT protocol (not OptoForce — the class was named for historical reasons but speaks the ATI NetFT wire format). The protocol involves:
- Sending a 8-byte command datagram to start/configure streaming
- Receiving 36-byte response datagrams: 3× uint32 header + 6× int32 measurements
- Dividing force values by 10000.0 and torque values by 100000.0 to get SI units

Valid polling rates (empirically verified): 5, 10, 20, 50, 100, 250, 500 Hz. The node is configured to 50 Hz.

### IP Discovery

The correct IP for the NetFT box was found by testing UDP port 49152 on both candidate addresses:

```bash
nc -zvu 192.168.0.5 49152   # refused
nc -zvu 192.168.0.6 49152   # succeeded — correct IP
```

### What the Node Exposes

**Published topic:**
- `/ft_sensor/wrench` (`geometry_msgs/WrenchStamped`) at 50 Hz

**Service:**
- `/ft_sensor/zero` — sends the `set_bias_1` command to the NetFT box, which subtracts the current reading from all future readings. Should be called with the gripper open and unloaded to remove gravity bias.

### Observed Values

At startup with the gripper mounted but not grasping anything:
- Fx ≈ 0.1 N, Fy ≈ 0.0 N, Fz ≈ −0.4 to +0.8 N (gravity bias from gripper weight ~0.4 N)
- Tx, Ty, Tz ≈ 0.001–0.01 N·m

These values are expected. Calling `/ft_sensor/zero` removes the bias.

---

## Step 6 — UR5 Arm Node

### Design Decision: Direct RTDE Wrapper vs. `ur_robot_driver`

The standard ROS 2 approach for Universal Robots arms is the `ur_robot_driver` package, which provides full integration with MoveIt 2, trajectory execution, and real-time control at 500 Hz. However, it has a mandatory hardware prerequisite: the **ExternalControl URCap** must be installed on the teach pendant.

A URCap is a plugin that runs inside the robot's PolyScope controller software. The ExternalControl URCap opens a socket that `ur_robot_driver` connects to in order to send motion commands. Without it, the driver has no way to send trajectories to the robot.

For this project, the existing `UR5_Interface` class in `ur5.py` already provides all needed motion primitives (`moveJ`, `moveL`, `speedL`, force-position control) via the `rtde_control` / `rtde_receive` libraries, which communicate over RTDE directly without requiring any pendant setup. The decision was made to wrap this existing class as a ROS node, avoiding the pendant installation step entirely.

**Trade-off:** This approach does not integrate with MoveIt 2's motion planning pipeline. If collision-aware path planning or Cartesian path planning with obstacle avoidance is needed in the future, migrating to `ur_robot_driver` would be required.

### Pose Representation

RTDE returns TCP pose as a 6-element vector `[x, y, z, rx, ry, rz]` where:
- `[x, y, z]` is position in meters in the robot base frame
- `[rx, ry, rz]` is the rotation as an **axis-angle vector** — the direction is the rotation axis, the magnitude is the rotation angle in radians

ROS 2 `geometry_msgs/Pose` uses a quaternion `[w, x, y, z]` for orientation. `ur5_node.py` converts between these representations on every publish and every service call using:

```
# Axis-angle → Quaternion
angle = ||rv||
axis  = rv / angle
q = (cos(angle/2), sin(angle/2)·axis_x, sin(angle/2)·axis_y, sin(angle/2)·axis_z)

# Quaternion → Axis-angle
angle = 2·arccos(w)
axis  = [x, y, z] / sin(angle/2)
rv    = angle · axis
```

### What the Node Exposes

**Published topics:**
- `/arm/joint_states` (`sensor_msgs/JointState`) at 10 Hz — joint names follow ROS convention: `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint`
- `/arm/tcp_pose` (`geometry_msgs/PoseStamped`) at 10 Hz — TCP pose in robot base frame, orientation as quaternion

**Services:**
- `/arm/move_j` (`magpie_msgs/MoveJoint`) — joint-space move; `async_mode=true` returns immediately, `false` blocks until complete
- `/arm/move_l` (`magpie_msgs/MoveLinear`) — Cartesian linear move; target as `geometry_msgs/Pose`
- `/arm/get_pose` (`magpie_msgs/GetPose`) — returns current TCP pose and joint angles
- `/arm/set_speed` (`magpie_msgs/SetSpeed`) — updates default linear and angular speed/acceleration
- `/arm/move_safe` (`std_srvs/Trigger`) — moves to `Q_safe = [12.30°, −110.36°, 95.90°, −75.48°, −89.59°, 12.33°]`
- `/arm/stop` (`std_srvs/Trigger`) — calls `ctrl.stopL()` immediately

### RTDE Register Conflict

When the full stack is launched while a previous test session's RTDE connection is still open (process killed with SIGTERM but RTDE not cleanly disconnected), the new `RTDEControlInterface` fails with:

```
RuntimeError: One of the RTDE input registers are already in use!
```

This happens because the UR controller only allows one active RTDE control client at a time. The fix is to ensure all previous node processes are fully terminated before launching. `destroy_node()` in `ur5_node.py` calls `self.ur5.stop()` which disconnects both RTDE interfaces cleanly.

---

## Step 7 — DeliGrasp Logic Node

### Overview

`deligrasp_node.py` is the top-level coordination node. It subscribes to camera and arm pose topics, runs object detection and 3D localisation, and then orchestrates the arm and gripper to execute a complete pick-and-place style grasp.

### Perception Pipeline

Object detection and 3D localisation proceed in four stages:

**Stage 1 — Open-vocabulary detection (Grounding DINO)**

Grounding DINO (Liu et al., 2023) is a zero-shot object detection model that accepts a free-text query (e.g., "bottle", "red cup") and returns bounding boxes with confidence scores. It is built on a DINO backbone with text-grounding via a BERT encoder. It is loaded from HuggingFace: `IDEA-Research/grounding-dino-tiny`.

The model is loaded once at node startup via:
```python
from magpie_perception.label_dino import LabelDINO
self.detector = LabelDINO()
boxes, labels, scores = self.detector.label(color_image, query, confidence_threshold)
```

`magpie_perception` is a Python library (not a ROS package) from the Correll Lab that wraps multiple detection and segmentation models with a consistent API.

**Stage 2 — Depth sampling**

The RealSense D405 publishes depth as a 16-bit unsigned integer image where each pixel value is depth in millimetres. To get the 3D position of a detected object:

1. Take the bounding box centroid `(u, v)` in pixels
2. Sample a 10×10 pixel neighbourhood around the centroid in the depth image
3. Discard invalid pixels (value = 0, indicating no depth return)
4. Take the median of the remaining values to reject noise and specular artefacts
5. Convert from mm to m: `depth_m = median / 1000.0`

**Stage 3 — Back-projection to camera frame**

Using the pinhole camera model and the intrinsic parameters from `/camera/camera/color/camera_info`:

```
X_cam = (u - cx) · depth_m / fx
Y_cam = (v - cy) · depth_m / fy
Z_cam = depth_m
```

where `fx`, `fy` are focal lengths in pixels and `cx`, `cy` is the principal point. These values are read from `CameraInfo.k` (the 3×3 intrinsic matrix in row-major order).

**Stage 4 — Transform to robot world frame**

The camera is mounted in the gripper palm with a known rigid transform relative to the UR5 TCP (Tool Centre Point). This transform `T_tcp→cam` is defined in `ur5.py` as:

```python
_CAMERA_XFORM = homog_xform(
    rotnMatx = R_krot([0, 0, 1], -π/2),   # -90° rotation about Z
    posnVctr = [0.0, 0.0, 0.120]           # 120mm forward along TCP Z axis
)
```

The current TCP pose `T_world→tcp` is read from `/arm/tcp_pose`. The object's world-frame position is:

```
T_world→cam = T_world→tcp · T_tcp→cam
p_world = T_world→cam · [X_cam, Y_cam, Z_cam, 1]ᵀ
```

### Grasp Execution Sequence

Once the 3D object position in world coordinates is known, the grasp proceeds as follows:

1. **Open gripper** — call `/gripper/open`
2. **Approach pose** — keep current TCP orientation, translate XY to object XY, set Z to `object_z + approach_height` (default 100mm above). Call `/arm/move_l` at 0.15 m/s
3. **Grasp descent** — translate TCP down to `object_z + grasp_z_offset` (default 20mm above object centre). Call `/arm/move_l` at 0.05 m/s (slow for safety)
4. **DeliGrasp** — call `/gripper/deligrasp` action with configured force parameters. Wait for result
5. **Retreat** — translate TCP back up by `approach_height`. Call `/arm/move_l` at 0.10 m/s
6. On any failure: open gripper and call `/arm/move_safe`

### Thread Safety — ReentrantCallbackGroup

The grasp service callback (`/grasp/execute`) needs to call other services (`/arm/move_l`, `/gripper/open`, `/gripper/deligrasp`) and wait for their results. In ROS 2's default single-threaded executor, calling `spin_until_future_complete()` from inside a callback would deadlock — the callback holds the executor thread, and `spin_until_future_complete` needs that thread to process the service responses.

The solution is to use a `ReentrantCallbackGroup` for all callbacks and run the node under a `MultiThreadedExecutor`:

```python
self.cbg = ReentrantCallbackGroup()

# Assign cbg to all subscriptions, service clients, and service servers
self.create_service(Trigger, 'grasp/execute', self._grasp_cb, callback_group=self.cbg)
self.cli_move_l = self.create_client(MoveLinear, '/arm/move_l', callback_group=self.cbg)
...

# In main():
executor = MultiThreadedExecutor()
executor.add_node(node)
executor.spin()
```

A `ReentrantCallbackGroup` tells the executor that callbacks in this group may be executed concurrently and may re-enter the executor. The `MultiThreadedExecutor` uses a thread pool to handle this. Without both of these, the grasp service would deadlock when it first tries to call `/arm/move_l`.

### Configuration Parameters

All parameters are set in `config/gripper_config.yaml` and can be overridden at launch:

| Parameter | Default | Description |
|---|---|---|
| `object_query` | `"object"` | Grounding DINO text prompt |
| `detection_confidence` | `0.3` | Minimum detection score (0–1) |
| `approach_height` | `0.10` | Metres above object for approach pose |
| `grasp_z_offset` | `0.02` | Metres above object centre for grasp |
| `initial_force` | `1.5` | Initial DeliGrasp contact force in N |
| `additional_force` | `0.2` | Force increment per slip iteration in N |
| `additional_closure` | `1.0` | Closure increment per slip iteration in mm |

### Triggering a Grasp

```bash
# Change the target object at runtime (no restart needed)
ros2 param set /deligrasp_node object_query "bottle"
ros2 param set /deligrasp_node detection_confidence 0.25

# Execute a grasp
ros2 service call /grasp/execute std_srvs/srv/Trigger {}

# Response:
# success: True
# message: 'Grasp complete — final aperture 24.3mm, force 2.1N'
```

---

## Step 8 — Full Stack Launch

### Launch File

`launch/gripper_control.launch.py` starts all five nodes with parameters from `config/gripper_config.yaml`:

```
gripper_node        ← MAGPIE gripper (serial)
ft_sensor_node      ← ATI Mini45 (UDP Ethernet)
tactile_sensor_node ← E-flesh stub (disabled)
ur5_node            ← UR5 arm (RTDE Ethernet)
deligrasp_node      ← perception + grasp orchestration
```

The RealSense camera is launched separately (it uses a different launch file from the `realsense2_camera` package).

```bash
# Terminal 1 — camera
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py

# Terminal 2 — full robot stack
source /opt/ros/humble/setup.bash
source ~/ws_ctrl/install/setup.bash
newgrp dialout  # or log out/in if dialout group was just added
ros2 launch magpie_control gripper_control.launch.py

# Terminal 3 — trigger a grasp
source /opt/ros/humble/setup.bash
source ~/ws_ctrl/install/setup.bash
ros2 param set /deligrasp_node object_query "bottle"
ros2 service call /grasp/execute std_srvs/srv/Trigger {}
```

### Full Startup Verification Checklist

```bash
# All nodes running
ros2 node list
# Expected: /gripper_node /ft_sensor_node /tactile_sensor_node
#           /ur5_node /deligrasp_node /camera/camera

# All topics publishing
ros2 topic hz /gripper/state                        # ~10 Hz
ros2 topic hz /ft_sensor/wrench                     # ~50 Hz
ros2 topic hz /arm/joint_states                     # ~10 Hz
ros2 topic hz /arm/tcp_pose                         # ~10 Hz
ros2 topic hz /camera/camera/color/image_raw        # ~10 Hz
ros2 topic hz /camera/camera/depth/image_rect_raw   # ~10 Hz

# Services available
ros2 service list | grep -E "gripper|arm|ft_sensor|grasp"

# Spot-check live values
ros2 service call /arm/get_pose magpie_msgs/srv/GetPose {}
ros2 topic echo /gripper/state --once
ros2 topic echo /ft_sensor/wrench --once
```

---

## Network and Hardware Map

| Device | IP / Port | Protocol | ROS Interface |
|---|---|---|---|
| Lab machine | 192.168.0.7 | — | — |
| UR5 CB3 controller | 192.168.0.4 TCP:30004 | RTDE | `/arm/*` |
| ATI Mini45 NetFT box | 192.168.0.6 UDP:49152 | NetFT datagram | `/ft_sensor/*` |
| Intel RealSense D405 | USB 2.1 → `/dev/video0` | UVC (USB Video Class) | `/camera/camera/*` |
| MAGPIE gripper (OpenRB-150) | USB → `/dev/ttyACM0` | Dynamixel Protocol 1.0 @ 1 Mbaud | `/gripper/*` |

---

## Dependency Installation Summary

```bash
# ROS packages (require sudo / admin)
sudo apt install ros-humble-realsense2-camera
sudo apt install ros-humble-ur   # installed but not used — ur_robot_driver requires URCap

# Python packages (user install)
pip install torch --index-url https://download.pytorch.org/whl/cpu
pip install transformers
pip install --upgrade Pillow   # system PIL too old for transformers (needs >=9.1.0)

# magpie_perception (cloned manually due to pyproject.toml src layout issue)
git clone https://github.com/correlllab/magpie_perception /tmp/magpie_perception
cp -r /tmp/magpie_perception/src/magpie_perception \
      ~/.local/lib/python3.10/site-packages/magpie_perception
```

**Note on `magpie_perception` install:** The package's `pyproject.toml` declares `src/` layout but is missing the `[tool.setuptools.packages.find] where = ["src"]` directive, causing `pip install` to produce an empty wheel. The workaround is to copy the package source directly into site-packages.

---

## Known Issues and Future Work

| Issue | Impact | Resolution |
|---|---|---|
| Camera on USB 2.1 port | 10 FPS instead of 30 FPS | Connect to blue USB 3.0 port |
| `dialout` group needs relogin | `newgrp dialout` required each session | One full logout/login makes it permanent |
| RTDE register conflict on restart | `ur5_node` fails if previous session not cleanly closed | Kill all node processes before re-launching |
| `magpie_perception` install workaround | Not pip-installable cleanly | Upstream fix: add `[tool.setuptools.packages.find]` to pyproject.toml |
| Grounding DINO on CPU only | Slow inference (~2–5 s per image) | Add GPU support when CUDA-capable hardware available |
| No collision avoidance | Arm moves in straight lines without checking obstacles | Future: migrate to `ur_robot_driver` + MoveIt 2 |
| Tactile sensors (E-flesh) not implemented | `tactile_sensor_node` is a stub | Implement when E-flesh ROS driver is available |
| `goal_aperture` hardcoded to 30mm | DeliGrasp starts from fixed width | Should be estimated from detection bounding box width + depth |

---

## Day 2 — Mentor Review and Refinements

### Mentor Feedback (Summary)

After completing the initial ROSification, the following feedback was received:

1. The F/T sensor **onboard the UR5** is the **OnRobot HEX-E**, not the ATI Mini45. The ATI Mini45 + NetFT box is a separate sensor not currently mounted on the robot.
2. The F/T sensor poll rate should be configurable — it already is via `poll_rate` in `gripper_config.yaml`.
3. `/arm/joint_states` and `/arm/tcp_pose` publish rate should be configurable. Default changed from 10 Hz → **500 Hz** (RTDE streams at 500 Hz; `UR5_Interface` already initialises the receive interface at `freq=500`).
4. The arm node should support **teach mode** (freedrive) as a toggleable service that blocks other motion commands while active.
5. The arm node should support **servoJ** and **servoL** for high-frequency streaming control, with `servoStop()` called on stop and shutdown.
6. DeliGrasp node should use the **`magpie_prompts`** library to derive the object query from a natural language instruction via LLM.
7. The camera namespace `/camera/camera/` is redundant — should be `/camera/gripper_camera/`.
8. A **standalone `/gripper/clear_error` service** should exist separately from `/gripper/reset_parameters`.
9. The deligrasp action server feedback should come from the **`/gripper/state` topic**, not internal serial reads.
10. Multiple detectors should be supported (Grounding DINO and OWL-ViT).
11. A **USB speed warning** should alert the user if the RealSense is on a USB <3.0 port.

---

### F/T Sensor Clarification

**Correction from Day 1:** The ATI Mini45 + NetFT box (`ft_sensor_node.py`) is **not mounted on the UR5**. It is a standalone sensor that may be integrated in the future.

The UR5 wrist-mounted F/T sensor is the **OnRobot HEX-E**, which communicates via the UR5's internal tool I/O. This sensor is not yet integrated as a separate ROS node. If future work requires wrist F/T, the OnRobot ROS driver or direct URScript access should be used.

---

### Step D2-1 — Teach Mode Toggle (`ur5_node.py`)

**Why teach mode matters:** Freedrive (teach mode) allows a human to physically move the arm to a desired pose by hand. While teach mode is active, the UR5 controller disables servo control. If a motion command (`moveJ`, `moveL`) were sent simultaneously, it would attempt to fight the human's motion, potentially causing a jerk or error. All motion callbacks must reject commands while teach mode is active.

**Implementation:**
- Added `/arm/teach_mode` service (`std_srvs/Trigger`) — each call toggles freedrive on/off
- `self._teach_mode` boolean tracks state internally
- `_teach_mode_blocked(response)` helper fills the response and returns `True` if motion is blocked
- Applied to: `move_j_callback`, `move_l_callback`, `move_safe_callback`

```python
def teach_mode_callback(self, request, response):
    self.ur5.toggle_teach_mode()
    self._teach_mode = not self._teach_mode
    state = 'enabled' if self._teach_mode else 'disabled'
    response.success = True
    response.message = f'Teach mode {state}'
    return response
```

**Usage:**
```bash
ros2 service call /arm/teach_mode std_srvs/srv/Trigger {}   # enable freedrive
# move arm by hand
ros2 service call /arm/teach_mode std_srvs/srv/Trigger {}   # disable freedrive
```

---

### Step D2-2 — ServoJ and ServoL (`ur5_node.py`)

ServoJ and ServoL allow streaming high-frequency position targets to the arm without blocking — used for real-time control loops (e.g., from a learned policy or teleoperation). Unlike `moveJ`/`moveL` which block until complete, servo commands are fire-and-forget and must be sent continuously at the control rate.

**RTDE servo parameters** (verified against SDU Robotics RTDE API docs):

| Parameter | Value | Valid Range | Meaning |
|---|---|---|---|
| `time` | 0.002 s | > 0 | Duration each call blocks — match your publish rate |
| `lookahead_time` | 0.1 s | [0.03, 0.2] | Smoothing window — reduces jerk |
| `gain` | 300 | [100, 2000] | Proportional position gain — higher = stiffer |

Note: `speed` and `acceleration` arguments are **not used** in the current RTDE library version — passing `0.0` is correct.

**Safety constraint:** These commands bypass collision avoidance and motion planning entirely. Small, incremental targets should be used. Always test with `approach_height` clearance before running near objects.

**Implementation:** Topic subscribers (not services) to support continuous streaming:
- `/arm/servo_j_cmd` (`sensor_msgs/JointState`) → `ctrl.servoJ()`
- `/arm/servo_l_cmd` (`geometry_msgs/PoseStamped`) → `ctrl.servoL()`

Both are blocked if `self._teach_mode` is active. `ctrl.servoStop()` is called in both `/arm/stop` and `destroy_node()` to exit servo mode cleanly before handing back regular control.

**Usage example (500 Hz loop in external node):**
```python
pub = node.create_publisher(JointState, '/arm/servo_j_cmd', 10)
timer = node.create_timer(0.002, lambda: pub.publish(joint_target_msg))
```

---

### Step D2-3 — Gripper Clear Error Service (`gripper_node.py`)

The Dynamixel AX12-A motors enter an overload error state when torque limits are exceeded (e.g., during a failed grasp or unexpected collision). In this state, both motors disable torque output and the gripper goes limp.

`/gripper/reset_parameters` resets all motor parameters AND opens the gripper — too destructive in mid-grasp. `/gripper/clear_error` calls `reset_packet_overload()` which only re-enables torque on both fingers, allowing recovery without disturbing other settings.

```bash
ros2 service call /gripper/clear_error std_srvs/srv/Trigger {}
```

---

### Step D2-4 — DeliGrasp Feedback from `/gripper/state` Topic (`gripper_node.py`)

**Previous behaviour:** The deligrasp action server collected force/aperture data via return values from `deligrasp_async()`, which in turn made direct serial reads inside the motor control loop.

**New behaviour:** The 10 Hz state publisher (`publish_state`) already reads aperture, force, and temperature from hardware and stores the latest reading in `self._latest_state`. During deligrasp execution, a `self._collecting_log` flag causes `publish_state` to also append each sample to `self._deligrasp_state_log`. When deligrasp completes, the result's `final_aperture`, `final_force`, and `force_log` are all derived from this topic-sourced data — not from internal serial reads.

**Why this is better:** A downstream node subscribing to `/gripper/state` sees the same data that is recorded in the grasp log. There is one source of truth for gripper state.

---

### Step D2-5 — LLM Integration (`deligrasp_node.py`)

DeliGrasp requires knowing *what* to grasp. A hardcoded `object_query` parameter works for scripted demos but doesn't scale to natural language commands. The `magpie_prompts` library provides a structured LLM prompt (`dg_command_enumerator`) that extracts object names from free-form instructions.

**How it works:**
1. User sets `llm_instruction` to a natural language command: `"pick up the red cup"`
2. The node calls OpenAI GPT-4o-mini with the `dg_command_enumerator` system prompt
3. The LLM returns a structured Python dict: `{'objects': ['red cup'], 'manipulation': 'pick', ...}`
4. The first object name is extracted and used as the Grounding DINO query

**Installation:**
```bash
git clone https://github.com/correlllab/magpie_prompts.git /tmp/magpie_prompts
cp -r /tmp/magpie_prompts/src/magpie_prompts ~/.local/lib/python3.10/site-packages/magpie_prompts
pip install openai absl-py colorama termcolor
```

Note: same `src/` layout issue as `magpie_perception` — `pip install .` produces an empty wheel. Direct copy workaround applied.

**Usage:**
```bash
# Set OPENAI_API_KEY in environment
export OPENAI_API_KEY=sk-...

ros2 param set /deligrasp_node use_llm true
ros2 param set /deligrasp_node llm_instruction "pick up the water bottle"
ros2 service call /grasp/execute std_srvs/srv/Trigger {}
# Node logs: LLM resolved "pick up the water bottle" → query="water bottle"
```

If `use_llm=false` or no API key is set, falls back to `object_query` parameter.

---

### Step D2-6 — Multiple Detector Support (`deligrasp_node.py`)

Added `detector_type` parameter (default: `grounding_dino`). Supported values:

| Value | Model | Notes |
|---|---|---|
| `grounding_dino` | IDEA-Research/grounding-dino-tiny | Zero-shot, text-prompted, ~2–5 s/image on CPU |
| `owlvit` | google/owlvit-base-patch32 | Zero-shot, faster than DINO on some hardware |

Switch at runtime:
```bash
ros2 param set /deligrasp_node detector_type owlvit
# Restart node for change to take effect (model loaded at startup)
```

---

### Step D2-7 — Camera Namespace (`deligrasp_node.py`)

The default `realsense2_camera` topic namespace `/camera/camera/` is redundant (outer `camera/` is the node namespace, inner `camera/` is the sensor module name). The subscriptions in `deligrasp_node.py` were updated to use `/camera/gripper_camera/` directly — no launch-file remapping needed.

**Updated subscription topics:**
```
/camera/gripper_camera/color/image_raw
/camera/gripper_camera/depth/image_rect_raw
/camera/gripper_camera/color/camera_info
```

Launch the camera with the matching namespace:
```bash
ros2 launch realsense2_camera rs_launch.py camera_name:=gripper_camera
```

**Note:** An earlier version used launch-file `remappings=[]` to remap `/camera/camera/` → `/camera/gripper_camera/` at launch time. This was removed — the node topic strings now match the camera directly, which is cleaner and works when running the node standalone without the launch file.

---

### Step D2-8 — USB Speed Warning (`deligrasp_node.py`)

At startup, `deligrasp_node` calls `lsusb -v -d 8086:0b5b` to check the RealSense D405 USB protocol version. If `bcdUSB < 3.0`, a ROS warning is logged:

```
[WARN] RealSense D405 connected at USB 2.10 — max ~10 FPS. Plug into a blue USB 3.0 port for 30 FPS.
```

This was confirmed active: the camera is currently on USB 2.10 and producing ~10 FPS.

---

### Colcon Workspace Symlink Fix

**Problem discovered:** `~/ws_ctrl/src/magpie_control` was a standalone directory copy of the repo (made during initial setup). Edits to `~/magpie_control/` were not reaching the colcon build — the build was compiling the stale May 14 copy.

**Fix:**
```bash
rm -rf ~/ws_ctrl/src/magpie_control
ln -s ~/magpie_control ~/ws_ctrl/src/magpie_control
colcon build --symlink-install
```

Now `~/magpie_control` is the single source of truth. All edits are immediately picked up on next build.

---

### Day 2 Verification Results

| Node | Status | Notes |
|---|---|---|
| `gripper_node` | Pass | Starts on `/dev/ttyACM0` with `sg dialout` workaround |
| `ft_sensor_node` | Pass | Connects to ATI Mini45 at 192.168.0.6:49152, 50 Hz |
| `ur5_node` | Pass | Connects to UR5 at 192.168.0.4; teach mode, servoJ/L, servoStop all registered |
| `deligrasp_node` | Pass | DINO loaded; USB 2.10 warning fires; subscribes to `/camera/gripper_camera/`; LLM ready |
| `magpie_perception` | Pass | LabelDINO imports cleanly |
| `magpie_prompts` | Pass | `dg_command_enumerator` prompt loads (requires `OPENAI_API_KEY` for live LLM calls) |

### Post-Review Audit Fixes

After an item-by-item audit against the mentor's original list, three additional gaps were found and corrected:

| Gap | Fix |
|---|---|
| `deligrasp_node.py` subscriptions still used `/camera/camera/` | Updated to `/camera/gripper_camera/` directly in node; launch-file remappings removed |
| `servoStop()` not called on `/arm/stop` or node shutdown | Added `ctrl.servoStop()` in `stop_callback` and `destroy_node` before `stopL`/`ur5.stop()` |
| `publish_rate` default was 10 Hz in both config and node | Changed to 500 Hz — RTDE receive interface already initialised at 500 Hz in `ur5.py` |

---

## Day 2 — Physical Hardware Testing

### Gripper Physical Test

**Prerequisites:** Gripper powered (12V supply on), OpenRB-150 USB plugged into `/dev/ttyACM0`, `dialout` group active.

**Observed results:**

| Test | Command | Result |
|---|---|---|
| Node init | `ros2 run magpie_control gripper_node` | `/dev/ttyACM0` detected and initialized |
| State read | `ros2 topic echo /gripper/state --once` | position=103.6mm, force=0N, temp=33.5°C |
| Close | `ros2 service call /gripper/close std_srvs/srv/Trigger {}` | Closed to ~0mm, 1.84N contact force measured |
| Open | `ros2 service call /gripper/open std_srvs/srv/Trigger {}` | Opened successfully |

---

## Day 3 — F/T Sensor, UR5 Arm, and Gemini Integration

### D3-1: Network Setup Discovery

The F/T sensor (192.168.0.6) and UR5 (192.168.0.4) are on a private robot subnet. Both require a **direct Ethernet connection** — WiFi routing passes ICMP (ping) but blocks UDP (F/T sensor) and the UR5 only accepts RTDE from local-subnet clients. The machine Ethernet port (`enp37s0`) must be connected to the robot network switch and receives DHCP address 192.168.0.7/24.

The robot PC at 100.65.73.59 (Tailscale) is a separate machine (`humanoid`) with no ROS workspace — all ROS code runs on the lab machine.

### D3-2: F/T Sensor Fix — Request-Response Mode

The sensor was not publishing data. Root cause: `recv_datum()` had been changed to assume continuous streaming mode (`set_speed 0x0082`), but this sensor uses **request-response mode** — each call must send `send_01` (0x0002, 1 sample) and wait for the reply. The `set_speed` command sets the firmware rate limit but does not trigger autonomous streaming.

**Fix in `ft_sensor.py`:** Restored `self.sock_r.send(self.cmd.COMMANDS['send_01'])` at the top of `recv_datum()`.

**Verified:** 49.95 Hz, F/T readings non-zero at rest. ✓

### D3-3: UR5 Fieldbus Conflict

`RTDEControlInterface` failed with "RTDE input registers already in use." Cause: a stale `ur5_node` process from the read-only test was still running and holding the RTDE session. Fix: `kill <pid>` (by PID, not `pkill`) to allow `destroy_node` to run `servoStop()` and release the session cleanly.

Secondary cause investigated: EtherNet/IP, PROFINET, and Modbus fieldbus adapters on the pendant — all confirmed already disabled. Pendant path: **Installation → Fieldbus**.

**Lesson:** Always kill `ur5_node` by PID so the RTDE session is released on shutdown. A `pkill` that skips `destroy_node` leaves a zombie RTDE session that blocks the next start.

### D3-4: UR5 Motion Test

Set speed to 20% before any motion:
```bash
ros2 service call /arm/set_speed magpie_msgs/srv/SetSpeed "{speed: 0.2, acceleration: 0.2}"
```

`move_safe` succeeded — arm moved to home position [12°, −110°, 96°, −75°, −90°, 12°] at 20% speed. `move_j` verified with wrist_3 rotation. All motion services confirmed functional.

### D3-5: poses.py Bug Fix

`combine_rot_and_trans_from_poses()` was called inside `translate_pose()` (lines 257, 261) but was commented out at the bottom of the file. Uncommented the function definition. Would have caused `NameError` if `translate_pose` was called with `dir_pose="origin"` or a pose matrix.

### D3-6: Gemini LLM Integration

Added Google Gemini vision call to `deligrasp_node.py` to replace hardcoded DeliGrasp parameters. Uses `mp_prompt_tc_vision_phys` system prompt from `magpie_prompts`.

**Parameters returned by LLM from camera image:**
1. `initial_force` (N) — initial gripper contact force
2. `additional_force` (N) — force increment per slip
3. `spring_constant` (N/m) — object stiffness, used to compute `additional_closure`

**SDK:** `google-genai` (new SDK), model `gemini-2.5-flash`. Enabled via `use_gemini: true` parameter. Requires `GEMINI_API_KEY` environment variable.

**Standalone test:** `python3 scripts/test_gemini_grasp.py --task "pick up the block"`

**Verified:** Gemini returned `initial_force=1.60N`, `additional_force=0.30N`, `spring_constant=1500 N/m` from a camera frame. ✓

**Note:** Free tier API keys from Google Cloud Console have `limit: 0` quota. Keys must be created from **aistudio.google.com** to get free tier quota.

All gripper services verified functional on hardware. See [TESTING.md](TESTING.md) for full command reference.

### D3-7: Gemini + Gripper Closed-Loop Test

Added `scripts/test_gemini_gripper.py` — end-to-end test without the arm:

1. Captures one RealSense frame
2. Calls Gemini 2.5 Flash with `mp_prompt_tc_vision_phys` to get grasp parameters
3. Opens gripper, sets force limit to Gemini's `initial_force`
4. 3-second countdown, then closes automatically — gripper stops on contact at the force limit
5. Holds 3 seconds, then releases

No `input()` pauses — fully automatic once the object is placed. The gripper's built-in force feedback handles contact detection and stop.

**Verified:** Gemini returned `initial_force=1.60N`, `additional_force=0.01N`, `spring_constant=1000 N/m` for a red block. Gripper closed to contact and released without user interaction. ✓

---

## Day 4 — DeliGrasp Descriptor Pipeline, New Home Position, RTDE Register Fix

### D4-1: DeliGrasp Descriptor Prompt (Text-Only Gemini)

The previous Gemini integration used a camera image to estimate grasp parameters. This was replaced with the **DeliGrasp descriptor prompt** — a text-only LLM call that derives physics-based grasp parameters from the object name alone, matching the approach in the DeliGrasp paper (deligrasp.github.io).

The descriptor prompt instructs Gemini to fill out a structured template describing:
- Object mass (g), spring constant (N/m), friction coefficient
- Goal aperture (mm), additional closure on slip (mm)

From these, grasp forces are computed using paper physics:
```
initial_force = clamp(mass_kg × 9.81 / friction, 0.15, 16.0)   # N
additional_force = max(0.05, additional_closure × k × 0.0001)   # N
```

**Changes in `deligrasp_node.py`:**
- Added `DG_DESCRIPTOR_PROMPT` constant at module level (verbatim from deligrasp.github.io)
- Added `_parse_descriptor(text)` — regex parser for the structured template response
- Added `_estimate_params_from_descriptor(object_name)` — calls Gemini text-only, parses result, computes forces
- Removed `gemini_task` parameter; `use_gemini` now enables the descriptor path (default `True`)
- Updated `_run_grasp_pipeline` step 6 to call `_estimate_params_from_descriptor(query)` instead of vision-based method

**Changes in `config/gripper_config.yaml`:**
- `use_gemini: true` (was `false`)
- `gemini_task` line removed

**Standalone test scripts rewritten:**
- `scripts/test_gemini_gripper.py` — text-only descriptor + gripper only (no arm, no camera)
- `scripts/test_gemini_arm_gripper.py` — text-only descriptor + arm + gripper (no camera for params)

---

### D4-2: New Safe Home Position (Gripper Mounted)

The old `Q_safe = [12.30°, −110.36°, 95.90°, −75.48°, −89.59°, 12.33°]` was set without the gripper mounted. With the gripper attached (190mm extension, 0.6kg), several joints were in unsafe positions relative to the table.

Used teach mode to find a new home position with gripper mounted and clear of all surfaces. Recorded via `ros2 topic echo /arm/joint_states --once`.

**New Q_safe:** `[34.65°, −105.31°, 101.03°, 1.90°, 38.37°, 188.78°]` → TCP `[−0.175, −0.333, 0.435]`

Updated `ur5.py` line 94.

**Gripper payload setup on pendant (required for safe motion):**
- Pendant → Installation → TCP → set payload: **0.6 kg**, CoG Z = **95 mm**

---

### D4-3: RTDE Register Conflict — Root Cause Identified

`RTDEControlInterface` failed with "RTDE input registers already in use" persistently across safety restarts and process kills. Previous understanding (Day 3) attributed this to a stale RTDE session — incorrect.

**Root cause:** UR CB3 Installation files store **Modbus client signal definitions** that occupy RTDE input registers 18–31 even when the Modbus fieldbus adapter is toggled off. The adapter toggle controls connectivity, not register allocation. On every robot boot, the Installation is reloaded and the registers are re-occupied.

**Fix:** Complete installation reset on pendant (Hamburger → Setup Robot → Installations → New Installation → Save). This creates a blank installation with no Modbus signals, permanently freeing all RTDE registers.

**Attempted workarounds that failed:**
- `FLAG_UPPER_RANGE_REGISTERS` — failed because all registers 18–31 were occupied (not just 18–23)
- `FLAG_NO_WAIT` — failed, different error
- `restart safety` + `power on` + `brake release` dashboard commands — registers re-occupied on reload

---

### D4-4: Emergency Stop Incident — Joint Flip on move_safe

**What happened:** Called `/arm/move_safe` from joint state [52.5°, −93.6°, 123.1°, **355.9°**, 81.8°, 183.7°]. Q_safe target wrist_1 = **1.9°**. Shortest path = 6°. RTDE `moveJ` took the **long path (354°)**, causing a violent wrist_1 rotation that triggered the robot emergency stop.

**Recovery:** Dashboard `restart safety` → `power on` → `brake release`. Physical e-stop button required release on pendant before robot would accept power-on.

**Root cause of flip:** `moveJ` picks joint paths based on angular distance in the joint controller's coordinate space. When one joint is near 0°/360°, the controller may choose the long path. This is a known limitation — the solution is to ensure Q_safe joints are set from the same arm configuration as the one that will call `move_safe` (no large joint angle discontinuities).

**Post-incident Q_safe update:** Updated to actual post-e-stop arm position: `[41.2°, −101.0°, 109.2°, 132.2°, 54.3°, 186.9°]` (TCP z = 0.480 m, arm safely upright).

**Change to `_safe_abort` in `deligrasp_node.py`:** Removed `/arm/move_safe` call from error recovery path. Now only opens gripper and stops arm in place — prevents triggering a joint flip on any grasp failure. (See Day 5 D5-1.)

---

### D4-5: Dry-Run Analysis Script

Added `scripts/test_dryrun_analyze.py` — runs the full DeliGrasp perception and Gemini parameter pipeline with **zero arm/gripper movement**. Outputs:
- Camera frame metadata (resolution, depth range)
- Grounding DINO detection (bounding box, confidence)
- 3D world position of detected object
- Computed approach and grasp Z heights
- Full Gemini descriptor response
- Computed grip params (initial_force, aperture, slip response)
- Annotated image saved to `/tmp/dryrun_detection.jpg`

**Verified (red block):** Detection 87.6% confidence, block at world (0.315, −0.265, 0.226 m), initial_force = 3.68 N.

---

### D4-6: LabelDINO API Fix

The installed `magpie_perception.label_dino.LabelDINO` class does not call `init()` in `__init__()` — the HuggingFace processor and model are only loaded when `init()` is called explicitly. Calling `label()` before `init()` raises `AttributeError: 'LabelDINO' object has no attribute 'processor'`.

Additionally, the installed version passes `box_threshold=` to `post_process_grounded_object_detection()`, but the installed version of `transformers` renamed this argument to `threshold=`.

**Fix in `deligrasp_node.py` and `test_dryrun_analyze.py`:** Bypass `LabelDINO` entirely. Call Grounding DINO directly via `AutoProcessor` + `AutoModelForZeroShotObjectDetection` from HuggingFace `transformers`, using the correct `threshold=` argument.

---

## Day 5 — Full Pipeline Run (2026-05-27)

### D5-1: safe_abort Fix — Stop in Place Instead of move_safe

**Problem:** `deligrasp_node._safe_abort()` called `/arm/move_safe` on any grasp failure. Given the Q_safe joint flip risk (D4-4), this was a safety hazard — a detection failure mid-grasp could trigger a 354° wrist rotation.

**Fix:** `_safe_abort()` now:
1. Calls `/gripper/open` — releases the object
2. Calls `/arm/stop` — halts arm immediately in current position

No joint-space move is made. The arm stays where it is, which is always safe relative to a joint flip.

Added `self.cli_stop` client (`/arm/stop`) to `DeliGraspNode.__init__`.

### D5-2: Full Pipeline Status at Start of Day

| Component | Status |
|---|---|
| UR5 arm (192.168.0.4) | Connected, RTDE running |
| RealSense D405 | USB connected, camera node publishing |
| MAGPIE gripper | USB connected, gripper_node responding |
| `ur5_node` | Running |
| `gripper_node` | Running |
| `deligrasp_node` | Pending start |

Arm speed set to **10% speed, 5% acceleration** before any motion.

### D5-3: Selectable Object Detector — ViT vs VLM

Two detection backends are now supported, selectable at runtime.

#### Grounding DINO (`--detector grounding_dino`, default)
- **Type:** Vision Transformer (ViT) fine-tuned for zero-shot detection
- **How it works:** Takes image + text query as input. A trained detection head produces bounding boxes + confidence scores directly.
- **Where it runs:** Locally on the lab machine. Model cached at `~/.cache/huggingface/`.
- **Speed:** ~2–5 s on CPU, <1 s on GPU.
- **API cost:** None. Set `TRANSFORMERS_OFFLINE=1` to suppress HuggingFace network calls.
- **Output:** Bounding box (xyxy pixels) + confidence score per detection.
- **Best for:** Reliable localization. Specialist model — only does detection.

#### Gemini VLM (`--detector gemini`)
- **Type:** Vision-Language Model (multimodal LLM)
- **How it works:** The full camera frame is encoded as JPEG and sent to `gemini-2.5-flash` alongside a text prompt asking for the bounding box. Gemini returns coordinates as text in `[y_min, x_min, y_max, x_max]` format normalized 0–1000.
- **Where it runs:** Google cloud API. Requires `GEMINI_API_KEY`.
- **Speed:** ~1–2 s, network-dependent.
- **API cost:** Tokens per image (free tier available at aistudio.google.com).
- **Output:** Single bounding box (no confidence score — assigned 0.9 as placeholder). Can be combined with the DeliGrasp descriptor in one call.
- **Best for:** Flexibility. A general-purpose model that can reason about unusual objects without retraining.

#### Key tradeoff
DINO is a specialist — precise pixel-level localization, no internet needed. Gemini is a generalist — treats detection as a language task, less precise on coordinates but can do detection + grasp parameter estimation in a single API call.

#### Code changes
- `deligrasp_node.py`: `_detect()` branches on `detector_type` ROS parameter. New `_detect_gemini()` method handles the Gemini vision path.
- `_load_perception()`: accepts `gemini` as valid type (no model to load at startup).
- `test_dryrun_analyze.py`: added `--detector grounding_dino|gemini` CLI argument.

#### Usage
```bash
# Local DINO (default, no API cost)
python3 ~/magpie_control/scripts/test_dryrun_analyze.py --object "red block"

# Gemini cloud vision
python3 ~/magpie_control/scripts/test_dryrun_analyze.py --object "red block" --detector gemini

# Full deligrasp_node with Gemini detector
ros2 run magpie_control deligrasp_node --ros-args -p detector_type:=gemini
```

---

### D5-4: SAM3 — Third Object Detector (Segment Anything Model 3)

SAM3 (Segment Anything Model 3) is Meta's unified promptable segmentation model, released 2026-03-27. Unlike Grounding DINO (which outputs bounding boxes) or Gemini VLM (which outputs a single text-format box), SAM3 produces a **pixel-level binary mask** for the detected object in addition to bounding boxes and confidence scores. The mask enables more accurate depth sampling — instead of a 10×10 pixel centroid patch, the depth is sampled over every pixel belonging to the object.

**Model specs:** ~848M parameters. Requires Python 3.12+, PyTorch 2.7+, CUDA 12.6+.  
**HuggingFace repo:** `facebook/sam3` (gated — access request required at huggingface.co/facebook/sam3).  
**SAM3 API:** `build_sam3_image_model()` + `Sam3Processor` — takes text prompt directly, no separate detection step.

#### Why a Subprocess Bridge

ROS 2 Humble mandates Python 3.10. SAM3 requires Python 3.12 (C extension `sam3._C` compiled against 3.12 only). These cannot coexist in one Python environment.

**Solution:** SAM3 runs in an isolated Python 3.12 virtual environment (`~/sam3_env`). The ROS stack calls it via `subprocess.run()`, passing the image path and text query as arguments and reading a JSON result back over stdout. The JSON carries base64-encoded mask data.

```
deligrasp_node.py (Python 3.10)
  → subprocess.run([~/sam3_env/bin/python3, scripts/sam3_infer.py, --image, --query])
    → SAM3 model in Python 3.12
    → JSON stdout: {"boxes": [...], "scores": [...], "mask_b64": "...", "mask_shape": [H, W]}
```

On any failure (model error, subprocess crash, timeout), the SAM3 path falls back to Grounding DINO automatically.

#### sam3_env Setup

```bash
# Install Python 3.12 (deadsnakes PPA)
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update && sudo apt install python3.12 python3.12-venv python3.12-dev

# Create isolated venv (never pollutes ROS Python 3.10)
python3.12 -m venv ~/sam3_env

# Install CUDA PyTorch for RTX 2070 (CUDA 12.8)
~/sam3_env/bin/pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128

# Install SAM3 from source
cd /home/user/sam3
~/sam3_env/bin/pip install -e .

# Install required extras
~/sam3_env/bin/pip install einops pycocotools psutil opencv-python-headless \
    scikit-image pandas matplotlib "numpy<2"
```

**Note on numpy:** SAM3 requires `numpy<2`. If `opencv-python` pulls numpy 2.x, force the downgrade with `pip install "numpy<2"` after.

#### HuggingFace Gated Access

SAM3 weights are gated — the model download will fail without approval.

1. Create an account at huggingface.co
2. Visit `huggingface.co/facebook/sam3` and click **"Request access"**
3. Wait for approval email (typically minutes to hours)
4. Login from the sam3_env:
   ```bash
   HF_HUB_OFFLINE=0 ~/sam3_env/bin/huggingface-cli login
   # Paste token from huggingface.co/settings/tokens
   ```

**Note:** If `TRANSFORMERS_OFFLINE=1` is set in `.bashrc` (which blocks HF downloads in ROS), the subprocess sets `HF_HUB_OFFLINE=0` explicitly before any imports. This overrides the env var for the subprocess only.

#### RTX 2070 (Turing) Float32 Patch

RTX 2070 (SM 7.5, Turing architecture) does not support BFloat16 hardware operations. SAM3 was designed for Ampere (A100/H100) and hardcodes `torch.bfloat16` throughout — in `torch.autocast()` contexts, explicit `.to(torch.bfloat16)` tensor casts, and image preprocessing. Two global replacements were applied to the SAM3 source:

```bash
# Pass 1: bfloat16 → float16 (initial patch)
find /home/user/sam3/sam3 -name "*.py" -exec sed -i 's/torch\.bfloat16/torch.float16/g' {} \;

# Pass 2: float16 → float32 (required because float16 autocast fought float32 model weights)
find /home/user/sam3/sam3 -name "*.py" -exec sed -i 's/torch\.float16/torch.float32/g' {} \;

# Clear compiled bytecode cache
find /home/user/sam3 -name "*.pyc" -delete
```

After these patches, SAM3 runs entirely in float32 — autocast contexts, image tensors, backbone features, and matmul inputs all use the same dtype. This is slower than bfloat16 on Ampere hardware but correct and stable on Turing.

**Remaining "bfloat16" text:** SAM3 source still contains ~17 occurrences of the string "bfloat16" in variable names (e.g., `backbone_fpn_bf16`) and comments. These are harmless — no actual `torch.bfloat16` dtype references remain.

#### Mask-Based Depth Sampling

With a SAM3 pixel mask, depth estimation is more accurate than the 10×10 centroid patch used by DINO/Gemini:

- **Centroid:** geometric mean of all `True` pixels in the mask (`xs.mean(), ys.mean()`). More accurate than the bounding box centre because the mask follows the object shape.
- **Depth:** median of all `depth[mask]` pixels with valid depth (> 0). Rejects noise and background pixels inside the bounding box that don't belong to the object.

A green semi-transparent overlay is drawn on the annotated detection image to visualize the mask.

#### New Files

| File | Description |
|---|---|
| `scripts/sam3_infer.py` | Standalone Python 3.12 subprocess script. Loads SAM3, runs inference, outputs JSON to stdout. |

#### Code Changes

| File | Change |
|---|---|
| `scripts/test_dryrun_analyze.py` | Added `detect_object_sam3()`, `--detector sam3` CLI option, mask overlay + centroid logic |
| `src/magpie_control/deligrasp_node.py` | Added `_detect_sam3()`, 4-tuple return from `_detect()`, mask-based depth sampling in `_run_grasp_pipeline()` |

#### Usage

```bash
# Dry-run with SAM3
export GEMINI_API_KEY=your_key
source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
python3 ~/magpie_control/scripts/test_dryrun_analyze.py --object "measuring tape" --detector sam3

# deligrasp_node with SAM3
ros2 run magpie_control deligrasp_node --ros-args -p detector_type:=sam3
```

**Note:** The `--detector sam3` path runs DINO first to get a bounding box, then passes the image to SAM3 for the pixel mask. If SAM3 fails for any reason, the result falls back silently to the DINO detection (mask = None, centroid depth patch used).

### D5-5: SAM3 Speed Optimisations — Socket Server + Parallel Execution

Two independent optimisations were added to reduce per-grasp latency.

#### Problem

SAM3 cold-loads the model from disk to GPU in ~12 s every time the node or dry-run script starts. Additionally, the Gemini descriptor was called sequentially after detection, adding another ~3 s.

#### Optimisation 1 — Persistent Background Socket Server

`sam3_infer.py` gained a `--socket` mode. Instead of loading the model fresh per call, start the server once and leave it running. Any client connects to the Unix socket `/tmp/sam3.sock`, sends a JSON request, receives a JSON response, and disconnects. The model stays resident on the GPU.

**Server start (one-time per session):**
```bash
~/sam3_env/bin/python3 ~/magpie_control/scripts/sam3_infer.py --socket
# Prints: {"status": "ready", "socket": "/tmp/sam3.sock"}  after ~12 s
# Leave this terminal open — the model is now always hot
```

**Protocol:** same JSON as stdin server — `{"image": "/tmp/x.jpg", "query": "object"}` in, `{"boxes": ..., "scores": ..., "mask_b64": ..., "mask_shape": ...}` out.

**Auto-detection in `deligrasp_node.py`:** at startup, if `/tmp/sam3.sock` exists the node uses the socket client; otherwise it falls back to spawning the inline stdin server as before. No flag needed.

**Result:** per-call SAM3 latency drops from ~12 s (cold) to ~1 s (warm socket).

#### Optimisation 2 — Parallel Detection + Gemini Descriptor

The Gemini descriptor needs only the object name — it is independent of detection output. Both are submitted to a `ThreadPoolExecutor` at the same time. On a typical Gemini call (~3 s) the descriptor finishes well before (or concurrent with) detection, adding zero extra latency.

Applied in both `scripts/test_dryrun_analyze.py` and `src/magpie_control/deligrasp_node.py`.

#### Combined Latency

| Mode | SAM3 | Gemini | Total |
|---|---|---|---|
| Cold (original) | 12 s | ~3 s (hidden by SAM3) | ~14–16 s |
| Socket server (new) | ~1 s | ~3 s (parallel) | ~4 s typical |

#### New / Changed Files

| File | Change |
|---|---|
| `scripts/sam3_infer.py` | Added `--socket [path]` mode: Unix socket server, `_run_socket_server()` |
| `scripts/test_dryrun_analyze.py` | Added `detect_object_sam3_socket()`, `--socket` flag, `start_sam3_server()` helper, labels cold/warm/socket in output |
| `src/magpie_control/deligrasp_node.py` | `_sam3_sock_path` state, auto-detects `/tmp/sam3.sock` at startup, socket client path in `_detect_sam3()` |

### D5-6: Segmented Point Cloud + PCA Grasp Orientation

#### Motivation

The previous depth approach collapsed the entire SAM3 mask to a single median depth value, losing all spatial structure. A full point cloud back-projects every masked pixel individually, giving object dimensions and orientation in addition to position.

#### Pipeline

```
SAM3 mask (H×W bool)  +  RealSense depth (H×W mm)
        │
        ▼
Back-project each masked pixel using camera intrinsics:
  x = (u - cx) * z / fx
  y = (v - cy) * z / fy
  z = depth[v, u] / 1000.0
        │
        ▼
Transform to world frame:  pts_world = TCP @ TCP_TO_CAM @ pts_cam
        │
        ▼
open3d statistical outlier removal (20 neighbours, 2σ threshold)
Strips stray depth-noise points at mask edges
        │
        ▼
PCA via numpy (np.linalg.eigh on covariance matrix):
  eigenvector 0 = major axis (longest dimension)
  eigenvector 1 = minor axis
  eigenvector 2 = normal axis
        │
        ▼
Grasp angle = atan2(major_y, major_x) + 90°
Gripper fingers close perpendicular to major axis
```

#### What open3d does

open3d is used for two things only — the rest is numpy:
- **Container:** `o3d.geometry.PointCloud` holds the XYZ array
- **Denoiser:** `remove_statistical_outlier()` removes points whose mean distance to their 20 nearest neighbours exceeds 2σ. Typical reduction: ~36,000 → ~35,700 points (edge noise stripped).

PCA, centroid, extent, and grasp angle are all computed with numpy.

#### Verified output (measuring tape, 2026-05-27)

```
Points (after denoise): 35699
Centroid (world):  x=-0.248  y=-0.643  z=0.079 m  (matches median method to <1 mm)
Extent:   major=7.9 cm  minor=9.2 cm  depth=4.8 cm
Major axis direction: [0.03, -1.00, 0.08]  (≈ world -Y)
Grasp angle (wrist Z): 1.4 deg
```

The measuring tape is circular (tape reel) so major ≈ minor. For elongated objects (pen, screwdriver) the grasp angle gives a meaningful wrist rotation.

#### New Files

| File | Description |
|---|---|
| `scripts/pointcloud_utils.py` | `build_segmented_pcd()`, `denoise_pcd()`, `analyse_pcd()`, `print_pcd_results()` |

#### Changed Files

| File | Change |
|---|---|
| `scripts/test_dryrun_analyze.py` | Added `--pcd` flag, calls point cloud pipeline after SAM3 detection |

#### Usage

```bash
python3 ~/magpie_control/scripts/test_dryrun_analyze.py \
  --detector sam3 --socket --pcd --object "measuring tape"
```

**Requires:** `--detector sam3` (needs pixel mask). Falls back gracefully with a warning if open3d is unavailable or mask is empty.

---

## Day 5B — Safety Hardening + Gripper Close Drop (2026-05-27)

### D5B-1: Dry-Run Script — Tunable Motion Parameters + Hard Safety Abort

Previously the dry-run script had `APPROACH_H = 0.10` and `GRASP_OFF = 0.02` hardcoded. If someone ran the real node with different ROS params (`--ros-args -p approach_height:=0.05`), the dry-run safety check would use the wrong numbers and give a false OK.

**Changes to `scripts/test_dryrun_analyze.py`:**
- Added `--approach-height` (default 0.10 m) and `--grasp-offset` (default 0.02 m) CLI flags
- Safety check now uses these values so it always mirrors whatever you'd pass to the node
- Negative values rejected immediately before ROS even starts
- Safety check changed from a **warning** to a **hard abort** (`sys.exit(1)`) — prints which pose failed and how to fix it, then exits. No longer possible to read the results and ignore an unsafe destination
- Safety constants (`GRIPPER_LENGTH`, `MIN_FINGERTIP_Z`) moved to module level with a comment: "MUST match deligrasp_node.py"

**Usage:**
```bash
# Mirror the node's ROS params exactly
python3 scripts/test_dryrun_analyze.py \
  --detector sam3 --socket --pcd \
  --approach-height 0.10 --grasp-offset 0.02
```

---

### D5B-2: Auto Object Identification from Camera

Previously `--object` was required (defaulted to `"block"`). Now if omitted, Gemini Vision identifies the main graspable object in the camera frame before the pipeline starts.

**How it works:**
1. Grab one camera frame after sensor data is received
2. Send JPEG to Gemini 2.5 Flash: *"What is the main graspable object in this image? Reply in 2–4 words."*
3. Print `Detected object: "..."` and use that name for the rest of the pipeline (detection, descriptor, point cloud)
4. Hard abort if Gemini cannot identify anything

**Usage:**
```bash
# Let Gemini identify the object:
python3 scripts/test_dryrun_analyze.py --detector sam3 --socket --pcd

# Specify explicitly (unchanged):
python3 scripts/test_dryrun_analyze.py --detector sam3 --socket --pcd --object "measuring tape"
```

---

### D5B-3: Floor Safety Limit — Gripper Close Drop Measured

**Problem:** When the gripper closes, the fingers rotate on an arc (Dynamixel pivot geometry). The fingertips arc downward slightly as they close. The safety check was not accounting for this — it only checked the TCP-based fingertip Z before closing, not what happens during the close.

**Measurement procedure (teach mode):**
1. Open gripper fully (both fingers symmetric at ~103 mm aperture)
2. Moved arm to lowest safe position with gripper open → TCP = 0.261 m → fingertip = 0.030 m
3. Closed gripper (reset_parameters + calibrate + close to get both fingers symmetric)
4. Moved arm to lowest safe position with gripper closed → TCP = 0.282 m → fingertip = 0.051 m
5. Close drop = 0.051 − 0.030 = **21 mm**

**Simplification:** Rather than tracking open vs. closed limits separately (with a `GRIPPER_CLOSE_DROP` parameter), a single floor limit was set to cover both:
- `MIN_FINGERTIP_Z = 0.047 m` — applies to all poses (approach, grasp, retreat)
- Any pose that would put TCP-based fingertip Z below 0.047 m is rejected before any movement

This means the arm never approaches a height where closing the gripper could hit the floor, regardless of the current pose.

**Updated in both `deligrasp_node.py` and `scripts/test_dryrun_analyze.py`.**

---

### D5B-4: Gripper USB Re-enumeration Recovery

**Problem encountered:** The OpenRB-150 controller re-enumerated from `/dev/ttyACM1` to `/dev/ttyACM0` after a USB reconnect during the session. The `gripper_node` had opened `/dev/ttyACM1` at startup and was now flooding `[TxRxResult] Port is in use!` — the service returned `success: True` but nothing moved.

**Diagnosis:** `temperature: 0.0` in `/gripper/state` is the indicator — real AX-12 motors always report ~33–45°C. Zero means the node is not talking to hardware.

**Recovery:**
```bash
# 1. Kill the stale node
kill <gripper_node_pid>

# 2. Restart — auto-detection finds the new port
sg dialout -c "bash -c 'source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash && ros2 run magpie_control gripper_node'"
```

**Additional symptom found:** After restart, one finger was not moving (asymmetric finger positions in `/gripper/state`). Fixed by:
```bash
ros2 service call /gripper/reset_parameters std_srvs/srv/Trigger {}
ros2 service call /gripper/calibrate std_srvs/srv/Trigger {}
```
Both fingers returned to symmetric operation (~50 mm each when open, total ~103 mm).

**Root cause:** The `reset_parameters` + `calibrate` sequence re-homes both motors from a known position. If one motor has a stale overload flag from the port conflict, this clears it.

**Note:** `temperature > 0` in `/gripper/state` is the quickest way to confirm the node is actually talking to hardware. If it reads 0.0 after startup, suspect a port mismatch.

---

### TODO: MuJoCo Collision Checking

The UR5 has a publicly available MuJoCo XML model (MJCF). In the future, this can be loaded into a collision-checking pipeline to give per-pose floor and obstacle clearance based on the full kinematic chain rather than the single fingertip Z heuristic. Parked for now — current safety checks are sufficient for tabletop operation.

---

## Day 6 — 2026-05-28 A: Arc-Based Fingertip Drop Safety

### D6A-1: Motivation — Replacing the Fixed Floor Constant

**Professor feedback:** The single fixed `MIN_FINGERTIP_Z = 0.047 m` was too blunt. It was simultaneously too conservative for large objects (causing the arm to refuse valid grasps near the floor) and not conservative enough for small objects (where the gripper closes almost fully, and the arc drop is larger). The correct approach: calculate the drop from the gripper's 4-bar linkage geometry as a function of how much the gripper closes for a given object.

---

### D6A-2: Gripper Arc Model — `src/magpie_control/gripper_arc.py`

**Source geometry:** `https://github.com/correlllab/h1_mujoco/blob/main/magpie/ur5e.xml`

The Magpie gripper fingers are driven by a **parallelogram 4-bar linkage** (a = c ≈ 0.04500 m, b = d ≈ 0.02236 m in the YZ plane of `base_top` frame). Because it is a parallelogram, the `finger_combined` body **translates without rotating**, so the vertical fingertip drop equals the crank-arc Z-component.

**Key parameter derivations from the XML:**
| Symbol | Value | Source |
|--------|-------|--------|
| Crank pivot Y | 0.038 m | `left_crank` body position |
| Crank length | 0.04500 m | distance pivot1→pivot2 in YZ |
| Initial crank angle φ₀ | 0.4836 rad | atan2(0.020953, 0.039824) |
| Pad Y offset | 0.02227 m | `pad_box` positions in `left_finger_combined` |
| Arc peak aperture | ~31.5 mm | when φ₀+θ = π/2 |

**Arc peak:** The crank peaks (maximum Z drop) at finger aperture ≈ 31.5 mm. For any goal aperture below 31.5 mm, the gripper passes through the peak during closing, so the worst-case drop occurs at ~31.5 mm — not at the final position.

**Calibration scale _K ≈ 1.377:** Fitted from measured 21 mm drop (103.6 → 0 mm, 2026-05-27).

**Drop table (from 103.6 mm open):**
| Goal aperture | Arc drop | Required safe grasp Z |
|--------------|----------|----------------------|
| 103.6 mm | 0 mm | 0.030 m |
| 80 mm | ~15 mm | 0.045 m |
| 50 mm | ~24 mm | 0.054 m |
| ≤ 31.5 mm | ~25 mm (peak) | 0.055 m |
| 0 mm (fully closed) | ~25 mm (peak applies) | 0.055 m |

**Public API:**
- `fingertip_drop(aperture_open_mm, aperture_close_mm)` — worst-case downward displacement (m) during closing; uses arc peak for goals ≤ 31.5 mm
- `safe_grasp_z(aperture_close_mm, floor_z=0.030)` — minimum open-gripper fingertip Z required so closing to the goal aperture clears the physical floor

---

### D6A-3: Safety Check Changes

**`deligrasp_node.py`:**
- Removed `MIN_FINGERTIP_Z = 0.047` → replaced with `HARD_FLOOR_Z = 0.030` (physical floor, open-gripper measurement)
- `check_pose_safe(tcp_matrix, label, aperture_close_mm=None)` now calls `safe_grasp_z(ap, HARD_FLOOR_Z)` for grasp poses — required Z adapts to goal aperture
- Descriptor collected early (after approach move, before grasp safety check) so `goal_aperture_mm` is available for the check

**`test_dryrun_analyze.py`:**
- Same `HARD_FLOOR_Z = 0.030` constant
- Descriptor collected alongside detection result (parallel, then both awaited)
- Safety pre-flight prints `goal_aperture`, `arc drop`, and `min_grasp_z` dynamically

**Practical effect:**
```
=== SAFETY PRE-FLIGHT ===
  Physical floor:   0.030 m
  Goal aperture:    30.0 mm  →  worst-case arc drop 24.9 mm
  Min grasp Z:      0.055 m  (floor + drop, 4-bar arc model)
  Approach: 0.165 m  ✓ SAFE
  Grasp:    0.085 m  (need ≥ 0.055)  ✓ SAFE
=========================
```
