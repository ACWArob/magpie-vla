# MAGPIE Hardware Testing Reference

Hardware test commands for each component. Run in order — each stage assumes the previous passed.

---

## Prerequisites (run before any stage)

```bash
# Source workspace (every terminal)
source /opt/ros/humble/setup.bash
source ~/ws_ctrl/install/setup.bash

# Kill any stale node processes
pkill -f gripper_node; pkill -f ft_sensor_node; pkill -f ur5_node; pkill -f deligrasp_node
```

---

## Stage 1 — Gripper

**Hardware required:** MAGPIE gripper powered (12V supply on), OpenRB-150 USB cable plugged in.

**Terminal 1 — start node:**
```bash
sg dialout -c "bash -c 'source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash && ros2 run magpie_control gripper_node'"

Open a second terminal and run the tests:

source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.
```

**Terminal 2 — run tests:**

```bash
# Check state (expect: position ~103mm, force ~0N, temp ~33°C)
ros2 topic echo /gripper/state --once

# Open gripper
ros2 service call /gripper/open std_srvs/srv/Trigger {}

# Close gripper
ros2 service call /gripper/close std_srvs/srv/Trigger {}

# Move to specific position (e.g. 50mm aperture, half speed)
ros2 service call /gripper/set_position \
  magpie_msgs/srv/SetGripperPosition "{position: 50.0, speed: 0.5}"

# Set force limit (1.5 N — gentle grasp)
ros2 service call /gripper/set_force \
  magpie_msgs/srv/SetGripperForce "{max_force: 1.5}"

# Calibrate (opens, waits 2s, closes — resets motor reference)
ros2 service call /gripper/calibrate std_srvs/srv/Trigger {}

# Clear motor overload error (re-enables torque without resetting all params)
ros2 service call /gripper/clear_error std_srvs/srv/Trigger {}

# Reset all parameters to defaults (also opens gripper)
ros2 service call /gripper/reset_parameters std_srvs/srv/Trigger {}
```

**Expected results:**

| Test | Expected |
|---|---|
| `/gripper/state` | position ~103mm when open, force ~0N, temp ~33–35°C |
| `/gripper/close` | position drops to ~0mm, force rises to ~1–2N |
| `/gripper/open` | position returns to ~103mm |
| `/gripper/set_position 50mm` | position ~50mm |
| `/gripper/clear_error` | `success: True, message: 'Motor torque re-enabled'` |

**Verified:** position=103.6mm open, 1.84N contact force on close. ✓

---

## Stage 2 — F/T Sensor

**Hardware required:** Machine Ethernet port connected to robot network switch (IP 192.168.0.7/24 via DHCP). F/T sensor powered at 192.168.0.6.

**Terminal 1 — start node:**
```bash
source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
ros2 run magpie_control ft_sensor_node --ros-args -p ip_address:=192.168.0.6 -p poll_rate:=50
```

**Terminal 2 — run tests:**

```bash
# Check one reading (expect: small non-zero values near 0, frame_id = ft_sensor)
ros2 topic echo /ft_sensor/wrench --once

# Verify publish rate (~50 Hz)
ros2 topic hz /ft_sensor/wrench --window 20

# Zero/bias the sensor (call while no force applied)
ros2 service call /ft_sensor/zero std_srvs/srv/Trigger {}
```

**Expected results:**

| Test | Expected |
|---|---|
| `/ft_sensor/wrench` | `frame_id: ft_sensor`, force/torque ~0 (small noise ±0.1 N) |
| `topic hz` | ~50 Hz |
| `/ft_sensor/zero` | `success: True` |

**Note:** Requires direct Ethernet to robot network (192.168.0.x subnet). WiFi-only will connect but return no data (UDP responses do not route back through gateway).

**Verified:** 49.95 Hz, F=[−0.1, 0.1, 0.4] N, T=[0.016, −0.005, −0.003] N·m at rest. ✓

---

## Stage 3 — UR5 Arm (read-only)

**Hardware required:** Machine Ethernet connected to robot network (192.168.0.7/24). UR5 powered on and initialized (brakes released). Fieldbus adapters (EtherNet/IP, PROFINET, Modbus) disabled on pendant — Settings → System → Fieldbus.

**Terminal 1 — start node:**
```bash
source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
ros2 run magpie_control ur5_node
```

**Terminal 2 — run tests:**
```bash
# Read TCP pose (position + orientation of end-effector)
ros2 topic echo /arm/tcp_pose --once

# Read all 6 joint positions (radians)
ros2 topic echo /arm/joint_states --once

# Call get_pose service (returns pose + joint array)
ros2 service call /arm/get_pose magpie_msgs/srv/GetPose {}

# Verify publish rate (~500 Hz)
ros2 topic hz /arm/tcp_pose --window 20
```

**Expected results:**

| Test | Expected |
|---|---|
| `/arm/tcp_pose` | `frame_id: base`, position in metres, quaternion orientation |
| `/arm/joint_states` | 6 joints named shoulder_pan through wrist_3, positions in radians |
| `/arm/get_pose` | `success: True`, pose + 6-element joint array |
| `topic hz` | ~500 Hz |

**Note:** If node fails with "RTDE input registers already in use", a fieldbus adapter is still enabled on the pendant. Disable it and restart the robot controller.

**Note:** Service type is `magpie_msgs/srv/GetPose` — not `std_srvs/srv/Trigger`.

**Verified:** 500 Hz, TCP at [−0.279, −0.392, 0.428] m, all 6 joints reporting, get_pose returns success: True. ✓

---

## Stage 4 — UR5 Arm (motion)

**Hardware required:** Same as Stage 3. **Clear the area around the arm before running any motion command.**

**Terminal 1 — start node:**
```bash
source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
ros2 run magpie_control ur5_node
```

**Terminal 2 — run tests:**
```bash
# 1. Set slow speed before any motion (20% speed/accel)
ros2 service call /arm/set_speed magpie_msgs/srv/SetSpeed "{speed: 0.2, acceleration: 0.2}"

# 2. Move to safe/home position — arm moves to [12°, -110°, 96°, -75°, -90°, 12°]
ros2 service call /arm/move_safe std_srvs/srv/Trigger {}

# 3. Verify pose at home
ros2 topic echo /arm/tcp_pose --once

# 4. move_j — small wrist rotation from home (+15° on wrist_3)
ros2 service call /arm/move_j magpie_msgs/srv/MoveJoint \
  "{joint_positions: [0.2147, -1.9254, 1.6737, -1.3175, -1.5637, 0.4770], speed: 0.0, acceleration: 0.0, async_mode: false}"

# 5. Verify joint positions changed
ros2 topic echo /arm/joint_states --once

# 6. Return to home
ros2 service call /arm/move_safe std_srvs/srv/Trigger {}

# 7. move_l — move 5 cm up in Z (linear Cartesian move)
ros2 service call /arm/move_l magpie_msgs/srv/MoveLinear \
  "{target_pose: {position: {x: -0.296, y: -0.177, z: 0.553}, orientation: {x: 0.7071, y: 0.7071, z: 0.0, w: 0.0003}}, speed: 0.0, acceleration: 0.0, async_mode: false}"

# 8. Verify TCP z increased by ~0.05 m
ros2 topic echo /arm/tcp_pose --once

# 9. Return to home
ros2 service call /arm/move_safe std_srvs/srv/Trigger {}

# 10. Teach mode (freedrive) — arm goes limp, you can push it by hand
ros2 service call /arm/teach_mode std_srvs/srv/Trigger {}
# Push the arm gently to verify freedrive is active
# Call again to disable
ros2 service call /arm/teach_mode std_srvs/srv/Trigger {}

# 11. Emergency stop (call mid-motion to test)
ros2 service call /arm/stop std_srvs/srv/Trigger {}
```

**Expected results:**

| Test | Expected |
|---|---|
| `set_speed` | `success: True, message: 'Speed set to 0.20, accel to 0.20'` |
| `move_safe` | Arm moves to home, `success: True, message: 'At safe position'` |
| `/arm/tcp_pose` at home | ~[−0.296, −0.177, 0.503] m |
| `move_j` | wrist_3 rotates ~15°, `success: True` |
| `/arm/joint_states` after move_j | joint[5] (wrist_3) changed from ~0.215 to ~0.477 rad |
| `move_l` | arm moves straight up 5 cm, `success: True` |
| `/arm/tcp_pose` after move_l | z increased by ~0.05 m to ~0.553 |
| `teach_mode` (1st call) | freedrive enabled — arm can be pushed by hand |
| `teach_mode` (2nd call) | freedrive disabled — arm holds position |
| `stop` | `success: True`, any motion halts immediately |

**Note:** `speed: 0.0` and `acceleration: 0.0` in move_j/move_l means "use the value set by set_speed" — the node falls back to the stored speed when 0 is passed.

**Important notes:**
- Always `set_speed` to a low value (0.2) before moving
- If node fails with "RTDE input registers already in use", check for stale ur5_node processes: `pkill -f ur5_node`
- Kill the node by PID (not pkill) to allow clean shutdown — avoids leaving a stale RTDE connection that blocks the next start

**Verified:** move_safe succeeded at 20% speed, arm moved to home position. ✓

---

## Stage 5 — Full Pipeline (camera + DeliGrasp)
*(to be added after hardware test)*
