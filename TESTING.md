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
*(to be added after hardware test)*

---

## Stage 3 — UR5 Arm (read-only)
*(to be added after hardware test)*

---

## Stage 4 — UR5 Arm (motion)
*(to be added after hardware test)*

---

## Stage 5 — Full Pipeline (camera + DeliGrasp)
*(to be added after hardware test)*
