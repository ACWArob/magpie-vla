# Sensor Poll Rate Measurements

**Date:** 2026-06-05  
**Measured with:** `scripts/measure_poll_rates.py` (10 s window)

## Results

| Sensor | Topic | Configured Hz | Actual Hz | Jitter | Status |
|---|---|---|---|---|---|
| Arm joint states (RTDE) | `arm/joint_states` | 500 | **500.0** | 0.5 ms | ✓ |
| Gripper state (Dynamixel) | `gripper/state` | 10 | **10.0** | 1.8 ms | ✓ |
| Camera color (RealSense D405) | `/camera/gripper_camera/camera/color/image_raw` | 10 | **10.0** | 3.9 ms | ✓ |
| F/T sensor (OptoForce) | `ft_sensor/wrench` | 50 | **NOT MEASURED** | — | ⚠ |

## Notes

- **Arm:** RTDE stream is rock solid — no dropped packets over 10 s window.
- **Gripper:** Dynamixel USB-serial; jitter is low and acceptable.
- **Camera:** RealSense D405 at default 10 Hz color stream. Depth stream not measured separately.
- **F/T sensor (OptoForce @ 192.168.0.5:49152):**
  - Device responds to ping (network reachable).
  - Node fails with `ConnectionRefusedError` on UDP port 49152 — firmware not listening.
  - Likely needs a power cycle to recover.
  - **TODO:** Power cycle OptoForce, re-run `ros2 run magpie_control ft_sensor_node`, then re-run `scripts/measure_poll_rates.py`.
  - Default configured rate is 50 Hz; hardware may support up to 250 Hz via `--ros-args -p poll_rate:=250`.

## Force Data Source

The grasp slip detection pipeline uses **gripper motor current sensing** (`gripper/state`), not the OptoForce.  
The OptoForce provides wrist-level 6-axis wrench data — useful for contact detection during arm motion but not currently used in the pipeline.
