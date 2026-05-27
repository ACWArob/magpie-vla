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

source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash

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

## Stage 5 — Gemini + Gripper (DeliGrasp descriptor → grasp)

**Hardware required:** MAGPIE gripper (12V + USB). No camera or arm needed.

Gemini estimates physics from the object name alone (text-only — no image).

**Prerequisites:**
```bash
export GEMINI_API_KEY=your_key_from_aistudio_google_com
```

**Terminal 1 — start gripper node:**
```bash
sg dialout -c "bash -c 'source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash && ros2 run magpie_control gripper_node'"
```

**Terminal 2 — run test:**
```bash
source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
python3 ~/magpie_control/scripts/test_gemini_gripper.py --object "wooden block"
```

**What it does:**
1. Sends `"Pick up the wooden block."` to Gemini 2.5 Flash with the DeliGrasp descriptor system prompt
2. Gemini returns structured description: mass, spring constant, friction coefficient, goal aperture, slip closure
3. Physics computed: `initial_force = mg/μ`, `add_force = max(0.05, k·Δx·0.0001)`
4. Opens gripper, sets force limit to computed `initial_force`
5. 3-second countdown — place object between fingers
6. Closes gripper (auto-stops at force limit on contact)
7. Holds 3 seconds, then opens

**Expected results:**

| Step | Expected |
|---|---|
| Gemini descriptor | Prints full structured description between `[start/end of description]` tags |
| Parsed params | mass, k, μ, aperture, closure all extracted |
| `initial_force` | Computed from `mg/μ`, typically 0.5–3.0 N for common objects |
| Gripper open | Opens to ~103 mm |
| Gripper close | Stops at object contact at computed force limit |
| Gripper open | Returns to ~103 mm |

**Note:** Get a free tier API key from **aistudio.google.com** — keys from Google Cloud Console have zero quota on free tier.

---

## Stage 5b — Object Detection: ViT vs VLM

Two detector backends are available for identifying the object in the camera frame. Choose one when running `test_dryrun_analyze.py` or `deligrasp_node`.

### Option A — Grounding DINO (ViT, default)

A Vision Transformer fine-tuned for zero-shot object detection. Runs locally — no API key or internet needed.

```bash
# Dry-run: detect block, get 3D position, get grip params — arm does not move
export GEMINI_API_KEY=your_key_from_aistudio_google_com
source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
python3 ~/magpie_control/scripts/test_dryrun_analyze.py --object "red block"
# --detector grounding_dino is the default, no flag needed
```

**Expected output:**
- Detection confidence score printed (e.g. `score=0.918`)
- Bounding box in pixels + centroid
- Depth at centroid, world XYZ, approach/grasp Z heights
- Gemini DeliGrasp descriptor + computed grip parameters
- Annotated detection image saved to `/tmp/dryrun_detection.jpg`

### Option B — Gemini VLM (cloud vision)

Sends the full camera frame to Gemini 2.5 Flash as a JPEG. Gemini returns the bounding box as text (`[y_min, x_min, y_max, x_max]`, 0–1000 normalized). Requires `GEMINI_API_KEY`.

```bash
export GEMINI_API_KEY=your_key_from_aistudio_google_com
source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
python3 ~/magpie_control/scripts/test_dryrun_analyze.py --object "red block" --detector gemini
```

**Expected output:**
- Gemini raw response printed (e.g. `Gemini VLM response: [234, 312, 456, 521]`)
- Bounding box converted from 0–1000 scale to pixel coordinates
- Same depth/3D/grip output as Option A

### Comparison

| | Grounding DINO | Gemini VLM |
|---|---|---|
| Runs | Locally | Google cloud |
| API key needed | No | Yes (`GEMINI_API_KEY`) |
| Speed | ~2–5 s CPU | ~1–2 s (network) |
| Confidence score | Yes (0–1) | No (fixed at 0.9) |
| Localization precision | High (trained detection head) | Lower (text output) |
| Unusual objects | Limited by training vocab | Handles anything |
| Internet required | No (set `TRANSFORMERS_OFFLINE=1`) | Yes |

**Note:** For `deligrasp_node`, set the detector via ROS parameter:
```bash
ros2 run magpie_control deligrasp_node --ros-args -p detector_type:=gemini
```
Default is `grounding_dino`.

---

## Stage 5c — SAM3 Object Detection (Segment Anything Model 3)

SAM3 is the third detector backend. It returns a pixel-level binary mask in addition to a bounding box, enabling more accurate depth sampling (entire object mask vs 10×10 centroid patch).

**Prerequisites:**

```bash
# HuggingFace access to facebook/sam3 must be approved
# sam3_env must be set up: ~/sam3_env/bin/python3 must exist
# Test sam3_env is functional:
~/sam3_env/bin/python3 -c "import sam3; print('SAM3 ok')"
```

**Setup (one-time, if not done):**

```bash
# Login to HuggingFace in sam3_env (required to download gated model weights)
HF_HUB_OFFLINE=0 ~/sam3_env/bin/huggingface-cli login
# Paste token from huggingface.co/settings/tokens
```

**Run dry-run test with SAM3:**

```bash
export GEMINI_API_KEY=your_key_from_aistudio_google_com
source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
python3 ~/magpie_control/scripts/test_dryrun_analyze.py --object "measuring tape" --detector sam3
```

**Expected output:**
- `SAM3 mask: XXXXX pixels  score=0.XXX` — mask pixel count and confidence
- `Centroid pixel: (u, v) (mask centroid)` — centroid from mask geometry, not box centre
- `Using SAM3 mask depth (XXXX pixels)` — depth sampled over entire mask
- Green overlay on `/tmp/dryrun_detection.jpg` showing segmented object
- Same depth/3D/approach-Z output as other detectors

**Fallback behaviour:**

If SAM3 fails for any reason (GPU error, timeout, model not downloaded), the test prints:
```
[WARN] SAM3 failed: <reason> — falling back to DINO
```
and continues with Grounding DINO results. No crash.

### Three-Way Detector Comparison

| | Grounding DINO | Gemini VLM | SAM3 |
|---|---|---|---|
| Type | ViT (detection head) | Multimodal LLM | Promptable segmentation |
| Runs | Local (GPU/CPU) | Google cloud | Local (GPU, Python 3.12) |
| Output | Bounding boxes | 1 bounding box | Bounding box + pixel mask |
| Depth sampling | 10×10 centroid patch | 10×10 centroid patch | All mask pixels (median) |
| Localization | High (trained head) | Lower (text output) | Highest (pixel-level) |
| Speed | ~2–5 s CPU / <1 s GPU | ~1–2 s (network) | ~3–8 s first run (GPU) |
| API key | No | Yes (`GEMINI_API_KEY`) | No (HF token for download only) |
| Requires internet | No | Yes | No (after first download) |
| Python | 3.10 | 3.10 | 3.12 subprocess |
| GPU dtype | float32 | N/A | float32 (Turing patch) |

**Default:** Grounding DINO — fastest, no API cost, sufficient precision for grasping.  
**Use SAM3 when:** Object depth needs to be accurate (irregular shapes, depth noise inside bounding box).  
**Use Gemini when:** Object is unusual or not in DINO's vocabulary, API key available.

---

## Stage 5d — SAM3 Socket Server (Persistent Warm Inference)

Tests the background socket server that keeps SAM3 hot between calls, reducing per-call latency from ~12 s to ~1 s.

**Prerequisites:**
- Stage 5c completed and verified (SAM3 working, float32 patch applied)
- Camera node running
- `~/sam3_env/bin/python3` functional

---

### Test A — Cold (single-shot, no server)

Baseline timing. Model loads fresh, runs once, exits. Use this to confirm the cold-load time on the current machine.

**Terminal 1 — camera:**
```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py camera_name:=gripper_camera
```

**Terminal 2:**
```bash
source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
export GEMINI_API_KEY=your_key
python3 ~/magpie_control/scripts/test_dryrun_analyze.py \
  --detector sam3 --object "measuring tape"
```

**Expected output:**
```
Running SAM3 (cold ~12s) + descriptor for "measuring tape" in parallel...
  SAM3 mask: XXXXX pixels  score=0.XXX
```
Total time: ~14–16 s (dominated by 12 s model load).

---

### Test B — Hot (background socket server)

Start the server once, then run the dry-run script. First terminal stays open for the whole session.

**Terminal 1 — camera:**
```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py camera_name:=gripper_camera
```

**Terminal 2 — SAM3 background server (leave running):**
```bash
~/sam3_env/bin/python3 ~/magpie_control/scripts/sam3_infer.py --socket
```
Wait for: `{"status": "ready", "socket": "/tmp/sam3.sock"}` (~12 s cold load, one-time).

**Terminal 3 — dry run (repeat as many times as needed):**
```bash
source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
export GEMINI_API_KEY=your_key
python3 ~/magpie_control/scripts/test_dryrun_analyze.py \
  --detector sam3 --socket --object "measuring tape"
```

**Expected output:**
```
Running SAM3 (socket ~1s) + descriptor for "measuring tape" in parallel...
  SAM3 mask: XXXXX pixels  score=0.XXX
```
Total time: ~3–5 s (SAM3 ~1 s, Gemini ~3 s in parallel). Re-running Terminal 3 immediately gives the same ~1 s SAM3 time — no reload.

**To verify server is being used:** check `/tmp/sam3.sock` exists while Terminal 2 is open.
```bash
ls -la /tmp/sam3.sock
```

---

### Speed Benchmark (optional)

Confirms cold vs warm timing in isolation without Gemini or ROS:

```bash
# Terminal 1 — camera required (grabs one frame then disconnects)
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py camera_name:=gripper_camera

# Terminal 2 — benchmark
source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
python3 ~/magpie_control/scripts/test_sam3_server.py --object "measuring tape" --runs 5
```

**Expected:**
```
Cold load:      ~12.0s  (one-time)
Warm inference: ~1.0s avg  (min ~0.98s, max ~1.2s)
Speedup:        ~11–12x faster after warmup
```

**Verified result (2026-05-27, RTX 2070):**
- Cold load: 11.9 s
- Warm inference: 1.02 s avg (5 runs)
- Speedup: 11.7×

---

## Stage 5e — Segmented Point Cloud + PCA Grasp Orientation

Builds a 3D point cloud from the SAM3 mask and depth image, denoises it, then computes the object centroid, physical dimensions, and optimal gripper rotation via PCA.

**Prerequisites:**
- Stage 5d completed (SAM3 socket server working)
- `open3d` installed: `pip install open3d`
- Camera node running, SAM3 background server running

**Terminal 1 — camera:**
```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py camera_name:=gripper_camera
```

**Terminal 2 — SAM3 background server:**
```bash
~/sam3_env/bin/python3 ~/magpie_control/scripts/sam3_infer.py --socket
```
Wait for `{"status": "ready", ...}`

**Terminal 3 — point cloud dry run:**
```bash
source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
export GEMINI_API_KEY=your_key
python3 ~/magpie_control/scripts/test_dryrun_analyze.py \
  --detector sam3 --socket --pcd --object "measuring tape"
```

**Expected output (additional section after detection):**
```
=== POINT CLOUD ANALYSIS ===
  Points (after denoise): ~35000–36000
  Centroid (world):  x=X.XXX  y=X.XXX  z=X.XXX m
  Extent:   major=X.X cm  minor=X.X cm  depth=X.X cm
  Major axis direction: [X.XX, X.XX, X.XX]
  Grasp angle (wrist Z): X.X deg
============================
```

**What to verify:**
- Centroid XYZ matches world-frame pos from depth median to within ~2 mm
- Extent values are physically plausible for the object (measuring tape ≈ 8×9 cm)
- No crash if mask is empty (falls back with `[WARN]`)

**Verified result (2026-05-27, measuring tape, RTX 2070):**
- Points after denoise: 35,699
- Centroid: (-0.248, -0.643, 0.079) m — matches median method to <1 mm
- Extent: major=7.9 cm, minor=9.2 cm, depth=4.8 cm
- Major axis: [0.03, -1.00, 0.08] (≈ world -Y)
- Grasp angle: 1.4° (circular tape, so near-zero is expected)

---

## Stage 5f — Safety Pre-flight + Auto Object Identification (2026-05-27)

Tests the hardened dry-run script: tunable motion params, hard abort on unsafe poses, and Gemini-based auto object identification.

**Prerequisites:**
- Camera node running
- SAM3 background server running (`/tmp/sam3.sock` exists)
- `GEMINI_API_KEY` exported

---

### Test A — Auto object identification (no `--object` flag)

```bash
source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
export GEMINI_API_KEY=your_key
python3 ~/magpie_control/scripts/test_dryrun_analyze.py \
  --detector sam3 --socket --pcd
```

**Expected output:**
```
No --object specified. Asking Gemini to identify the object...
  Detected object: "<object name>"
Running SAM3 (socket ~1s) + descriptor for "<object name>" in parallel...
```
Then the full detection + point cloud + safety + grip params output as normal.

---

### Test B — Safety pre-flight check output

With a detected object, the output should include:

```
  Motion params: approach_height=0.100 m  grasp_offset=0.020 m
  Approach: fingertip Z=X.XXX m  TCP Z=X.XXX m
  Grasp:    fingertip Z=X.XXX m  TCP Z=X.XXX m

=== SAFETY PRE-FLIGHT ===
  Floor limit: 0.047 m  (closed-gripper floor, applies to all poses)
  Approach fingertip Z: X.XXX m  ✓ SAFE
  Grasp    fingertip Z: X.XXX m  ✓ SAFE
=========================
```

**If object is too close to the floor**, both lines show `✗ UNSAFE` and the script exits with:
```
ABORTED — arm would crash into floor with these parameters.
Adjust --approach-height / --grasp-offset, or move object higher.
```

---

### Test C — Custom motion params

```bash
python3 ~/magpie_control/scripts/test_dryrun_analyze.py \
  --detector sam3 --socket --pcd \
  --approach-height 0.15 --grasp-offset 0.01
```

**Expected:** safety check uses 0.15 m and 0.01 m values (not hardcoded 0.10/0.02).

---

### Floor limit constants

| Constant | Value | Source |
|---|---|---|
| `MIN_FINGERTIP_Z` | 0.047 m | Teach mode measurement 2026-05-27 |
| `GRIPPER_LENGTH` | 0.231 m | `magpie_tooltip[2]` in ur5.py |

Both constants must match between `deligrasp_node.py` and `test_dryrun_analyze.py`.

---

## Stage 6 — Arm + Gripper (coordinated motion)

**Hardware required:** UR5 powered + initialized, MAGPIE gripper mounted on end-effector (12V + USB). Ethernet to robot network. **Clear the area around the arm before running.**

**Gripper constraints:**
- Adds **0.6 kg** payload
- Extends **190 mm** (7.5 in) forward from the wrist
- **80 mm** wide at the fingertips — keep clearance on both sides
- **Cables restrict wrist rotation** — do not rotate wrist_3 beyond ±30° from home

---

**Pendant setup (do once before this stage):**

1. On the pendant go to **Installation → Payload**
2. Set **mass = 0.6 kg**
3. Set **center of gravity Z = 95 mm** (roughly halfway along the gripper)
4. Save and confirm — this prevents the arm from triggering a protective stop when it moves with the gripper

---

**Terminal 1 — start arm node:**
```bash
source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
ros2 run magpie_control ur5_node
```

**Terminal 2 — start gripper node:**
```bash
sg dialout -c "bash -c 'source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash && ros2 run magpie_control gripper_node'"
```

**Terminal 3 — run tests:**
```bash
source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash

# 1. Set slow speed (10%)
ros2 service call /arm/set_speed magpie_msgs/srv/SetSpeed "{speed: 0.1, acceleration: 0.1}"

# 2. Return to home
ros2 service call /arm/move_j magpie_msgs/srv/MoveJoint \
  "{joint_positions: [0.6048, -1.8380, 1.7633, 0.0331, 0.6697, 3.2949], speed: 0.0, acceleration: 0.0, async_mode: false}"

# 3. Confirm TCP pose at home (expect x≈-0.175, y≈-0.333, z≈0.435)
ros2 topic echo /arm/tcp_pose --once

# 4. Open gripper
ros2 service call /gripper/open std_srvs/srv/Trigger {}

# 5. Set gentle force limit
ros2 service call /gripper/set_force magpie_msgs/srv/SetGripperForce "{max_force: 1.5}"

# 6. Move UP 5 cm — safe direction first, confirms arm responds correctly
ros2 service call /arm/move_l magpie_msgs/srv/MoveLinear \
  "{target_pose: {position: {x: -0.175, y: -0.333, z: 0.485}, orientation: {x: 0.063, y: 0.697, z: -0.714, w: 0.019}}, speed: 0.0, acceleration: 0.0, async_mode: false}"

# 7. Return to home
ros2 service call /arm/move_j magpie_msgs/srv/MoveJoint \
  "{joint_positions: [0.6048, -1.8380, 1.7633, 0.0331, 0.6697, 3.2949], speed: 0.0, acceleration: 0.0, async_mode: false}"

# 8. Move DOWN 3 cm — place object below gripper first, verify clearance visually
ros2 service call /arm/move_l magpie_msgs/srv/MoveLinear \
  "{target_pose: {position: {x: -0.175, y: -0.333, z: 0.405}, orientation: {x: 0.063, y: 0.697, z: -0.714, w: 0.019}}, speed: 0.0, acceleration: 0.0, async_mode: false}"

# 9. Close gripper on object
ros2 service call /gripper/close std_srvs/srv/Trigger {}

# 10. Lift back to home
ros2 service call /arm/move_j magpie_msgs/srv/MoveJoint \
  "{joint_positions: [0.6048, -1.8380, 1.7633, 0.0331, 0.6697, 3.2949], speed: 0.0, acceleration: 0.0, async_mode: false}"

# 11. Open gripper to release
ros2 service call /gripper/open std_srvs/srv/Trigger {}
```

**Expected results:**

| Step | Expected |
|---|---|
| `set_speed` | `success: True, message: 'Speed set to 0.10, accel to 0.10'` |
| `move_j` home | Arm at x≈−0.175, y≈−0.333, z≈0.435 |
| `gripper/open` | Gripper opens to ~103 mm |
| `set_force 1.5N` | `success: True` |
| `move_l` up 5 cm | Arm rises to z≈0.485, `success: True` |
| `move_j` home | Returns to z≈0.435 |
| `move_l` down 3 cm | Arm descends to z≈0.405, fingertips ~215 mm above table |
| `gripper/close` | Gripper closes, stops at object contact (~1.5 N) |
| `move_j` home (lift) | Arm lifts object back to home height |
| `gripper/open` | Gripper opens, object released |

**Important notes:**
- **Always move UP first** — confirms arm direction before going toward the table
- **Never rotate wrist** — keep orientation fixed, cables will catch
- TCP z is the **wrist** — fingertips are ~190 mm lower; at home (z=0.435) fingertips are at z≈0.245
- Home joints: `[0.6048, -1.8380, 1.7633, 0.0331, 0.6697, 3.2949]` rad = `[34.65°, -105.31°, 101.03°, 1.90°, 38.37°, 188.78°]`
- If the arm triggers a protective stop, check pendant payload settings (0.6 kg, CoG Z=95 mm)

---

## Stage 7 — Full Pipeline (camera + Gemini + arm + gripper)

**Hardware required:** UR5 powered + initialized, MAGPIE gripper mounted on end-effector (12V + USB), RealSense camera mounted and plugged in, Ethernet to robot network. **Clear the area around the arm before running.**

**Prerequisites:**
```bash
export GEMINI_API_KEY=your_key_from_aistudio_google_com
```

Set `use_gemini: true` in [config/gripper_config.yaml](config/gripper_config.yaml) and set `gemini_task` to your task description:
```yaml
deligrasp_node:
  ros__parameters:
    use_gemini: true
    gemini_task: "pick up the block"
    object_query: "block"          # text prompt for object detector
    approach_height: 0.10          # m above detected object
    grasp_z_offset: 0.02           # m descent below approach for grasp
```

Rebuild after changing config:
```bash
cd ~/ws_ctrl && colcon build --symlink-install --packages-select magpie_control && source install/setup.bash
```

---

**Option A — launch everything with one command:**
```bash
sg dialout -c "bash -c 'source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash && ros2 launch magpie_control gripper_control.launch.py'"
```

Then start the camera separately (the launch file does not include it):
```bash
source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
ros2 launch realsense2_camera rs_launch.py camera_name:=gripper_camera
```

---

**Option B — four separate terminals (easier to read logs):**

**Terminal 1 — arm node:**
```bash
source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
ros2 run magpie_control ur5_node
```

**Terminal 2 — gripper node:**
```bash
sg dialout -c "bash -c 'source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash && ros2 run magpie_control gripper_node'"
```

**Terminal 3 — camera node:**
```bash
source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
ros2 launch realsense2_camera rs_launch.py camera_name:=gripper_camera
```

**Terminal 4 — deligrasp node:**
```bash
source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
ros2 run magpie_control deligrasp_node --ros-args --params-file ~/magpie_control/config/gripper_config.yaml
```

---

**Trigger a grasp (any terminal):**
```bash
source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash

# Check all nodes are ready
ros2 node list

# Send a grasp action goal
ros2 action send_goal /deligrasp magpie_msgs/action/DeliGrasp \
  "{instruction: 'pick up the block'}"
```

**What it does:**
1. Object detector (Grounding DINO) finds the object in the color frame using `object_query`
2. Depth image + camera intrinsics convert the 2D detection to a 3D point in camera frame
3. Live TCP pose transforms that point to world (base) frame
4. Arm moves to approach pose (`approach_height` above the object)
5. Arm descends to grasp pose (`grasp_z_offset` below approach)
6. Gemini 2.5 Flash is called **text-only** with `"Pick up the {object_name}."` and the DeliGrasp descriptor prompt — no image. Returns mass, spring constant, friction coefficient, aperture, slip closure. Physics computed: `initial_force = mg/μ`, `add_force = max(0.05, k·Δx·0.0001)`
7. DeliGrasp action closes gripper with force control — stops at `initial_force`, increments on slip
8. Arm retreats to approach height

**Expected results:**

| Step | Expected |
|---|---|
| `ros2 node list` | `/ur5_node`, `/gripper_node`, `/deligrasp_node` all present |
| Object detection | deligrasp_node logs object detected with bounding box + confidence |
| Gemini descriptor | deligrasp_node logs full descriptor text + parsed mass/k/μ/aperture + computed forces |
| Arm approach | Arm moves to `approach_height` above object |
| Arm descend | Arm descends to grasp pose |
| Gripper close | Gripper stops at contact, logs force reached |
| Arm retreat | Arm lifts back to approach height |

**Important notes:**
- The TCP is the wrist — with gripper mounted the fingertips are ~230 mm lower. Set `approach_height` conservatively (0.10–0.15 m) to avoid the gripper hitting the table on descent
- If deligrasp_node fails to detect the object, increase `detection_confidence` downward (e.g. 0.2) or adjust `object_query` to match the object more precisely
- `GEMINI_API_KEY` must be exported in the same terminal that starts deligrasp_node (Option B Terminal 4) or the whole launch shell (Option A)
