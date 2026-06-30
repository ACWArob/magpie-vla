# magpie_control

Autonomous grasping stack for the UR5 + MAGPIE gripper. Detects any object by name, computes a force-controlled grasp using physics-based parameters (DeliGrasp via Gemini), and adapts in real-time using depth-based slip detection and a persistent Kalman-filtered grasp memory that improves with each attempt. Logs full VLA training data including a Gemini-based grasp quality reward signal.

Grasp pose planning runs both **PCA** (point-cloud principal axes) and **GraspGenX** (NVlabs cross-embodiment 6-DOF grasp model) in parallel, with Gemini arbitrating across all candidates. See [docs/pickup_pipeline.md](docs/pickup_pipeline.md) for the current `notebooks/magpie_collect.ipynb`-based pickup pipeline, including the GraspGenX/SAM3 VRAM coexistence design. The diagram and cell names below describe the original `magpie_demo.ipynb` walkthrough.

---

## Architecture overview

```
┌──────────────────────────────────────────────────────────────────────┐
│  magpie_demo.ipynb  (primary interface)                              │
│                                                                      │
│  c01         Launch nodes + GraspMemory                              │
│  mem_manager ACTIVE_OVERRIDE + per-object memory management          │
│  c02         Import ROS shared libs into kernel                      │
│  c03         Constants (APPROACH_H, GRIPPER_LEN, TABLE_Z, _TCP_TO_CAM …)│
│  c04         MagpieNode ROS client (arm + gripper + slip guard)      │
│  measure_table  Measure TABLE_Z on clear table (before placing obj)  │
│  c14         [optional] DeliGrasp manual run + radar chart           │
│  17ea9af9    Full pickup — DeliGrasp inline, slip guard, place-down  │
│  vla_logger  Save result + true_force_label + quality placeholder    │
│  dg_summary  Post-grasp Gemini quality score (0–1) + JSONL patch     │
└──────────────────────────────────────────────────────────────────────┘
         │                    │                    │
    ROS2 nodes          scripts/               data/
  ur5_node             slip_guard_node.py     grasp_log/
  gripper_node         grasp_memory.py          force_priors.json
  realsense2           pointcloud_utils.py      embeddings.npz
  sam3_infer.py        seed_ycb_priors.py        grasps_YYYY-MM-DD.jsonl
  slip_guard_node.py   measure_poll_rates.py     *_image_detect.jpg
                       vlm_consistency.py        *_image_grasp.jpg
                                                 *_image_held.jpg
                                                 *_pca_plot.jpg
```

### Key scripts

| Script | Purpose |
|---|---|
| `slip_guard_node.py` | ROS2 node — depth-primary slip detection at 10Hz; re-clamps grip if object drops >1mm during lift |
| `grasp_memory.py` | Persistent force priors: named Kalman filter per object + DINOv2 RAG for new objects |
| `pointcloud_utils.py` | Depth→world point cloud, dual-scan merge, spatial crop, PCA grasp angle, multi-candidate Gemini arbiter (`rank_grasp_angles_visual`) |
| `grasp_detectors/graspgenx_zmq.py` | ZMQ client for the GraspGenX server (cross-embodiment 6-DOF grasp poses, MAGPIE gripper conditioning) |
| `run_graspgenx_server.sh` | Launches the GraspGenX ZMQ server (`:5557`); preloaded at notebook startup, coexists with SAM3 in VRAM all session |
| `seed_ycb_priors.py` | Pre-populate force priors from YCB dataset (50 objects) — so first grasp isn't cold |
| `measure_poll_rates.py` | Measure actual ROS publish rates for all sensors |
| `vlm_consistency.py` | Test Gemini detection/DeliGrasp stability across N repeated calls |

---

## How to run

### Prerequisites — check once per session

```bash
# Source ROS2 + workspace
source /opt/ros/humble/setup.bash
source ~/ws_ctrl/install/setup.bash

# Verify nodes are up
ros2 topic list | grep -E "gripper|ur5|camera|slip"
```

If any nodes are missing, re-run **cell c01** — it kills and restarts all processes including SAM3 and the slip guard.

---

### Step 1 — Seed grasp memory (once ever, or after clearing data/)

```bash
cd ~/magpie_control
python3 scripts/seed_ycb_priors.py
```

Seeds `force_priors.json` with physics-based initial estimates for 50 common objects. Uses `F_min * 1.5` safety margin from YCB masses + empirical friction coefficients. Safe to re-run — skips objects that already have n≥2 real grasps.

---

### Step 2 — Open the notebook

```bash
cd ~/magpie_control
jupyter notebook notebooks/magpie_demo.ipynb
```

**Required cells — run in order at the start of every session:**

| Cell | What it does | Must run? |
|---|---|---|
| `c01` | Kills old processes; launches ur5_node, gripper_node, realsense2, sam3_infer.py, slip_guard_node; initialises GraspMemory | **Yes** |
| `mem_manager` | Sets `ACTIVE_OVERRIDE`; prints all objects currently in memory with n, force mean, std | **Yes** |
| `c02` | Loads ROS2 Python libs into kernel namespace | **Yes** |
| `c03` | Sets `APPROACH_H`, `GRIPPER_LEN`, `TABLE_Z`, `HARD_FLOOR_Z`, `_TCP_TO_CAM`, `SAM3_SOCK` | **Yes** |
| `c04` | Creates `MagpieNode` Python client (wraps all ROS services) | **Yes** |
| `measure_table` | Drives arm to clear-table position, records `TABLE_Z` from median depth — **run before placing object** | **Yes, before object** |
| `c14` | Runs DeliGrasp manually + plots radar chart — useful for inspection/tuning | No — runs inline in 17ea9af9 |

**Switching objects mid-session:**
Set `ACTIVE_OVERRIDE = "object name"` in `mem_manager` and re-run it before `17ea9af9`. Set it back to `""` to resume auto-detection.

---

### Step 3 — Full grasp (main flow)

Run **17ea9af9** directly. Every sub-step runs automatically.

```
17ea9af9 — full pickup sequence
──────────────────────────────────────────────────────────────────────

1. SNAPSHOT + GEMINI DETECTION
   - node.spin(8) → get stable color + depth frame
   - Gemini 2.5 Flash: "What is the main graspable object? 2–4 words."
   - ACTIVE = first 4 words of response, lowercased
   - If ACTIVE_OVERRIDE is set in mem_manager, overrides Gemini result

2. SAM3 SEGMENTATION (with close-up retry)
   - Sends frame + ACTIVE query to SAM3 via Unix socket (/tmp/sam3.sock)
   - Returns bounding box, confidence score, binary mask
   - If no detection: moves arm 2cm closer, retries up to 3×
   - Saves detection crop (_det_crop) immediately — fp_color is overwritten later

3. ROUGH XY POSITION
   - Median depth at mask centroid → camera-frame 3D point → world frame
   - Used only to position arm above object for top-down scan
   - Also builds Scan 1 PCD from detection position (side view)

4. TOP-DOWN SCAN + POINT CLOUD MERGE
   - Arm moves directly above object at APPROACH_H + GRIPPER_LEN
   - Second SAM3 call from top-down → mask2
   - Builds Scan 2 PCD (top-down)
   - If both scans have ≥10 points: merge → denoise → pts_use  [merged]
   - If scan 2 blocked: use whichever scan has more points          [side-only]
   - Spatial crop: 12cm radius + Z bounds — removes table bleed-through

5. PCA → GRASP ANGLE
   - top_layer(pts_use): top 30% of points by Z (avoids table surface)
   - PCA of top layer → major/minor axes, centroid, extent_mm
   - obj_w_mm = minor axis length (gripper spans this)
   - Gemini smart_grasp_angle(): vision + geometry → strategy
       symmetric: grip at centroid along minor axis
       rotated:   grip at angle offset from minor axis
       custom:    Gemini specifies angle directly from image
   - PCA figure saved to bytes (_pca_plot_bytes) for quality assessment

6. FORCE PARAMETERS
   a. Fast path (skip DeliGrasp):
      - Triggers if n≥5 AND std<1.0N for this specific object
      - Synthesises GP from memory: initial_force = memory_mean,
        force_cap = clip(memory_mean * 2.5, memory_mean * 1.1, 16N)
      - Saves ~3–5s Gemini API call

   b. DeliGrasp inline (if c14 not run):
      - Sends frame + object name to Gemini with structured template
      - Extracts: mass_g, k (N/m), mu (friction coefficient)
      - Search-grounded override: second call with Google Search tool
        to retrieve real specs; overwrites mass/mu/k if found
      - k-based deformation cap:
            soft  (k < 400 N/m): force_cap = k × 0.012  [12mm crush limit]
            stiff (k ≥ 400 N/m): force_cap = k × 0.025  [25mm — structure holds]
        clipped to [1.0, 16.0] N

   c. Memory prior (always retrieved):
      - Named Kalman state first (exact object name match)
      - DINO RAG fallback for new objects: k=5 nearest by cosine similarity,
        min similarity 0.5, weighted-mean force. Returns n=0 so DeliGrasp
        is never skipped on a new object's first grasp.

   d. Blend (when n≥2):
      _w = min(0.9, n / 4.)          # 90% memory weight at n=4, stays there
      cf = clip(_w * memory + (1-_w) * DeliGrasp, 0.5, force_cap)
      if n≥3: force_cap = max(k-cap, memory_mean * 1.5)   # memory can raise cap

   e. Slip threshold:
      slip_thresh = clip(F_min * 0.9, 0.15, force_cap * 0.9)
      where F_min = mass_g/1000 * 9.81 / (2 * mu)

7. APPROACH + DESCEND
   - Open gripper fully
   - Move to approach pose (above object, correct rotation)
   - Descend to grasp_pose Z = p_obj[2] - grasp_off + GRIPPER_LEN
     where grasp_off = clip(obj_height / 2, 10mm, 40mm)
   - Snapshot at grasp position (snap_grasp) before closing

8. GRASP LOOP (up to 4 attempts)
   - Pre-position: node.set_pos(obj_w_mm + 8mm) before first attempt
     Parks gripper just above contact so AX-12 has short travel → better torque
   - Each attempt: clear_error → set_force(cf) → close_g()
   - Wait 2.5s for motor to settle, spin ROS 8×
   - Contact check: force ≥ slip_thresh → contact, else slip
   - On slip: cf += 1.0N (capped at force_cap), retry
   Note: "slip" in the loop means force below threshold, NOT physical sliding.
   The slip guard handles real slips during lift.

9. LIFT WITH SLIP GUARD
   - lift_force = min(force_cap, cf + 0.5)
   - set_force(lift_force) before moving — tightens grip before arm rises
   - force_step = clip(k / 2000, 0.15, 1.0) N — soft objects get smaller increments
   - slip_guard_enable(lift_force, slip_thresh, obj_u, obj_v, force_step)
   - Arm rises to ap_rot (approach height)
   - slip_guard_disable() + capture snap_held

10. SLIP GUARD EVENTS → TRUE FORCE LABEL (DAgger/HER)
    - Parses RECLAMP events from slip guard log
    - true_force_label = final force after all corrections
    - force_label_error = true_force_label - init_force_pred
    - This is the training target, not the initial prediction

11. HOLD CHECK
    - force_ok:    s_lift.force ≥ slip_thresh
    - aperture_ok: s_lift.position ≥ obj_w_mm * 0.4
    - held = force_ok AND aperture_ok

12. CONTROLLED PLACE-DOWN (if held)
    - Carry to home_pick (arm home height, same XY)
    - Lower to grasp_pose[2,3] + 8mm (just above pickup Z)
    - open_g() → wait 0.8s for object to settle
    - Rise back to home_pick

13. RESULT DICT
    Full state captured for VLA training — see VLA data format section.
    → vla_logger saves to JSONL + images
    → dg_summary patches with Gemini quality score
```

---

### Step 4 — After logging (run immediately after 17ea9af9)

```
vla_logger cell:
  - Writes result to data/grasp_log/grasps_YYYY-MM-DD.jsonl
  - Saves snap_grasp, snap_held, PCA plot as JPG files
  - Calls gm.update(ACTIVE, true_force_label, _det_crop) → Kalman update
  - grasp_quality written as null (placeholder)

dg_summary cell:
  - Sends snap_grasp + snap_held + _pca_plot_bytes to Gemini 2.5 Flash
  - Prompt asks for 0.0–1.0 quality score with strict calibration:
      1.0       = perfect grip, object undeformed, centred in fingers
      0.7–0.9   = good grip, minor misalignment, no deformation
      0.4–0.6   = marginal — visible deformation or poor contact
      0.0–0.3   = crushing, failed contact, or object clearly slipped
  - Patches last JSONL record: grasp_quality, grasp_quality_reason, grasp_quality_category
  - Prints full summary: force prediction vs truth, memory state, quality score
```

---

### Step 5 — After 4+ grasps of the same object

System blends heavily toward memory and eventually skips DeliGrasp:

```
Memory confident (n=5, std=0.28N) — skipping DeliGrasp
Blended force: 3.8N (memory 90% + GP 10%)
```

Blend weight = `min(0.9, n/4)`:
- n=1: 25% memory, 75% DeliGrasp
- n=2: 50% memory, 50% DeliGrasp
- n=4+: 90% memory, 10% DeliGrasp

At n≥5 with std<1N, DeliGrasp API call is skipped entirely.

---

### Teach mode (move arm by hand)

```bash
source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
ros2 service call /arm/teach_mode std_srvs/srv/Trigger
# move arm freely, then call again to disable
ros2 service call /arm/teach_mode std_srvs/srv/Trigger
```

Or from the notebook: `node.teach()` / `node.unteach()`

---

### Open gripper manually

```python
node.open_g()          # from notebook
```

```bash
ros2 service call /gripper/open std_srvs/srv/Trigger
```

---

### Emergency / recovery

```bash
# Gripper stuck closed (AX-12 overload — torque shuts off after ~1–2s of overload)
ros2 service call /gripper/clear_error std_srvs/srv/Trigger
ros2 service call /gripper/open std_srvs/srv/Trigger

# UR5 RTDE connection dropped ("End of file" or NoneType errors)
# → re-run cell c01 in the notebook
# → on pendant: ensure external control program is running (play button)

# Slip guard manual control
ros2 service call /slip_guard/enable std_srvs/srv/Trigger
ros2 service call /slip_guard/disable std_srvs/srv/Trigger
ros2 topic echo /slip_guard/events
```

---

### VLM consistency test

```bash
# Capture live frame and test N=20 calls
python3 scripts/vlm_consistency.py --capture --n 20

# Or use a saved image
python3 scripts/vlm_consistency.py path/to/image.jpg --object "red cube" --n 20
```

Tests Gemini stability: runs N repeated calls for object detection, grasp strategy, and DeliGrasp parameters. Reports mean/std per field. Results saved to `tests/vlm_consistency_YYYY-MM-DD_HHMM.md`.

---

### Sensor poll rates

```bash
source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
python3 scripts/measure_poll_rates.py
```

Expected: arm ~500Hz, gripper 10Hz, camera 10Hz. Results saved to `tests/poll_rates.md`.

---

## Grasp memory system

### Storage

- `data/grasp_log/force_priors.json` — one Kalman state per object: `{x, P, n}`
  - `x` = force estimate (N)
  - `P` = uncertainty (N²)
  - `n` = number of real grasps
- `data/grasp_log/embeddings.npz` — DINOv2 ViT-S/14 embeddings + force labels + object names

### Kalman filter parameters

```
Q  = 0.25 N²   process noise  — force can vary between placements
R  = 0.50 N²   measurement noise — slip-guard + placement variability
P0 = 4.00 N²   initial uncertainty — wide prior on first real grasp
```

Update equations:
```
P_pred = P + Q
K      = P_pred / (P_pred + R)
x_new  = x + K * (true_force - x)
P_new  = (1 - K) * P_pred
n     += 1
```

### Query priority

1. **Named Kalman state** — exact object name match (lowercased). Always preferred for known objects.
2. **DINO RAG** — for truly new objects (no named prior): embed detection crop with DINOv2 ViT-S/14 (384-dim, L2-normalised), cosine similarity against all stored embeddings. Top-5 by similarity (min threshold 0.5), weighted-mean force. Returns `n=0` so DeliGrasp is never skipped on a new object.
3. **None** — returns `{force_mean: None, n: 0, source: 'none'}`

### Blend logic

```python
_w = min(0.9, n / 4.)                         # 0.25 at n=1 → 0.90 at n=4+
cf = clip(_w * memory_mean + (1-_w) * cf_gp, 0.5, force_cap)
if n >= 3:
    force_cap = max(force_cap, memory_mean * 1.5)  # real data overrides physics
```

### Skip DeliGrasp

Triggers when `n >= 5 AND std < 1.0N` for this specific object. Saves ~3–5s Gemini call. A minimal GP is synthesised from memory so force_cap is still computed.

### Training labels (DAgger / HER)

`true_force_label` = force reported by slip guard after all re-clamps during lift. This is the corrected label — what force was actually needed, not what was predicted. Use this as the VLA training target. `force_label_error_n = true_force_label - init_force_pred` measures prediction quality.

---

## Slip guard

### Operation

Depth-primary slip detection running on the wrist/palm RealSense D405 at 10Hz during the lift phase only (never during approach/descend).

- **Reference depth** locked on the first camera frame after `enable` — object pixel location `(obj_u, obj_v)` passed from notebook so depth is sampled at the correct spot, not image centre.
- **Depth channel (primary):** if `current_depth - ref_depth > 1.0mm` → re-clamp
- **Force channel (secondary):** if `force < slip_thresh * 0.6` → re-clamp
- **Re-clamp:** `clear_error → set_force(target + force_step) → close_g` (async, non-blocking)
- **Reference shift:** after each depth re-clamp, reference is shifted forward to avoid cascading triggers on accumulated slip

### Parameters

```
SLIP_DEPTH_MM        = 1.0mm   depth increase that triggers re-clamp
FORCE_SLIP_RATIO     = 0.6     force below thresh * this → re-clamp (backup)
FORCE_STEP_N         = 1.0N    default re-clamp increment (overridden per object)
MIN_RECLAMP_INTERVAL = 0.3s    minimum gap between re-clamps (avoids thrashing)
DEPTH_SAMPLE_RADIUS  = 25px    radius around object pixel for depth sampling
```

`force_step` is set per object via the config topic: `clip(k / 2000, 0.15, 1.0)` N — soft objects (low k) get smaller increments to avoid overcorrecting.

### Config topic

`/slip_guard/config` — `Float32MultiArray` with 5 elements:
```
[force_n, slip_thresh, obj_u, obj_v, force_step]
```
Published by `node.slip_guard_enable(...)` in c04 before the arm rises.

---

## Point cloud pipeline

```
depth image + SAM3 mask
        │
        ├─ Scan 1: detection position (side view)
        │   build_segmented_pcd() → raw PCD in world frame
        │   denoise_pcd() → pts_cln1
        │
        ├─ Scan 2: top-down position (arm directly above object)
        │   second SAM3 call from new viewpoint → mask2
        │   build_segmented_pcd() → raw PCD
        │   denoise_pcd() → pts_cln2
        │   check_view_quality() → fill ratio, point count, ok flag
        │
        ├─ Merge strategy:
        │   both scans OK + ≥10pts each → vstack → denoise → [merged]
        │   only scan 2 OK             →                      [top-down]
        │   scan 2 blocked             → larger of scans      [side-only]
        │
        ├─ top_layer(): top 30% by Z — avoids table surface in PCA
        │
        └─ analyse_pcd(): PCA of top layer
               centroid, major axis, minor axis, extent_m
               │
               └─ smart_grasp_angle(): PCA → angle + Gemini strategy override
                     Returns: angle_deg, strategy, reason
```

`obj_w_mm = pca['extent_m'][1] * 1000` — minor axis is gripper span.

`grasp_off = clip(obj_height / 2, 10mm, 40mm)` — finger centre at object equator.

**Important:** run `measure_table` on an empty scene to capture `TABLE_Z`. This is used to compute `obj_height_est = max(p_obj[2] - TABLE_Z, 10mm)`. Re-estimating from a scene with the object present is unreliable.

---

## AX-12 gripper behaviour

The MAGPIE gripper uses Dynamixel AX-12A servo motors with a 4-bar linkage.

- **Overload shutdown:** if torque exceeds limit for ~1–2s (e.g., gripping too tight or object wedged), the AX-12 shuts off torque. Recovery: `node.clear_err()` re-enables the motor at current position, then `node.set_force()` + `node.close_g()`.
- **Low force at wide apertures:** the linkage geometry means the motor produces less grip force when the fingers are far apart. Pre-positioning to `obj_w_mm + 8mm` before `close_g()` gives the motor a short travel and better mechanical advantage at contact.
- **Force units:** `set_force(N)` sets max motor torque in Newtons of grip force (calibrated). `gripper/state.force` is the current grip force in N.

---

## VLA data format

Each row in `data/grasp_log/grasps_YYYY-MM-DD.jsonl`:

```json
{
  "object":              "measuring tape",
  "held":                true,
  "grasp_log": [
    {"attempt": 1, "ap": 27.8, "force": 7.09, "cmd_force": 14.4, "status": "contact"},
    {"attempt": 2, "ap": 28.1, "force": 7.71, "cmd_force": 14.4, "status": "contact"}
  ],
  "final_force":         8.10,
  "final_aperture":      27.6,
  "slip_thresh":         6.48,
  "lift_force_cmd":      14.9,
  "obj_pos_world":       [-0.186, -0.621, 0.074],
  "grasp_angle_deg":     12.3,
  "grasp_strategy":      "symmetric",
  "obj_width_mm":        27.8,
  "gp_mass_g":           250.0,
  "gp_mu":               0.5,
  "gp_k":                1200.0,
  "sg_fires":            0,
  "sg_events":           [],
  "true_force_label":    8.1,
  "init_force_pred":     14.4,
  "force_label_error_n": -6.3,
  "image_detect_path":   "data/grasp_log/20260609_143201_image_detect.jpg",
  "image_grasp_path":    "data/grasp_log/20260609_143201_image_grasp.jpg",
  "image_held_path":     "data/grasp_log/20260609_143201_image_held.jpg",
  "pca_plot_path":       "data/grasp_log/20260609_143201_pca_plot.jpg",
  "tcp_grasp":           [[...], [...], [...], [...]],
  "grasp_quality":       0.85,
  "grasp_quality_reason":"Firm grip, object centred, no visible deformation",
  "grasp_quality_category": "good",
  "timestamp":           "2026-06-09T14:32:05.123"
}
```

**Key fields:**

| Field | Description |
|---|---|
| `true_force_label` | Force after all slip-guard corrections — use as training target |
| `init_force_pred` | Initial DeliGrasp/memory prediction before grasp |
| `force_label_error_n` | `true - pred` (positive = underestimated, negative = overestimated) |
| `sg_fires` | Number of slip-guard re-clamps during lift |
| `grasp_quality` | Gemini visual quality score 0–1 (null until dg_summary runs) |
| `grasp_strategy` | `symmetric`, `rotated`, or `custom` |
| `image_grasp_path` | Camera image right before gripper closes (pre-contact) |
| `image_held_path` | Camera image post-lift (shows actual grip quality) |
| `pca_plot_path` | PCA overlay figure (major/minor axes on point cloud) |

---

## ROS2 node API

### Gripper (`/gripper/*`)

| Service | Type | Description |
|---|---|---|
| `/gripper/open` | Trigger | Open fully |
| `/gripper/close` | Trigger | Close to contact (motor stall detection) |
| `/gripper/set_force` | SetGripperForce | Set max grip force (N) |
| `/gripper/set_position` | SetGripperPosition | Move to absolute position (mm) |
| `/gripper/clear_error` | Trigger | Re-enable motor after AX-12 overload shutdown |
| `/gripper/calibrate` | Trigger | Calibrate open/close limits |

Topic: `/gripper/state` (10Hz) — `position` mm, `force` N, `temperature` °C, `contact_detected` bool

**Notebook wrappers (c04):**
```python
node.open_g()            # open fully
node.close_g()           # close to contact
node.set_force(n)        # set force limit in N
node.set_pos(mm)         # move to position in mm
node.clear_err()         # clear AX-12 overload
node.gs                  # read current GripperState
```

### Arm (`/arm/*` or `/ur5/*`)

| Method | Description |
|---|---|
| `node.move(pose, spd)` | Move TCP to 4×4 pose matrix at speed (m/s) |
| `node.teach()` | Enable freedrive (teach) mode |
| `node.unteach()` | Disable freedrive mode |
| `node.tcp` | Read current TCP pose (4×4 numpy matrix) |
| `node.spin(n)` | Spin ROS executor n times (flush callbacks) |

### Camera

Topic: `/camera/gripper_camera/camera/color/image_raw` — RGB, 10Hz
Topic: `/camera/gripper_camera/camera/depth/image_rect_raw` — uint16 mm, 10Hz

```python
node.color     # latest RGB frame (numpy HxWx3 uint8)
node.depth     # latest depth frame (numpy HxW uint16, mm)
node.caminfo   # camera intrinsics (k matrix: fx, fy, cx, cy)
```

### Slip guard (`/slip_guard/*`)

| Service | Description |
|---|---|
| `/slip_guard/enable` | Start monitoring — captures reference depth on first frame |
| `/slip_guard/disable` | Stop monitoring |

Topic: `/slip_guard/config` — `Float32MultiArray [force_n, slip_thresh, obj_u, obj_v, force_step]`
Topic: `/slip_guard/events` — timestamped event log strings (REFERENCE, RECLAMP, ENABLED, DISABLED)

```python
# c04 wrapper:
node.slip_guard_enable(force_n, slip_thresh, obj_u, obj_v, force_step)
node.slip_guard_disable()
```

---

## SAM3 segmentation

SAM3 runs as a subprocess (`sam3_infer.py`) and communicates via Unix socket at `/tmp/sam3.sock`. The notebook sends a JPEG path + text query; SAM3 returns boxes, scores, and a RLE-encoded binary mask.

- Model requires HuggingFace token (`HF_TOKEN` in `.env`) for first download
- Float32 patch applied automatically in `sam3_infer.py` for RTX 2070 compatibility
- If socket is missing: re-run c01 to restart the subprocess
- SAM3 and the GraspGenX grasp-pose server coexist in VRAM (measured ~3.9GB + ~0.8GB, ~3GB headroom on an 8GB card) and both stay resident for the whole session — SAM3 is never killed to make room for GraspGenX, so it stays available for the pre-descent centering check and live recalibration mid-pickup

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

### Environment variables (`.env` in repo root)

```
GEMINI_API_KEY=your_key_here
HF_TOKEN=your_huggingface_token
```

---

## Common issues

| Symptom | Fix |
|---|---|
| `AttributeError: NoneType.success` | UR5 RTDE dropped — re-run c01, check pendant (external control program must be running) |
| Gripper closes but doesn't hold / immediately drops | AX-12 overload: `ros2 service call /gripper/clear_error std_srvs/srv/Trigger` then `open` |
| SAM3 no detection | Move arm 2cm closer (auto-retry does this); or check `/tmp/sam3.sock` exists (re-run c01) |
| `TABLE_Z not set` | Run `measure_table` cell on a clear table before placing the object |
| All grasp attempts show "slip" but object is held | Normal when DeliGrasp overcalls force — "slip" in the loop means `force < slip_thresh`, not physical sliding. The slip guard during lift handles real slips. |
| Force is 0.000N across multiple attempts | Caused by extra `set_pos` call between attempts — each retry should start from `close_g()` fresh, not a manually set intermediate position |
| Wrong object detected | Set `ACTIVE_OVERRIDE = "object name"` in `mem_manager` and re-run it before `17ea9af9` |
| Object crushed (too much force) | Check `gp_k` — low k (soft object) gets 12mm deform cap, so force_cap should be low. If memory n≥3 is overriding cap too high, reset: `del gm._priors['name']; gm._save_priors()` in mem_manager |
| Memory prior wrong for this object | In `mem_manager`: `del gm._priors['object name']; gm._save_priors()` resets to cold start |
| DINOv2 slow on first load | Downloads ~84MB once to `~/.cache/torch/hub`; subsequent loads are instant |
| F/T sensor timeout | Check ethernet cable to 192.168.0.5; `ping 192.168.0.5` |
| PCA shows table surface | Spatial crop not isolating object — check SAM3 mask is tight; verify `measure_table` was run on a clear scene |
