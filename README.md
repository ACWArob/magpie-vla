# magpie_control

Autonomous grasping stack for the UR5 + MAGPIE gripper. Detects any object by name, computes a force-controlled grasp using physics-based parameters (DeliGrasp), and adapts in real-time using depth-based slip detection and a persistent grasp memory that improves with each attempt. Logs VLA training data with a Gemini-based grasp quality reward signal.

---

## Architecture overview

```
┌──────────────────────────────────────────────────────────────────────┐
│  magpie_demo.ipynb  (primary interface)                              │
│                                                                      │
│  c01  Launch nodes + GraspMemory                                     │
│  c03  Constants (APPROACH_H, GRIPPER_LEN, TABLE_Z, _TCP_TO_CAM …)   │
│  c04  MagpieNode ROS client (arm + gripper + slip guard)             │
│  measure_table  TABLE_Z (run on clear table before placing object)   │
│  c14  [optional] DeliGrasp manual run + radar chart visualisation    │
│  17ea9af9  Full pickup — DeliGrasp runs inline automatically         │
│  vla_logger  Save result + true_force_label + quality placeholder    │
│  dg_summary  Post-grasp summary + Gemini grasp quality score (0–1)  │
└──────────────────────────────────────────────────────────────────────┘
         │                    │                    │
    ROS2 nodes          scripts/               data/
  ur5_node             slip_guard_node.py     grasp_log/
  gripper_node         grasp_memory.py          force_priors.json
  ft_sensor_node       pointcloud_utils.py      embeddings.npz
  realsense2           seed_ycb_priors.py        grasps_YYYY-MM-DD.jsonl
  sam3_infer.py        measure_poll_rates.py     *_pca_plot.jpg
  slip_guard_node.py   vlm_consistency.py
```

### Key scripts

| Script | Purpose |
|---|---|
| `slip_guard_node.py` | ROS2 node — depth-primary slip detection during lift; re-clamps grip if object drops >1mm |
| `grasp_memory.py` | Persistent force priors: DINOv2 RAG + Kalman filter — improves predictions across sessions |
| `pointcloud_utils.py` | Depth→world point cloud, dual-scan merge, spatial crop, PCA grasp angle |
| `seed_ycb_priors.py` | Pre-populate force priors from YCB dataset (50 objects) so first grasp isn't cold |
| `measure_poll_rates.py` | Measure actual publish rates for all sensors |
| `vlm_consistency.py` | Test Gemini stability across N calls (auto-detect, strategy, DeliGrasp params) |

---

## How to run

### Prerequisites — check once per session

```bash
# Source ROS2 + workspace
source /opt/ros/humble/setup.bash
source ~/ws_ctrl/install/setup.bash

# Verify nodes are up
ros2 topic list | grep -E "gripper|ur5|camera"
```

If any nodes are missing, re-run **cell c01** in the notebook — it kills and restarts all processes.

---

### Step 1 — Seed grasp memory (once ever, or after clearing data/)

```bash
cd ~/magpie_control
python3 scripts/seed_ycb_priors.py
```

Seeds force priors for 50 common objects (YCB dataset + lab objects). Safe to re-run — skips objects that already have real grasp data.

---

### Step 2 — Open the notebook

```bash
cd ~/magpie_control
jupyter notebook notebooks/magpie_demo.ipynb
```

**Required cells — run in order at the start of every session:**

| Cell | What it does | Must run? |
|---|---|---|
| `c01` | Launches all ROS nodes + SAM3 + slip guard + GraspMemory | **Yes** |
| `c02` | Loads ROS shared libs into kernel | **Yes** |
| `c03` | Sets constants (APPROACH_H, GRIPPER_LEN, etc.) | **Yes** |
| `c04` | Creates MagpieNode ROS client | **Yes** |
| `measure_table` | Measures TABLE_Z — **run on clear table before placing object** | **Yes, before object** |
| `c14` | DeliGrasp manual run + radar chart — **optional**, useful for inspection | No — runs inline |

---

### Step 3 — Full grasp (main flow)

**Run 17ea9af9 directly — DeliGrasp now runs automatically inside the pickup cell.**

```
17ea9af9 → Full pickup sequence:
  1. Gemini auto-detects object name
  2. SAM3 segments object (with close-up retry)
  3. Arm moves above object
  4. Dual scan: detection-pos + top-down → merged point cloud
  5. PCA → grasp angle + strategy (Gemini override for complex shapes)
     PCA figure saved to bytes for quality assessment
  6. Force params: DeliGrasp inline (or skip if memory confident)
     → k-based deformation cap: force_cap = k * 0.012 (12mm max crush)
     → reclamp step = k/2000 (soft objects get gentler slip recovery)
     → GraspMemory prior blended in when n ≥ 2
  7. Tighten grip to lift_force before rising (avoids marginal grip at lift)
  8. Lift with slip guard active (depth-primary, 10Hz camera)
     snap_held captured post-lift for quality assessment
  9. Controlled place-down: carry to home → lower to table Z → open → rise
 10. result dict → vla_logger → dg_summary

vla_logger → Saves to data/grasp_log/grasps_YYYY-MM-DD.jsonl
              Saves snap_held image + PCA plot as JPG
              Updates GraspMemory Kalman filter with true_force_label
              Writes grasp_quality=null placeholder (filled by dg_summary)

dg_summary → Prints force/aperture/strategy/memory state
              Sends snap_grasp + snap_held + PCA overlay to Gemini
              Gets grasp_quality 0.0–1.0 + category + reason
              Patches last JSONL entry with quality score
```

---

### Step 4 — After 5+ grasps of the same object

The system skips DeliGrasp automatically:

```
Memory confident (n=7, std=0.31N) — skipping DeliGrasp
Blended force: 3.8N (memory 70% + GP 30%)
```

Force prediction improves each run via Kalman update.

---

### Teach mode (move arm by hand)

```bash
source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
ros2 service call /arm/teach_mode std_srvs/srv/Trigger
# move arm, then call again to disable
ros2 service call /arm/teach_mode std_srvs/srv/Trigger
```

Or from the notebook: `node.teach()` / `node.unteach()`

---

### Emergency / recovery

```bash
# Gripper stuck closed (AX-12 overload)
ros2 service call /gripper/clear_error std_srvs/srv/Trigger
ros2 service call /gripper/open std_srvs/srv/Trigger

# UR5 RTDE connection dropped ("End of file")
# → re-run cell c01 in the notebook (kills and restarts ur5_node)
# → on pendant: ensure external control program is running (play button)

# Slip guard
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

Results saved to `tests/vlm_consistency_YYYY-MM-DD_HHMM.md`.

---

### Sensor poll rates

```bash
source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
python3 scripts/measure_poll_rates.py
```

Expected: arm 500Hz, gripper 10Hz, camera 10Hz. F/T sensor requires hardware connection on 192.168.0.5:49152.

---

## Grasp memory system

Each grasp updates `data/grasp_log/force_priors.json` (Kalman filter per object) and `data/grasp_log/embeddings.npz` (DINOv2 ViT-S embedding + true_force_label).

At query time:
1. **Named Kalman state** — per-object force prior, updated after every grasp
2. **DINO RAG** — for new objects only: embed detection crop, retrieve visually similar past grasps, weighted-mean force. n=0 so DeliGrasp is never skipped on a new object's first grasp.
3. **Blend with DeliGrasp** — weight = `min(0.7, n/10)` — at n=7 it's 70% memory
4. **Skip DeliGrasp** — if n≥5 and std<1N for THIS object, no Gemini API call needed

Training labels: `true_force_label` = force after slip-guard corrections (DAgger/HER relabeling). `force_label_error_n` = gap between initial prediction and truth.

---

## Point cloud pipeline

```
depth image + SAM3 mask
        │
        ├─ Scan 1: detection position (side view) → pts_cln1
        ├─ Scan 2: top-down position              → pts_cln2
        │
        ├─ Spatial crop: 12cm radius around object centroid + Z bounds
        │   (eliminates table surface that leaks through the SAM3 mask)
        │
        ├─ Merge (pts_cln1 + pts_cln2) → pts_use
        │
        └─ PCA (top 30% by Z) → grasp angle, major/minor axes
               │
               └─ Gemini strategy override for complex shapes
```

**Important:** run `measure_table` on an empty scene before placing the object — this captures TABLE_Z (needed for grasp depth). The background point cloud (BG_PTS) is also captured for debugging, but is **not** used in the main pipeline: at VLA inference time you won't have a background scan, so training data must not depend on one.

---

## VLA data format

Each row in `grasps_YYYY-MM-DD.jsonl`:

```json
{
  "object": "red cube",
  "held": true,
  "grasp_log": [{"attempt": 1, "ap": 30.6, "force": 4.23, "status": "contact"}],
  "true_force_label": 4.5,
  "init_force_pred": 3.8,
  "force_label_error_n": 0.7,
  "sg_fires": 1,
  "grasp_angle_deg": 31.7,
  "grasp_strategy": "symmetric",
  "obj_width_mm": 52.3,
  "gp_mass_g": 120.0,
  "gp_mu": 0.6,
  "gp_k": 800.0,
  "image_detect_path": "data/grasp_log/20260609_143201_image_detect.jpg",
  "image_grasp_path": "data/grasp_log/20260609_143201_image_grasp.jpg",
  "timestamp": "2026-06-09T14:32:05.123"
}
```

`true_force_label` is the corrected force after slip-guard interventions — use this as the training target, not the initial Gemini prediction.

---

## ROS2 node API

### Gripper (`/gripper/*`)

| Service | Type | Description |
|---|---|---|
| `/gripper/open` | Trigger | Open fully |
| `/gripper/close` | Trigger | Close to contact |
| `/gripper/set_force` | SetGripperForce | Set max force (N) |
| `/gripper/set_position` | SetGripperPosition | Move to position (mm) |
| `/gripper/clear_error` | Trigger | Re-enable after AX-12 overload shutdown |
| `/gripper/calibrate` | Trigger | Calibrate open/close limits |

Topic: `/gripper/state` (10Hz) — `position` mm, `force` N, `temperature` °C, `contact_detected` bool

### Arm (`/arm/*` or `/ur5/*`)

| Service | Description |
|---|---|
| `/arm/teach_mode` | Toggle freedrive (teach) mode |

### Slip guard (`/slip_guard/*`)

| Service | Description |
|---|---|
| `/slip_guard/enable` | Start monitoring (captures reference depth on first frame) |
| `/slip_guard/disable` | Stop monitoring |

Topic: `/slip_guard/config` — `Float32MultiArray [force_n, slip_thresh, obj_u, obj_v]`
Topic: `/slip_guard/events` — event log strings

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
| `AttributeError: NoneType.success` | UR5 RTDE dropped — re-run c01, check pendant external control program |
| Gripper closes but doesn't hold | AX-12 overload: `ros2 service call /gripper/clear_error std_srvs/srv/Trigger` |
| SAM3 no detection | Move arm 2cm closer; or check `/tmp/sam3.sock` exists (re-run c01) |
| `TABLE_Z not set` warning | Run `measure_table` cell on a clear table before placing the object |
| F/T sensor timeout | Check ethernet cable to 192.168.0.5; try `ping 192.168.0.5` |
| PCA shows table surface | Spatial crop not isolating object — check that SAM3 mask is tight and `_det_crop` is captured before `fp_color = color2` reassignment |
| DINOv2 slow on first load | Model downloads ~84MB once to `~/.cache/torch/hub`; subsequent loads are instant |
