# MAGPIE Pickup Pipeline

## Overview

The pickup pipeline takes a single command — "pick up the object" — and executes a full grasp cycle: detect, localise, plan, grasp, lift, and record training data. It runs in two Jupyter cells from `notebooks/magpie_collect.ipynb`.

---

## Hardware

| Component | Role |
|---|---|
| UR5e arm | 6-DOF manipulator |
| MAGPIE gripper | Custom 2-finger parallel jaw (AX-12 servos) |
| RealSense D435i | Wrist-mounted RGBD camera (10 Hz colour, depth aligned) |
| ATI force/torque sensor | Wrist wrench at 250 Hz |

---

## Software Stack

```
Gemini 2.5 Flash          — object identification, grasp strategy, DeliGrasp physics priors
SAM3 (Segment Anything 3) — pixel-accurate object mask from text query
GraspGenX (NVlabs)        — cross-embodiment 6-DOF grasp pose generation (MAGPIE gripper)
Open3D                    — point cloud build, denoise, PCA
DeliGrasp                 — adaptive grip force from object physics
LeRobot                   — 10 Hz episode recorder for VLA training
Rerun                     — live + offline trajectory visualiser
```

---

## Pipeline Steps

### 1. Auto-Setup (start of cell 7)
- Loads saved camera calibration from `data/last_calibration.json` (clocking angle Rz, z-offset, lateral offset) — no manual calibration cell needed after first session
- TABLE_Z is auto-measured from the depth ring around the detected object — no clear-table step needed

### 2. Object Detection
- **Gemini** looks at the wrist camera image and names the object (2–4 words)
- **SAM3** uses that name to produce a pixel mask; retries by moving 2 cm closer if it misses

![Detection](../data/grasp_log/20260629_143039_image_detect.jpg)

### 3. Point Cloud Build
Two scans are merged for a complete 3D model:
1. Detection scan — arm at home height, wide view
2. Close-up scan — arm moves 5 cm closer, denser points

Both scans are back-projected through the calibrated `_TCP_TO_CAM` extrinsic and merged via centroid alignment. Background (table) is subtracted using the pre-captured `BG_PTS` cloud.

### 4. Grasp Angle Selection
PCA on the top-layer point cloud gives the object's major/minor axes. Three strategies compete:
- `symmetric` — 45° (works for any object)
- `short_side` — grip across the narrow axis (most stable)
- `long_side` — grip along the long axis (for flat objects)

**GraspGenX** (NVlabs cross-embodiment model, conditioned on MAGPIE's swept volume) proposes a 6-DOF grasp pose. Its angle is compared against the PCA winner; Gemini arbitrates if they disagree by more than 3°.

![PCA Plot](../data/grasp_log/20260629_143039_pca_plot.jpg)

### 5. Grasp Execution
```
open gripper
→ approach (APPROACH_H = 10 cm above object)
→ rotate wrist to grasp angle
→ pre-descent centering check (SAM3 mask centroid vs image centre, ≤80 px threshold)
→ descend to grasp Z (object midpoint − arc compensation for finger drop)
→ DeliGrasp close: Gemini estimates mass/stiffness/friction → force setpoint
→ slip guard: monitors wrist Fz; re-closes if grip slips
→ lift to approach height
```

![Grasp](../data/grasp_log/20260629_143039_image_grasp.jpg)
![Held](../data/grasp_log/20260629_143039_image_held.jpg)

### 6. DeliGrasp Force Policy
After contact, Gemini is queried with the wrist camera image to estimate:
- Object mass (g)
- Spring constant / stiffness (N/m)
- Coefficient of friction

These feed a physics-based force model (`F = mg / (2μ)`) with safety margins. The gripper closes incrementally, checking for slip between each step.

### 7. VLA Data Recording
Throughout the grasp, `VLARecorder` samples at 10 Hz:
- **State**: TCP pose (x,y,z,rx,ry,rz), gripper aperture (mm), grip force (N), wrist Fz (N)
- **Action**: commanded TCP pose + grip open/close
- **Image**: wrist camera RGB (848×480)

Episodes pass a **reward gate** (object held AND grasp quality ≥ 0.6) before being committed to the LeRobot dataset at `data/lerobot_magpie/`. Each episode is flushed to disk immediately after commit — no data loss if the kernel dies.

---

## Calibration

### Camera Clocking (Rz)
`auto_calibrate()` jogs the arm ±30 mm in world X and Y, measures the pixel shift of the detected object, and finds the rotation angle that best predicts those shifts. Two-stage search: coarse 4-quadrant then fine ±45° at 1° resolution. Result saved to `data/last_calibration.json`.

### Z-Offset
Camera-to-TCP distance = 0.144 m (0.120 m nominal + 0.024 m measured mount offset). This sets the TABLE_Z correctly (~33 mm above base).

### Lateral Offset
`_CAM_XY_OFFSET_M` in constants cell compensates for the colour lens not being centred in the camera housing (two IR holes visible). Tunable by observing consistent grasp miss direction.

---

## Running It

```
notebooks/magpie_collect.ipynb
  Cell 0:   title / notes
  Cells 1–6: startup (ROS nodes, imports, constants, Demo node)
  Cell 7:   full pickup  ← run this each grasp
  Cell 8:   summary + commit + Rerun  ← run after each grasp
```

Between grasps: reset the object, re-run cells 7 → 8.

---

## Dataset

| Field | Value |
|---|---|
| Format | LeRobot v3 (parquet + MP4 video) |
| repo_id | `magpie/grasp` |
| Root | `~/magpie_control/data/lerobot_magpie/` |
| FPS | 10 Hz |
| State dim | 9 (x,y,z,rx,ry,rz, grip_mm, grip_force, wrist_fz) |
| Action dim | 7 (x,y,z,rx,ry,rz, grip_cmd) |

Visualise a recorded episode:
```bash
python -m lerobot.scripts.lerobot_dataset_viz \
  --repo-id magpie/grasp \
  --root ~/magpie_control/data/lerobot_magpie \
  --episode-index 0
```
