# Code & Hardware Update — June 29, 2026

Progress since June 17 update. Focus: close the loop from detection → grasp → training data.

---

## 1. GraspGenX Integration (Complete)

Replaced prior grasp detectors with **GraspGenX** (NVlabs, ICRA 2026) — a cross-embodiment model conditioned on the gripper's swept volume mesh. Key advantage: no per-gripper training required; the MAGPIE gripper runs zero-shot.

- Server runs on `:5557` via ZMQ, generates 50 grasp candidates in ~0.7s, confidence up to 0.99
- Integrated into pickup pipeline as `GRASP_METHOD='graspgenx'`; falls back to PCA if server is unavailable
- VRAM budget: GraspGenX (~3 GB) and SAM3 (~4 GB) swap on the RTX 2070 (8 GB); SAM3 auto-restarts after each grasp *(superseded June 30 — see [June_30_Update.md](./June_30_Update.md): measured coexistence, swap removed)*

**Files:** `scripts/grasp_detectors/graspgenx_zmq.py`, `scripts/run_graspgenx_server.sh`, `scripts/grasp_detectors/GRASPGENX.md`

---

## 2. Camera Calibration (Fixed)

Two calibration errors were identified and corrected:

| Error | Root Cause | Fix |
|---|---|---|
| 7 cm XY offset | Camera clocking (Rz) angle wrong | `auto_calibrate()` — coarse 4-quadrant + fine ±45°@1° sweep |
| 5 cm Z offset | z_off hardcoded at 120 mm, true value 144 mm | Updated `_TCP_TO_CAM` z_off to 0.144 m |

`HARD_FLOOR_Z` was also hardcoded at 60 mm — after correcting z_off, TABLE_Z dropped to ~33 mm, making the hardcoded floor 27 mm *above* the table. Fixed: `HARD_FLOOR_Z = TABLE_Z` (derived from measurement, not hardcoded).

Calibration result is now saved to `data/last_calibration.json` and auto-loaded at the start of each pickup — no manual calibration cell required after first session.

---

## 3. Grasp Angle Bug Fix

All grasp angle strategies were producing identical outputs due to a mathematical identity:

```python
# Bug: (x + 90) % 180 % 90 == x % 90  — short_side collapsed to symmetric
angle = float(angle) % 90.

# Fix: preserve full half-turn range, clamp downstream
angle = float(angle) % 180.          # in pointcloud_utils.py
ang   = min(float(ang) % 180., 90.)  # clamp to wrist limit in pickup cell
```

Strategies now produce distinct angles (e.g. for a 30° object: symmetric=30°, short_side=90°).

---

## 4. Fingertip Arc Compensation

The MAGPIE gripper's 4-bar linkage causes fingertips to drop ~21 mm from fully open to fully closed. Previously uncompensated, causing grasps to land too low.

Fix: `gfz += fingertip_drop(105., grip_aperture_mm)` before descending, where `grip_aperture_mm` is estimated from the object's PCA minor axis. For a ~55 mm object this adds ~11 mm of upward correction.

**File:** `src/magpie_control/gripper_arc.py` (model), applied in pickup cell.

---

## 5. Streamlined Data Collection Pipeline

### Auto-TABLE_Z
TABLE_Z is now measured automatically from the depth ring around the SAM3-detected object — the table surface is visible around the block. No separate "clear table" step.

### Auto-Calibration Load
On each pickup, calibration is loaded from `data/last_calibration.json` if it exists and is less than 7 days old. First session still requires a manual calibration run.

### Episode Flush
After each successful grasp, the episode is immediately flushed to disk (`flush_episode()` closes the parquet/video writers). Files are valid even if the kernel dies mid-session. Previously, an interrupted session left corrupted parquet files that blocked subsequent runs. Auto-repair logic in `VLARecorder._ensure_ds()` also handles this case.

### Simplified Notebook
`notebooks/magpie_collect.ipynb` — 9 cells total, only the essential startup + pickup + summary cells. No scrolling through the full demo notebook during data collection.

**Session flow:** cells 1–6 (startup once) → cell 7 (pickup) → cell 8 (summary + commit + Rerun). Repeat cells 7–8 for each grasp.

---

## 6. Pre-Descent Centering Check

After wrist rotation and before descending, SAM3 re-detects the object and checks if its centroid is within 80 px of the image centre. Warns if the gripper is likely to miss laterally. Non-blocking — proceeds regardless, but the offset is printed for calibration tuning. *(superseded June 30 — see [June_30_Update.md](./June_30_Update.md): now corrective, with a live-recalibration escalation path)*

---

## 7. VLA Data Pipeline (Operational)

| Component | Status |
|---|---|
| LeRobot recorder (10 Hz) | Working |
| Reward gate (held + quality ≥ 0.6) | Working |
| Rerun live view | Working (`VLA_RERUN='spawn'`) |
| Rerun offline replay | Auto-opens after each commit |
| Dataset format | LeRobot v3, parquet + AV1 video |

First successful episodes recorded June 15 and June 29. Dataset at `data/lerobot_magpie/`.

![Detection](./20260629_143039_image_detect.jpg)
![Grasp](./20260629_143039_image_grasp.jpg)
![Held](./20260629_143039_image_held.jpg)

---

## 8. NSF ACCESS

Account approved and ready. Next step: rsync dataset to NSF storage and run a baby ACT training job to verify the compute pipeline before committing GPU hours.

---

## Current Status

| Goal | Status |
|---|---|
| AutoGrasp pipeline deployed | ✅ Complete |
| Camera calibration | ✅ Fixed (XY + Z) |
| GraspGenX integration | ✅ Working |
| LeRobot data collection | ✅ Pipeline ready, collecting |
| Rerun live + offline | ✅ Working |
| Sensor poll rates documented | ✅ Complete (June 17) |
| VLM consistency tests | ✅ Complete (June 17) |
| NSF ACCESS | ✅ Account ready |
| Policy training | ⏳ Pending — need ~50 episodes first |
| Harder objects / fruits | ⏳ Next after episode target hit |
| Placement at specified pose | ⏳ July scope |
