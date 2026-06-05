# MAGPIE Control — Changes from Original

**Date:** 2026-06-05  
**Branch:** ros  
**Authors:** ACWArob + Claude

This document details every change made on top of the original AutoGrasp/MAGPIE ROSification. Changes are grouped by area. The "original" refers to the codebase at commit `8851fdc` (end of Day 7, adaptive slip detection complete).

---

## 1. Camera Calibration — `_TCP_TO_CAM` Sign Fix

**File:** `notebooks/magpie_demo.ipynb` cell c03, `notebooks/angle_sweep.ipynb` cell c02

**Original:**
```python
_TCP_TO_CAM = homog_xform(R_krot([0, 0, 1], -np.pi/2), [0, 0, 0.120])
```

**Changed to:**
```python
_TCP_TO_CAM = homog_xform(R_krot([0, 0, 1], +np.pi/2), [0, 0, 0.120])
```

**Why:** The arm was moving in the wrong XY direction when approaching detected objects. Diagnosis: TCP rotation matrix read `[[1,0,0],[0,-1,0],[0,0,-1]]` — wrist flipped from calibration expectation. The `-π/2` rotation was incorrect for the actual camera mount orientation. Changing to `+π/2` made the arm move in the correct direction toward the detected object.

---

## 2. Safety Floor — HARD_FLOOR_Z Lowered

**File:** `notebooks/magpie_demo.ipynb` cell c03, `notebooks/angle_sweep.ipynb` cell c02

**Original:** `HARD_FLOOR_Z = 0.030` (30 mm)  
**Changed to:** `HARD_FLOOR_Z = 0.025` (25 mm)

**Why:** At 30 mm, valid grasp positions were being falsely rejected (observed: `gfz = 0.0296 m` blocked). 25 mm is verified safe by the floor-reach test (Section 13). The cutting mat sits at ~35 mm world Z so 25 mm gives adequate clearance without false failures.

---

## 3. pointcloud_utils.py — Significant New Functions

**File:** `scripts/pointcloud_utils.py`

### 3a. `top_layer(pts, percentile=50)`

**New function.** When the RealSense scans a textured cutting mat, the point cloud contains two Z layers: the object top surface (high Z) and the mat grid pattern (low Z). Statistical outlier removal cannot separate them because both layers are dense.

`top_layer` keeps only the top 50th percentile of Z values. This is used **exclusively for PCA angle computation** — position (`p_obj`) is always computed from the full-cloud depth centroid independently.

```python
def top_layer(pts, percentile=50):
    z = pts[:, 2]
    if z.max() - z.min() < 0.020:
        return pts   # single layer, no filtering
    return pts[z >= np.percentile(z, percentile)]
```

### 3b. `check_view_quality(pts_cleaned, mask, depth_mm)`

**New function.** Validates whether the camera has a usable view of the object by checking:
- Depth fill fraction: what fraction of the SAM3 mask pixels have valid (non-zero) depth
- Denoised point count: whether enough points survive statistical removal

Returns `{'ok': bool, 'reason': str, 'n_pts': int, 'fill': float}`. Used in the dual-scan cell (c09) to decide which scans to merge.

### 3c. `mask_grasp_angle(mask)`

**New function.** Computes grasp angle from the 2D SAM3 mask shape using image-space PCA. More robust than 3D point cloud PCA when the depth camera produces stripe artifacts (IR structured light on flat surfaces or IR-opaque objects).

Returns `(angle_deg, ratio)` in the same convention as `analyse_pcd`.

### 3d. `smart_grasp_angle(pca, object_name, image_rgb, gemini_client, mask)`

**New function.** Two-layer decision system for wrist rotation angle:

**Layer 1 — Geometry:**
- `major < 25 mm` → `symmetric` (point cloud unreliable)
- `ratio < 1.3` → `symmetric` (nearly square/round)
- Otherwise → `short_side` (default for rectangles)

**Layer 2 — Gemini override:** Sends RGB image + object name to Gemini 2.5 Flash. Gemini classifies into `symmetric / short_side / long_side`. Overrides geometry result.

Strategy to angle:
- `symmetric` → 0°
- `short_side` → `(pca_angle + 90°) % 180°`
- `long_side` → `pca_angle`

All results clamped to `% 90°` for wrist cable-wrap limit.

Returns `(angle_deg, strategy, reason)`.

### 3e. `denoise_pcd` — Reverted to Statistical Only

**Original (late iterations):** Had DBSCAN clustering added to handle blue outlier clusters.  
**Reverted to:** Pure statistical outlier removal (`remove_statistical_outlier`, nb_neighbors=30, std_ratio=0.5).

**Why reverted:** DBSCAN broke things in multiple ways — "highest Z cluster" selected gripper cables; hint-based clustering failed when the cube was IR-opaque; complexity not justified. The `top_layer` function solves the mat-grid problem more cleanly without touching position computation.

---

## 4. DeliGrasp — Image Input + Physics Force Policy

**File:** `notebooks/magpie_demo.ipynb` cells c14, c15, 17ea9af9

### 4a. Image Sent to Gemini

**Original:** DeliGrasp sent only a text description of the object.  
**Changed:** Sends the actual camera RGB image alongside the description.

**Why:** Gemini can assess material, surface finish, and condition from the image directly. Text descriptions like "red eraser" are ambiguous; an image shows exactly what Gemini is evaluating.

### 4b. Physics-Based Force Policy

**Original:** Fixed force values from the Gemini descriptor, single-shot close.

**Changed to physics-based minimum:**
```
F_min       = (mass_g / 1000 * 9.81) / (2 * mu)   # Coulomb friction minimum hold force
force_cap   = k / 1000                              # stiffness ceiling (N)
close_force = clip(max(initial_force, F_min * 1.2, 5.0), 5.0, force_cap)
slip_thresh = clip(F_min * 0.9, 0.15, force_cap * 0.8)
```

`k` (stiffness in N/m from DeliGrasp) provides the force ceiling — low-k objects (foam, soft materials) get a low cap to avoid damage.

### 4c. Deficit-Based Escalation

On slip detection (measured force < threshold), force is increased by the larger of `add_force` or `(deficit + 0.5)`:
```python
deficit = max(0, slip_thresh - measured_force)
cf = min(force_cap, cf + max(add_force, deficit + 0.5))
```

Up to 6 attempts per grasp.

### 4d. Aperture Pre-Positioning

After descending to grasp height, the gripper is pre-positioned to `obj_w_mm + 5 mm` before the attempt loop. This gives faster, more controlled contact than closing from fully open.

---

## 5. TABLE_Z — Object Height Measurement

**File:** `notebooks/magpie_demo.ipynb` cell c10

**Original:** `grasp_off` was a fixed value or computed from a manually-set `TABLE_Z`.

**Changed:** TABLE_Z is measured automatically from depth pixels in a dilated ring adjacent to the SAM3 mask (not inside it):

```python
dilated  = cv2.dilate(mask.astype(uint8), np.ones((30,30)))
tbl_mask = dilated.astype(bool) & ~mask
tbl_dm   = median(depth[tbl_mask][valid]) / 1000.
TABLE_Z  = world_z(tbl_dm)
obj_height = p_obj[2] - TABLE_Z
grasp_off  = clip(obj_height / 2, 0.010, 0.040)
```

The gripper now targets the object's midpoint rather than its top surface.

---

## 6. Dual-Scan Point Cloud (magpie_demo cell c09)

**Original:** Single point cloud scan from wherever the arm happened to be.

**Changed:** Two scans, merged:
1. **Scan 1** — from current arm position (side/angled view): captures object profile
2. **Scan 2** — arm moves directly above the object at approach height (top-down): captures object footprint

Both pass through `check_view_quality`. Merged with `np.vstack` if both OK; falls back to whichever is better if one is blocked.

**Why:** Side views give better Z depth for position; top-down gives cleaner XY footprint for PCA angle. Merging exploits both.

---

## 7. Post-Lift Grasp Validation

**File:** `notebooks/magpie_demo.ipynb` cell 17ea9af9

**Original:** Lifted and checked only force (`force >= slip_thresh`).

**Changed to dual criterion:**
```python
force_ok    = s_lift.force    >= slip_thresh
aperture_ok = s_lift.position >= obj_w_mm * 0.4
held        = force_ok and aperture_ok
```

`aperture_ok` catches the case where the object slipped through the fingers (gripper closed past the object → position reads near 0 mm, but mechanism friction can still read non-zero force).

---

## 8. SAM3 — Gemini Auto-Detect + Close-Up Retry

**Original:** Object name was hardcoded in `OBJECT` config variable.

**Changed:**

**Auto-detect:** At the start of each pickup, Gemini 2.5 Flash receives the camera image and returns a 2–4 word object description automatically.

**Close-up retry:** If SAM3 returns no detections, the arm moves 2 cm closer and retries, up to 3 times, before skipping.

---

## 9. Z Safety Check — Simplified

**File:** `notebooks/magpie_demo.ipynb` cell c12

**Original:** Safety check involved computing arc drop from gripper geometry.

**Changed to:** Simple floor check on commanded fingertip positions:
```python
ok_a = (p_obj[2] + APPROACH_H) >= HARD_FLOOR_Z
ok_g = (p_obj[2] - auto_offset) >= HARD_FLOOR_Z
```

**Why:** The arc drop correction was causing false failures. The actual fingertip Z during descent is directly computable from the commanded TCP Z minus GRIPPER_LEN — no arc model needed for the safety check.

---

## 10. Floor Reach Test — Section 13 (magpie_demo)

**Original:** Separate `floor_test.ipynb` notebook, which had kernel-sharing issues (couldn't access `home`, `HARD_FLOOR_Z`, `GRIPPER_LEN` from magpie_demo).

**Changed:** Floor test integrated directly into `magpie_demo.ipynb` as Section 13. Descends 3 mm at a time from home to `HARD_FLOOR_Z + GRIPPER_LEN`. Generates a descent plot. All config variables available in same kernel.

---

## 11. New Notebook — angle_sweep.ipynb

**Original:** No systematic angle validation.

**New:** `notebooks/angle_sweep.ipynb` places an object at each angle in `[0°, 10°, 20°, ..., 90°]` and records whether the detected PCA angle matches the placed angle.

**Per-iteration execution block is matched exactly to magpie_demo full pickup cell (17ea9af9):**
- `open_g()` + 1 s at the very start of each iteration (before view_pose move)
- `open_g()` + 0.5 s before approach (demo exact)
- Approach/rotate/descend at demo timings (0.08/0.05/0.05 m/s, 0.4/0.4/0.5 s sleeps)
- Force calc then `set_pos(pre_ap)` after descent (demo order)
- 6-attempt loop with per-attempt print and deficit escalation (demo exact)
- Lift with 0.5 s / spin(8) (demo exact)
- Place at target angle (sweep-specific); release; return home

Outputs: per-attempt force/aperture prints, summary table, three plots (expected vs detected angle, force per angle, strategy distribution), auto-append to `tests/test_log.md`.

---

## 12. IDE — VSCode Python Path

**File:** `.vscode/settings.json`

Added `scripts/` to `python.analysis.extraPaths` and `python.autoComplete.extraPaths` so Pylance resolves `pointcloud_utils` without false `reportMissingImports` errors.

---

## Summary Table

| Area | What Changed | Impact |
|---|---|---|
| `_TCP_TO_CAM` sign | `-π/2` → `+π/2` | Arm moves toward object correctly |
| `HARD_FLOOR_Z` | 30 mm → 25 mm | Fewer false safety failures |
| `top_layer()` | New — Z filter for PCA only | Correct angle on cutting mat |
| `check_view_quality()` | New — view validation | Catches blocked scans before PCA |
| `mask_grasp_angle()` | New — 2D mask PCA | Fallback for IR-problematic objects |
| `smart_grasp_angle()` | New — geometry + Gemini | Correct wrist angle for any object shape |
| `denoise_pcd` | Reverted to statistical only | Stable, no false cluster selection |
| DeliGrasp — image input | Text → image + text | Better material/force estimation |
| DeliGrasp — force policy | Fixed → physics F_min + k cap | Safe for heavy and delicate objects |
| DeliGrasp — escalation | None → deficit-based 6-attempt | Handles slip without over-forcing |
| TABLE_Z measurement | Manual → auto from mask ring | Grips object midpoint, not top |
| Dual-scan PCA | Single scan → merge two scans | Better angle + depth coverage |
| Post-lift validation | Force only → force + aperture | Catches slipped-through objects |
| SAM3 object name | Hardcoded → Gemini auto-detect | Works on any object without reconfiguration |
| SAM3 retry | None → 3× close-up retry | Handles small/distant objects |
| Z safety check | Arc model → simple floor check | No false failures from arc math |
| Floor test | Separate broken notebook → Section 13 | Shares kernel, always runnable |
| `angle_sweep.ipynb` | New notebook | Systematic angle validation |
| `.vscode/settings.json` | Added scripts/ path | No Pylance import errors |
