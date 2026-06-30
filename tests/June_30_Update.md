# Code & Hardware Update — June 30, 2026

Progress since June 29 update. Focus: stop GraspGenX and SAM3 fighting over VRAM, and make grasp-angle planning faster and smarter.

---

## 1. SAM3 + GraspGenX VRAM Coexistence (measured, not assumed)

The June 29 doc assumed GraspGenX (~3 GB) and SAM3 (~4 GB) couldn't both fit on the 8 GB RTX 2070, so the pipeline killed SAM3 with `SIGKILL` before every GraspGenX call and restarted it afterward. That assumption was never actually tested — it just got carried forward as a design constraint.

Built two probe scripts (`vram_probe.py`, then a more rigorous `vram_probe2.py` that samples VRAM in a background thread during 3 real `GraspGenXZMQ.detect()` calls) and measured directly:

| Stage | VRAM used | VRAM free |
|---|---|---|
| Baseline (SAM3 only) | ~3.9 GB | ~4.3 GB |
| + GraspGenX server loaded | +0.8 GB | ~3.5 GB |
| Peak during real inference (both alive) | ~4.8 GB | **~3.0 GB** |

SAM3 stayed responsive (socket connectable) throughout. GraspGenX cold-starts in ~10s, not the previously-assumed 60s.

**Result: they coexist with ~3 GB of headroom.** Removed the kill/swap entirely:
- GraspGenX server is now preloaded in the startup cell (cell 1) so it's already warm before the first grasp
- Both models stay resident for the **entire session** — no per-grasp kill or reload of either
- This also fixes something the swap design had silently broken: the **pre-descent centering check and live recalibration** (`auto_calibrate()`) both depend on SAM3 being alive mid-pickup. Under the swap design SAM3 was dead at exactly the point those checks needed it, so recalibration could never actually trigger. It works again now because SAM3 is never killed.

**Files:** `notebooks/magpie_demo.ipynb` (cell 1, cell 40), `notebooks/magpie_collect.ipynb` (synced cells)

---

## 2. Parallel Grasp-Angle Planning

PCA and GraspGenX angle computation used to run sequentially even though they're independent (PCA only needs the point cloud; GraspGenX only needs the point cloud + mask). GraspGenX inference now fires in a background `threading.Thread` immediately after the point cloud is built, overlapping the ~350 ms of PCA computation + matplotlib visualization on the main thread. The main thread joins the GraspGenX thread right before the angle is needed.

Thread-safety note: all data the background thread reads (`pts_use`, `mask_use`, `fp_tcp`, `fp_ci.k`, `TABLE_Z`, `CAMERA_MOUNT_Z_OFFSET_M`, `strategy`) is captured as **default arguments** at thread-function-definition time, not read live from the main thread — this avoids needing locks since each value is frozen at thread creation.

Net effect: GraspGenX inference is now close to free time-wise (it's hidden behind PCA), instead of adding its own latency on top.

---

## 3. Multi-Candidate Gemini Arbiter

Previously the arbiter only compared PCA vs. a single GraspGenX angle and asked Gemini to break a tie if they disagreed by more than 3°. GraspGenX now returns up to 3 clustered candidate angles (15° clustering window), and all of them — plus the PCA angle — are pooled into one candidate list for `rank_grasp_angles_visual()`. Gemini looks at the live wrist image once and ranks across every candidate, rather than only ever seeing one GraspGenX proposal.

**File:** `scripts/pointcloud_utils.py` (`rank_grasp_angles_visual`)

---

## Current Status

| Goal | Status |
|---|---|
| SAM3 + GraspGenX coexistence | ✅ Measured and shipped — no more swap |
| Parallel PCA / GraspGenX | ✅ Working — background thread |
| Multi-candidate arbiter | ✅ Working — up to 4 candidates (PCA + 3 GraspGenX clusters) |
| Live recalibration during centering check | ✅ Working again (was silently broken under the swap design) |
| `anygrasp` reference in `grasp_compare.ipynb` | ⏳ Open — flagged, not yet investigated |
