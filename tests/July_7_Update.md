# July 7 Update — V0 Milestone: Collect → Train (NSF) → Deploy, Autonomous Picks

**The headline:** the full VLA loop closed today. 60 self-collected episodes → ACT policy
trained on NSF ACCESS (DeltaAI) in ~1 hour → deployed on the arm the same day →
**autonomous visual pick-and-place** (reach, self-rotate, close, lift, place). On mentor
deadline.

---

## 1. V0 dataset (yesterday → this morning)

- **60 clean episodes / 24k frames** in a dedicated LeRobot dataset (`data/lerobot_v0`),
  100% save rate, mean quality reward 0.87
- Collected largely hands-free by the **AUTO-COLLECT self-resetting loop**
  (recorded pick→place-at-centre, then unrecorded scatter to a random pose/angle)
- Audited: all trajectories real motion (40–59cm), zero stuck episodes

## 2. Training on NSF ACCESS (DeltaAI)

- Full runbook written: **[docs/NSF_TRAINING.md](../docs/NSF_TRAINING.md)** — reproducible
  without assistance; presentation template: **[docs/NSF_ACCESS_SLIDES.md](../docs/NSF_ACCESS_SLIDES.md)**
- Environment fights (all documented): ARM/aarch64 CPU-torch trap, torchvision VideoReader
  removal (use pytorch **2.7.0 module**, not newest), `module load cuda` for libcufile,
  `--policy.push_to_hub=false` (first job died in 33 s without it)
- **Result: 100k steps in 62 minutes on one GH200** (~33 epochs, batch 8), loss 1.0 → 0.043
  converged. Cost: ~1 GPU-hour of the 100-hour allocation
- Policy pulled back to `models/act_v0` (51.6M params), loads + infers on the lab RTX 2070
  at **11 ms/tick**

## 3. Deployment (`notebooks/v0_test.ipynb` — new, standalone)

Standalone notebook: drivers → node → GO-HOME (median frame-0 pose of the dataset itself)
→ DEPLOY (10 Hz servoL streaming with safety envelope: XY box, Z floor incl. GRIPPER_LEN,
step/rot-rate caps, tilt-only orientation gate with **free wrist yaw**).

**Deploy bugs found & fixed (verified by offline replay — the model itself reproduces
training trajectories to 1.9 mm):**

| Bug | Cause | Fix |
|---|---|---|
| 10 s open-loop lurches | ACT default `n_action_steps=100` | re-infer every 1 s (`=10`); NOTE: temporal ensembling made rotation WORSE (averages rotvecs across the 180° flip) |
| wild wrist swings | rotation-vector sign flips near 180° | per-tick canonicalization + 12°/tick rate limit |
| edge grasps | orientation cap blocked yaw alignment | tilt-only cap, yaw free |
| mechanical stutter | driver `_SERVO_TIME=0.002` (2 ms bursts) | 0.1 s to match the 10 Hz stream |
| hover, never closes | demos' finger **pre-position (~51 mm) is not in the action space** (grip is binary) — script did it exogenously | deploy shim reproduces it at grasp height; policy then closes ~0.9 s later, matching demo timing |
| RTDE script deaths | blocking gripper service calls starved the servo stream → watchdog kill | async gripper calls; `/arm/stop` (servoStop) before/after streaming |

**Results:** repeated clean autonomous picks in-zone, including off-centre blocks
(visual tracking confirmed). Post-grasp handoff to a scripted place at centre.

## 4. What V0 measured (the research findings)

1. **Competence ≈ data support**: reliable within the ±4 cm training scatter; at 5–7 cm the
   reach still tracks but the descent regresses to the training mean → miss
2. **Angle supervision was poisoned**: 42/62 episodes grasped at ~90° regardless of block
   angle (quality judge passed `issue=angle` grasps at 0.8) → policy **freezes on a 45°
   block** (imitation averages conflicting modes into inaction)
3. **Binary grip action is insufficient** — the pre-position lived outside the action space
4. ~Half of every episode is carry/place footage irrelevant to "pick up" (and its phase
   ambiguity caused post-grasp dithering)

## 5. Next steps — V1 (target: end of week)

Systematic coverage, not random (per discussion):

- **5×5 position grid @ 3 cm (±6 cm) × 7 angles (0–90° per 15°)** = 175 combos + edge
  reps ≈ **220 episodes**, each jittered ±1 cm/±5°
- **One pickup per episode**: record pick→lift only; the carry+place TO THE NEXT GRID POINT
  is the (unrecorded) reset → ~1 min/episode → **~4 robot-hours for the full set**
- Recorder changes: **gripper aperture (mm) as a continuous action channel**, episode ends
  at grasp + 5 cm lift, dead compute-gap frames trimmed
- Angle-consistent supervision: flat-face snap during collection; reward gate **rejects**
  `issue=angle` episodes
- Train on DeltaAI (same runbook, ~2–3 GPU-hours), eval = the same grid → per-cell success
  heat-map
- Throughput ceiling with the new loop: ~50–60 episodes/hour → **1–2k episodes/week** on one
  robot → multi-object datasets become a for-loop (task string already recorded per episode)
