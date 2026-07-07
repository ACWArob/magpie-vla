# MAGPIE Data Standard — the lab method for collecting policy training data

Version 1.0 (2026-07-08). Applies to V1 and everything after. Rationale + citations in
[V1_data_research.md](V1_data_research.md); V0 evidence in [../tests/July_7_Update.md](../tests/July_7_Update.md).

## The loop

```
1 DEFINE  →  2 CONTRACT  →  3 COVER  →  4 GATE+AUDIT  →  5 MAP  →  6 FILL/EXPAND  → (repeat)
```

### 1. DEFINE the support (before any collection)
Declare the exact manifold the policy must own, in the ROBOT's world frame centered on the
collection home pose (never human/table reference):
- V1: positions 5×5 grid @ 3cm (±6cm around home) × angles 0–90° in 15° steps (7)
- The policy will work INSIDE this and nowhere else (interpolation-only generalization).

### 2. CONTRACT — consistency rules (violations poison the dataset)
- One canonical motion profile: same speeds, heights, phase order, every episode.
- Grasp angle MUST match the object's flat-face orientation (minAreaRect snap). No exceptions.
- Everything the policy must reproduce is IN the action space: target pose (waypoint-style,
  keep) + **gripper aperture in mm (continuous)**. No script-side events the policy can't express.
- Episode = the task and nothing else: record reach → grasp → +5cm lift, verified held. STOP.
  The carry/place is the (unrecorded) reset.

### 3. COVER — systematic enumeration, robot-automated
- Sequencer walks a persisted coverage ledger of (cell, angle) combos; the PLACE leg of
  episode N sets up episode N+1 at the next combo (+ jitter ±1cm, ±5°). One pickup/episode.
- 1–2 episodes per combo (per-cell reps saturate; spend budget on new cells).
- +~10% edge/corner reps (boundary of the support matters most).
- +~15% scripted RECOVERY episodes: small deliberate offset at approach, the pipeline's
  re-localise corrects it — recording correction behavior.
- Misses retry the same cell; ledger marks a combo done only when the episode SAVED.

### 4. GATE + AUDIT
- Reward gate (existing): held (camera verdict) AND quality ≥ 0.6.
- NEW: reject `issue=angle` episodes outright — angle-inconsistent supervision is the
  single most poisonous defect (V0: 42/62 at ~90° → policy froze on 45° blocks).
- Pre-training audit (10 min, mandatory): (a) executed-angle histogram — should be ~uniform
  across the 7 bins; (b) per-cell coverage count — no holes; (c) episode length
  distribution — tight, no carry-tail outliers. Any anomaly → fix the GENERATOR, recollect.

### 5. MAP — eval on the collection grid
Deploy the trained policy AT each grid cell (subset of angles) → per-cell success heat-map.
Failure becomes an address ("cell (4,2) @ 30°"), not a vibe. The heat-map is also the
publishable figure.

### 6. FILL, then EXPAND ONE axis
- Red cells → targeted recollection of exactly those combos (a ~1h fill run), retrain.
- Green map → expand a single axis per version: wider positions → heights → new object →
  clutter/lighting. Never two axes at once (the map can't attribute changes).
- Multi-object = object becomes a grid axis; ~50 episodes/object across many objects beats
  hundreds on one (scaling laws).

## Standing numbers (V1 instance)
| Parameter | Value |
|---|---|
| Grid | 5×5 @ 3cm, centered on collection home |
| Angles | 0,15,30,45,60,75,90° |
| Jitter | ±1cm position, ±5° angle, per episode |
| Budget | 175 combos + ~20 edge reps + ~30 recovery ≈ 225 |
| Episode | reach→grasp→+5cm lift (held-verified), ~20s, ONE pickup |
| Throughput | ~50–60 episodes/hour → ~4–5 robot-hours |
| Gate | held (camera) ∧ quality ≥0.6 ∧ issue ≠ angle |
| Training | DeltaAI per docs/NSF_TRAINING.md (~2–3 GPU-hours) |
| Eval | same grid, per-cell heat-map |
