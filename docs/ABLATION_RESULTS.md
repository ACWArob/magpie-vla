# Input-Ablation Results — dropping absolute position/orientation from the policy state

*On record 2026-07-21. Interpolation (in-zone) block complete for the first
variant; extrapolation (OOD ring) still pending — see "What's left".*

## The experiment

act_v1's state input is `[x, y, z, rx, ry, rz, grip, force, wrist_fz]` (9 dims,
absolute pose included). Hypothesis (mentor, 2026-07-14): a policy handed
absolute position/orientation can **memorize** position→action instead of
*looking*, which is great in-zone and falls off a cliff the moment position is
OOD (act_v1's measured ±9cm result). The wrist camera already gives a
near position-invariant view, so the fix is to **remove the absolute state
channels** and force the policy to use vision.

Three variants trained from scratch on the *same* 229 episodes (DeltaAI jobs
2690361/362/363), differing only in which state columns were dropped:

| Variant | Dropped | State dims | Model |
|---|---|---|---|
| **act_noXTheta** | `x`, `rz` (yaw) | 7 | `models/act_noXTheta` |
| act_noForce | `grip_force` | 8 | `models/act_noForce` |
| act_noBoth | `x`, `rz`, `grip_force` | 6 | `models/act_noBoth` |

Same eval instrument as act_v1: 5×5 position grid × 4 angles (0/25/45/70°),
lift-verify (8cm), grade 0–1, resume-safe. 100 combos = interpolation. A ±9cm
OOD ring = extrapolation.

## Result 1 — act_noXTheta, interpolation (100-grasp grid)  ✅ COMPLETE

**Dropping absolute x + yaw did not cost in-zone grasping — it slightly improved it.**

| Metric | act_v1 (baseline, absolute) | **act_noXTheta (no x/θ)** | Δ |
|---|---|---|---|
| Pick rate | 97 / 100 | **96 / 100** | −1 (within noise) |
| Mean grade | 0.696 | **0.813** | **+0.117** |
| 0° (aliasing case) | 22/25 · grade 0.58 | **24/25 · grade 0.72** | **+2 picks, +0.14 grade** |
| 25° | 25/25 · 0.81 | 23/25 · 0.82 | −2 picks, ≈grade |
| 45° | 25/25 · 0.73 | 24/25 · 0.87 | −1 pick, +0.14 |
| 70° | 25/25 · 0.68 | 25/25 · 0.83 | +0.15 |
| First-try close | — | 99 / 100 | — |
| Seated aperture | — | 26.7 ± 7.4 mm | — |

Figures: `docs/figures/eval_noXTheta_grid_heatmap.png`,
`docs/figures/eval_noXTheta_grid_summary.png`.
Per-rollout motion (wrist mp4 + 10Hz TCP/aperture) recorded for 91 combos in
`data/eval_motion/` for the policy-vs-autograsp movement comparison.

**Read:** the pick rate is statistically identical (one grasp), but **grade
rose across every angle** and the **0° aliasing case — act_v1's weakest column —
improved most** (0.58 → 0.72). So removing the absolute channels did not force
the policy to give anything up in the trained zone; if anything the grasps are
cleaner. That is the precondition for the OOD test to mean something: had
in-zone grasping collapsed, an OOD number would be uninterpretable.

## Result 2 — act_noXTheta, extrapolation (OOD ring)  ✅ MEASURED (stopped early at 20)

**Dropping x/θ did NOT fix the extrapolation cliff — the rate is statistically
identical to baseline. What changed is *how* it fails.**

| | act_v1 (absolute) | **act_noXTheta (no x/θ)** |
|---|---|---|
| OOD ring | 2/15 (13%) · 95% CI [4, 38]% | **3/20 (15%) · 95% CI [5, 36]%** |
| Fisher exact | — | **p = 1.00 (indistinguishable)** |

Sample: 20 of a planned 52-combo spread (user-stopped); covers the left column,
bottom edge, and one corner combo. Data: `data/eval_noXTheta_ood.json`; every
rollout has a wrist mp4 + 10 Hz TCP/aperture trace in `data/eval_motion/`.

By position — the picks hug the trained zone, the corners are 0-for-everything:

| Ring position | Picks | Distance outside zone |
|---|---|---|
| (−9,−3), (−9,−6) | 3 / 7 | 3 cm (adjacent to zone) |
| (−9,−9), (−3,−9), (+3,−9), (+9,−9) | 0 / 13 | 3–6.5 cm (edges + corners) |

All 3 picks were at rotated placements (25°/45°) — same surface pattern as
v1's 2 ring successes.

### The mechanism (measured from the 20 recorded trajectories)

- **It looks.** Close position tracks the commanded block position: r = 0.92
  (x-axis). This is the behavior the ablation was designed to produce, and v1
  almost certainly lacks (its state-x was a perfect shortcut in training).
- **It clamps.** Close-x never exceeds **−6.6 cm** against blocks commanded at
  −9. The trained placements span ±6 cm — the policy walks toward the block and
  stops at the exact boundary of its training data. Mean undershoot 4.7 cm.
- **Fails close on empty air** (aperture → −5 mm, fully shut) at the boundary;
  the 15% "successes" are geometry, not generalization: a ~3 cm undershoot on a
  5 cm block still catches an edge at ring cells adjacent to the zone, never at
  corners 6–7 cm out.

**Why (three stacked causes):** (1) ACT regresses absolute action targets, and
every training target lies inside ±6 cm — a regressor cannot emit values outside
the support of its labels, so the output manifold is clipped at the boundary it
was trained on (the measured −6.6 wall). (2) The wrist view at home height sees
the whole mat, so "block at 9 cm" is a genuinely novel scene, mapped to the
nearest familiar one. (3) Behavioral cloning interpolates; nothing in it
extrapolates.

**Bottom line:** removing x/θ moved the failure from *"doesn't look"* to
*"looks, but can only act inside the box it has seen."* The cliff migrated from
the state distribution to the **action** distribution. The variant that attacks
that directly — relative/delta actions, where a far block becomes a sequence of
in-distribution short hops under the 10-step receding horizon — is on record as
the next experiment but **not started** (user decision 2026-07-22: understand
this data first).

**v1's mechanism is predicted, not measured** (its old ring run predates
recording). The `b3` cell in `ablation_noXTheta.ipynb` runs act_v1 on the
identical ring with recording, whenever robot time allows — predicted signature:
close positions *uncorrelated* with the block.

## Result 3 — act_noForce, interpolation (100-grasp grid)  ✅ COMPLETE — the surprise

**Removing grip force cost 20 points of pick rate in-zone. Both of us (user and
assistant) predicted it would be redundant. It is not — and the *way* it fails
reveals what the channel was actually for.**

| | act_v1 | act_noXTheta | **act_noForce** |
|---|---|---|---|
| Pick rate | 97/100 | 96/100 | **76/100** |
| Mean grade | 0.696 | 0.813 | **0.639** |
| Fisher vs the other two | — | — | **p ≤ 0.0001** |

By angle — a clean gradient that tracks how much alignment correction each case
needs:

| Angle | Picks | Grade | Yaw error |
|---|---|---|---|
| 25° | 22/25 | 0.81 | 8.8° |
| 0° | 19/25 | 0.64 | 20.7° |
| 45° | 22/25 | 0.72 | 20.5° |
| **70°** | **13/25** | **0.38** | **30.9°** |

Failure taxonomy (24 fails): **20 closed-on-empty-air** (fingers fully shut,
ap ≈ −5 mm), 4 closed-then-dropped, 0 never-closed — the gripper hardware
executed every close; the policy closed in the wrong place. Data:
`data/eval_noForce_grid.json`, 100 recorded rollouts (`noforce_*`), figures
`eval_noForce_grid_heatmap/summary.png`.

### Mechanism (measured from the 100 trajectories)

- **Fails close 1.8 ± 1.1 cm off block center** (picks: 0.9 ± 0.6; noXTheta:
  0.8 ± 0.7). On a 5 cm block, ~2 cm puts the finger line at the edge — the
  close clips a corner, shoves the block out, and shuts on air (confirmed on
  video: wrist frame at the close moment shows fingers on the block's corner).
- **Closes fire late** (mean step 102 vs noXTheta's 93; up to +4.6 s early in
  the run) and with incomplete yaw correction (see the 70° column).
- Close **height** is unchanged — descent is fine; it is the lateral/yaw
  alignment and the commit timing that break.

### Interpretation — force was the phase clock, not touch confirmation

The naive prediction said force is redundant for a rigid block because the
aperture plateau already signals contact. The data killed that: contact
confirmation was never the channel's main job. In every training episode,
grip force traces one clean arc — flat during centering/descent, ramp at
squeeze, high during hold. It is the single input that unambiguously encodes
*which phase of the grasp the policy is in*. Two supporting facts: (a)
`wrist_fz` is all-zero in the training data (verified), so grip force was the
**only live touch channel the policy ever had**; (b) with it removed, the
commit-to-close decision decouples from centering progress — the policy
hesitates, then closes while still misaligned, failing worst exactly where the
most correction is needed (70°).

**One-line version: the force input's role was temporal, not tactile — it told
the policy *when* it was, not *what* it was touching.**

### Registered prediction for act_noBoth (falsifiable, test next)

noBoth drops x/θ *and* force → it should exhibit **both** measured pathologies:
noForce's late/off-center closes in-zone (pick rate ≈ 76% or worse, worst at
70°) *plus* noXTheta's track-then-clamp on the OOD ring (~15%, clamped at
±6 cm). If noBoth lands near noXTheta's 96% instead, the phase-clock story is
wrong. Written down before the run on 2026-07-22.

Open checks before this is paper-firm: (1) training-loss parity on DeltaAI
(rules out "noForce just converged badly" — needs Duo login); (2) the act_v1
in-zone re-slice for the instrument caveat below.

## Honest caveats (read before quoting these numbers)

1. **The instrument changed between the two runs.** act_v1 (97/100) was measured
   before today's eval fixes; act_noXTheta was measured after them (ordered
   close chain, re-grip-while-carrying, gripper preflight gate). The re-grip fix
   in particular keeps a grasp seated through the lift, which plausibly lifts
   grade a few points independent of the policy. **To make the grade comparison
   airtight, re-slice ~20 act_v1 combos on this machine with the current
   instrument.** The *pick-rate* comparison (96 vs 97) is robust to this; the
   *grade* delta (+0.117) is the part that needs the re-slice before it goes in
   a paper.
2. **This is one variant.** act_noForce and act_noBoth interpolation blocks are
   not run yet.
3. **The headline claim is about extrapolation and is NOT yet tested.** Grade
   parity in-zone is necessary but not the hypothesis. See below.

## What's left

Scoreboard (extrapolation was the headline question — now answered):

| Policy | Input | In-zone | OOD ring |
|---|---|---|---|
| act_v1 | absolute | 97% · 0.70 | 2/15 (13%) |
| **act_noXTheta** | no x/θ | **96% · 0.81 ✅** | **3/20 (15%) ✅ — same rate, different mechanism** |
| act_noForce | no force | **76% · 0.64 ✅ (−20 pts — the surprise)** | — |
| act_noBoth | no x/θ/force | ? | ? |
| act_relative (not trained) | no x/θ + delta actions | — | the variant the mechanism data points at |

Pending, in order:
1. **act_noBoth · grid + OOD** ⟵ NEXT (tests the registered prediction in Result 3)
2. `b3` — act_v1 on the recorded ring (failure-mode figure), when robot time allows
3. ~20-combo act_v1 in-zone re-slice (the instrument caveat)
4. act_relative — train offline from the same 229 eps (no robot needed); deliberately deferred
5. training-loss parity check on DeltaAI (one command, needs Duo)

## Hardware note (why this took a full day)

The run was repeatedly stalled by the OpenRB-150 gripper board dropping off USB
on a ~8s cycle. Root cause found late: **ModemManager probing the CDC-ACM port**
(no udev rule protected it) — fixed by `scripts/99-openrb-150.rules`
(`ID_MM_DEVICE_IGNORE` + a stable `/dev/openrb` symlink). Along the way five
real software faults in the eval loop were fixed and committed (ordered close
chain, re-grip while carrying, short-timeout + reboot escalation, gripper
preflight health gate, auto-reap of abandoned GPU-holding kernels). None of the
recorded data is contaminated — every failure occurred *outside* a recorded
combo (crash paths `continue` without writing).
