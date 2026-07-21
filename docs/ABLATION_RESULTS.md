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

## What's left (the actual hypothesis test)

The whole point of dropping absolute pose is the **OOD cliff**. act_v1 fell from
97% in-zone to **~8%** on the ±9cm ring — it interpolated but did not
extrapolate. The open question:

> **Does act_noXTheta hold up on the OOD ring where act_v1 collapsed?**

Pending blocks:
- **act_noXTheta · OOD ring** (`b2`) — the headline experiment
- **act_noForce · grid** — is force needed in-zone?
- **act_noBoth · grid + OOD** — both channels dropped

Target table (extrapolation column is the result that matters):

| Policy | Input | In-zone | OOD ring |
|---|---|---|---|
| act_v1 | absolute | 97% · 0.70 | ~8% |
| **act_noXTheta** | no x/θ | **96% · 0.81 ✅** | **? ← pending** |
| act_noForce | no force | ? | — |
| act_noBoth | no x/θ/force | ? | ? |

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
