# V1.1 — The 0° Fix: root cause, patch, and execution plan

*2026-07-13. Companion: [V1_EVAL_REPORT.md](V1_EVAL_REPORT.md) (the failures),
[OFFLINE_TEST_REPORT.md](OFFLINE_TEST_REPORT.md) (Findings A–C).*

## Root cause (found at the line level)

The flat-face angle in the pickup cell is computed **mod 90**
(`_flat_cloud = float(_brect) % 90.`). Mod-90 is *discontinuous at the 0/90 boundary*:
a perfectly straight block reads `0.3°` or `89.7°` depending on pixel noise alone. Both
readings pass the mod-90 consistency gate (distance to the flat face ≈ 0 either way), so
the expert arbitrarily executed a ~0° or ~90° wrist rotation on physically identical
scenes.

**Measured in the dataset** (`data/v1_1_episode_filter.json`): of 62 straight-block
episodes, **47 rotated 85–90° and 15 rotated 2–5° — perfectly bimodal, nothing between**.
Imitation learning averages the modes → the deployed policy rotates ~43° above straight
blocks → all 3 eval failures (0°: 88% success, 0.58 grade vs 100%/0.73–0.81 elsewhere).

Contributing failure: the documented "reject `issue=angle`" gate was never enforced in
code (Finding A) — 86/88 flagged episodes passed on reward alone, so nothing caught the
divergent supervision at commit time.

## The patch (all code landed 2026-07-13, commit 15e195e)

1. **Boundary canonicalization** (pickup cell, all notebook copies): for symmetric
   footprints, a flat-face reading > 84° executes as `ang − 90` (≈0° rotation — the
   physically identical grasp). Straight blocks now always get the SAME demonstration.
   Mid-angle conventions untouched → no new divergence with the surviving 182 episodes.
2. **Judge prompt** scores mod-90-equivalent grips as aligned (ANGLE_OFF to the NEAREST
   face, range −45..45); a remaining `angle` flag means genuinely skewed.
3. **Hard angle gate** at commit: `issue=angle` now rejects, period (Finding A enforced).
4. **Traceability**: every new episode records its grid `combo_id` (Finding C), and the
   staging notebooks can no longer add episodes (Finding B).

## Execution plan (~half a robot-day + 1 cluster job)

1. **Collect ~50 replacement straight-block episodes** with the patched expert
   (`v1_1_collect.ipynb` sequencer, straight-angle combos only — 25 cells × {0°} × 2 reps
   equivalent). Expect every one to execute ≈0° rotation. (~50 min)
2. **Train on the filtered set**: 182 kept old episodes (`keep` list in
   `data/v1_1_episode_filter.json` — excludes 47 far-mode + 3 stowaways) + the new ~50:
   ```
   --dataset.episodes='[...keep + new indices...]'   # supported by lerobot DatasetConfig
   ```
   150k steps on DeltaAI as before (~1.5 h). Output → `models/act_v1_1`.
3. **Re-run the identical 100-grasp eval** (`v1_eval.ipynb`, POLICY_DIR → act_v1_1,
   fresh results file `data/v1_1_eval_100.json`). (~3 h unattended)

## Success criteria (pre-registered)

| Metric | V1 measured | V1.1 target |
|---|---|---|
| 0° success | 22/25 (88%) | 25/25 |
| 0° mean grade | 0.58 | ≥ 0.75 |
| 0° yaw error (deploy) | ~41° on failures | < 10° median |
| 25/45/70° success | 75/75 | no regression (≥ 73/75) |
| Overall | 97/100 | ≥ 98/100 |

If 0° improves but mid angles regress, the exclusion removed too much support — fallback
is re-including far-mode episodes with their action yaws relabeled (riskier; only if
needed).

## Why this also strengthens the paper

Before/after heat-maps on the identical instrument close the loop:
**measure → predict (audit) → confirm (eval) → fix (V1.1) → verify (re-eval)** — and the
fix is a *data* patch with the architecture, hyperparameters, and training recipe frozen,
which is the paper's thesis in one experiment.
