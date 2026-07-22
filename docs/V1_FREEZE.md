# V1 PIPELINE FREEZE — tag `v1-pipeline` (2026-07-22)

*The complete, working V1 pipeline as it stood at the end of the input-ablation
study. V2 development starts after this point and is free to break anything —
`git checkout v1-pipeline` resurrects exactly this state. On-disk artifacts
(models/, data/ — gitignored) are listed below so they don't get orphaned.*

## What this pipeline is (one paragraph)

Scripted AutoGrasp expert (SAM3 segmentation + minAreaRect angle + DeliGrasp
force + GraspGenX/PCA arbitration) collects grasps on a 5×5×4-angle grid,
gated by held-through-lift AND VLM judge ≥ 0.6 → 10 Hz LeRobot dataset (229
episodes, 442 MB) → ACT (51.6M params) trained 100k steps on DeltaAI (~1.5 h)
→ deployed at 10 Hz with n_action_steps=10, no ensembling → graded on the
100-grasp grid instrument with lift-verify, auto-recovery, resume-safe JSON,
per-rollout motion recording.

## Headline numbers (all in docs/ABLATION_RESULTS.md + V1_EVAL_REPORT.md)

| Policy | In-zone | OOD ring |
|---|---|---|
| act_v1 | 97/100 · 0.696 | 2/15 (13%) |
| act_noXTheta | 96/100 · 0.813 | 3/20 (15%) — tracks (r=.92), clamps at ±6.6cm |
| act_noForce | 76/100 · 0.638 | — |
| act_noBoth | 94/100 · 0.793 | (pending) |

Key findings on record: non-additive ablation 2×2 (reliance-not-necessity);
0° label-aliasing defect replicated across all four input sets; OOD cliff
lives in the absolute action space (relative-actions variant designed, not
trained).

## Git-tracked pipeline (all at tag `v1-pipeline`)

| Piece | File |
|---|---|
| Collection (expert + gate + recorder) | `notebooks/v1_collect.ipynb` (V1.1 variant: `v1_1_collect.ipynb`) |
| Eval instrument (the 100-grid) | `notebooks/v1_eval.ipynb` |
| Ablation evals (as-run) | `notebooks/ablation_noXTheta / noForce / noBoth.ipynb` (+ `b3` v1-ring cell) |
| Dataset ablation builder | `scripts/make_ablation_dataset.py` |
| Cluster push+train | `scripts/push_train_ablations.sh`, `docs/NSF_TRAINING.md` |
| Driver bringup + health | `scripts/bringup_gripper_ft.sh`, `scripts/99-openrb-150.rules` (udev, installed), `scripts/gpu_reap.py` |
| Docs | `FULL_PIPELINE.md`, `DECISION_LEDGER.md`, `ABLATION_RESULTS.md`, `V1_EVAL_REPORT.md`, `NEXT_WEEK_PLAN.md`, `WORKSHOP_PAPER_DRAFT.md` |
| Figures | `docs/figures/` (grid heatmaps/summaries per policy, talk/, spur/, appendix/) |

## On-disk artifacts (NOT in git — do not delete during V2 work)

| Artifact | Size | What |
|---|---|---|
| `models/act_v0, act_v1, act_noXTheta, act_noForce, act_noBoth` | 198 MB each | trained checkpoints |
| `data/lerobot_v1` | 442 MB | the 229-episode dataset (videos + parquet) |
| `data/lerobot_v1_noXTheta / _noForce / _noBoth` | 20 MB each | ablation datasets (videos symlinked into lerobot_v1) |
| `data/eval_motion/` | 370 MB | ~320 recorded rollouts: `run_*` (noXTheta), `noforce_*`, `noboth_*` — per-rollout wrist mp4 + 10 Hz TCP/aperture JSON |
| `data/v1_eval_100.json`, `eval_noXTheta_grid/_ood.json`, `eval_noForce_grid.json`, `eval_noBoth_grid.json`, `v1_ood_ring.json` | — | eval results (resume-safe ledgers) |
| `data/v1_1_coverage.json`, `v1_1_episode_filter.json` | — | V1.1 collection assets (ready, unused) |
| DeltaAI: `~/lerobot_v1*`, `~/act_*_run/` | — | cluster-side copies (jobs 2690361/362/363) |

## Hardware facts V2 inherits (hard-won 2026-07-21)

- Gripper board = OpenRB-150 on `/dev/ttyACM0` (+ `/dev/openrb` symlink via udev
  rule). ModemManager MUST be kept off it (rule installed). Health = state
  topic publishing, not services registered.
- AX-12: torque decays ~1–2 s holding → re-grip on a cadence while carrying;
  clear_error resets force to 2 N → ordered clear→set_force→close, never async.
- VSCode kernel restarts leak VRAM → `gpu_reap.py` (wired into eval runners).

## To resurrect V1 exactly

```bash
git checkout v1-pipeline          # code+docs at freeze
# models/ + data/ are untouched on disk; nothing else needed
```
