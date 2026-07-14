# Next-Week Plan — on record 2026-07-14 (grid returns ~2026-07-21)

*Two orthogonal fixes for act_v1's two measured failures, sequenced around grid availability.
Nothing here is started yet — this is the agreed plan to pick up next week.*

## The two failures and their fixes (orthogonal — they stack)

| Failure | Measured | Fix | Needs grid? |
|---|---|---|---|
| 0° aliasing (in-zone) | 88% / grade 0.58 at 0° | **V1.1**: canonicalize square-object angle labels + ~50 replacement episodes | **YES** (0° grid placements) |
| OOD extrapolation cliff | 97% → 13% at ±9cm | **Input-ablation** (mentor, 2026-07-14): drop absolute `x` + orientation `θ` from the policy STATE input so it must locate the block visually (wrist cam ⇒ position-invariant view); optionally switch to relative/delta actions for consistency | **NO** — reuses existing 229 episodes |

## Why input-ablation should help the OOD cliff (mentor's insight, made precise)

The state input `[x, y, z, rx, ry, rz, grip, force, wrist_fz]` includes absolute pose. A policy
given absolute position can memorize position→action instead of *looking* — great in-zone,
cliff the instant position is OOD (exactly the ±9cm result). The wrist camera already gives a
near position-invariant view (block-under-gripper looks the same everywhere), so the absolute
state input is the thing that goes OOD. Drop `x`/`θ` → forced to use vision → should generalize.
Subtlety: absolute action output needs the **relative/delta action** variant to be fully
consistent (same image at 2 positions can't map to 2 absolute targets).

## Sequence (agreed: option 3 — ablation first, then V1.1 with the winner, then compare)

1. **This/next week, NO robot — input-ablation experiment.** Reuses `data/lerobot_v1` (229 eps).
   Build offline: re-processor that (a) drops `x,rx,ry,rz` from state, (b) computes delta actions.
   Train 2–3 configs on DeltaAI (~1.5h each, unattended): baseline / input-drop / input-drop+relative.
   Re-run grid + OOD eval. → isolates the spatial-generalization effect on the SAME data.
2. **Next week, grid back — V1.1 collection.** ~50 straight-block replacement episodes
   (`data/v1_1_coverage.json` ledger already built; patched expert already committed — the
   `[angle canon]` boundary fix + enforced angle gate). Grid needs re-taping — **no recalibration**
   (grid is robot-frame relative to the hardcoded canonical home; TABLE_Z auto-measured).
3. **Combine + compare.** Retrain final policy on V1.1 data + the winning input representation.
   Re-eval on the identical instrument.

## Target result table

| Policy | Data | Input | In-zone | OOD ring |
|---|---|---|---|---|
| V1 baseline | grid | absolute | 97% | 13% |
| V1 + ablation | grid | no x/θ (+rel?) | ? | **? ← the OOD test** |
| V1.1 + ablation | grid + label fix | no x/θ | **~100%?** | ? |

## Ready-to-go assets (already committed)

- V1.1 ledger: `data/v1_1_coverage.json` (50 straight + 5 boundary probes)
- V1.1 patched expert: 0/90 canonicalization + enforced angle gate (commit 15e195e)
- Dual train script: `scripts/v1_1_train_prep.sh` (V1.1 + 60-subset ablation)
- Episode filter: `data/v1_1_episode_filter.json` (182 kept, drops 47 far-mode + 3 stowaways)
- **TODO to build (no robot, do anytime):** the input-ablation re-processor + 2–3 train configs

## Other queued (no robot, anytime)

- Judge-vs-human agreement: `data/judge_sheet/` — you + mentor score 50 images blind (~30 min each)
- Citations pass (`docs/paper_refs.bib`, VERIFY tags) + LaTeX port once venue chosen
- 60s demo video (any block, no grid needed) for slides/portfolio

## Also on record

- Conference ladder (not started): new object → placement → SmolVLA/other VLA → live memory → MuJoCo sim. Workshop first.
- Gate-7 (judge determinism) + gate-13 (ensembling) corrections from 2026-07-14 are in `docs/TODAY_TESTS_RESULTS.md`.
