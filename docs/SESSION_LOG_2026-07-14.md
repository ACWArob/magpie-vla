# Session Log — 2026-07-14 (AutoGrasp test day + paper package)

*Curated summary of the working session. Raw transcript archived locally at
`~/magpie_chatlogs/session_2026-07-14_autograsp-tests.jsonl` (not in repo — 332 MB, public
repo). This file is the human-readable index of what changed and why.*

## Headline outcomes

- **act_v1 measured at 97/100** (95% CI 91.5–99.0), rotated blocks 75/75, in a graded eval.
- **OOD ring test**: 97% in-zone → **13% at ±9 cm** — a hard extrapolation cliff (the policy
  interpolates within its data, does not extrapolate). New `figures/appendix/ood_ring.png`.
- **Live method bake-off** (36 placements, ground truth): minAreaRect **4.5°** ≪ mask-PCA
  11.5° ≈ GraspGenX 11.5° < depth-PCA 12.7° < fixed-90° **22.5°**. Detectors all 36/36;
  SAM3 1.1 s, HSV 1 ms, Gemini 6.5 s.
- **Two honest self-corrections** (both reinforce the "offline metrics miss deployment
  failures" thesis): judge determinism is NOT bit-exact at temp=0 (max 0.25 spread);
  ensembling is 0.8× on replay, NOT the "3× worse" deployment claim → scoreboard gate-13
  de-fabricated.

## What got built / fixed

- **today_tests.ipynb** — test-day orchestrator (OOD ring, live bake-off, judge determinism,
  ensembling A/B). Audited: fixed OOM, ensembling-faithfulness, comment counts; opt-in
  eval overrides that never clobber frozen v1 artifacts.
- **Bake-off bugs fixed** (both silently killed ggx + pca3d): K passed as 3×3 not flat;
  `build_segmented_pcd` returns a tuple that wasn't unpacked.
- **V1.1 fixes committed** (for next week): 0/90 boundary canonicalization; the angle-reject
  gate ENFORCED (was documented-but-never-wired — 86/88 flagged episodes had passed);
  AUTO_RECORD=False in staging notebooks; combo-id traceability.
- **Paper package** written earlier in the week: WORKSHOP_PAPER_DRAFT, DECISION_LEDGER
  (14 gates + scoreboard), OFFLINE_TEST_REPORT, V1_EVAL_REPORT, paper_refs.bib,
  paper_results_table.{csv,tex}, fig1_pipeline.png, Wilson CIs.
- **Presentation assets**: SUMMER_TALK_SLIDES (10-min, +OOD slide +metric slide +two-
  scorekeepers slide), PIPELINE_SLIDES, NSF_TUTORIAL (mentor-format step-by-step),
  PORTFOLIO_PROMPT, SLIDE_PROMPTS. Gmail drafts with slide-change instructions.
- **Recovered** DECISION_LEDGER.md after it was blanked to 0 lines (restored from git).

## Key decisions on record

- **Grid can be removed** — re-taping needs no recalibration (robot-frame, TABLE_Z
  auto-measured). Only V1.1 collection needs the grid → deferred to next week, no penalty.
- **Next-week sequence** (docs/NEXT_WEEK_PLAN.md): input-ablation experiment (mentor's idea —
  drop absolute x/θ from state input to fix the OOD cliff; NO robot, reuses 229 eps) →
  V1.1 collection (grid back, fixes 0° aliasing) → combine + compare. The two fixes are
  orthogonal.
- **Workshop first, conference later** — conference ladder (new object → placement →
  other VLA → memory → sim) on record, not started.

## Commits this session (branch ros, pushed to both remotes)

today_tests orchestrator + audit fixes · bake-off K/tuple fixes · OOD + bake-off results
doc + figures · T2b/T2c results + scoreboard correction · V1.1 fixes + plan · NEXT_WEEK_PLAN
· propagation into ledger/comparison/summer-talk. HEAD ≈ 1ee04b8.

## Still open (no robot — anytime)

Judge-vs-human agreement (`data/judge_sheet/`, you + mentor 30 min) · citations + LaTeX port
(pick venue) · 60 s demo video · **build the input-ablation re-processor + train configs**.
