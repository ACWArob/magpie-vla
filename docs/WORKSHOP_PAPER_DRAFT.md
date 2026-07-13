# Measure, Design, Verify: A Self-Generating Data Pipeline for Real-Robot Grasp Policies
*Workshop paper draft v0.1 — 2026-07-13. All numbers measured; sources in brackets. LaTeX
table at `docs/paper_results_table.tex`; figures referenced from `docs/figures/`.*

## Abstract

We present a pipeline in which a real robot generates, judges, and curates its own
imitation-learning data — zero teleoperation, zero human labels — and show that *designed*
data beats *collected* data at fixed budget. A scripted expert with a physically-grounded
reward gate (episode kept only if the object is measurably held AND a VLM judge scores the
grasp ≥0.6) collects 66.7 episodes/hour unattended. An initial 60-episode dataset with
random placement trained a policy whose replay error was 5.9 mm — yet it froze on rotated
objects and missed beyond its placement scatter. We diagnose three data defects
(supervision collapse: 71% of grasps at one angle; inexpressible action events: binary
gripper hiding finger pre-positioning; 40% off-task frames), redesign collection around
them (5×5 grid × 7 angles with jitter; continuous aperture actions; episodes end at lift),
and retrain the same architecture on 229 episodes. On a 100-grasp graded hardware
evaluation spanning every grid cell × 4 angles, the new policy picks **97/100**
(95% CI 91.5–99.0), including **75/75 rotated blocks** (the predecessor's failure case),
with no spatial falloff at the workspace edge. Critically, the dataset audit *predicted*
the remaining failure mode (a 0°/90° label aliasing on square objects) before deployment,
and all three eval failures occurred exactly there — while offline replay and chunk-level
probes of the trained policy showed no trace of it. We conclude that for scripted-data
pipelines, dataset distribution audits and closed-loop graded evaluation are load-bearing;
offline prediction metrics are necessary but cannot surface mode-averaging defects.

## 1. Introduction

- Imitation learning for manipulation is data-limited; teleoperation couples data volume
  to human time 1:1.
- Claim: a scripted expert + success filtering + *designed coverage* is a legitimate and
  measurable alternative at lab scale (the real-world analogue of MimicGen-style
  generation).
- Contributions:
  1. A closed-loop pipeline (collect → judge → audit → train → deploy → grade) where every
     stage's decision is measured (Decision Ledger, §5/appendix).
  2. A measured V0→V1 case study: three diagnosed data defects, three designed fixes,
     froze→100% on the rotated-block case at equal architecture/recipe.
  3. A negative result with methodological weight: the surviving data defect (0/90 label
     aliasing) was invisible to offline replay AND to chunk-level probes of the trained
     policy, but was flagged by a distribution audit and confirmed exactly at deployment.
  4. The instrument itself: an unattended 100-grasp graded evaluation protocol (lift
     verification, hardware-fault quarantine, resume-safe ledger) reusable across policy
     versions.

## 2. Related work (compressed; expand from V1_data_research.md)

- **Scripted/generated demonstrations**: MimicGen, SkillMimicGen, DemoGen — success-filtered
  machine-generated data trains BC policies; robomimic's caveat that *mixed-quality*
  machine data hurts BC → our reward gate supplies the filtering.
- **Data quality axes**: Belkhale et al. — action divergence vs transition diversity; our
  V0 angle collapse is action divergence measured in the wild; grid coverage maximizes
  transition diversity at fixed budget.
- **Coverage/scaling**: Lin et al. data-scaling laws (diversity > count); spatial-support
  interpolation studies; MOVE's +76% from spatial variability — our 5×5×7 grid + jitter is
  the deliberate version.
- **Action representation**: AWE waypoints (our declared-target actions are waypoint-style
  natively); ACT/ALOHA multimodality — mode averaging as our observed freeze.
- **Evaluation**: our graded protocol relates to per-condition success reporting;
  grading (not just success) is what exposed the aliasing (0°: 88% success but 0.58 grade).

## 3. Pipeline (Figure 1: `figures/appendix/fig1_pipeline.png`)

3.1 **Scripted expert**: SAM3 text-query segmentation → closed-loop centering →
minAreaRect flat-face grasp angle (deterministic) → re-localisation at the grasp angle
(cancels co-rotating camera/finger offsets) → DeliGrasp force closure → slip-guarded lift.
3.2 **Reward gate**: physically held (aperture band through lift) ∧ VLM judge ≥0.6
(temperature 0). 99.1% save rate over 234 attempts (CI 96.9–99.8); mean reward 0.865.
3.3 **Designed coverage**: 5×5 positions (3 cm) × 7 angles (15°) + jitter (±1 cm/±5°) +
~15% scripted recovery episodes; ledgered and resume-safe.
3.4 **Action space**: aperture-mm as a continuous channel; the 104→width+8→20 staircase
appears in 37.7% of frames — inexpressible in the binary predecessor (measured: the V0
policy's grip output spans [0, 1.01] regardless of input).
3.5 **Audit gate** (ships-or-not): episode count, angle histogram (collapse), coverage
holes, length outliers. The V1 audit passed but warned "0° bin LOW" — see §6.
3.6 **Training**: ACT (51.6M) from scratch (ImageNet ResNet18 backbone only), 150k steps,
~1.5 h on one NSF ACCESS GH200; normalization statistics frozen into the checkpoint.
3.7 **Deployment**: 10 Hz; execute 10 of 100 predicted actions then re-infer (temporal
ensembling rejected: measured 3× rotation-error inflation from rotation-vector averaging);
rotvec branch canonicalization; tilt-capped but yaw-free safety envelope; policy hands off
to a scripted place after a settled grasp verdict.

## 4. Evaluation protocol (the instrument)

100 combos = 25 cells × {0°, 25°, 45°, 70°} (90°≡0° for a square object). Per combo: the
scripted expert stages the block (or the policy's own previous clean pick is placed
directly at the next pose — measured unbiased: 98.2% vs 95.3% subsequent success);
policy rollout with the arm frozen at close (policies trained on lift-terminated episodes
begin lifting; ungated motion invalidated grading); **lift verification** (8 cm; aperture
must remain seated); grade ∈ [0,1] with penalties for re-grips, yaw error vs placed angle,
and unseated aperture. Hardware-fault quarantine: a stall fingerprint (aperture pinned
wide through retries) triggers driver reboot + unrecorded retry — a dying gripper once
wrote 36 spurious "policy failures" in one hour; none reached the final ledger.

## 5. Results

**Headline (Table 1 / `paper_results_table.tex`):** 97/100 picks (CI 91.5–99.0), mean
grade 0.70. Rotated blocks 75/75 (CI 95.1–100). Straight blocks 22/25 (CI 70.0–95.8) —
all three failures of the run. No spatial falloff: 97% at the 6 cm ring (CI 89.3–99.1).
Same-protocol replay: V0 5.9 mm / 0.93°, V1 1.8 mm / 1.13° — both "fine," neither
predictive of deployment (see §6).

**Per-gate measurements** (scoreboard figure + Decision Ledger): angle-collapse 71%→19.7%;
minAreaRect vs PCA frame-to-frame jitter 0.31° vs 1.65° (p90 0.47° vs 12.39°); SAM3
deterministic (repeat-query mask IoU 1.0000) but 1050 ms (10× the control tick — hence
detector-free deployment); throughput 66.7 eps/hr confirmed from an independent ledger.

**Quality ceiling is mechanical**: 94/97 picks required ≥1 close re-grip (servo torque
recovery race), costing ~0.1 grade each — the gap between 97% success and 0.70 mean grade.

## 6. The aliasing case study (the paper's argument)

1. **Design-time**: square blocks at 0° and 90° are the same scene; both labels entered
   the dataset (the mod-90 consistency gate saw no deviation — and the documented
   `issue=angle` hard-reject was, we later found, never enforced in code: 86/88 flagged
   episodes passed on reward alone).
2. **Audit-time**: the angle histogram flagged "0° LOW" — noted, misread as benign.
3. **Train/replay-time**: nothing. Replay medians 1.8 mm/1.13°; **chunk-tail probes from
   ambiguous start frames predict the correct final yaw even for 0°/90° episodes (median
   0.8° vs 0.6° for mid angles)** — teacher-forced context masks mode averaging entirely.
4. **Deploy-time**: the policy rotates ~43° above straight blocks; all 3 eval failures at
   0°, two with ~41° yaw error; 0° grade 0.58 vs 0.81 at 25°.
5. **Lesson**: distribution audits and closed-loop graded evaluation are the only stages
   in this pipeline capable of surfacing label-aliasing defects. Offline accuracy metrics
   — even chunk-level, even at the ambiguous decision point — cannot.

Patch (V1.1, planned): canonicalize angle labels for symmetric objects + ~35 replacement
episodes + retrain; re-run the identical protocol → before/after heat-maps.

## 7. Limitations

Single object class (square block) and single embodiment so far; V2 extends to
multi-object. Judge validated against physical outcome but not yet against human raters
(experiment queued). Placement ground truth during collection is the expert's own
(mitigated in eval by independent scripted staging). The 3-episode post-training dataset
drift (setup pickups recording) is documented; provenance now pinned. Grade weights are
task-specific.

## 8. Reproducibility

Repo: github.com/ACWArob/magpie-vla (branch `ros`). Pins: lerobot==0.4.4, numpy==1.26.4,
transformers<5. Cluster recipe: docs/NSF_TRAINING.md (validated twice). Full decision
record: docs/DECISION_LEDGER.md; failure ledger: docs/FULL_PIPELINE.md §10; raw eval
ledger: data/v1_eval_100.json schema in docs/V1_EVAL_REPORT.md.

## Appendix map

- A. Decision Ledger + gate scoreboard (14 gates, MEASURED/CONSTRAINT/TO-MEASURE)
- B. Offline test report (replay, cross-replay, detector/angle/stability, ledger mining)
- C. Eval instrument details + failure-ladder design
- D. Live bake-off protocol (angle accuracy vs ground truth; Gemini/GGX legs)
- E. NSF ACCESS training runbook

## Figure plan

| Fig | File | Caption sketch |
|---|---|---|
| 1 | appendix/fig1_pipeline.png | pipeline + the one-grid-three-uses loop |
| 2 | v0/fig1_angle_collapse.png + v0_v1/compare_angles_lengths.png | V0 defects → V1 design |
| 3 | v1_eval_heatmap.png | per-angle grade heat-maps, the 0° column visible |
| 4 | appendix/offline_aperture_staircase.png + offline_actionspace.png | action-space argument |
| 5 | appendix/gate_scoreboard.png | every decision quantified |
| 6 | appendix/offline_replay_v1.png | replay ≠ deployment (the §6 argument) |
