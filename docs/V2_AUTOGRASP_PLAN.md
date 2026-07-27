# AutoGrasp V2 — collection architecture for varied, delicate objects

*Researched + decided 2026-07-22. Goal: by Mon 2026-07-27, start collecting on
new objects (banana, strawberry, cup, …) — picked DELICATELY and CONSISTENTLY,
with placement recorded for later. Policy stays OURS, trained from scratch on
our own data (ACT baseline; no VLA fine-tuning). V1 is frozen at tag
`v1-pipeline` and untouched.*

## STATUS (2026-07-27) — what's built vs pending

**Built + offline-verified (no robot), branch `v2-dev`, pushed to `fork` + `personal`:**

| Piece | File | Tests |
|---|---|---|
| shape-gated grasp ladder | `src/magpie_control/v2/grasp_ladder.py` | 6 |
| planner (ladder + priors) | `src/magpie_control/v2/grasp_planner.py` | 6 |
| state builder (real wrist_fz) | `src/magpie_control/v2/state_builder.py` | (in 9) |
| deformation + weight gates | `deformation_check.py` · `weight_estimate.py` | 9 + 6 |
| episode meta (phase/task) | `src/magpie_control/v2/episode_meta.py` | 3 |
| whole-pipeline self-test | `scripts/v2_selftest.py` | 60 checks |
| object-agnostic notebook | `notebooks/v2_collect.ipynb` | cells 1–4 run headless |

Verified against **live SAM3** on real images: concept-prompt → mask → sane plan.
Bugs caught offline before they hit hardware: 8N seed would crush produce (→1.5N);
strawberry grasped by the leaves (→thickness-weighted center); deformation target was
the seed not the weight-reconciled force (caught on the apple).

**Pending (needs the robot):** port V1 motion helpers into notebook cells 6–7 (reuse the
proven grasp cell), SAM3 part-prompt test ("handle"), live ladder-vs-GraspGenX bake-off on
real produce. **Object order:** cube (control) → 3D-printed apple → banana → strawberry → cup.

## The architecture in one diagram

```
                       ┌── per-object PRIOR (GraspMemory: width/force/angle, 51 objs known)
                       ▼
 prompt ("strawberry") → SAM3 concept segmentation → mask + depth
                                   │
                     shape-gated GEOMETRY LADDER  ←— the V2 change
                     rectangularity = area/minAreaRect_area
                       ├─ box-like   → minAreaRect      (measured best: 4.5° on cubes)
                       ├─ elongated  → skeleton-perp    (banana)
                       └─ otherwise  → polygon-antipodal (angle + WIDTH out)
                                   │            (GraspGenX candidates as cross-check)
                                   ▼
                     DeliGrasp adaptive force (mentor's method — BUILT for this:
                     12 delicate/deformable items incl. produce, mass/friction/
                     compliance inferred → minimally-deforming force ramp)
                                   │  + slip_guard reclamp during lift/transport
                                   ▼
                     lift-verify → VLM judge (+ deformation check for produce)
                                   ▼
                     gentle PLACE-DOWN (guarded move on wrist-Fz contact) — RECORDED
                                   ▼
                     LeRobot episode (10 Hz, 9-dim state W/ REAL wrist_fz) + reward gate
```

## What changes vs V1 — component by component, with the evidence

| Component | V1 | V2 | Why (evidence) |
|---|---|---|---|
| Object spec | hardcoded red-block HSV+SAM3 | **SAM3 concept prompt per object** | SAM3 is open-vocab by design; zero code per new object |
| Grasp angle | minAreaRect only | **shape-gated ladder** (gate→minAreaRect/skeleton/polygon) | prototype VERIFIED on our stack 07-22: gate reads 0.96 cube / 0.40 crescent and routes correctly; rungs disagree ~70° on non-boxes — the silent error V2 would have eaten |
| Grasp width | minAreaRect side | **polygon pair width, per-object prior fallback** | polygon rung outputs width directly; GraspMemory priors give the DeliGrasp prepos |
| Force | DeliGrasp (overkill on rigid cube) | **DeliGrasp in its home turf** | the paper's benchmark IS 12 delicate/deformable items (food, produce, toys) spanning 2 orders of magnitude in mass; compliance feedback even measures ripeness → extra judge signal |
| wrist_fz | **recorded as all-zero (verified bug)** | **wire /ft_sensor/wrench into the recorder** | FT node exists and publishes; the channel was zeroed in every V1 episode. Free, real signal for gentle place + slip + future policies |
| Hold discipline | re-grip cadence (07-21 fix) | same + slip_guard active on compliant objects | AX-12 facts unchanged; slip matters more with soft objects |
| Episode end | lift + verify | **+ transport + gentle place-down (guarded Fz contact)** | placement data collected from day 1 → the same dataset later trains place, no re-collection; guarded-move placing is standard force-control practice |
| Judge | held + VLM grade ≥0.6 | same **+ deformation/squish check** (compare pre/post width; DeliGrasp compliance readout) | "delicately" must be graded, not assumed |
| Coverage | 5×5 grid ±6 cm | **wider + randomized placements per object** | our measured OOD finding: the policy's action support ends exactly where collection ended (−6.6 cm wall). The cliff is a DATA boundary — collection width is the fix |
| Per-object state | none | **GraspMemory priors updated per episode** (already built, 51 objects) | consistency across episodes = priors, not luck; already in the stack, underused |
| 3D model | Stage 0 | keep accumulating per object (background) | feeds GraspGenX real geometry now; CoACD part decomposition + MuJoCo later (ShapeGrasp path for "grasp the handle") |
| Policy | ACT from scratch | **unchanged** — ACT on our data | user decision; ablation says minimal-state input set is strongest (96%/0.81 noXTheta). Optional later bake-off: lerobot-native DiffusionPolicy/VQ-BeT, also from scratch on the same data — native multimodality attacks the 0°-style aliasing and gives real best-of-N diversity; NOT a V2 blocker |

## Explicitly rejected (and why)

- **VLA fine-tuning (SmolVLA/π0.5)** — user call: the point is our own policy on
  our own data. (Noted: Correll-lab has a SmolVLA guide if this ever revisits.)
- **LERF-TOGO-style NeRF part grasping** — needs per-scene multi-view capture;
  we are a single wrist cam at 10 Hz.
- **Heavy 6-DoF grasp nets (Contact-GraspNet, AnyGrasp)** — ≥8 GB-class VRAM or
  restrictive licensing (AnyGrasp SDK); GraspGenX already covers the
  point-cloud lane within budget. Lightweight options (GG-CNN port ~1 day,
  EconomicGrasp) stay on the bake-off bench, not in the critical path.
- **Raw triangulation as the grasp method** — representation, not decision;
  the useful resolution is convex pieces (CoACD), which waits for the
  accumulated 3D model (part-grasping phase, not Monday).

## Part-directed grasping (requirement #2, designed-for now, built later)

Everything above takes a MASK as input — so "pick up the cup by the handle"
is the same pipeline with a part prompt. Two routes, both preserved:
1. **SAM3 part prompt** ("handle") → part mask → same ladder. 10-minute
   empirical test on our wrist cam decides how much comes free.
2. **Geometric parts**: accumulated 3D model → CoACD convex pieces → Gemini
   picks the task piece (ShapeGrasp pattern) → grasp that piece. Robust when
   language segmentation fails; same mesh exports to MuJoCo.

## Every stage of the pipeline: options considered, chosen or rejected, why

*(V2 counterpart of DECISION_LEDGER.md — the FULL walk, drivers to eval.
"Looks similar to V1" is deliberate where it happens: the loop shape is
measured to work (67 eps/hr → 97% policy). Every "keep" below is defended,
not defaulted. ✅ chosen · ❌ rejected · ⏸ deferred with trigger.)*

### Layer A — hardware & drivers

**A1. Arm + motion.** UR5e via ur5_node/RTDE — ✅ keep (no alternative in the
room). moveL-singularity escape via joint moves — ✅ keep (measured recovery).
Speedup 1.35× — ✅ keep for transit, ❌ for descend near produce (bruise risk):
descend at V1 speed.
**A2. Gripper fingers.** **Bare AX-12 fingers (V1) — ✅ KEEP, no hardware
change (user decision 2026-07-27: must work out of the box with these
grippers).** Evidence it suffices for delicate produce: the gripper resolves
force to a **measured min of 0.15N** (deligrasp_node), DeliGrasp's adaptive
grasp SEEDS at 1.5N and ramps up only until slip stops, and V1 already
**learned stable holds down to 0.50N** on delicate items (eraser, foam brick,
markers, racquetball). DeliGrasp itself was benchmarked on 12 delicate/
deformable produce items on this exact gripper class. So delicacy is a
SOFTWARE property here: low force seed + DeliGrasp adaptive ramp + delicate cap
+ deformation gate + slip_guard. Fingertip pads — ❌ not needed (dropped).
Soft gripper swap — ❌ different robot. Tactile (eFlesh/TouchIQ) — ⏸ August.
**A3. Camera.** Single wrist D405 — ✅ keep: schema stability with V1 data,
and every V2 method consumes its mask/depth. Add overhead cam — ❌ for now:
new calibration + schema break mid-summer; revisit only if occlusion-during-
place proves blocking. Known D405 issue: IR-stripe artifacts on glossy
surfaces (already bit us once — the mask_grasp_angle fix exists because of
it) → V2 rule: **2D mask is authoritative; depth is advisory** on shiny
produce.
**A4. FT sensor.** Exists, publishes, was never recorded (wrist_fz all-zero
verified) — ✅ wire into recorder + use for guarded place. No alternative
considered seriously; it's free.
**A5. Infra discipline.** udev/ModemManager rule, verified-open resets,
re-grip cadence, gpu_reap — ✅ keep verbatim (all are responses to measured
failures from 07-21/22).

### Layer B — perception

**B1. Detection (what & where).** Gemini bbox detect (V1) — ✅ keep as the
what/where oracle + arbiter; it is not the per-frame bottleneck. Grounding-
DINO/OWL-ViT — ❌ another VRAM tenant duplicating what Gemini+SAM3 cover.
LocateAnything — ❌ GPU (backlogged since June).
**B2. Segmentation.** **SAM3 concept prompt per object — ✅ chosen** (open-
vocab, zero code per object, part-prompt upside). HSV — ❌ dies on multi-
color produce. SAM3 part-level prompting — ⏸ Day-2 10-min test decides how
much of "grasp the handle" is free.
**B3. Point cloud / 3D.** build_segmented_pcd + centroid — ✅ keep for
position; per-object 3D model accumulation — ✅ keep in background (feeds
GGX now, CoACD parts + MuJoCo later). Shape completion nets — ❌ VRAM for
marginal gain at tabletop ranges.

### Layer C — grasp synthesis

**C1. Angle + width.** (unchanged from earlier draft) minAreaRect-only — ❌;
PCA/depth-PCA/fixed-90 — ❌ measured losers; raw triangulation — ❌
representation-not-decision; GG-CNN/GR-ConvNet — ⏸ bench (port cost);
EconomicGrasp/AnyGrasp/Contact-GraspNet — ❌ VRAM/licensing; superquadrics/
CoACD — ⏸ needs accumulated model. **Shape-gated ladder + GraspGenX
cross-check — ✅**, Day-2 live bake-off is the final referee.
**C2. Centering target — a real V2 change.** V1 centered on the MASK
CENTROID — correct for a cube, WRONG for a strawberry (centroid includes the
leaves; the grasp point is the body). **✅ center on the chosen grasp-pair
midpoint from the ladder, not the centroid.** Keep the V1 re-localise-at-
grasp-angle trick (it fixed the offset drift; same logic applies).
**C3. Approach direction.** Top-down + tilt-cap (V1 deploy constraint) — ✅
keep for V2 start: every method and the whole eval instrument assume it.
Full 6-DoF approach (GGX offers it) — ⏸ trigger: an object the bake-off
shows is >20% worse top-down (e.g., stemmed cherry); costs IK-near-singularity
risk we've already been burned by.
**C4. Grip force.** Fixed 16 N — ❌ crushes produce. Learned — ❌ bootstrap.
Tactile — ⏸ August. **DeliGrasp — ✅** (mentor's method; its benchmark IS
delicate produce).
**C5. Retry policy.** V1's 3 grip-retries — ❌ for produce: each retry
bruises; **✅ max 1 retry on delicate objects, then re-detect** (the judge's
deformation term audits whether even 1 was too many).

### Layer D — verify, judge, record

**D1. Physical success check.** Held-through-lift + aperture-in-band — ✅
keep, but the band becomes **per-object from GraspMemory priors** (20–40 mm
was cube geometry, meaningless for a banana).
**D2. Grading agent — NOT replaced, upgraded.** Physical-only — ❌ misses
"held but crushed". **Gemini VLM judge — ✅ kept** (built a 97% policy; ±0.25
noise documented and tolerable for a gate) **+ deformation/squish term — ✅
added** (pre/post width + DeliGrasp compliance readout — physical and
deterministic). Value heads as gate (RL white paper) — ❌ circular (policy
grades its own training data) + uncalibrated until it passes its own offline
AUROC gate; deploy-time scorer only, later at most a second opinion. Human
labels — ❌ defeats the thesis (agreement sheet stays as *validation*).
**D3. Recorder.** 10 Hz LeRobot, start at first manipulation move — ✅ keep.
**Additions ✅: real wrist_fz, per-frame expert phase label, per-episode task
string** (all near-free; phase = our own force-phase-clock finding). Extra
cameras/tactile in schema — ❌/⏸ hardware.
**D4. Reward gate.** held AND grade ≥0.6 — ✅ keep threshold to stay
comparable with V1 data quality; deformation term gates *in addition*.
**D5. Grasp memory / priors.** GraspMemory per-object width/force priors —
✅ promote from passive log to ACTIVE input (prepos width, force seed,
aperture band). Kalman-style update already implemented.

### Layer E — reset & episode economics

**E1. Reposition.** Scripted place at next target + direct-place optimization
— ✅ keep; the place leg becomes the **guarded-Fz gentle set-down** and is
RECORDED (pick AND place data from day 1). Drop-back (V1 eval style) — ❌
wastes the return trip and bruises fruit.
**E2. Object degradation — NEW stage V1 never had.** Real produce degrades
over 100 grasps. Options: single real fruit (❌ episode 60 is grasping a
different object than episode 1 — label drift), **rotate 3–4 instances +
artificial produce for bulk with real-fruit validation subsets — ✅ chosen**,
all-artificial (❌ compliance/DeliGrasp signals go fake). Judge's deformation
term doubles as the degradation detector → auto-swap prompt to the operator.
**E3. Session orchestration.** Fixed script (V1) — ❌ single-object only;
**Gemini proposes next object + placement (AutoRT-lite, ~20 lines) — ✅** for
multi-object sessions + diversity knobs (mat rotation, lighting, distractors
— DROID lesson).

### Layer F — train, deploy, eval

**F1. Training recipe.** DeltaAI slurm ACT 100k — ✅ keep verbatim (proven
3× this month). State-input set for act_v2 — record ALL 9+ channels, choose
input at training time; ablation says minimal-state is strongest on rigid,
but force may be genuinely informative for compliant objects → that's a
planned A/B (act_v2 vs act_v2_noForce), not an assumption either way.
**F2. Deploy loop.** 10 Hz, n_action_steps=10, no ensembling, rotvec
canonicalize+rate-limit, tilt-only ori-cap — ✅ keep all (each was a measured
deploy fix). Best-of-N/abort — ⏸ RL white-paper phases, separate go/no-go.
**F3. Eval instrument for V2 — must change, and this is easy to miss.**
Yaw-error grading is MEANINGLESS for radially-symmetric fruit; the 20–40 mm
aperture band is cube-specific. **✅ per-object grade schema: aperture band
from priors, yaw term only for objects with a defined long axis, deformation
term added, fresh-instance rule for eval runs.** Grid protocol + lift-verify
+ resume-safety + motion recording — ✅ keep identical (comparability with
the entire V1/ablation corpus).

## What the top labs do — adopted where it fits our one arm + 8 GB

| Source (lab / venue) | Their lesson | Our V2 adoption (cost) |
|---|---|---|
| QT-Opt — Google, 580k grasps, 7 robots | success detection must be PHYSICAL and self-supervised (did the object leave the scene) | validates our lift-verify + aperture gate; no change. Also calibrates ambition: their 96% took 580k grasps — our lane is small-data + high-quality expert, argue quality-per-sample |
| AutoRT — DeepMind | a VLM orchestrates WHAT to collect next | **Gemini proposes next object + placement** during multi-object V2 sessions (Gemini already in the loop; ~20 lines) |
| BridgeData V2 — Berkeley, CoRL | language annotation on every episode enables everything later | **store a task string per episode** ("pick up the strawberry") — trivial now, unlocks language/part-conditioning later |
| DROID — Stanford++ | scene diversity, not volume, drives generalization | procedural diversity knobs during collection: mat rotation, lighting, distractor objects (free) |
| MimicGen / SkillGen — NVIDIA, CoRL | few seeds → many demos via segment-wise adaptation; SEGMENTED skills beat monolithic trajectories | **record the expert's phase label per frame** (approach/center/descend/squeeze/lift/place — the expert already knows). Costs nothing, and our own ablation showed phase information is load-bearing (the force-as-phase-clock finding). Also the blueprint for the sim-twin multiplication paper (P2) |

## Monday plan (week of 07-27)

**Day 1 — bench, no robot:** ship the ladder as `src/magpie_control/v2/grasp_ladder.py`
(prototype exists); unit tests on saved masks; wire wrist_fz into the recorder;
add the deformation check to the judge.
**Day 2 — robot, bake-off:** SAM3 prompts on banana/strawberry/cup (incl. the
part-prompt test); live angle bake-off on real objects: ladder vs minAreaRect
vs GraspGenX (same instrument as the V1 bake-off; new DECISION_LEDGER gate).
**Day 3 — robot:** DeliGrasp force validation per object (squish audit),
guarded-Fz place-down, GraspMemory priors warm-up (~5 grasps/object).
**Day 4–5 — collection:** wider-coverage randomized placements, 100 episodes
on object #1 (per roadmap), gate + deformation judge live. Nightly: push +
train ACT-v2 on DeltaAI with the V1 recipe.

Gate to call V2 collection "working": ≥60 gated episodes/hr on a non-box
object with zero squish-fails in the last 20 (V1's rate was 67/hr on the cube).

## Key references

DeliGrasp (Xie, Lavering, Correll): arxiv.org/abs/2403.07832 — the 12-item
delicate benchmark. SAM3 concept segmentation: arxiv.org/html/2511.16719.
ShapeGrasp (decomposition+LLM parts): arxiv.org/pdf/2403.18062. Skeleton-based
grasping: ieeexplore.ieee.org/document/6651547. GG-CNN (bench option):
github.com/dougsm/ggcnn. EconomicGrasp (bench option):
github.com/iSEE-Laboratory/EconomicGrasp. Diffusion Policy / VQ-BeT: native in
our pinned lerobot 0.4.4 (from-scratch, later bake-off only). QT-Opt:
arxiv.org/abs/1806.10293. AutoRT: arxiv.org/pdf/2401.12963. BridgeData V2:
proceedings.mlr.press/v229/walke23a. MimicGen: arxiv.org/pdf/2310.17596.
SkillMimicGen: arxiv.org/abs/2410.18907. Phase-frame labels + task strings are
the two cheapest adoptions and both feed later training directly.
