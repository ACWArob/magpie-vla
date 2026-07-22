# AutoGrasp V2 — collection architecture for varied, delicate objects

*Researched + decided 2026-07-22. Goal: by Mon 2026-07-27, start collecting on
new objects (banana, strawberry, cup, …) — picked DELICATELY and CONSISTENTLY,
with placement recorded for later. Policy stays OURS, trained from scratch on
our own data (ACT baseline; no VLA fine-tuning). V1 is frozen at tag
`v1-pipeline` and untouched.*

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
