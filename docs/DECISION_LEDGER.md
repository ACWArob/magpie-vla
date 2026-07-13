# Decision Ledger — every fork on the way to AutoGrasp, with the rejection reason

*Workshop-appendix source. Every decision gate in building the pipeline: the options on the
table, what we chose, and WHY each alternative lost — as a measurement, a hard constraint,
or a logical argument. Each reason is tagged:*

- **[MEASURED]** — we have the number (source cited)
- **[CONSTRAINT]** — hard fact (VRAM, license, API) — no experiment needed
- **[TO-MEASURE]** — judgment call at the time; experiment queued to quantify it
  (→ `notebooks/appendix_bakeoff.ipynb` unless noted)

![gate scoreboard](figures/appendix/gate_scoreboard.png)
*One panel per gate: chosen (green) vs rejected (red); amber "est." bars are literature/
vendor numbers — every other number measured on our hardware. Sources: OFFLINE_TEST_REPORT.md,
V1_EVAL_REPORT.md, V0_ANALYSIS.md.*

---

## Gate 1 — How do we get demonstrations at all?
**Options:** human teleoperation · kinesthetic teaching · **scripted expert (AutoGrasp)** ✔

| Rejected | Why |
|---|---|
| Teleoperation | [MEASURED] our scripted expert collects **66.7 episodes/hr with ZERO human-hours** (V1, 229 eps in 3.4h). Teleop needs a human present for every second of collection; even matching the rate costs 1:1 human time, and ALOHA-class setups need custom rigs. Also [MEASURED]: expert consistency — robomimic shows plain BC degrades on inconsistent demonstrators; scripted = perfectly consistent by construction. |
| Kinesthetic teaching | [CONSTRAINT] UR5e freedrive + recording at 10 Hz while a human holds the arm contaminates the wrist camera view (hands in frame) and the FT signal. Never viable for vision policies. |

**Litmus:** the resulting data trained a policy that scored 97/100 — scripted data was not second-class (MimicGen regime, not noisy-RL regime).

## Gate 2 — Object detector
**Options:** Gemini 2.5 Flash boxes · OWL-ViT-class open-vocab · LocateAnything (NVIDIA) · **SAM3** ✔

| Rejected | Why |
|---|---|
| Gemini box | [CONSTRAINT] a box has no pixels → no segmented point cloud → no geometry downstream. Kept for object *identification* only. Latency leg: live bake-off. [MEASURED — OFFLINE_TEST_REPORT §4] note: on the red block a 2 ms HSV threshold matches SAM3 100%/1px — SAM3's justification is text-query GENERALITY (V2 multi-object), and its 1050 ms measured latency (10× the tick budget) is why deploy runs detector-free end-to-end. |
| OWL-ViT-class | [CONSTRAINT] same box-only limitation; superseded early. |
| LocateAnything | [CONSTRAINT] does not fit the RTX 2070 (8 GB) beside the rest of the stack without int4 quantization — backlogged, not rejected on merit. |

**Why SAM3 won:** pixel-accurate mask from a text query = the input every downstream stage
(point cloud, PCA, minAreaRect, GGX) actually consumes. [MEASURED] runs in ~4.5 GB, coexists
with GraspGenX (1.8 GB) inside the 8 GB budget — measured, not estimated.

## Gate 3 — Object pose from the detection
**Options:** depth point-cloud PCA · **2D mask PCA / minAreaRect** ✔ · bare centroid

| Rejected | Why |
|---|---|
| Depth (3D) PCA | [MEASURED] RealSense IR stripe artifacts corrupt the depth on flat surfaces → 3D principal axes swing wildly frame to frame; the 2D mask fix was confirmed on a cube at 45° (2026-06). |
| Bare centroid (no orientation) | [MEASURED] this IS the V0 behaviour: no angle authority → 42/62 episodes grasped at ~90° → the policy froze on rotated blocks. Eval confirms: a fixed-90° strategy's angle error grows linearly with block rotation (→ bake-off fig C). |

## Gate 4 — Grasp planner
**Options:** PCA-only · GraspGen · GSNet · AnyGrasp · **GraspGenX** ✔ (+ PCA kept as prior)

| Rejected | Why |
|---|---|
| AnyGrasp | [CONSTRAINT] licensing/weight-access friction — could not be integrated cleanly. |
| GSNet | [TO-MEASURE if ever needed] evaluated qualitatively in the June bake-off, not adopted; no cross-embodiment story for the MAGPIE gripper. |
| GraspGen (original) | [CONSTRAINT] superseded by GraspGenX, which ships an explicit cross-embodiment path — we contributed the MAGPIE gripper embodiment. |
| PCA-only | [MEASURED, partial] works on boxes; no 6-DOF proposals for irregular objects (the V2 fruit target). Kept as the angle prior + fallback. |

**Why GraspGenX won:** [CONSTRAINT] only candidate with a maintained cross-embodiment
interface for a custom gripper. [MEASURED] 1.8 GB VRAM measured alongside SAM3 — fits.

## Gate 5 — Grasp-angle authority (who decides the final angle?)
**Options:** Gemini multi-candidate arbiter · mask-PCA · **minAreaRect flat-face snap + geometric gate** ✔

| Rejected | Why |
|---|---|
| Gemini arbiter | [MEASURED] the V0 smoking gun: arbiter chose ~90° regardless of block angle → **71% of the dataset in one angle bin** → mode-averaging froze the policy on rotated blocks (V0_ANALYSIS fig 1). Also non-deterministic across calls. |
| mask-PCA alone | [MEASURED — OFFLINE_TEST_REPORT §5] frame-to-frame jitter 1.65° median / **12.39° p90** vs minAreaRect's 0.31°/0.47° on identical frames; PCA-vs-rect disagree 12.7° median on the same mask (near-square ill-conditioning) while rect is detector-independent to 0.3°. |

**Result of the fix:** [MEASURED] worst angle bin 71% → **19.7%** in V1; deploy eval:
75/75 on rotated blocks.

## Gate 6 — Grip force
**Options:** fixed force · **DeliGrasp (VLM physics priors) + measured width** ✔

- Fixed force: [MEASURED] the AX-12 overload story — a force right for a light object drops
  heavy ones and crushes soft ones; DeliGrasp's per-object force closed the gap. And
  [MEASURED] gripper-measured width replaced the PCD width guess after the PCD ran ~mm off
  (aperture is ground truth, depth is an estimate).

## Gate 7 — What enters the dataset? (the reward gate)
**Options:** keep everything · human labeling · **physically-held AND VLM judge ≥ 0.6** ✔

| Rejected | Why |
|---|---|
| Keep everything | [MEASURED — literature + our audit] robomimic: mixed-quality data hurts plain BC. ⚠ CORRECTION (OFFLINE_TEST_REPORT Finding A): the angle-flag reject was documented but NEVER enforced — 86/88 `issue=angle` attempts passed on reward alone. V1.1 must enforce it or fix the flag semantics. |
| Human labeling | [MEASURED] 229 episodes × ~30 s/label ≈ 2 h of human time per collection run — the exact cost the pipeline exists to delete. Judge agreement with humans: [TO-MEASURE → judge-agreement experiment, queued]. |

**Save rate at scale:** [MEASURED] 99.1% over 231 attempts — the gate rejects little, but
what it rejects matters (angle-inconsistent supervision).

## Gate 8 — Data format
**Options:** custom format · ROS bags · **LeRobot v0.4.4** ✔

- ROS bags / custom: [CONSTRAINT] every trainer we target (ACT, π0, SmolVLA) consumes
  LeRobot natively; a custom format buys nothing and costs a converter. Version PINNED
  because [MEASURED] the reader must match the writer (video-decode API drift bit us on
  the cluster).

## Gate 9 — Coverage strategy
**Options:** random scatter · large-scale random (QT-Opt style) · **grid × angle + jitter** ✔

| Rejected | Why |
|---|---|
| Random scatter | [MEASURED] V0: 71% angle collapse + misses beyond the ±4 cm scatter. Interpolation-only generalization measured on hardware. |
| QT-Opt-scale random | [MEASURED] works at 100k+ grasps — 3 orders of magnitude more robot-hours than our budget (229 episodes, 3.4 h). |

**Result:** [MEASURED] V1 eval shows NO spatial falloff (97% at the 6 cm ring).

## Gate 10 — Action space
**Options:** binary gripper · **aperture-mm continuous** ✔

- Binary: [MEASURED] deployed V0-style mapping closed at step 0 on a V1 policy; the
  pre-position event was physically real but inexpressible → needed a hand-coded shim.
  With aperture-mm the policy emitted its own 86→59→55 mm staircase and the shim was
  deleted. Rule extracted: the action space must contain every event the policy must
  reproduce.

## Gate 11 — Policy architecture
**Options:** π0 / big VLA · SmolVLA fine-tune · diffusion policy · **ACT from scratch** ✔

| Rejected | Why |
|---|---|
| π0-class (3B+) | [CONSTRAINT] does not fit the 8 GB deployment GPU beside SAM3+GGX; inference latency incompatible with 10 Hz on a 2070. |
| SmolVLA fine-tune | [TO-MEASURE — queued as the V2-era comparison row] not rejected; deferred. At 229 episodes, from-scratch ACT already reaches 97/100 — fine-tuning has little headroom to demonstrate on this task. |
| Diffusion policy | [TO-MEASURE — unexplored] no constraint; ACT's single-forward-pass chunk (6–7 ms) already 15× under the 100 ms tick budget. |

## Gate 12 — Training compute
**Options:** lab RTX 2070 · CURC · **NSF ACCESS DeltaAI** ✔

- 2070: [MEASURED] fully occupied by the perception stack (4.5+1.8 GB) during collection;
  and GH200 trains at 0.037 s/step → 150k in ~1.5 h vs an estimated overnight+ locally.
- CURC: [CONSTRAINT] reserved for future big runs; DeltaAI allocation already approved and
  a run costs ~1–2% of it.

## Gate 13 — Deploy-time chunk handling
**Options:** temporal ensembling (ACT default) · **n_action_steps=10, no ensembling** ✔

- Ensembling: [MEASURED] averaging rotation vectors across chunks is invalid near 180° —
  offline replay showed **3× worse rotation error** with ensembling ON. This is the
  clearest pure-measurement rejection in the project.

## Gate 14 — Evaluation methodology
**Options:** anecdotal demos · **graded 100-grasp grid protocol** ✔

- Anecdotes: [MEASURED, by contrast] the graded protocol caught what demos can't: 0°
  succeeds 88% but at 0.58 quality — success-rate alone (let alone demos) would have
  hidden the aliasing. Same protocol re-runs for V1.1/V2 → before/after comparability.

---

## Experiments queued to convert [TO-MEASURE] → [MEASURED]

| # | Experiment | Fills | Where |
|---|---|---|---|
| 1 | Detector bake-off (rate/latency/agreement) | Gate 2 | `appendix_bakeoff.ipynb` figs A |
| 2 | Angle-strategy error vs ground truth | Gates 3, 5 | `appendix_bakeoff.ipynb` figs B, C |
| 3 | Judge-vs-human agreement on a fixed grasp-image set | Gate 7 | small notebook, ~50 images, queued |
| 4 | SmolVLA fine-tune comparison row | Gate 11 | V2 era |

*Maintenance: when an experiment lands, replace the [TO-MEASURE] tag with the number and
cite the figure. This document + FULL_PIPELINE.md §10 (failure ledger) together are the
complete "why is it built this way" record.*
