# PROJECT CONTEXT BRIEF — paste this into any new Claude/Gemini chat

> You are helping with an ongoing robot-learning research project. This document gives you
> full context so the researcher doesn't have to re-explain. Read it all before answering.
> Last updated: 2026-07-08.

## Who / what / goal

Undergraduate researcher ("Adam", CU Boulder-affiliated lab) building an **autonomous
grasping + VLA (vision-language-action) learning pipeline** on real hardware, mentored
weekly. Long-term research interest: **perpetual/continual learning — a VLA with a live,
continually-updated memory**. Near-term: iterate versioned pick-up policies (V0 → V1 → V2
multi-object), collect high-quality data fast, train on NSF ACCESS supercomputers, publish.

## Hardware / software stack

- **UR5e arm** (ROS2 Humble driver; ONLY linear `moveL` + `servoL` streaming — no moveJ from
  notebooks; watch for singularities near the top-down pose; `/arm/move_safe` recovers)
- **MAGPIE gripper**: custom 2-finger, AX-12 servos. Quirks: aperture scale reads ~20mm
  under reality (self-consistent), torque cuts out after ~1-2s holding (fix: clear_error +
  re-assert force), force reading unreliable (a held block can read "dropped")
- **RealSense D435i** wrist camera (RGBD, the ONLY camera)
- Lab PC: **RTX 2070 8GB** — runs SAM3 segmentation (~3.9GB, subprocess over Unix socket,
  ~90s boot) + GraspGenX grasp model (ZMQ :5557, ~0.8GB) + policy inference (11ms/tick)
- Everything driven from **Jupyter notebooks** in repo `magpie_control` (branch `ros`):
  - `notebooks/magpie_collect.ipynb` — scripted autograsp + data collection (cells: 1 =
    launch ALL drivers+SAM3+GraspGenX as kernel children; 7 = full pickup; 8 = Gemini
    quality judge + episode commit; AUTO-COLLECT cell = hands-free loop)
  - `notebooks/v0_test.ipynb` — standalone policy deployment (drivers → node → GO-HOME →
    10Hz DEPLOY loop with safety envelope)
- **Gemini 2.5 Flash** used inside the pipeline (object ID, grasp-angle arbiter, quality
  judge = reward gate, temperature=0). API key in env — NEVER print it.
- Data format: **LeRobot v0.4.4** (pin this version everywhere), 10Hz episodes,
  state=[xyz, rotvec, grip_mm, grip_force, wrist_fz], action=[target xyz, rotvec, grip 0/1]

## Scripted autograsp pipeline (the data generator)

Cell 7 does: SAM3 detect → move above object → closed-loop centering → point cloud (PCA) +
GraspGenX (angle only) → Gemini arbiter picks grasp angle → re-localise AT the grasp angle
(critical calibration fix) → descend → DeliGrasp force close → slip-guard lift → place.
Episodes recorded at 10Hz, saved ONLY if held AND Gemini quality ≥0.6 ("reward gate").
Camera verdict (judge looks at after-lift image) overrides the unreliable force sensor for
held/dropped. Self-calibrating gripper offset (TCP-local, in `data/last_calibration.json` —
if grasps suddenly go off-centre, CHECK THIS FILE; known-good gripper_offset_tcp_m ≈
[-0.0004, -0.0152], angle offset small; self-cal can run away — clamps added).

## Current state (as of 2026-07-08)

**V0 = DONE (mentor milestone hit on deadline 07-07):**
- 60 clean episodes (mean quality 0.87) collected via auto-collect loop in `data/lerobot_v0`
- ACT policy (51.6M) trained on **NSF ACCESS DeltaAI**: 100k steps in 62 min on one GH200,
  loss 0.043. Runbook: `docs/NSF_TRAINING.md` (ARM pitfalls: pip installs CPU torch —
  uninstall it; use module `python/miniforge3_pytorch/2.7.0` + `module load cuda`;
  `--policy.push_to_hub=false` required; smoke-test dataset load before sbatch).
  Login: `aabid@dtai-login.delta.ncsa.illinois.edu`, partition `ghx4`, account `bgcd-dtai-gh`.
- Policy deployed (`v0_test.ipynb`): **autonomous picks work** — visual reach, self-rotation
  to 90°, descend, close, lift, scripted place handoff.
- Deploy lessons (all fixed): re-infer every 1s not 10s (`n_action_steps=10`); do NOT use
  ACT temporal ensembling (averages rotation vectors across the 180° flip — makes rotation
  worse); canonicalize rotvec branch per tick; tilt-only orientation cap with FREE wrist
  yaw; driver servo time must match stream period (was 2ms bursts = stutter); gripper
  service calls must be async during servo streaming (blocking calls → UR watchdog kills
  the RTDE script; recover = pendant + relaunch drivers); finger PRE-POSITION to ~51mm at
  grasp height is required before the policy will close (it was script-side in demos, not
  in the binary action space — deploy shims it).

**V0's measured limits (the reason for V1):**
1. Competence ≈ training support: picks inside the ±4cm scatter zone; 5-7cm out → hesitant
   creep, descent regresses to training mean, miss. (Interpolation-only generalization.)
2. Angle supervision poisoned: 42/62 demos grasped at ~90° regardless of block angle →
   policy FREEZES on a 45° block (imitation averages conflicting modes into inaction).
3. Binary grip action insufficient (pre-position not expressible).
4. ~50% of each episode is off-task carry/place footage (also caused post-grasp dithering).

## V1 plan (research-backed, docs/V1_data_research.md has the full review + 20-paper reading list)

Target: collected + trained by ~2026-07-10. **Systematic coverage, not random** (validated
by Data Scaling Laws 2410.18647, spatial-generalization studies, MOVE 2512.04813, Geometric
Entropy 2606.20871 — diversity ≫ quantity; interpolation-only; consistency prevents
mode-averaging):
- **5×5 position grid @3cm (±6cm) × 7 angles (0–90° per 15°)** = 175 cells, jitter
  ±1cm/±5° per cell, +~30 edge/corner reps, +~30 scripted perturb-then-correct "recovery"
  episodes (RaC/IntervenGen-style) ≈ **235 episodes ≈ 4-5 robot-hours**
- Recorder changes: **gripper aperture (mm) as continuous action** (replaces binary),
  episode **ends at grasp + 5cm lift** (place = unrecorded reset to the NEXT grid cell —
  one pickup per episode, ~1min each), trim dead compute-gap frames
- Angle-consistent supervision: grasp angle snapped to the block's flat face (minAreaRect);
  reward gate REJECTS `issue=angle` episodes
- Keep: declared-target waypoint actions (AWE-validated), scripted-expert consistency,
  success filtering (MimicGen-style — our pipeline is a real-world MimicGen)
- Eval = the same grid → per-cell success heat-map
- Throughput: ~50-60 episodes/hr → 1-2k/week feasible → after V1, multi-object = same
  machinery + task-string conditioning (already recorded per episode)

## Working style / preferences (respect these)

- Direct, fast iteration; hates over-engineering ("no extra shit"); wants ONE change at a
  time with clear run instructions. If an explicit instruction won't work, SAY SO and ask
  before substituting a "better" approach.
- Jupyter gotcha: edits to .ipynb on disk require **File → Reload Notebook from Disk**
  before running (Ctrl+S in a stale editor clobbers disk edits). Kernel restart is only
  needed for .py changes; it KILLS the robot drivers (they're children of the kernel that
  launched them — re-run the drivers cell after).
- Robot safety: hand on e-stop for new motions; block near table center avoids UR5
  singularities; never open the gripper at height (place low or it throws the block).
- When grasps regress: check `data/last_calibration.json` first, then the shape model
  (`data/grasp_log/shape_log.json`), then camera/driver state — the MODEL is rarely the
  problem (verify by offline replay of training episodes before blaming it).

## Key file map

| Path | What |
|---|---|
| `notebooks/magpie_collect.ipynb` | scripted autograsp + collection (the data engine) |
| `notebooks/v0_test.ipynb` | policy deployment |
| `data/lerobot_v0/` | V0 dataset (60 eps); `models/act_v0/` = trained policy (local only) |
| `docs/NSF_TRAINING.md` | validated DeltaAI training runbook |
| `docs/NSF_ACCESS_SLIDES.md` | general ACCESS presentation + runbook template |
| `docs/V1_data_research.md` | data-strategy literature review + reading list |
| `tests/July_7_Update.md` | V0 milestone report |
| `scripts/vla_recorder.py` | 10Hz LeRobot recorder + reward gate |
| `scripts/grasp_memory.py` | per-object priors/shape memory |
| `data/last_calibration.json` | gripper offset calibration (check when grasps drift) |

## How to help me

Typical asks: debug pickup/deploy runs from pasted logs, edit notebook cells (give exact
code + which cell), plan V1/V2 experiments, training-job help (DeltaAI), research reviews,
mentor-facing docs/slides. Assume the hardware quirks above are real and constrain designs.
Prefer minimal, verifiable changes; give run instructions after any edit; when uncertain,
propose the plan BEFORE touching the working pipeline.
