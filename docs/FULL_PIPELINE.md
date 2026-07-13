# MAGPIE VLA Pipeline — The Complete Record

*Everything, end to end: hardware → scripted expert → data → training → deployment → evaluation
→ automation. Every option we explored, what we chose, and why. Last updated 2026-07-09.*

*Companion docs: [pickup_pipeline.md](pickup_pipeline.md) (scripted-expert internals),
[DATA_STANDARD.md](DATA_STANDARD.md) (the 6-step data method), [V0_ANALYSIS.md](V0_ANALYSIS.md)
(V0 post-mortem), [V1_data_research.md](V1_data_research.md) (literature review),
[V0_V1_COMPARISON.md](V0_V1_COMPARISON.md) (measured comparison),
[NSF_TRAINING.md](NSF_TRAINING.md) (cluster runbook).*

---

## 0. The one-paragraph story

A scripted expert ("autograsp") picks up a red block on a real UR5e, judged episode-by-episode
by a VLM reward gate. Good episodes are recorded in LeRobot format at 10 Hz. Coverage is
*designed* (5×5 position grid × 7 angles, jittered), not random. The dataset trains an ACT
policy from scratch on an NSF ACCESS supercomputer (DeltaAI, ~1.5 GPU-hours). The policy
deploys back on the arm at 10 Hz inside a safety envelope and is measured on the same grid it
was trained on (100-grasp graded eval). No teleoperation, no human demonstrations, no human
labels anywhere in the loop.

```
 SAM3 + GraspGenX + DeliGrasp            LeRobot v0.4.4                ACT 51.6M
┌──────────────────────────┐   10Hz    ┌───────────────┐   rsync    ┌─────────────┐
│  SCRIPTED EXPERT (real   │──────────▶│ DATASET       │───────────▶│ TRAIN       │
│  arm, grid coverage)     │  reward   │ 229 episodes  │  DeltaAI   │ 150k steps  │
└──────────────────────────┘  gate     └───────────────┘  GH200     └──────┬──────┘
        ▲                     (Gemini judge ≥0.6 + held)                   │ rsync back
        │ scripted reset / block placement                                 ▼
┌──────────────────────────┐                                       ┌─────────────┐
│  GRID EVAL (100 graded   │◀──────────────────────────────────────│ DEPLOY 10Hz │
│  grasps, heat-maps)      │        same arm, safety envelope      │ servoL loop │
└──────────────────────────┘                                       └─────────────┘
```

---

## 1. Hardware

| Component | Details | Quirks that shaped the design |
|---|---|---|
| **UR5e arm** | ROS2 Humble driver; `moveL` (blocking) + `servoL` (streaming) | Refuses moveL while in servo state ("unreachable or singular") — every deploy loop must call `/arm/stop` before scripted moves. Singular configs near top-down pose: `/arm/move_j` + `/arm/move_safe` (JOINT moves) escape what moveL can't. |
| **MAGPIE gripper** | Custom 2-finger parallel jaw (Correll lab), 2× Dynamixel AX-12, 1 Mbaud, protocol 1.0, IDs 1/2 | See §1.1 — the single largest source of debugging in the project. |
| **RealSense D435i** | Wrist-mounted RGBD, the ONLY camera | Camera extrinsic `_TCP_TO_CAM`: Rz(−90°) clocking, z_off = 0.144 m (0.120 + 0.024 measured mount error). IR stripe artifacts corrupt depth-PCA → the 2D mask-PCA fix (§3.3). |
| **ATI FT sensor** | Wrist wrench 250 Hz | Feeds `wrist_fz` into the state vector. |
| **Lab PC** | RTX 2070 **8 GB** | The VRAM budget rules everything: SAM3 ~4.5 GB + GraspGenX ~1.8 GB + ACT ~1 GB + CUDA overhead ≈ full. Stale Jupyter kernels (~0.5 GB each) have caused OOM — kill them before deploy. |
| **Compute cluster** | NCSA **DeltaAI** (NSF ACCESS), GH200 nodes: ARM CPU + H100-class GPU | ARM changes Python packaging entirely (§5.2). |

### 1.1 The AX-12 gripper — full quirk list (each cost real debugging time)

1. **Torque overload shutdown**: holding against resistance for ~1–2 s trips the overload
   protection — torque dies, object drops on lift. Fix: `grip_refresh` (clear_error →
   set_force → close) between incremental lift steps.
2. **The 2N trap**: `/gripper/clear_error` calls `reset_packet_overload(force_limit=2)` —
   it re-enables torque **at 2N**, far too weak to move the fingers. *Every* motion command
   after a clear_error silently stalls unless force is re-asserted first. This one bug
   masqueraded as three different symptoms over the project: close-stalls during deploy,
   "calibration not accurate," and "gripper doesn't open sometimes."
3. **Calibrate service is a placebo**: `/gripper/calibrate` is just open → 2 s → close. AX-12
   encoders are **absolute** (potentiometer, 0–300°) — there is no software zero to restore.
   A hard collision can only slip the horn physically, which no service call fixes.
   The real reset (commit 6736267) = **full driver reboot**: pkill + relaunch `gripper_node`
   re-runs `Gripper.__init__` (torque=200 written to both servos, USB port reopened), then
   open at 16N and **verify aperture reads >80 mm**, retry once, loud warning if still shut.
4. **Aperture scale reads ~20 mm under reality** (self-consistent): a 46 mm block seated
   reads ~27 mm; full open (104 mm command) reads ~90–104 mm. All thresholds in the deploy
   loop are in *measured* units.
5. **Force reading unreliable**: a firmly held block can read as "dropped" — never use
   force alone as the held/dropped verdict; use aperture bands.
6. **USB adapter can drop off the bus** (2026-07-09 hardware failure): symptom = node dies
   with "Could not find dynamixel port," `/dev/ttyACM0` missing. Diagnosis path that worked:
   direct dynamixel ping (PortHandler, 1 Mbaud, protocol 1.0, read model number of IDs 1/2).
   Fix was physical replug; the driver-reboot reset (#3) now recovers the software half
   automatically.

---

## 2. Driver / ROS layer

All hardware runs as ROS2 nodes launched as **children of the Jupyter kernel** (subprocess.Popen
with sourced workspaces, logs to `/tmp/log_*.txt`):

- `ur5_node` — `/arm/move_l`, `/arm/move_j`, `/arm/move_safe`, `/arm/stop`, `/arm/teach_mode`,
  `/arm/servo_l_cmd` (streaming topic), `/arm/tcp_pose`. Servo-time parameter was **2 ms**
  originally → 98 ms freeze between bursts = mechanical stutter; now **0.1 s** (matches the
  10 Hz command rate).
- `gripper_node` — `/gripper/{open,close,set_position,set_force,calibrate,clear_error,
  reset_parameters}` + `/gripper/state` (10 Hz aperture/force/temp) + DeliGrasp action server.
- `realsense2_camera` — namespaced `/camera/gripper_camera`.
- `ft_sensor_node` — `ft_sensor/wrench`.
- `slip_guard` — depth-based slip detector with force-reclamp (used by the scripted expert's
  lift).
- **SAM3** — too big to import in-process (float32 patch needed, ~90 s boot): runs as a
  subprocess bridge over a Unix socket (`/tmp/sam3.sock`), JSON in / boxes+scores+mask out.
- **GraspGenX** — ZMQ server on :5557 (NVlabs GraspGen weights + MAGPIE gripper embodiment).

**Operational rules learned the hard way:** drive services from the notebook KERNEL, not an
ephemeral shell (children die with their parent); restarting the collect kernel kills every
driver (v0_test/v1_test have their own lightweight driver cell for exactly this reason);
kill stale kernels before loading a policy or CUDA OOMs.

---

## 3. Perception + grasp planning (the scripted expert's brain)

### 3.1 Object detection — options explored

| Option | Status | Notes |
|---|---|---|
| Gemini 2.5 Flash (VLM box) | kept for object *identification* | temperature=0 everywhere for determinism |
| OWL-ViT-class open-vocab detectors | superseded | early stack |
| **SAM3** (Segment Anything 3) | **CHOSEN** — pixel-accurate mask from text query | needed a float32 patch + subprocess bridge to live on the 2070; HF access approved |
| LocateAnything (NVIDIA bbox) | back-burner | needs int4 quant to fit 8 GB; revisit with bigger GPU |

### 3.2 Grasp planner — the bake-off

Four candidates compared on the same objects (`GRASP_METHOD` swap in the collect notebook):

| Planner | Verdict |
|---|---|
| **PCA on segmented point cloud** | baseline; kept as fallback and as the *angle* prior |
| GraspGen (original) | superseded by GraspGenX |
| GSNet | evaluated, not adopted |
| AnyGrasp | dropped (licensing/weight access friction) |
| **GraspGenX** (NVlabs cross-embodiment + MAGPIE gripper) | **CHOSEN** — live grasps confirmed 2026-06-30; coexists with SAM3 in VRAM (measured) |

Design constraint that shaped this: 8 GB VRAM → planners must cold-start on demand or coexist;
GraspGenX (~1.8 GB) + SAM3 (~4.5 GB) fit together.

### 3.3 Grasp angle — options explored (this is where V0 was poisoned)

1. **3D PCA on the point cloud** — corrupted by IR stripe artifacts in depth.
2. **2D PCA on the SAM3 mask** (`mask_grasp_angle()`) — fixed the stripe problem; confirmed
   on a cube at 45°.
3. **Gemini multi-candidate arbiter** (image + candidate overlay) — unreliable off-axis;
   picked ~90° regardless of block angle → **the V0 angle collapse (42/62 episodes at 90°)**.
4. **minAreaRect flat-face snap + geometric gate** — **CHOSEN for V1**: deterministic,
   snaps the grasp to the block's flat face, rejects episodes where achieved angle deviates
   from the block angle (`issue=angle` → not saved).

Remaining known flaw: **0°/90° aliasing** — a square block at 0° and 90° is the same physical
scene, but the mod-90 gate let both labels through (15 eps @0°, 45 eps @90° of identical
scenes) → imitation mode-averaging → the trained policy rotates to ~43° on straight blocks.
Uniquely-labelled angles (15–75°) deploy cleanly. Fix queued (V1.1): canonicalize the label
for square objects + ~35 replacement episodes + retrain.

### 3.4 Grip force — DeliGrasp

Gemini estimates object physics priors (mass, friction) → DeliGrasp adaptive force closure.
Runs once in background (skipped at n≥2 of the 3 physical grip attempts — the 3 retries are
intentional, they improve seating). Gripper-measured width (`s_lift.position` after a clean
hold) replaced the PCD width guess as ground truth.

### 3.5 Calibration — options explored

- **Self-tuning calibration** (judge feedback → `GRASP_ANGLE_OFFSET`): drifted 3× during V0
  collection → **frozen constants** for V1 (consistency contract).
- **Off-centre grasp drift**: camera offset (frozen at scan angle) was lumped with finger
  offset (rotates with grasp) → left/right drift depending on angle. Fix: **re-localise the
  object AT the grasp angle** before descending, so both offsets co-rotate.
- `calibrate` gripper service: placebo — see §1.1.3.

---

## 4. Data: recording, action space, coverage, quality gate

### 4.1 Recorder

`scripts/vla_recorder.py` (`VLARecorder`) — dedicated ROS node with its own executor, samples
at 10 Hz: wrist camera + arm state + **declared action**. Episodes buffer in RAM and commit to
the LeRobot dataset **only if they pass the reward gate**; failures go to
`<root>_attempts.jsonl` (never the trainable set — but never deleted either: they're the
audit trail).

- **State (9-dim)**: `[x, y, z, rx, ry, rz, grip_mm, grip_force, wrist_fz]`
- **Action (7-dim)**: `[target x, y, z, rx, ry, rz, grip]`
- **Actions are declared waypoint targets**, not measured next-states — AWE-validated design
  (waypoint targets beat dense actions for ACT by 4–28% in low-data regimes). This was
  accidental-then-kept: the scripted expert *has* explicit targets, so we record them.

### 4.2 The reward gate (what makes scripted data trustworthy)

Commit criteria: physically **held** (aperture band through lift) AND **Gemini quality judge
≥ 0.6** (image of the grasp, temperature=0, thinking_budget=0 for determinism). Judge rubric
was recalibrated 2026-07-01 after it capped dead-centre grasps at 0.60: centring now has a
tolerance (~1/5 width), 0.8–0.95 band = "very good and trainable." If clean grasps score ~0.6
again, the rubric has drifted.

This gate is the project's philosophical core: **a physically-grounded write filter**. It is
what makes machine-generated data land in the MimicGen regime (success-filtered, works) rather
than the noisy-RL regime (hurts BC).

### 4.3 Action space — V0 vs V1 (the single most consequential change)

| | V0 | V1 |
|---|---|---|
| Grip action | **binary 0/1** | **aperture in mm** (continuous) |
| Finger pre-position | happened physically but was INVISIBLE in the action space | in-band: staircase 104 (open) → width+8 (pre-position) → 20 (squeeze) |
| Consequence | deployed policy couldn't reproduce pre-positioning; needed a hand-coded shim | policy pre-positions its own fingers (observed: 86→59→55 mm staircase from its own outputs); shim deleted |

Rule extracted: **the action space must contain every event the policy must reproduce.**
Corollary: V0 and V1 datasets **cannot be mixed in training** — the normalizer would smear
binary 0/1 against 20–104 mm values. act_v1 trained on the 229 V1 episodes only.

### 4.4 Coverage strategy — options explored

| Option | Verdict |
|---|---|
| Random scatter ±4 cm, random angle (V0) | measured failure: interpolation-only generalization, 71% of grasps at one angle, misses beyond the scatter |
| Large-scale random (QT-Opt style, 100k+) | works but needs 3 orders of magnitude more robot-hours |
| **Systematic grid + jitter (V1)** | **CHOSEN**: 5×5 positions @ 3 cm × 7 angles @ 15° + jitter ±1 cm/±5° — every cell guaranteed in the interpolation support; literature converged on exactly this (MOVE's +76%, spatial-support studies) |
| Recovery episodes | added at ~15% rate: deliberate XY offset at approach → scripted correction recorded (RaC/IntervenGen-inspired; V0 had zero correction data) |
| Edge reps | extra repetitions at boundary cells |

V1 also **ends episodes at lift** (V0 wasted ~40% of frames on carry/place footage that
caused post-grasp dithering) — median episode 39.6 s → 11.2 s.

### 4.5 Collection infrastructure

- **AUTO-COLLECT loop** (V0): hands-free self-reset — pick wherever → place at target →
  record next episode; block scatter via random placement in a 6×6 cm box.
- **Grid sequencer** (V1, `v1_collect.ipynb`): 225-combo ledger (`data/v1_coverage.json`),
  serpentine order, resume-safe. Completed 229 episodes at **66.7 eps/hr** (2.1× V0), save
  rate 99.1%.
- **Resilient sequencer** (V1.1, `v1_1_collect.ipynb`): auto-recovery on miss/crash (servo
  release + gripper driver reboot w/ verified open + home + retry same combo); stops only on
  3 consecutive crashes or block-not-found. Namespace lesson: `run_cell` shares the kernel
  namespace — sequencer vars are `_sq_*` prefixed after a collision with the pickup cell's
  `_cur` broke everything. Output lesson: ~20k lines froze the VS Code renderer → per-episode
  detail goes to `data/v1_collect_log.txt`, screen gets one line per 10 episodes.

### 4.6 The audit (DATA_STANDARD step 4 — the gate before any training)

Run on the finished dataset, must pass before pushing to the cluster:
episode count ≥ target, angle histogram (collapse check: worst bin < 45%), grid coverage
(0 holes in 25 cells), episode-length outliers, frames/durations sane. The V1 audit read:
229 eps, angle bins ~uniform [15,30,39,31,33,36,45], 25/25 cells, median 112 frames/11.2 s,
0 outliers — and one warning ("0° LOW") that we misread as benign but was actually the
0/90 aliasing announcing itself. Lesson recorded: audit warnings get investigated, not
rationalized. In the automated pipeline (§7) the audit is the **decision-maker**.

---

## 5. Training

### 5.1 Model — options explored

| Option | Verdict |
|---|---|
| **ACT** (Action Chunking Transformer, 51.6M params) | **CHOSEN for V0/V1**: right size for ~50–250 episodes, trains in ~1 GPU-hour, 6–11 ms inference on the 2070 |
| π0 / π0.5 | too big for the deployment GPU; fine-tune candidates for V2+ |
| SmolVLA fine-tune | queued as a V2-era comparison row (architecture axis) |
| Diffusion policy | not explored yet |

**What the training actually is**: from-scratch behavioral cloning ("basic VLA training"),
NOT fine-tuning. Only the vision backbone (ResNet18) starts from ImageNet weights. ACT
ignores the task string (it's recorded for future VLA compatibility). ~33 epochs over the
dataset at 100k steps / batch 8.

### 5.1.1 The policy itself — exact I/O and architecture (act_v1 config)

**Inputs** (one observation per 10 Hz tick):
- `observation.images.wrist` — RGB 3×480×848 from the D435i (the only camera)
- `observation.state` — 9-dim: `[x, y, z, rx, ry, rz, grip_mm, grip_force, wrist_fz]`

**Output**: a **chunk of 100 future actions** (`chunk_size=100`), each 7-dim:
`[target x, y, z, rx, ry, rz, grip]` — for V1 `grip` is **aperture in mm** (V0: binary 0/1).
Actions are *declared waypoint targets* (§4.1), so a predicted action is "where the arm
should be heading," not a velocity.

**Architecture** (ACT = Action Chunking Transformer, CVAE variant; 51.6M params, ~197 MB):
- **Vision**: ResNet18 backbone (ImageNet-pretrained, fine-tuned during training) → feature
  map → flattened tokens with sinusoidal position embeddings.
- **Transformer**: encoder 4 layers / decoder 1 layer, `dim_model=512`, 8 heads,
  feedforward 3200. Decoder cross-attends vision tokens + state token + latent token and
  emits the 100-action chunk in one shot (no autoregression → single forward pass, 6–7 ms
  on the RTX 2070).
- **CVAE**: a 4-layer VAE encoder (training only) compresses the ground-truth action chunk
  into a 32-dim latent `z`; the decoder is conditioned on it. At inference `z=0` (the
  distribution mean). Purpose: absorb demonstrator style variation so the deterministic
  decode is clean — with a scripted expert there's little style variance, but it also mops
  up jitter.
- **Loss**: L1 reconstruction on the action chunk + KL regularizer (`kl_weight=10`),
  lr 1e-5, AdamW.

**What is actually learned vs. fixed**:
| Component | Trained? |
|---|---|
| ResNet18 vision backbone | fine-tuned (starts from ImageNet) |
| Transformer encoder/decoder, action head, VAE encoder | from scratch |
| Normalization statistics | **not learned** — computed from the dataset (mean/std per feature) and frozen into the pre/post processors |
| Task string | ignored by ACT entirely |

**Normalization is part of the model**: the pre-processor normalizes image/state with the
*training dataset's* statistics; the post-processor un-normalizes predicted actions back to
metres/radians/mm. This is WHY V0 and V1 data can't be mixed (a 0/1 grip channel and a
20–104 mm grip channel produce nonsense shared statistics), and why the processor files must
travel with the weights.

**What training outputs** (pulled back to `models/act_<tag>/`):
```
config.json            — architecture + I/O shapes (everything quoted above)
model.safetensors      — the 51.6M weights (~197 MB)
policy_preprocessor.json  + …normalizer_processor.safetensors    — input normalization stats
policy_postprocessor.json + …unnormalizer_processor.safetensors  — action un-normalization
train_config.json      — full training provenance (dataset, steps, seed, lr schedule)
```
Plus per-`save_freq` checkpoints (`010000/`…`150000/`, `last` symlink) left on the cluster.

**Chunking at deploy time**: trained `chunk_size=100` (10 s of actions), but deployment
overrides `n_action_steps=10` — execute only the first 1 s of each predicted chunk, then
re-infer with fresh observations (§6.1: chosen over temporal ensembling, which corrupts
rotation vectors). The config on disk says `n_action_steps=100`; every deploy cell sets it
to 10 at load time.

### 5.2 Where — options explored

| Option | Verdict |
|---|---|
| Lab 2070 | busy running perception; too slow |
| **DeltaAI (NSF ACCESS)** | **CHOSEN**: GH200, ~0.037 s/step → 100k ≈ 1 h; costs ~1–2% of a small allocation per run |
| CURC (university cluster) | reserved for future big runs |

### 5.3 The recipe that works (validated 2026-07-06/08; full runbook in NSF_TRAINING.md)

- `module load python/miniforge3_pytorch/2.7.0 + cuda`, venv `--system-site-packages` —
  DeltaAI is **ARM**: plain pip torch is CPU-only, and the *newest* module (2.10+) removed
  torchvision `VideoReader` which lerobot needs. Newest ≠ best.
- `lerobot==0.4.4` pinned everywhere (must match the version that wrote the dataset).
- `--policy.push_to_hub=false` REQUIRED (job dies in 33 s without it).
- **5-second smoke test on the login node before every submit**: load one sample through
  LeRobotDataset (forces real video decode) — catches 90% of first-job failures.
- V0: 100k steps, 62 min, loss 0.043. V1: 150k steps, ~1.5 h.
- Pull back `checkpoints/last/pretrained_model/` → `models/act_<tag>/`.

### 5.4 Loading the checkpoint locally — gotchas

- **groot stub**: lerobot's package init imports every policy; the unrelated `groot` policy
  crashes under our Python → stub its modules before importing ACTPolicy (baked into every
  deploy cell + `auto_train_cycle.sh`).
- **config.device drives the safetensors load target**: for a real CPU fallback you must set
  `cfg.device='cpu'` before retrying `from_pretrained`, or it OOMs twice.
- Environment pins: `numpy==1.26.4` (ROS/cv2/matplotlib compiled against 1.x),
  `transformers<5` (hf-hub API compat) — a rogue upgrade broke both on 2026-07-08/09.
- Offline verification **before touching the arm**: replay training episodes through the
  policy, compare predicted vs recorded actions. V0: 1.9 mm mean position error → "model and
  recipe are fine, data is the bottleneck" (this measurement is what redirected all effort
  to data design).

---

## 6. Deployment (policy → real arm)

`notebooks/v0_test.ipynb` (act_v0) and `v1_test.ipynb` (act_v1) — standalone (own driver
cell, no SAM3/GraspGenX needed; the policy is end-to-end from pixels).

### 6.1 The 10 Hz control loop — every design choice and the failure it answers

| Choice | Failure it fixes |
|---|---|
| `n_action_steps=10`, **NO temporal ensembling** | ensembling averages rotation vectors across chunks — invalid near 180° (made rotation 3× worse in offline replay). Small chunks = re-infer every 1 s with fresh observations. |
| **Rotvec branch canonicalization** (pick the ±2π branch nearest the current pose) | 180° rotation-vector flips caused full-speed lurches |
| **Rotation-rate limit 12°/tick + tilt gate 25°, yaw FREE** | the original total-angle cap blocked the yaw alignment the policy needs for rotated blocks (caused edge grasps); tilt-only keeps the tool upright while letting it spin |
| **EMA position smoothing (α=0.5) + step cap 22 mm/tick** | jitter + runaway steps. Originally α=0.4/15 mm; loosened after trust established — the policy was fighting the cap (119 clamps/run → cap raised, clamps now mostly the rotation limiter during large yaw swings) |
| **XY box ±10 cm around start + Z floor = table + GRIPPER_LEN + 3 mm** | fingertips physically cannot reach the table |
| Servo-time 0.1 s | 2 ms bursts + 98 ms freeze = stutter |
| **Aperture-mm action mapping** (V1): <35 mm → force-close chain (clear→force 16N→close), 35–90 mm → set_position (policy pre-positioning), >90 mm while closed → open; >4 mm change threshold; force re-asserted on EVERY path incl. opens | V0's binary threshold misread aperture-mm outputs (closed instantly at step 0); the 2N trap stalled opens |
| **Retry-on-stall** (aperture >40 mm and unchanged ±1.5 mm, 2 s after close → re-fire chain, max 3) | AX-12 close stalls at first contact when the async clear→force race loses; mirrors the scripted expert's grip_refresh |
| **Settle-then-band handoff** (wait until aperture stops moving, up to 8 s; then <15 mm = air miss, >45 mm = bad grip, else held → scripted place) | "held" was being declared mid-close at 57 mm; and a fall-through bug once declared "held" at 90.6 mm after a slip — the held verdict must be the `else`, never the fall-through |
| **Division of labour**: policy = perceive/reach/align/pre-position/close; script = lift-carry-place | matches how the training episodes end (at lift); post-grasp frames were the V0 dithering source |
| GO HOME between rollouts: /arm/stop → two-stage move (vertical, then lateral) to the dataset's median frame-0 pose → gripper driver reboot + verified open | servo-state refusals; diagonal swoops; every gripper failure mode in §1.1 |

Deploy verified on the arm (2026-07-08/09, act_v1): rotated-block picks at ~20°, ~40°, 45°
(the case V0 froze on), off-centre +5 cm, corner cells reach with graceful degradation.
Inference 6–7 ms/tick on the 2070.

### 6.2 Deploy bugs found on the way (kept as a checklist for future policies)

1. Chunking/ensembling rotation averaging (offline replay caught it before the arm did).
2. Rotvec 180° branch flips.
3. Total-orientation cap vs yaw-free tilt cap.
4. Servo-time 2 ms.
5. Binary-vs-aperture grip mapping mismatch.
6. The 2N trap in every gripper path.
7. Handoff fall-through verdict.
8. config.device on CPU fallback; VRAM stale kernels.

---

## 7. Evaluation

- **V0**: informal — reliable in-zone, froze on rotated blocks (mode averaging), missed
  beyond the scatter. Documented in V0_ANALYSIS.md with 4 figures.
- **V1**: **100-grasp graded grid eval** (`v1_eval.ipynb`): 5×5 cells ×
  angles {0°, 25°, 45°, 70°} (90° ≡ 0° on a square block). Per grasp: scripted place →
  policy rollout (arm freezes at close — the policy learned to lift and shook blocks loose) →
  **lift-verify** (8 cm up, aperture must hold the seated band — slip-outs
  fail) → grade 0–1 (penalties: extra re-grips −0.1 each, yaw error vs block angle up to
  −0.4, aperture outside 20–40 mm −0.2) → place directly at the next combo (clean picks)
  or via the scripted stagehand. Resume-safe JSON (`data/v1_eval_100.json`); failure ladder:
  hardware stalls reboot+retry unrecorded, policy fails total-reset, 5-in-a-row relaunches
  the driver stack, 10 stops. Outputs: per-angle grade heat-maps + summary plots →
  `docs/figures/v1_eval_heatmap.png`, `v1_eval_summary.png`.
- **V1 MEASURED (2026-07-13, full report [V1_EVAL_REPORT.md](V1_EVAL_REPORT.md))**:
  **97/100 picks, mean grade 0.70**. Rotated blocks (25/45/70°): **100% (75/75)** — the
  V0-freeze case solved. Straight blocks (0°): 88% and grade 0.58 — all 3 failures of the
  run, two with ~41° yaw error = the 0/90 aliasing confirmed on hardware (V1.1's target).
  No spatial falloff (97% at the 6 cm ring — designed coverage worked). Grip-authority tax:
  94/97 picks needed ≥1 stall re-grip (mechanical, the main quality limiter).
- The grid heat-map is the project's standard result format — the same 5×5×angle grid used
  for collection is used for evaluation, and the planned paper ablations (§9) all report on it.

---

## 8. Automation (the robot's night shift)

`scripts/auto_train_cycle.sh <tag> [steps]` — one command: **audit gate → rsync push →
remote smoke test → sbatch → poll → pull checkpoints → local load test**. Unattended-Duo
solved via SSH **ControlMaster** (`Host deltaai`, ControlPersist 12h): authenticate once,
every subsequent ssh/rsync/sbatch rides the socket.

Current audit gate: episodes ≥ 50, worst angle bin < 45%. Must-harden list before full trust:
grid-coverage holes, length outliers, reward distribution (mean ≥ 0.8), one random-trajectory
motion check, and a named-diagnosis file on failure. **In the automated pipeline the audit is
the decision-maker** — nothing ships to the cluster that a human wouldn't have shipped.

End state (designed, partially built): collect (resilient sequencer) → audit → train → pull
→ eval, fully hands-free overnight. This is also the infrastructure the memory-paper's
"sleep consolidation" needs (§9).

---

## 9. Research directions & the paper map

- **P1 (workshop, writable now)**: autograsp data methodology — scripted expert + grid
  coverage + contracts + audit; the V0→V1 measured story (66.7 eps/hr, no teleop, no labels).
- **P2**: MuJoCo **sim twin** — replay/record simulation alongside every real episode,
  validate the twin on V1 replays, then sim+real vs real-only at equal robot-hours
  (data multiplication).
- **P3**: **continuous self-judged memory** — frozen policy + growing episodic store
  (LeRobot episodes, SigLIP/DINOv2 index) + semantic store (grasp_memory force/shape/angle
  priors — already live as a scalar prototype) + the reward gate as the physically-grounded
  WRITE FILTER + wake (retrieval-conditioned policy, no gradient steps) + sleep (nightly
  DeltaAI fine-tune via §8). Staged: retrieval demo → VINN-style baseline →
  retrieval-conditioned ACT → wake/sleep/both on grid heat-maps. Novelty sweep done
  2026-07-08 (closest: MemoryVLA, behavior retrieval/VINN, "Learning to Forget" — gap we
  fill: physically-grounded write filter + live store on real hardware, grid-measured).
- **P4 (conference, fall)**: the combination — "self-improving grasping with zero human
  demos/labels: scripted bootstrap + sim multiplication + live memory," core figure a 2×2
  ablation (sim × memory) on the same eval grid.
- Version ladder: **V0** (3×3, done) → **V1** (5×5 + rotation, deployed, eval running) →
  **V1.1** (aliasing patch + auto-cycle) → **V2** (multi-object: fruit, 100 eps each) →
  memory/sim thrusts.

---

## 10. Full failure→fix ledger (chronological highlights)

| Failure (measured) | Root cause | Fix |
|---|---|---|
| Object drops on lift | AX-12 overload shutdown | grip_refresh + incremental lift |
| Judge scores clean grasps 0.60 | rubric's zero-tolerance centring cap | tolerance + 0.8–0.95 trainable band |
| Left/right grasp drift that rotates with angle | camera offset frozen at scan angle | re-localise at the grasp angle |
| V0 policy freezes on 45° block | 42/62 episodes at 90° (Gemini arbiter) → mode averaging | minAreaRect snap + angle gate + grid×angle coverage |
| V0 misses beyond ±4 cm | interpolation-only support | 5×5 grid + jitter + edge reps |
| Policy can't pre-position fingers | binary grip action hid the event | aperture-mm action space |
| Deploy rotation 3× worse than replay | temporal ensembling averaging rotvecs | n_action_steps=10, no ensembling |
| Full-speed lurch mid-rollout | rotvec 180° branch flip | branch canonicalization + rate limit |
| Policy can't rotate to block | total-orientation cap | tilt-only cap, yaw free |
| Mechanical stutter | 2 ms servo-time | 0.1 s |
| Policy closes at step 0 (V1) | binary threshold on aperture-mm output | aperture mapping |
| Close stalls at 41–57 mm | 2N trap + async race | force re-assert in chain + retry-on-stall |
| "Held" declared at 90.6 mm after slip | handoff fall-through | settle-wait + band check, held = else only |
| Straight block grasped at ~43° | 0/90 aliasing in labels | V1.1: canonicalize + replacement episodes (queued) |
| Gripper won't open / calibration "inaccurate" | 2N trap on open paths + placebo calibrate | driver reboot + verified open (16N) everywhere |
| CUDA OOM on CPU fallback | config.device stayed 'cuda' | sync cfg.device on both attempts |
| lerobot import crash | transformers 5.x rogue upgrade | pin transformers<5 (and numpy==1.26.4) |

---

*Maintenance note: when a design choice changes, update BOTH the relevant section here and
the failure ledger — this document is the project's single source of "why is it built this
way."*
