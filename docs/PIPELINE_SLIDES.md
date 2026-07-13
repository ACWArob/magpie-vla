# AutoGrasp → Policy: The Full Pipeline — Slide Deck Source (bi-weekly, 15 min)

> Format: one `## Slide N` block per slide — title, bullets, speaker notes. Paste into
> Gemini / Google Slides as-is. Figure files referenced per slide live in `docs/figures/`.
> Pacing: ~1 min/slide, 14 slides + backup. Companion deck: NSF_ACCESS_SLIDES.md (already
> presented) covers the cluster mechanics — slide 9 here just points at it.

---

## Slide 1 — Title
**AutoGrasp → Data → Policy: a robot that generates its own training data**
- End-to-end: scripted expert → quality-gated dataset → ACT policy on NSF ACCESS → deployed on the same arm
- Zero teleoperation, zero human demonstrations, zero human labels
- Everything measured: collection throughput, dataset quality, per-cell deployment success

*Speaker notes: The one-sentence claim of this talk — the robot collects its own demonstrations, judges them itself, and the resulting policy picks blocks it has never been shown, including the rotated ones the previous version failed on. 15 minutes, 4 acts: expert, data, training, deployment.*

---

## Slide 2 — The loop at a glance
**IMAGE: keep the ASCII diagram as a styled graphic (or rebuild in Slides shapes)**
```
 SAM3 + GraspGenX + DeliGrasp             LeRobot v0.4.4              ACT 51.6M
┌──────────────────────────┐   10 Hz    ┌───────────────┐   rsync   ┌────────────┐
│  SCRIPTED EXPERT         │───────────▶│ DATASET       │──────────▶│ TRAIN      │
│  (real arm, grid)        │  reward    │ 229 episodes  │  DeltaAI  │ 150k steps │
└──────────────────────────┘  gate      └───────────────┘  ~1.5 h   └─────┬──────┘
        ▲                (VLM judge ≥0.6 AND physically held)             │
        │ scripted reset / block placement                                ▼
┌──────────────────────────┐                                       ┌────────────┐
│  GRID EVAL               │◀──────────────────────────────────────│ DEPLOY     │
│  (100 graded grasps)     │        same arm, 10 Hz, safety box    │ 10 Hz loop │
└──────────────────────────┘                                       └────────────┘
```
- One robot, one wrist camera, one week per version iteration

*Speaker notes: Walk the arrows once. Key point: the SAME 5×5 grid appears three times — collection coverage, evaluation, and (later) the paper figures. That symmetry is deliberate and is the methodology contribution.*

---

## Slide 3 — Act 1: the scripted expert (the data generator)
**IMAGE: a wrist-cam frame with the SAM3 mask overlay (grab one from any episode) — right half of slide**
- Pipeline per pick: **SAM3** text-query segmentation → move above → closed-loop centering → point cloud → grasp angle (minAreaRect flat-face snap) → **re-localise at the grasp angle** → descend → **DeliGrasp** force closure → slip-guarded lift
- Runs on the lab RTX 2070 (8 GB): SAM3 ~4.5 GB + GraspGenX ~1.8 GB coexist
- Self-resetting: places the block at the next grid target itself → hands-free collection

*Speaker notes: Expert stack detail if asked: SAM3 runs as a subprocess over a Unix socket (too big to import in-process on 8 GB); GraspGenX is the NVlabs cross-embodiment grasp model with our MAGPIE gripper embodiment; DeliGrasp turns VLM physics priors into grip force. The "re-localise at the grasp angle" step killed a systematic left/right drift — camera and finger offsets only cancel when measured at the final angle.*

---

## Slide 4 — The reward gate: the robot judges itself
- Episode is committed ONLY if: **physically held** (aperture band through lift) AND **VLM quality judge ≥ 0.6** (Gemini, temperature 0, image of the grasp)
- Fails go to an attempts log — auditable, never trainable
- This is the difference between "machine-generated data hurts BC" (robomimic) and "scripted data works" (MimicGen): **success-filtering + consistency**
- Save rate at scale: **99.1%** over 231 attempts (the expert is that reliable)

*Speaker notes: The write filter is physically grounded — the block either stayed in the gripper or it didn't — plus a semantic check for grasp quality. This same filter later becomes the memory-paper's write criterion, so it's load-bearing beyond data collection.*

---

## Slide 5 — Act 2: what we learned about DATA (V0, the honest failure)
**IMAGE: figures/v0/fig1_angle_collapse.png (left) + figures/v0/fig2_support_vs_grid.png (right)**
- V0: 60 episodes, random placement ±4 cm, random angle → trained fine (loss 0.043, replay error 1.9 mm) → **deployment told the truth:**
  - Picks succeed only INSIDE the training scatter — interpolation, not extrapolation
  - **42/62 episodes grasped at ~90° regardless of block angle** → policy FROZE on a 45° block (imitation averages conflicting modes into inaction)
  - ~40% of every episode was carry/place footage → post-grasp dithering
- Figure: `docs/figures/v0/fig1_angle_collapse.png` + `fig2_support_vs_grid.png`

*Speaker notes: This slide is the pivot of the talk. The model was never the problem — offline replay proved 1.9mm accuracy on training data. The DATA was the problem, in three measurable ways. Every V1 design choice on the next slide answers one of these three findings.*

---

## Slide 6 — V1 data design: every choice answers a measured failure
**IMAGE: figures/v0_v1/compare_angles_lengths.png (bottom strip, under the table)**
| V0 finding | V1 response |
|---|---|
| interpolation-only | **5×5 grid @3 cm × 7 angles @15° + jitter** — designed support, 0/25 holes |
| angle collapse (71% @90°) | deterministic flat-face angle + reject `issue=angle` → worst bin **19.7%** |
| binary grip hid finger pre-positioning | **aperture-mm continuous action** (104 → width+8 → 20) |
| 40% off-task frames | episode **ends at lift** → 39.6 s → **11.2 s** median |
| zero recovery behavior | ~15% scripted perturb→correct episodes |
- Throughput: **66.7 episodes/hour** (2.1× V0) → 229 episodes in one afternoon
- Figure: `docs/figures/v0_v1/compare_angles_lengths.png`

*Speaker notes: Literature agrees with all of this (data-scaling-laws: diversity beats count; MimicGen: scripted+filtered works; AWE: waypoint actions win in low data) — but the point is we didn't import these from papers, we measured each failure ourselves first. The action-space row is the deepest one: the policy must be ABLE to express every event it needs to reproduce.*

---

## Slide 7 — The audit: data ships only if it passes
**IMAGE: screenshot of the audit cell output (angle histogram + '0° LOW' warning line highlighted)**
- Before any training run: episode count, angle histogram (collapse check), grid coverage holes, length outliers
- V1 audit: 229 eps · angle bins ~uniform · 25/25 cells · 0 outliers → PASS
- One warning ("0° bin LOW") we misread — it was the **0°/90° aliasing** announcing itself (next slide's punchline)
- In the automated overnight pipeline, the audit is the **decision-maker** (no human in the loop)

*Speaker notes: Lesson learned live: audit warnings get investigated, not rationalized. The 0/90 aliasing — a square block at 0° and 90° is the same physical scene with two different labels — slipped through and shows up in deployment as ~43° rotations on straight blocks. Fix is queued (V1.1): canonicalize labels + ~35 replacement episodes.*

---

## Slide 8 — Act 3: the policy (what actually gets trained)
- **ACT** (Action Chunking Transformer w/ CVAE), 51.6M params — from scratch, NOT fine-tuning (only ResNet18 backbone starts from ImageNet)
- In: wrist RGB (3×480×848) + 9-dim state [pose, aperture, force, wrist-Fz]
- Out: a **chunk of 100 future actions**, 7-dim each [target pose + gripper aperture-mm]
- Loss: L1 on the chunk + KL (CVAE) · batch 8 · 150k steps
- Normalization stats are computed from the dataset and FROZEN into the checkpoint — why V0/V1 data can't be mixed

*Speaker notes: If asked "why not fine-tune a big VLA": at 229 episodes and an 8 GB deployment GPU, ACT is the right operating point — trains in ~1.5 GPU-hours, runs at 6-7 ms/tick. SmolVLA fine-tune is the planned V2-era comparison. The task string is recorded but ACT ignores it — future-proofing for VLA swaps.*

---

## Slide 9 — Training on NSF ACCESS (1 slide — full deck exists)
- DeltaAI (NCSA), one GH200 node: **100k steps ≈ 1 hour**, ~1–2% of a small allocation per run
- Push dataset (rsync) → smoke-test one sample on the login node → sbatch → pull `checkpoints/last/pretrained_model/` back
- Fully scripted end-to-end: `auto_train_cycle.sh` = audit → push → train → pull, one Duo login (SSH ControlMaster)
- Details + pitfalls: previous presentation (NSF_ACCESS_SLIDES)

*Speaker notes: One sentence per bullet and move on — this audience saw the NSF deck two weeks ago. New since then: the whole cycle is now one command; the vision is collect→audit→train→pull as the robot's unattended night shift.*

---

## Slide 10 — Act 4: deployment (policy → real arm at 10 Hz)
**IMAGE: photo/short clip of the arm mid-pick on a rotated block (phone video still works great here)**
- Loop: observation → predict 100-action chunk → execute first 10 (1 s) → re-infer. **No temporal ensembling** (averaging rotation vectors is invalid near 180° — measured 3× worse)
- Safety envelope: XY box ±10 cm · Z floor at table+gripper+3 mm · tilt ≤25° with **yaw FREE** · rotation ≤12°/tick · step ≤22 mm
- Gripper mapping: policy commands aperture directly — <35 mm = force-close · 35–90 = pre-position · stall-retry ×3
- Division of labour: policy perceives/reaches/aligns/closes → scripted lift-carry-place (episodes end at lift, so must the policy's job)

*Speaker notes: Every constraint on this slide exists because removing it broke something specific: ensembling wrecked rotations, the old total-angle cap blocked the yaw alignment rotated blocks need, 2 ms servo bursts caused mechanical stutter. Inference is 6-7 ms per tick — the 2070 is nowhere near the bottleneck.*

---

## Slide 11 — Does it work? (measured: 100-grasp eval, 2026-07-13)
**IMAGE: figures/v1_eval_heatmap.png (full width — this is the money figure)**
- **97 / 100 picks, mean grade 0.70** — fully unattended protocol
- **Rotated blocks (25°/45°/70°): 100% (75/75)** — the exact case V0 froze on
- Straight blocks (0°): 88%, grade 0.58 — ALL 3 failures of the run, right where the audit warned
- No spatial falloff: 100% centre · 97% @3 cm · 97% @6 cm — designed coverage did its job
- Policy pre-positions its own fingers (86→59→55 mm staircase — nobody coded that)

*Speaker notes: Lead with 97 and the 75/75. Then point at the 0° column of the heat-map: the three red cells are the aliasing from slide 7 confirmed on hardware — two failures rotated ~41° off before closing. The failure mode was PREDICTED by the dataset audit before the policy ever ran. That's the methodology working: measure → predict → confirm → patch (V1.1).*

---

## Slide 12 — Measuring it properly: the 100-grasp graded eval
- 5×5 grid × 4 angles {0°, 25°, 45°, 70°} — same grid as collection · scripted stagehand repositions the block between rollouts → fully unattended
- Success requires surviving an 8 cm **lift-verify**; every grasp graded 0–1 (yaw error vs block angle, seated aperture, re-grip count)
- Failsafe ladder: any fail → full reset · 5 in a row → relaunch entire driver stack · hardware stalls never pollute the score
- Grading separates success from quality: 0° *succeeds* 88% but at 0.58 grade — success-rate alone would hide the aliasing
- Secondary finding: 94/97 picks needed ≥1 stall re-grip (AX-12 grip authority) — the main QUALITY limiter, mechanical not learned
**IMAGE: figures/v1_eval_summary.png (4-panel: success by angle, grade by angle, yaw-error hist, aperture hist)**

*Speaker notes: The eval instrument itself is a contribution: policies that learn to lift shook blocks loose mid-grade (fixed: freeze at close), a dying gripper once wrote 36 fake policy failures in an hour (fixed: hardware circuit breaker — reboot, retry, ledger stays clean), and clean picks stage the next combo themselves (57/100 direct placements). Same instrument re-runs unchanged for V1.1 and V2 — before/after heat-maps on identical protocol.*

---

## Slide 13 — What's next (the ladder)
**IMAGE: the 0° column of v1_eval_heatmap.png cropped, labelled 'V1.1 target' — before/after teaser**
- **V1.1** (days): fix 0/90 aliasing (canonical labels + ~35 replacement episodes) → retrain → re-eval on the SAME grid → before/after heat-maps
- **V2** (weeks): multi-object (fruit), 100 episodes each — tests whether the method transfers past blocks
- **Research thrusts** (fall): MuJoCo sim-twin data multiplication · continual-learning memory with the reward gate as write filter · combination paper (2×2 ablation on the same grid)
- Automation end-state: collect → audit → train → pull → eval, fully unattended overnight

*Speaker notes: The eval grid is the connective tissue: V1.1's before/after, V2's transfer, and both research thrusts all report on the same heat-map format. That's what makes the eventual paper's ablation table coherent.*

---

## Slide 14 — Summary
- A scripted expert + a physically-grounded reward gate = **training data with zero human time**
- Data design beats data volume: 229 designed episodes >> 60 random ones — every improvement traceable to a measured failure
- Same-day loop: collect (3.5 h) → train (1.5 h on ACCESS) → deploy → measure
- The policy does what the data taught: rotated-block picks that were impossible two weeks ago
- Everything versioned, audited, and reproducible: `docs/FULL_PIPELINE.md`

*Speaker notes: Close on the loop-closure: two weeks ago the deliverable was "a policy trained on the cluster." Today it's "a policy measurably better than the last one, because we measured WHY the last one failed and designed the data to fix it." Questions.*

---

## Backup slide A — The gripper war stories (if hardware questions come)
- AX-12 quirks: torque overload shutdown · clear_error leaves 2N (every motion after needs force re-assert) · calibrate service is placebo (absolute encoders) · real reset = driver reboot + verified open
- Aperture scale reads ~20 mm under reality — all thresholds in measured units
- Serial port contention ("Port is in use") under sustained load → circuit breaker in the eval

## Backup slide B — Exact training artifacts (if reproducibility questions come)
- `models/act_v1/`: config.json (architecture) · model.safetensors (197 MB) · pre/post processor stats · train_config.json (full provenance)
- Pins that matter: lerobot==0.4.4 (matches dataset writer) · numpy 1.26.4 · transformers<5
- Offline replay verification BEFORE the arm: 1.9 mm mean position error (V0)

## Backup slide C — Cost/throughput table
| Metric | V0 | V1 |
|---|---|---|
| Episodes | 60 | 229 |
| Collection | 1.9 h | 3.4 h |
| Episodes/hour | 31.6 | 66.7 |
| Train time (GH200) | 62 min | ~90 min |
| Median episode | 39.6 s | 11.2 s |
| Worst angle bin | 71% | 19.7% |
| Deploy: rotated blocks | froze | **100% (75/75)** |
| Deploy: overall (100-grasp eval) | n/a | **97%, grade 0.70** |
