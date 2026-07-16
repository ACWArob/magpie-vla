# AutoGrasp — Summer Research Talk (10 min) — Slide Deck Source

> 18 slides ≈ 31 s each (60 s for the background slide) ≈ 9.8 min + questions. AUDIENCE: assume no robotics/ML background — slide 2 carries the vocabulary. Format: `## Slide N` blocks — title,
> bullets, speaker notes — paste into Gemini/Claude/Slides as-is.
> Framing: a RESEARCH TIMELINE (what I asked → what I measured → what I learned),
> not a systems walkthrough. Image URLs are live (public repo). `[VIDEO]` = your saved clips.

---

## Slide 1 — Title (0:00)
**AutoGrasp: A Robot That Generates, Judges, and Fixes Its Own Training Data**
- Summer 2026 · UR5e + custom MAGPIE gripper · one wrist camera · one undergrad
- Question I started with: *can a robot collect its own imitation-learning data — no
  teleoperation, no human labels — and get better because of how the data is DESIGNED?*
- Answer (spoiler): 97/100 picks, and the most useful result was a failure

*Speaker notes: 15 seconds on the question, then "here's the summer as a timeline." Sets the research framing immediately — this is about data, measurement, and one diagnosed failure, not a demo reel.*

---

## Slide 2 — Background: how a robot learns to act (the 60-second primer) (0:35)
- **How it acts (the process):** a robot "policy" is a **see → decide → move** loop — camera image in, motor command out, ~10× a second
- **How it learns:** by **imitation** — copying many examples of the task done right (like learning to drive by watching, *not* by crashing). No trial-and-error, no reward tuning
- **What a "VLA" is** (you may have heard the term): **Vision-Language-Action** — the popular kind also reads a text instruction ("pick up the red block"). Ours is a lean cousin called ACT, same family
- **The catch:** every example normally comes from a **human teleoperating** the robot — hundreds of demos, hours of human time
- **This summer's question: what if the robot makes its own examples — and can we make them GOOD?**

IMAGE (fills the slide): https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/talk/intro_vla.png

*Speaker notes: 55-60 seconds, unhurried — the diagram carries it. Point at panel 1: "this is all the robot does — see, decide, move, ten times a second." Panel 2: "the models you've heard of, VLAs, also take a language command; ours is a simpler one in the same family." Then the punchline that sets up the whole talk: "it learns by copying examples — so the examples ARE the product. This summer was about the robot making its own."*

---

## Slide 3 — The timeline at a glance (1:35)
**May → mid-June: build · late June: the data engine · July 7: first policy · July 8: designed data · July 13: 97/100 + a root-caused failure**
- Draw as a horizontal timeline bar with 5 milestones (build it in Slides — 5 dots, one label each)
- Every claim in this talk has a number behind it; all measured on real hardware

*Speaker notes: 20 seconds. "I'll spend the least time on the infrastructure and the most time on what the data taught us." Point at July 13 — "that's where the interesting science is."*

---

## Slide 4 — May–June: the platform (2:06)
- ROS2 stack: UR5e arm, custom 2-finger gripper (hobby servos — they fight back), wrist RGBD, force-torque
- Perception on an 8 GB GPU: SAM3 text-query segmentation + GraspGenX grasp proposals — coexisting in VRAM by measurement, not hope
- Everything driven from notebooks; every sensor's poll rate measured before trusting it (June 17 update)

IMAGE (right half): https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/sam3_wrist_overlay.png

*Speaker notes: 35 seconds, no more. One war story if pacing allows: the gripper's servos silently lose torque after an error-clear — that single quirk cost more debugging than any ML component. "Real hardware taxes everything."*

---

## Slide 5 — Late June: AutoGrasp, the data engine (2:37)
- A scripted expert picks the object: segment → center → choose grasp angle from geometry → force-controlled close → verified lift
- **The reward gate — the idea I care about**: an episode enters the dataset ONLY if
  (a) the object measurably stayed held through the lift, AND (b) a vision-language judge scores the grasp ≥ 0.6
- The robot resets the scene itself → **66.7 episodes/hour, zero human minutes**

[VIDEO: 15-20s clip of one full autonomous collection cycle — pick, judge, place at next spot]

*Speaker notes: This slide is the method contribution. "Physically-grounded self-labeling: the block either stayed in the gripper or it didn't — the VLM only refines that." Save rate at scale: 99.1% over 234 attempts.*

---

## Slide 6 — July 7: first policy — and the first lesson (3:08)
- 60 episodes, randomly placed → ACT policy (51.6M), trained on an NSF supercomputer in 62 minutes
- Same-day deployment: autonomous visual picks on the real arm ✓
- Offline replay error: **1.9 mm** — the model imitates its data almost perfectly
- So we're done? No.

*Speaker notes: 30 seconds. Land the setup: "the model was never the problem." Next slide is the reveal. If you have the V0 first-pick video, 5 seconds of it here.*

---

## Slide 7 — What does "training" actually optimize? (the metric) (3:39)
- The network is trained to minimize **imitation error**: for each moment in a demonstration, predict the next ~10 seconds of motion — the loss is the **distance between predicted and demonstrated actions** (arm pose + gripper aperture, an L1 loss)
- What we watch during training: the loss curve (ours converges ~0.04 in normalized units) — and after training, **replay error in physical units**: median **1.8 mm position / 1.1° rotation / 0.16 mm gripper** against held-out demonstration frames
- **The catch (remember this)**: both numbers measure *how well the policy copies* — neither measures whether the task succeeds

*Speaker notes: 35 seconds. This slide earns its place twice: (1) the audience needs to know what the number being optimized IS — "distance to the demonstrated motion, nothing about success or reward"; (2) it plants the bomb that goes off twice later — a policy can copy near-perfectly and still fail (slide 8), and copying-accuracy metrics are provably blind to the failure we found (slide 15). Say the last bullet slowly.*

---

## Slide 8 — Finding #1: policies interpolate; they don't extrapolate (4:10)
- Picks succeed ONLY inside the training placement zone; beyond it, the descent regresses to the training mean → miss
- And on rotated blocks the policy **froze completely**
- Forensics: **71% of all training grasps executed at ~90°** regardless of block angle — when demonstrations disagree, imitation averages them into inaction

IMAGE: https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/v0/fig1_angle_collapse.png

*Speaker notes: 40 seconds — this is the first research beat. "Mode averaging: rotate-vs-don't averaged together is neither." The histogram IS the diagnosis; deployment behavior was predictable from data statistics alone. That idea drives the whole rest of the summer.*

---

## Slide 9 — July 8: design the data like an experiment (4:41)
| Measured V0 defect | V1 design response |
|---|---|
| interpolation-only | 5×5 grid × 7 angles + jitter — designed support |
| 71% angle collapse | deterministic geometric angle rule |
| binary gripper hid finger pre-positioning | continuous aperture action |
| 40% off-task frames | episodes end at lift |
- 229 episodes in 3.4 h — **43 minutes of actual robot trajectory, 442 MB**

*Speaker notes: 40 seconds. "Every row is a measured failure mapped to a designed fix — nothing aesthetic." The 43-minutes number lands well: the whole policy fits in less robot time than a lunch break.*

---

## Slide 10 — The action-space point (worth its own 30 seconds) (5:12)
- The old action space was gripper open/close (binary). The expert physically pre-positions its fingers — the policy **could not even represent** that event
- New: gripper aperture in millimeters as a continuous action
- The trained policy then reproduced the pre-positioning staircase ON ITS OWN — nobody coded it

IMAGE: https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/appendix/offline_aperture_staircase.png

*Speaker notes: Rule extracted: "the action space must contain every event the policy must reproduce." Measured proof the old one couldn't: the V0 policy's gripper output spans [0, 1.01] no matter what you show it.*

---

## Slide 11 — Measuring properly: the evaluation instrument (5:43)
- 100 grasps, fully unattended: 25 positions × 4 block angles, the SAME grid as collection
- Success requires surviving an 8 cm lift; every grasp GRADED 0–1 (alignment, seating, retries)
- Hardware faults are quarantined (reboot + retry, never recorded) — a dying gripper once wrote 36 fake "policy failures" in an hour

[VIDEO: 10-15s time-lapse of the eval running unattended]

*Speaker notes: 35 seconds. "The instrument is a contribution too — it re-runs identically for every future policy, so results stay comparable forever." Grading vs pass/fail matters on the next slide.*

---

## Slide 12 — July 13: the result (6:14)
- **97 / 100 picks** (95% CI 91.5–99.0) · mean grade 0.70
- **Rotated blocks: 75/75 — the exact case the first policy froze on**
- No spatial falloff: 97% at the workspace edge
- All from 43 minutes of self-collected robot data

IMAGE (full width): https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/v1_eval_heatmap.png

*Speaker notes: SLOW DOWN HERE — this is the money slide, give it its full 40 seconds. Point at the three red cells: "and I want to talk about those three failures, because they're the best part."*

---

## Slide 13 — Where the scores come from: two scorekeepers, zero humans (6:45)
**Scorekeeper 1 — during COLLECTION (curates the data): a vision-language model**
- Shown photos of the grasp (at close + after lift), asked — real prompt, condensed:
  *"From the after-lift image, is the object still held? SCORE 0.0–1.0. Name the primary
  problem (none / angle / placement / centering / stability). Measure how the object sits
  between the fingers…"* — temperature 0, so the same photos always score the same
- Episode kept ONLY if physically held **and** score ≥ 0.6 · mean kept score: 0.865

**Scorekeeper 2 — during EVALUATION (grades the policy): pure arithmetic, no AI**
- start at 1.0 → **−0.1** per extra close attempt → up to **−0.4** for gripper-vs-block
  angle error → **−0.2** if not seated (finger gap outside 20–40 mm) → **0** if it doesn't
  survive the 8 cm lift
- **The 0.70 = the average of 100 such grades.** Every input is a sensor reading

*Speaker notes: 40 seconds — this slide answers "where does 0.70 come from" before anyone asks. The punchline matters for credibility: the headline numbers (97/100, grade 0.70) contain NO learned or VLM component — pure measured physics — so "the AI graded itself" is not a possible objection. The VLM only curates training data upstream, and even there it's anchored to a physical fact (the block stayed held or it didn't).*

---

## Slide 11b — But where does it STOP working? (the competence boundary)
- I tested one grid-step OUTSIDE the training zone (±9 cm instead of ±6 cm)
- **97% inside → 8% outside (8/96).** A hard cliff, not a gentle fade — and the few that worked were rotated so their centroid fell back near the trained zone
- The policy interpolates within its data and does not extrapolate past it — 9 cm is well within the arm's reach, so this is a *learned* limit, not a mechanical one

IMAGE: https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/appendix/ood_ring.png

*Speaker notes: 30 seconds. This is the honest boundary of the result — and it's WHY the systematic grid matters: the grid is exactly the region the policy works in. "To make the workspace bigger, you don't hope for generalization — you collect more grid." Ties the coverage design to a measured limit.*

---

## Slide 14 — The three failures: all at 0° — and we predicted them (7:16)
- The dataset audit had flagged "0° bin LOW" before training — we misread it as benign
- All 3 failures at 0°, two rotated ~41° off before grasping
- 0° *succeeds* 88% of the time but at 0.58 quality — **success-rate alone would have hidden this; the grading exposed it**

IMAGE: https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/appendix/v1_eval_0deg_column.png

*Speaker notes: 40 seconds. The narrative beat: measure → predict → confirm. "The failure mode was visible in the data distribution before the policy ever ran."*

---

## Slide 15 — Root cause: one discontinuity (7:47)
- A square block at 0° and 90° is the SAME scene — but the angle code works modulo 90, which is discontinuous at that boundary: a straight block reads 0.3° or 89.7° on pixel noise alone
- So the expert demonstrated BOTH "stay" and "rotate 90°" on identical scenes: **47 episodes vs 15 — perfectly bimodal**
- Imitation averaged the two modes → the ~43° freeze

*Speaker notes: 40 seconds, the deepest technical beat. "We traced a deployed-policy behavior all the way down to a single line of geometry code." If asked how found: executed-rotation histogram over the dataset — two clean spikes, nothing between.*

---

## Slide 16 — Finding #2 (the one that generalizes): offline metrics are blind to this (8:18)
- Replay error: 1.8 mm — clean. Chunk-level probes from the ambiguous start frames: predict the CORRECT angle even on 0° episodes
- **No offline metric we could construct shows the defect.** It exists only closed-loop
- ⇒ dataset distribution audits + graded hardware evaluation are load-bearing; offline accuracy is necessary but cannot catch mode-averaging

*Speaker notes: 35 seconds. This is the research claim a general audience should take home. "If you only validate policies offline, this entire failure class is invisible to you."*

---

## Slide 17 — The fix, pre-registered (8:49)
- Boundary canonicalization (one rule: near-90° readings on symmetric objects execute as ≈0°) + the angle-consistency gate now actually enforced + ~50 replacement episodes
- Success criteria written down BEFORE running: 0°→25/25, grade ≥0.75, zero regression elsewhere
- Re-run the identical 100-grasp instrument → before/after heat-maps *(insert result if run by talk day)*

*Speaker notes: 30 seconds. "Pre-registering the criteria keeps us honest — the fix has to clear a bar we set before seeing its results." If V1.1 results exist by the talk: THIS becomes the closing money slide.*

---

## Slide 18 — Summary + what's next (9:20)
- **A robot collected 43 minutes of its own data, judged it itself, and the resulting policy picks at 97%** — including the rotations its predecessor failed
- The best result was a failure: predicted by an audit, invisible offline, root-caused to one line
- Next: workshop paper (drafted) · multi-object + placement (the strawberry test) · continual-learning memory on top of this loop
- Repo: github.com/ACWArob/magpie-vla — every number in this talk is reproducible from it

[VIDEO: closing 10s — best rotated-block pick, full speed]

*Speaker notes: End at ~9:30, leaving buffer. Last line: "The robot's data got better because we measured why it was bad — that's the whole method." Questions.*

---

## Backup slide — Everything is a measured decision (8:49)
- Every fork of the summer — which detector, which grasp planner, which architecture, why no temporal ensembling — recorded as chosen-vs-rejected WITH the number that decided it

IMAGE (full width): https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/appendix/gate_scoreboard.png

*Speaker notes: 25 seconds, light touch: "I won't walk through it — the point is the habit. Every 'why did you use X' has a quantitative answer." It's also the appendix of the workshop paper in progress.*

---

## Assets checklist
- Images: all URLs above are live (raw.githubusercontent, branch ros)
- Your videos: (1) full collection cycle ~15s → slide 4 · (2) eval time-lapse ~10s → slide 9 · (3) best rotated pick ~10s → slide 16 · optional V0 first-pick → slide 5
- If V1.1 eval lands before the talk: add its 0° column next to slide 14 as before/after
