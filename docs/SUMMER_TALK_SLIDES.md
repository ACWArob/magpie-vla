# AutoGrasp — Summer Research Talk (10 min) — Slide Deck Source

> 17 slides ≈ 33 s each (60 s for the background slide) ≈ 9.6 min + questions. AUDIENCE: assume no robotics/ML background — slide 2 carries the vocabulary. Format: `## Slide N` blocks — title,
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

## Slide 2 — Background: how robots learn from examples (60 seconds) (0:35)
- A **robot policy** = a neural network: camera image in → motion command out, 10×/second
- It learns by **imitation**: show it many examples of a task done right, it learns to copy — this is how most modern robot learning works (you may have heard "VLA models" — same family)
- The usual price: a human **teleoperates** the robot for every single example — hundreds of demonstrations, hours of human time
- **This summer's question: what if the robot makes its own examples — and can we make them GOOD?**

*Speaker notes: 50-60 seconds, unhurried — everything later depends on these four ideas: policy, imitation, demonstrations, and "data quality decides everything." Explicitly say: "no reinforcement learning here, no reward shaping — just copying good examples. Which means the examples ARE the product."*

---

## Slide 3 — The timeline at a glance (1:35)
**May → mid-June: build · late June: the data engine · July 7: first policy · July 8: designed data · July 13: 97/100 + a root-caused failure**
- Draw as a horizontal timeline bar with 5 milestones (build it in Slides — 5 dots, one label each)
- Every claim in this talk has a number behind it; all measured on real hardware

*Speaker notes: 20 seconds. "I'll spend the least time on the infrastructure and the most time on what the data taught us." Point at July 13 — "that's where the interesting science is."*

---

## Slide 4 — May–June: the platform (2:08)
- ROS2 stack: UR5e arm, custom 2-finger gripper (hobby servos — they fight back), wrist RGBD, force-torque
- Perception on an 8 GB GPU: SAM3 text-query segmentation + GraspGenX grasp proposals — coexisting in VRAM by measurement, not hope
- Everything driven from notebooks; every sensor's poll rate measured before trusting it (June 17 update)

IMAGE (right half): https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/sam3_wrist_overlay.png

*Speaker notes: 35 seconds, no more. One war story if pacing allows: the gripper's servos silently lose torque after an error-clear — that single quirk cost more debugging than any ML component. "Real hardware taxes everything."*

---

## Slide 5 — Late June: AutoGrasp, the data engine (2:41)
- A scripted expert picks the object: segment → center → choose grasp angle from geometry → force-controlled close → verified lift
- **The reward gate — the idea I care about**: an episode enters the dataset ONLY if
  (a) the object measurably stayed held through the lift, AND (b) a vision-language judge scores the grasp ≥ 0.6
- The robot resets the scene itself → **66.7 episodes/hour, zero human minutes**

[VIDEO: 15-20s clip of one full autonomous collection cycle — pick, judge, place at next spot]

*Speaker notes: This slide is the method contribution. "Physically-grounded self-labeling: the block either stayed in the gripper or it didn't — the VLM only refines that." Save rate at scale: 99.1% over 234 attempts.*

---

## Slide 6 — July 7: first policy — and the first lesson (3:14)
- 60 episodes, randomly placed → ACT policy (51.6M), trained on an NSF supercomputer in 62 minutes
- Same-day deployment: autonomous visual picks on the real arm ✓
- Offline replay error: **1.9 mm** — the model imitates its data almost perfectly
- So we're done? No.

*Speaker notes: 30 seconds. Land the setup: "the model was never the problem." Next slide is the reveal. If you have the V0 first-pick video, 5 seconds of it here.*

---

## Slide 7 — Finding #1: policies interpolate; they don't extrapolate (3:47)
- Picks succeed ONLY inside the training placement zone; beyond it, the descent regresses to the training mean → miss
- And on rotated blocks the policy **froze completely**
- Forensics: **71% of all training grasps executed at ~90°** regardless of block angle — when demonstrations disagree, imitation averages them into inaction

IMAGE: https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/v0/fig1_angle_collapse.png

*Speaker notes: 40 seconds — this is the first research beat. "Mode averaging: rotate-vs-don't averaged together is neither." The histogram IS the diagnosis; deployment behavior was predictable from data statistics alone. That idea drives the whole rest of the summer.*

---

## Slide 8 — July 8: design the data like an experiment (4:20)
| Measured V0 defect | V1 design response |
|---|---|
| interpolation-only | 5×5 grid × 7 angles + jitter — designed support |
| 71% angle collapse | deterministic geometric angle rule |
| binary gripper hid finger pre-positioning | continuous aperture action |
| 40% off-task frames | episodes end at lift |
- 229 episodes in 3.4 h — **43 minutes of actual robot trajectory, 442 MB**

*Speaker notes: 40 seconds. "Every row is a measured failure mapped to a designed fix — nothing aesthetic." The 43-minutes number lands well: the whole policy fits in less robot time than a lunch break.*

---

## Slide 9 — The action-space point (worth its own 30 seconds) (4:53)
- The old action space was gripper open/close (binary). The expert physically pre-positions its fingers — the policy **could not even represent** that event
- New: gripper aperture in millimeters as a continuous action
- The trained policy then reproduced the pre-positioning staircase ON ITS OWN — nobody coded it

IMAGE: https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/appendix/offline_aperture_staircase.png

*Speaker notes: Rule extracted: "the action space must contain every event the policy must reproduce." Measured proof the old one couldn't: the V0 policy's gripper output spans [0, 1.01] no matter what you show it.*

---

## Slide 10 — Measuring properly: the evaluation instrument (5:26)
- 100 grasps, fully unattended: 25 positions × 4 block angles, the SAME grid as collection
- Success requires surviving an 8 cm lift; every grasp GRADED 0–1 (alignment, seating, retries)
- Hardware faults are quarantined (reboot + retry, never recorded) — a dying gripper once wrote 36 fake "policy failures" in an hour

[VIDEO: 10-15s time-lapse of the eval running unattended]

*Speaker notes: 35 seconds. "The instrument is a contribution too — it re-runs identically for every future policy, so results stay comparable forever." Grading vs pass/fail matters on the next slide.*

---

## Slide 11 — July 13: the result (5:59)
- **97 / 100 picks** (95% CI 91.5–99.0) · mean grade 0.70
- **Rotated blocks: 75/75 — the exact case the first policy froze on**
- No spatial falloff: 97% at the workspace edge
- All from 43 minutes of self-collected robot data

IMAGE (full width): https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/v1_eval_heatmap.png

*Speaker notes: SLOW DOWN HERE — this is the money slide, give it its full 40 seconds. Point at the three red cells: "and I want to talk about those three failures, because they're the best part."*

---

## Slide 12 — The three failures: all at 0° — and we predicted them (6:32)
- The dataset audit had flagged "0° bin LOW" before training — we misread it as benign
- All 3 failures at 0°, two rotated ~41° off before grasping
- 0° *succeeds* 88% of the time but at 0.58 quality — **success-rate alone would have hidden this; the grading exposed it**

IMAGE: https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/appendix/v1_eval_0deg_column.png

*Speaker notes: 40 seconds. The narrative beat: measure → predict → confirm. "The failure mode was visible in the data distribution before the policy ever ran."*

---

## Slide 13 — Root cause: one discontinuity (7:05)
- A square block at 0° and 90° is the SAME scene — but the angle code works modulo 90, which is discontinuous at that boundary: a straight block reads 0.3° or 89.7° on pixel noise alone
- So the expert demonstrated BOTH "stay" and "rotate 90°" on identical scenes: **47 episodes vs 15 — perfectly bimodal**
- Imitation averaged the two modes → the ~43° freeze

*Speaker notes: 40 seconds, the deepest technical beat. "We traced a deployed-policy behavior all the way down to a single line of geometry code." If asked how found: executed-rotation histogram over the dataset — two clean spikes, nothing between.*

---

## Slide 14 — Finding #2 (the one that generalizes): offline metrics are blind to this (7:38)
- Replay error: 1.8 mm — clean. Chunk-level probes from the ambiguous start frames: predict the CORRECT angle even on 0° episodes
- **No offline metric we could construct shows the defect.** It exists only closed-loop
- ⇒ dataset distribution audits + graded hardware evaluation are load-bearing; offline accuracy is necessary but cannot catch mode-averaging

*Speaker notes: 35 seconds. This is the research claim a general audience should take home. "If you only validate policies offline, this entire failure class is invisible to you."*

---

## Slide 15 — The fix, pre-registered (8:11)
- Boundary canonicalization (one rule: near-90° readings on symmetric objects execute as ≈0°) + the angle-consistency gate now actually enforced + ~50 replacement episodes
- Success criteria written down BEFORE running: 0°→25/25, grade ≥0.75, zero regression elsewhere
- Re-run the identical 100-grasp instrument → before/after heat-maps *(insert result if run by talk day)*

*Speaker notes: 30 seconds. "Pre-registering the criteria keeps us honest — the fix has to clear a bar we set before seeing its results." If V1.1 results exist by the talk: THIS becomes the closing money slide.*

---

## Slide 16 — Everything is a measured decision (8:44)
- Every fork of the summer — which detector, which grasp planner, which architecture, why no temporal ensembling — recorded as chosen-vs-rejected WITH the number that decided it

IMAGE (full width): https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/appendix/gate_scoreboard.png

*Speaker notes: 25 seconds, light touch: "I won't walk through it — the point is the habit. Every 'why did you use X' has a quantitative answer." It's also the appendix of the workshop paper in progress.*

---

## Slide 17 — Summary + what's next (9:17)
- **A robot collected 43 minutes of its own data, judged it itself, and the resulting policy picks at 97%** — including the rotations its predecessor failed
- The best result was a failure: predicted by an audit, invisible offline, root-caused to one line
- Next: workshop paper (drafted) · multi-object + placement (the strawberry test) · continual-learning memory on top of this loop
- Repo: github.com/ACWArob/magpie-vla — every number in this talk is reproducible from it

[VIDEO: closing 10s — best rotated-block pick, full speed]

*Speaker notes: End at ~9:30, leaving buffer. Last line: "The robot's data got better because we measured why it was bad — that's the whole method." Questions.*

---

## Assets checklist
- Images: all URLs above are live (raw.githubusercontent, branch ros)
- Your videos: (1) full collection cycle ~15s → slide 4 · (2) eval time-lapse ~10s → slide 9 · (3) best rotated pick ~10s → slide 16 · optional V0 first-pick → slide 5
- If V1.1 eval lands before the talk: add its 0° column next to slide 14 as before/after
