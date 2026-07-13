# Portfolio case-study prompt — paste into Claude

*Generated 2026-07-13. All image URLs verified live (public repo ACWArob/magpie-vla,
branch ros). Swap the format line if your portfolio uses something other than HTML.*

```
Build me a single-page portfolio case study (self-contained HTML, mobile-friendly,
clean/minimal — white background, dark navy text #1B2430, orange accent #D9560B). I'm an
undergraduate robotics researcher; this is the flagship project on my portfolio. Write it
in first person, confident but factual — every number below is real and measured.

TITLE: "Teaching a Robot to Teach Itself: from a ROS2 stack to a 97% grasp policy"

STRUCTURE — a vertical timeline with 6 chapters, each with 1-2 images (URLs below),
a short story paragraph, and a "what I learned" line:

CHAPTER 1 — The foundation (May 2026)
Modernized a UR5e + custom MAGPIE gripper stack onto ROS2: arm control, gripper driver,
wrist RealSense camera, force-torque sensing, all driven from Jupyter. Built the
perception stack: SAM3 segmentation (squeezed onto an 8GB RTX 2070 via a subprocess
bridge), point clouds, grasp-pose planning with NVIDIA's GraspGenX (I contributed our
gripper's embodiment).
Image: https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/sam3_wrist_overlay.png
(caption: live SAM3 segmentation from the wrist camera, 0.96 confidence)

CHAPTER 2 — The robot that grades its own homework (June 2026)
Built "AutoGrasp": a scripted expert that picks objects using force-adaptive grasping
(DeliGrasp — VLM physics priors), then JUDGES each attempt itself: an episode only enters
the training dataset if the object measurably stayed held AND a vision-language judge
scores the grasp >= 0.6. Result: 66.7 training episodes per hour with zero human
minutes — no teleoperation, no labeling.

CHAPTER 3 — The honest failure that taught the most (early July)
First policy (60 random-placement episodes -> ACT, trained in 62 minutes on an NSF
supercomputer): replayed its training data at 1.9mm accuracy... and froze on rotated
blocks in the real world. Forensics: 71% of the training grasps were at one angle —
imitation learning averages conflicting demonstrations into inaction. The model was
never the problem. The data was.
Image: https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/v0/fig1_angle_collapse.png
(caption: the smoking gun — the angle histogram that explained the freeze)

CHAPTER 4 — Designing data like an engineer (July 8)
Rebuilt collection around the measured failures: a 5x5 position grid x 7 angles with
jitter instead of random placement; a continuous gripper-aperture action space (the old
binary one literally could not express finger pre-positioning — over a third of every
trajectory); episodes end at lift. 229 episodes in one afternoon at 2.1x the speed,
audited before anything ships to the cluster.
Image: https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/appendix/offline_aperture_staircase.png
(caption: the new action space — the policy later reproduced this staircase on its own)

CHAPTER 5 — 97/100 (July 13)
The new policy, evaluated by a fully-unattended instrument I built (the robot stages the
block, the policy attempts the pick, an 8cm lift verifies it, every grasp graded 0-1,
hardware faults quarantined so they never pollute the score): 97/100 picks, including
100% (75/75) on rotated blocks — the exact case the old policy failed. And the 3 failures?
All at 0 degrees, exactly where the dataset audit had warned — a label-aliasing bug I then
traced to a single mod-90 discontinuity in the angle code, measured (47 vs 15 episode
bimodality), and patched with pre-registered success criteria.
Images: https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/v1_eval_heatmap.png
(caption: 100 graded grasps, colour = quality)
https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/appendix/fig1_pipeline.png
(caption: the full loop — collection, judging, training and evaluation share one grid protocol)

CHAPTER 6 — Everything is measured (ongoing)
Every design decision in the pipeline — which detector, which grasp planner, which
architecture, why no temporal ensembling — is documented with the number that decided it,
chosen-vs-rejected. Currently: workshop paper in preparation; next: multi-object
generalization and a continual-learning memory.
Image: https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/appendix/gate_scoreboard.png
(caption: the decision scoreboard — every fork in the project, quantified)

FOOTER: link to github.com/ACWArob/magpie-vla + "built with UR5e, ROS2, LeRobot, ACT,
SAM3, GraspGenX, NSF ACCESS (DeltaAI)". Add a small stats strip near the top:
"97/100 picks · 75/75 rotated · 66.7 episodes/hr unattended · 0 human labels · 1.5 GPU-hours to train"

Keep total reading time ~3 minutes. The images carry the story; don't pad the text.
```
