# Claude prompts — two matching PowerPoint decks (total ≤15 min)

Generated 2026-07-13. Paste each into claude.ai separately. Images are raw GitHub links
(public repo ACWArob/magpie-vla, branch ros) — all verified reachable.

## Prompt 1 — Pipeline deck (~9 min, 9 slides)

```
Make me a Microsoft PowerPoint (.pptx) presentation, 9 slides, to present in ~9 minutes.

DESIGN (use exactly this, it must match a second deck I'm making):
- Clean white background, dark navy text (#1B2430), safety-orange accent (#D9560B) for titles/highlights
- One idea per slide, max 4 bullets, big readable text (min 18pt body)
- Numbers and metrics in a monospace font, bold
- No clipart, no emoji, no gradients

CONTENT — "AutoGrasp → Data → Policy: a robot that trains itself":

Slide 1 (title): "A Robot That Generates Its Own Training Data" — scripted expert → self-judged dataset → ACT policy trained on NSF ACCESS → 97/100 picks. Zero teleoperation, zero human labels.

Slide 2 (the loop): Scripted expert picks blocks (real UR5e arm) → reward gate (VLM judge ≥0.6 AND physically held) → LeRobot dataset → train on DeltaAI supercomputer (~1.5h) → deploy at 10Hz → evaluate on the same grid. Draw this as a simple cycle diagram with boxes and arrows.

Slide 3 (the expert): SAM3 segmentation → center over block → pick angle from block geometry → grasp with force control → episode auto-saved only if it passes the quality gate. 99.1% save rate over 231 attempts, fully hands-free.
Image: https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/sam3_wrist_overlay.png

Slide 4 (V0, the honest failure): 60 random-placement episodes trained fine (replay error 1.9mm) but deployment failed 3 measurable ways: only works inside the training zone; 42/62 episodes grasped at 90° regardless of block angle → policy FROZE on rotated blocks; 40% wasted footage.
Image: https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/v0/fig1_angle_collapse.png

Slide 5 (V1 data design): every V0 failure got a designed answer — 5×5 grid × 7 angles (designed coverage), deterministic angle labels (worst bin 71%→19.7%), gripper aperture as a continuous action, episodes end at lift (39.6s→11.2s). 229 episodes at 66.7/hour, 2.1× faster.
Image: https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/v0_v1/compare_angles_lengths.png

Slide 6 (the audit): data ships to the cluster only if it passes checks (count, angle histogram, coverage holes). One warning — "0° bin LOW" — we misread; it predicted the exact deployment failure. Lesson: audit warnings get investigated.
Image: https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/v1_audit_angles.png

Slide 7 (RESULTS — the money slide): 100-grasp evaluation, fully unattended. 97/100 picks. Rotated blocks: 100% (75/75) — the exact case V0 froze on. All 3 failures at 0° — exactly where the audit warned. No spatial falloff (97% at the grid edge).
Image (full width): https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/v1_eval_heatmap.png

Slide 8 (evidence detail): success by angle, quality grades, alignment error — the grading separates "picked it up" from "picked it up well" (0° succeeds 88% but at 0.58 quality — the aliasing shows in quality first).
Image: https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/v1_eval_summary.png

Slide 9 (next): V1.1 = fix the 0° label aliasing, ~35 replacement episodes, retrain, re-run the SAME eval → before/after heat-maps. Then V2 multi-object. Long-term: simulation twin + continual-learning memory.
Image: https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/v1_eval_0deg_column.png

Add one-sentence speaker notes per slide.
```

## Prompt 2 — NSF deck (~5 min, 5 slides)

```
Make me a Microsoft PowerPoint (.pptx) presentation, 5 slides, to present in ~5 minutes.
It follows another deck in the same session, so it must match this design EXACTLY:

DESIGN:
- Clean white background, dark navy text (#1B2430), safety-orange accent (#D9560B) for titles/highlights
- One idea per slide, max 4 bullets, big readable text (min 18pt body)
- Numbers and metrics in a monospace font, bold
- No clipart, no emoji, no gradients

CONTENT — "Training on NSF ACCESS: lab → supercomputer → lab in one evening":

Slide 1 (what + why): NSF ACCESS = free, proposal-based time on national supercomputers. We use DeltaAI (NCSA, GH200 GPU nodes). Rule of thumb: local machine develops and deploys, the cluster trains. Our whole project used ~2% of a small allocation.

Slide 2 (the pipeline): prepare dataset locally → rsync up (Duo once, then a persistent SSH socket) → one-time environment setup → sbatch training job → rsync the trained policy back. First trip: one evening. Every trip after: ~20 minutes of attention. Draw as a simple 5-step flow.

Slide 3 (the 4 pitfalls we hit, so you don't): ARM CPUs mean pip installs a CPU-only torch (use the site's module); newest module ≠ best (an API we needed was removed); always "module load cuda"; the framework tried to upload our model to a public hub and killed the job in 33s (one flag fixes it).

Slide 4 (the 5-second rule): before submitting any overnight job, load ONE data sample interactively on the login node — it exercises the exact code path the job runs and catches ~90% of failures before the queue. Never trust an overnight job you haven't smoke-tested.

Slide 5 (results + automation): 150k training steps in ~1.5 hours on one GH200. The whole cycle is now ONE script: audit the data → push → train → pull → verify, riding a single morning login. The measured outcome of the policies it trained is in the previous deck (97/100).

Add one-sentence speaker notes per slide.
```
