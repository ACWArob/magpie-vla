# SPUR Final Talk — AutoGrasp (8 min, SPUR template format)

> Follows the SPUR skeleton: Title → Introduction → Methods → Results → Conclusions →
> Acknowledgements → References, with the section-nav sidebar on the right. **Minimal text**
> (2–3 short bullets max per slide — the image carries it). 8 min ≈ 1 slide/min → 9 content
> slides + title + ack + refs. Colours: **burgundy #7A2E39 · orange #D97A1E · beige #E6D8BF**
> (the recolored `figures/spur/` images already match). Images are live raw-GitHub links.
> Base: `https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/`

---

## Slide 1 — TITLE  [nav: Title]
**Teaching a Robot to Collect Its Own Training Data**
- [Presenter Name]
- Correll Lab · Department of Computer Science
- University of Colorado Boulder
- Mentor: William Xie · PI: Prof. Nikolaus Correll

IMAGE (right side, like the SPUR title layout): `spur/competence.png`
*(or a photo/still of the arm+gripper if you have one — a real robot photo is stronger)*

*Notes: 20s. "This summer I taught a robot to make its own training data — and I want to show you it worked, and the one place it didn't."*

---

## Slide 2 — INTRODUCTION  [nav: Introduction]
**Robots learn by copying — but who makes the examples?**
- Robots learn to grasp by **imitation**: copy many demonstrations
- Normally a **human teleoperates** the robot for every example — hours of human time
- **My question: can the robot make its OWN examples — and make them good?**

IMAGE: `spur/intro_vla.png`

*Notes: 60s — the primer. See→decide→move, learns by copying, the term "VLA." Land: "the examples ARE the product."*

---

## Slide 3 — METHODS  [nav: Methods]
**AutoGrasp: a robot that generates and grades its own data**
- A scripted "expert" picks the object and **judges each grasp itself**
- An episode is kept only if the object **stayed held** AND a vision model scores it **≥ 0.6**
- **67 grasps / hour · zero human labels**

IMAGE: `spur/pipeline.png`

*Notes: 50s. The reward gate is the idea — "the block either stayed in the gripper or it didn't."*

---

## Slide 4 — METHODS  [nav: Methods]
**I designed the data like an experiment**
- Systematic **grid** of positions × angles — not random placement
- Every failure of the first version mapped to a specific fix

IMAGE: `spur/coverage.png`

*Notes: 40s. "Random placement leaves holes; a grid guarantees the robot sees every case."*

---

## Slide 5 — RESULTS  [nav: Results]
**97 out of 100 grasps**
- Trained on **43 minutes** of the robot's own data
- Colour = grasp quality across the whole workspace

IMAGE (full width): `spur/heatmap.png`

*Notes: 45s — the money slide. Slow down. "97 out of 100 — and I want to show you those failures in a second."*

---

## Slide 6 — RESULTS  [nav: Results]
**The case the first version couldn't do**
- Version 0 (random data) **froze** on rotated blocks
- Version 1 (designed data): **75 / 75** — same model, only the data changed

IMAGE: `spur/v0_v1.png`

*Notes: 40s. "Identical network, identical training — the only difference is the data. That's the whole point."*

---

## Slide 7 — RESULTS  [nav: Results]
**And where it still fails**
- One grid-step **outside** the trained zone → **8 / 96** (a hard cliff)
- The policy **interpolates** within its data; it does **not extrapolate**

IMAGE: `spur/competence.png`

*Notes: 40s — the honest finding. "This isn't a bug, it's the measured boundary of what the data taught it."*

---

## Slide 8 — CONCLUSIONS  [nav: Conclusions]
**What I found**
- A robot collected its own data, judged it, and reached **97%** — no human labels
- **Data design mattered more than data volume**
- The most useful result was a **measured failure**

*Notes: 35s. No image — let the words land. This is the takeaway slide.*

---

## Slide 9 — CONCLUSIONS  [nav: Conclusions]
**What's next**
- Fix the remaining failure — remove absolute position from the robot's input so it relies on vision
- More objects (fruit), and placing them at a target
- Toward a robot that keeps getting better on its own

*Notes: 25s. Optional — merge into slide 8 if you're over time.*

---

## Slide 10 — ACKNOWLEDGEMENTS  [nav: Acknowledgements]
- **William Xie** — mentor, Correll Lab
- **Prof. Nikolaus Correll** — PI, Correll Lab
- **SPUR Program** · Engineering Excellence Fund *(include if your SPUR was donor-funded — check)*
- **NSF ACCESS (DeltaAI)** — supercomputer time

*Notes: 15s. Always thank whoever funded you.*

---

## Slide 11 — REFERENCES  [nav: References]
**Tools & models used:**
- SAM3 — segmentation (Meta) · GraspGenX — grasp planning (NVlabs) · DeliGrasp — grip force
- Gemini 2.5 Flash — detection + quality judge (Google) · ACT — the policy (Zhao et al. 2023)
- LeRobot — dataset + training (Hugging Face)

**Ideas:** MimicGen (Mandlekar 2023) · Data Scaling Laws (Lin 2024)

*Notes: don't read these — on the slide for credit. This slide honestly reflects how much we
stood on: six real tools end-to-end.*

---

## Build notes
- Use the **SPUR template** structure (section-nav sidebar). **Skip the 'Engineering Career
  Hub' footer** — that's the career center's branding on their teaching slides; the student
  EXAMPLE in the packet has no footer. Keep slide numbers. The sidebar is optional (the
  example didn't use it) but signals structure nicely.
- Keep the recolored `spur/` images — they're already burgundy/beige/orange to match.
- Two videos, if you record them: a collection cycle → slide 3; a rotated-block pick → slide 6.
- 8-min cut if tight: drop slide 9 (fold into 8) and slide 4 (fold the grid idea into slide 3).
