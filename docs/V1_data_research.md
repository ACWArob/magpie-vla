# V1 Data Strategy — Research Review & Pipeline Audit (2026-07-08)

**Question:** for the red-block pick task, how should V1's ~200 episodes be distributed —
random placement (like V0) or systematic coverage of every position × angle — and what else
about our autograsp data generator should change to produce better policy training data?

**Method:** literature review (10 sources, 2021–2026) + audit of our own V0 evidence
(60 episodes, trained policy, measured deployment behavior).

---

## 1. What we already measured ourselves (V0 ground truth)

These are *our* datapoints, and everything in the literature should be read against them:

| V0 observation | Implication |
|---|---|
| Picks succeed **inside** the ±4cm training scatter; at 5–7cm the reach tracks but the descent regresses to the training mean → miss | Policy generalizes by **interpolation only** |
| 42/62 episodes grasped at ~90° regardless of block angle → policy **freezes** on a 45° block (averages "rotate" vs "don't" into inaction) | Conflicting supervision at similar states is **worse than no data** — imitation averages modes |
| Binary grip action; the finger pre-position lived outside the action space | Action space must contain **every event the policy must reproduce** |
| ~50% of each episode is carry/place footage; post-grasp phase ambiguity caused dithering | Episode should end when the *task* ends |
| Offline replay: 1.9mm position error on training data | The *model and training recipe are fine* — data is the bottleneck |

## 2. What the literature says

### 2.1 Coverage strategy: interpolation is all you get
- **[Data Scaling Laws in Imitation Learning](https://arxiv.org/abs/2410.18647)** (Lin et al., 2024): policy generalization follows a **power law in the *diversity* of environments/objects, not demo count** — beyond ~50 demos per environment/object, more demos of the same thing add almost nothing. For us, "diversity" at V1 scale = **position × angle cells**, and 60 demos of a narrow zone was exactly the predicted waste.
- **[Spatial generalization study](https://zhaohengyin.github.io/res/pdf/point.pdf)** (Yin et al.): success is determined by whether the test position falls **within the convex support of demo positions** — grid/full coverage substantially outperforms clustered demos; extrapolation beyond the demo region fails. This is *precisely* our ±4cm boundary finding, measured independently.
- **[MOVE](https://arxiv.org/pdf/2512.04813)** (2025): demos from static/fixed spatial configurations "significantly restrict the diversity of spatial information"; injecting per-demo spatial variability gives **+76% relative success** over static collection. Their related-work uses **5×5 tabletop grids with randomized placements around each center** — literally the grid+jitter design.

**Verdict on random vs systematic:** at *large* scale (QT-Opt-style 100k+ self-supervised grasps), random placement works because random eventually covers everything. At **our budget (~200 episodes), random sampling leaves holes and over-samples the center** — V0's angle histogram (42/62 at 90°) is what uncontrolled sampling produces. Systematic enumeration guarantees every cell is in the interpolation support. The literature's caveat: pure lattice points create their own distribution artifacts → **jitter around each grid point**. So: **grid × angle enumeration + small jitter = the correct V1 design.** The user's instinct is what the field converged on.

### 2.2 Data quality: consistency and action representation
- **[robomimic / What Matters in Learning from Offline Human Demonstrations](https://arxiv.org/abs/2108.03298)** (Mandlekar et al., 2021): data quality dominates algorithm choice; BC performs best on **consistent** demonstrations; mixed-quality data hurts plain BC badly. → Our *scripted* generator is actually an advantage here: it produces near-identical motion profiles every episode (a perfectly consistent "demonstrator"), IF we stop letting inconsistent supervision through (the 90°-angle poisoning was an inconsistency we manufactured).
- **[Data Quality in Imitation Learning](https://arxiv.org/abs/2306.02437)** (Belkhale, Cui, Sadigh, NeurIPS 2023): formalizes the two axes — **action divergence** (same state → different actions = bad) and **transition diversity** (more distinct states covered = good). V1's design maps 1:1: kill action divergence (angle-consistent supervision, reject `issue=angle`), maximize transition diversity (grid coverage).
- **[AWE: Waypoint-Based Imitation Learning](https://arxiv.org/pdf/2307.14326)** (Shi et al., 2023): replacing dense next-state actions with **waypoint targets improves ACT by 4–28%**, especially in low-data regimes (shorter effective horizon → less error compounding). → Important nuance for us: our "declared target pose" action scheme **is already waypoint-style — keep it**. It is not a flaw; the flaw was only the *binary gripper channel* riding along with it.
- **ACT multimodality** ([ACT guide](https://www.roboticscenter.ai/learn/action-chunking-transformers), [Mobile ALOHA](https://arxiv.org/html/2401.02117)): when demos disagree at similar states, the averaged prediction is "a poor choice for either" mode — the formal name for our 45°-block freeze. ALOHA-class tasks train on 20–50 consistent demos; consistency is what makes that tiny count work.

### 2.3 Scripted generation is a legitimate strategy (we're accidentally doing MimicGen)
- **[MimicGen](https://arxiv.org/abs/2310.17596)** (Mandlekar et al., 2023) + [SkillMimicGen](https://arxiv.org/pdf/2410.18907), [DemoGen](https://demo-generation.github.io/): generate 50k demos from ~200 human seeds by **replaying object-centric motion segments across systematically varied object poses, with success-based filtering**. Policies trained on this data work. → Our autograsp pipeline is a *real-world* MimicGen: a scripted expert that adapts to the object pose and only keeps successful, quality-gated episodes. The literature validates the whole approach — scripted ≠ second-class data, *if* filtered and consistent.
- Contrast: robomimic found machine-generated (RL-rollout) data hurts plain BC — but that data was *mixed-quality*. Ours is success-filtered + quality-judged, which is the MimicGen regime, not the noisy-RL regime.

### 2.4 Robustness: recovery data (the axis we have zero of)
- **[RaC](https://arxiv.org/pdf/2509.07953)** (2025), **[IntervenGen](https://arxiv.org/pdf/2405.01472)**: standard demos underrepresent failures/corrections; adding **recovery segments** (perturb → correct) scales robustness with far fewer episodes than more clean demos. → V0's descent-regression failure showed the policy has *no* correction behavior — it has never seen a state where the block is slightly off-center at grasp height. A cheap scripted version: at ~15% of V1 episodes, insert a small deliberate XY offset at approach, then let the pipeline's re-localise correct it — recording a *correction* the policy can imitate.
- **[Real-Time Chunking](https://arxiv.org/abs/2506.07339)** (Black et al., 2025; [lerobot docs](https://huggingface.co/docs/lerobot/rtc)): smooth async chunk execution — the principled version of what we hand-rolled (1s re-inference). Worth adopting when we move to bigger/slower policies (π0/SmolVLA-class); at ACT-51M with 11ms inference it's not the bottleneck.

## 3. Audit: our autograsp pipeline as a data generator

| Component | Verdict | Action for V1 |
|---|---|---|
| Scripted expert w/ success filtering + quality judge | ✅ MimicGen-class design, keep | tighten: reject `issue=angle` |
| Declared-target (waypoint) actions | ✅ AWE-validated, keep | — |
| Binary grip action | ❌ hides the pre-position event | **aperture (mm) as continuous action** |
| Random scatter ±4cm | ❌ under-covers, angle-collapses | **grid: 5×5 pos (3cm) × 7 angles (15°) + jitter ±1cm/±5°** |
| Grasp angle chosen ~90° regardless of block | ❌ action divergence (Belkhale axis 1) | flat-face snap during collection; angle must match block |
| Episode = pick + carry + place (~40s) | ❌ half is off-task; phase ambiguity | **end at grasp + 5cm lift**; place = unrecorded reset to next grid cell |
| Dead compute-gap frames (arm idle) | ❌ teaches "hover is an action" | trim stationary frames |
| Consistent scripted motion profile | ✅ consistency is BC's best friend | keep identical speeds/heights every episode |
| No failure/recovery states | ❌ zero correction ability | **~15% perturb-then-correct episodes** (scripted, cheap) |
| Reward gate (held + quality ≥0.6) | ✅ success filtering per MimicGen | keep; camera-verdict held |

## 4. The V1 protocol (research-final)

1. **Coverage**: 5×5 position grid @3cm (±6cm) × 7 angles (0–90° per 15°) = 175 cells, one
   episode each, jitter ±1cm/±5° per cell; +~30 extra episodes on edge/corner cells
   (boundary cells are where interpolation support matters most); +~30 perturb-and-correct
   episodes ≈ **~235 episodes ≈ 4–5 robot-hours** with the 1-pickup-per-episode loop.
2. **Action space**: waypoint target pose (keep) + **gripper aperture in mm** (new).
3. **Supervision consistency**: grasp angle snapped to the block's flat face; reward gate
   rejects `issue=angle`; identical motion profile every episode.
4. **Episode boundary**: record reach→grasp→5cm lift, verified held; carry+place to the
   next grid cell is the unrecorded reset.
5. **Eval = the same grid**: per-cell success heat-map (interpolation support made visible) —
   also the exact figure the scaling-laws papers use.
6. **Deferred, with reasons**: RTC (needed at π0-scale latency, not ACT-51M), MOVE-style
   camera-motion augmentation (single fixed wrist cam), DAgger/human intervention (do the
   scripted recovery variant first — it's free).

## 5. One-paragraph answer

For a ~200-episode budget on a single object, the field's evidence is unambiguous:
**systematic enumeration of the position×angle support (with per-cell jitter) beats random
sampling**, because BC policies generalize almost purely by interpolation inside the demo
support ([scaling laws](https://arxiv.org/abs/2410.18647), [spatial study](https://zhaohengyin.github.io/res/pdf/point.pdf), [MOVE](https://arxiv.org/pdf/2512.04813)) and random draws at this budget leave
holes and mode-collapse (our own 42/62-at-90° histogram). Equal-or-more important than
coverage is **supervision consistency** ([robomimic](https://arxiv.org/abs/2108.03298), [Belkhale](https://arxiv.org/abs/2306.02437)) — one consistent scripted expert whose grasp
angle always matches the block, with the gripper in the action space — plus a small dose of
**scripted recovery data** ([RaC](https://arxiv.org/pdf/2509.07953)/[IntervenGen](https://arxiv.org/pdf/2405.01472)) for robustness. Our pipeline is
structurally a real-world [MimicGen](https://arxiv.org/abs/2310.17596) and that is a validated way to make training data —
V1 is about pointing it at the right distribution.

---

## 6. Reading list (study links, tiered by relevance to V1)

**Tier 1 — decides V1 design:**
- Data Scaling Laws in Imitation Learning — https://arxiv.org/abs/2410.18647 (+ https://data-scaling-laws.github.io/)
- Spatial generalization of visual imitation learning — https://zhaohengyin.github.io/res/pdf/point.pdf
- Geometric Entropy: When Trajectory Diversity Helps and Hurts (JUN 2026) — https://arxiv.org/pdf/2606.20871
- Data Quality in Imitation Learning (NeurIPS'23) — https://arxiv.org/abs/2306.02437
- MOVE: Motion-Based Data Collection for Spatial Generalization (DEC 2025) — https://arxiv.org/pdf/2512.04813

**Tier 2 — validates the pipeline approach:**
- MimicGen — https://arxiv.org/abs/2310.17596 ; SkillMimicGen — https://arxiv.org/pdf/2410.18907 ; DexMimicGen — https://arxiv.org/pdf/2410.24185 ; DemoGen — https://demo-generation.github.io/
- robomimic study — https://arxiv.org/abs/2108.03298 (+ https://robomimic.github.io/study/)
- AWE: Waypoint-Based Imitation Learning — https://arxiv.org/pdf/2307.14326 (+ https://lucys0.github.io/awe/)
- RaC: Scaling Recovery and Correction (2025) — https://arxiv.org/pdf/2509.07953 ; IntervenGen — https://arxiv.org/pdf/2405.01472

**Tier 3 — data curation layer:**
- SCIZOR: Self-Supervised Data Curation (2025) — https://arxiv.org/abs/2505.22626
- DemInf: Data Curation with Mutual Information (2025) — https://arxiv.org/abs/2502.08623
- Curating Demonstrations using Online Experience (2025) — https://arxiv.org/pdf/2503.03707
- Diversity You Can Actually Measure (MAR 2026) — https://arxiv.org/pdf/2603.11634

**Tier 4 — deployment/inference:**
- Real-Time Chunking (RTC) — https://arxiv.org/abs/2506.07339 (+ https://huggingface.co/docs/lerobot/rtc)
- Training-Time Action Conditioning for Efficient RTC (DEC 2025) — https://arxiv.org/pdf/2512.05964
- Deployment-Time Reliability of Learned Robot Policies (MAR 2026) — https://arxiv.org/pdf/2603.11400
- ESPADA: Semantics-Aware Demo Downsampling (DEC 2025) — https://arxiv.org/pdf/2512.07371

**Context:**
- Mobile ALOHA — https://arxiv.org/html/2401.02117
- Unified Understanding of Robot Manipulation survey (2025) — https://arxiv.org/pdf/2510.10903
- PGDG: Physically Grounded Data Generation (MAY 2026) — https://arxiv.org/pdf/2605.21710
