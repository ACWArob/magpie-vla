# Test-Day Results — OOD ring + live method bake-off (2026-07-14)

*Two hardware tests run today on act_v1 (frozen). Companion:
[V1_EVAL_REPORT.md](V1_EVAL_REPORT.md) (the in-zone 97/100), [DECISION_LEDGER.md](DECISION_LEDGER.md).*

---

## Test 1 — Out-of-distribution ring (the extrapolation question)

**Setup:** the ±9 cm ring — one 3 cm grid step *outside* everything the policy trained on
(training support ends at ±6 cm) — graded on the identical 100-grasp instrument. 15 of the
24 ring combos ran before the arm driver dropped; enough to answer the question.

![OOD ring](figures/appendix/ood_ring.png)

| | In-zone (±6 cm, trained) | **OOD ring (±9 cm)** |
|---|---|---|
| Pick success | 97% (75/75 rotated) | **13% (2/15)** |
| Mean grade | 0.70 | **0.08** |

**The finding — a hard extrapolation cliff.** One grid step past the training support and
the policy collapses from 97% to 13%. The 2 picks that survived (‑9,‑6 @70° grade 0.55;
‑9,‑3 @45° grade 0.67) were edge/partial grabs, not clean. This is the **measured boundary
of the policy's competence** and it confirms the V0 lesson at V1 scale: **these policies
interpolate within their data support and do not extrapolate beyond it.** 9 cm is inside the
arm's reach and the deploy safety box, so this is a *learned* limit, not a mechanical one.

*Paper value:* this is the quantified "support boundary" figure — it makes the coverage
argument (gate 9) concrete: the grid isn't over-engineering, it's exactly the region where
the policy works, and nothing works outside it. To extend the workspace you must extend the
data, not hope for generalization.

---

## Test 2 — Method bake-off, LIVE with ground truth (36 placements)

**Setup:** the scripted expert places the block at 36 known positions × angles; every
detector and every angle strategy runs on the *same* frame, scored against the **known
placed angle** (ground truth the offline tests couldn't provide).

![bake-off live](figures/appendix/bakeoff_live_angles.png)

### Angle accuracy vs the true placed angle (mod-90, median)

| Method | Median error | Verdict |
|---|---|---|
| **minAreaRect (V1 choice)** | **4.5°** | ✅ best — and it's what we deploy |
| mask-PCA | 11.5° | 2.5× worse — the near-square ill-conditioning, now measured against truth |
| GraspGenX yaw | 11.5° | 6-DOF planner; its yaw isn't tuned for flat-face alignment |
| depth-PCA (gate 3) | 12.7° | worst PCA variant — IR-stripe corruption, quantified |
| fixed 90° (V0 behaviour) | 22.5° | the mode-averaging cause, now a number: 22.5° off on average |

**minAreaRect wins decisively — 4.5° vs everything else's 11–22°.** This closes gates 3 and
5 with *ground-truth accuracy* (the offline tests only had frame-to-frame consistency).
Notably, "fixed 90°" — the V0 no-angle strategy — sits at **22.5° median error**, which is
mechanistically exactly why V0 froze: it was systematically ~22° off the block.

### Detectors (all 36/36 detected the block)

| Detector | Detection | Median latency |
|---|---|---|
| SAM3 (deployed) | 36/36 | 1109 ms |
| Gemini box | 36/36 | 6497 ms |
| HSV threshold | 36/36 | **1 ms** |

Confirms the offline finding on live frames: on a red block all three detect 100%; SAM3's
value is text-query generality (V2), and its ~1.1 s latency (11× the 10 Hz tick) is why
deployment is detector-free. Gemini's 6.5 s is a network round-trip — unusable in any loop.

---

## What these two tests bought

| Claim | Before today | Now (measured) |
|---|---|---|
| Policy competence has a hard support boundary | assumed from V0 | **97% → 13% at one step past ±6 cm** |
| minAreaRect beats other angle methods on *accuracy* | had consistency only | **4.5° vs 11–22° vs ground truth** |
| fixed-90° (V0) is systematically wrong | narrated | **22.5° median error — the freeze cause quantified** |
| depth-PCA broken by IR stripes | observed | **12.7° — worst of the five** |
| Detectors on live frames | offline only | **all 36/36; SAM3 1.1 s, HSV 1 ms confirmed live** |

**Not run today:** T2b (judge determinism) and T2c (ensembling A/B) — no-robot, ~15 min,
still queued. OOD ring is 15/24; the remaining 9 combos would only sharpen an already-clear
cliff.

*Raw data: `data/v1_ood_ring.json`, `data/appendix_bakeoff.json`. act_v1 and its results
untouched — both tests wrote their own files.*
