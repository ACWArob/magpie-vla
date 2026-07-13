# Execution Plan — workshop paper in 3 weeks (2026-07-14 → 2026-08-01)

*Everything below is prepped and committed. V1 artifacts (act_v1, its dataset slice,
v1_eval results) are frozen — nothing here modifies them. Robot days assume the arm is
free ~4h; every long run is unattended with the failsafe ladder.*

## Week 1 — data + training (the V1.1 arc)

**Day 1 (Mon) — robot day: V1.1 collection + bake-off**
1. v1_1_collect: restart kernel → cells 1–6 → GO HOME
2. New cell (or before the sequencer): `COVERAGE_LEDGER = 'data/v1_1_coverage.json'`
3. Run sequencer → 55 combos (~1h). **Watch the first 3 episodes for the
   `[angle canon]` line** — that's the fix firing. Every straight block should execute ≈0°.
4. Same session: appendix_bakeoff.ipynb → last two cells (~45 min) → live bake-off done
5. Evening: `ssh deltaai exit` (Duo) → `bash scripts/v1_1_train_prep.sh`
   → submits BOTH jobs (act_v1_1 + the 60-subset ablation) → check `tail -f` for loss lines

**Day 2 (Tue) — robot day: the two policy evals (unattended)**
1. Morning: pull both checkpoints (commands printed by the prep script), groot-stub load test
2. Run **v0_eval.ipynb** (~3h unattended) → the measured V0 comparison row
3. Afternoon/overnight-style: **v1_eval.ipynb with `POLICY_DIR='models/act_v1_1'` and a
   fresh `_EV_RES` name** (copy the notebook or edit the two constants — do NOT overwrite
   `data/v1_eval_100.json`) → the before/after
4. Check V1.1 against the pre-registered criteria in V1_1_PLAN.md

**Day 3 (Wed) — no robot: subset eval + analysis**
1. Eval act_v1s60 on the instrument (~3h, can also slide to Day 4)
2. I regenerate: before/after heat-maps, updated scoreboard (ablation + V0-measured bars
   turn green), updated comparison table + V1_EVAL_REPORT v2
3. You + mentor: score `data/judge_sheet/labels_template.csv` blind (30 min each,
   independently) → I compute agreement → gate 7 goes MEASURED

**Day 4–5 — writing**
1. Pick the venue (decides page limit + template) — ask mentor Day 3 at the latest
2. I port WORKSHOP_PAPER_DRAFT.md → LaTeX, real citation pass from V1_data_research links
3. Record the 60s video: 3 rotated-block picks + 1 straight (V1.1 showing the fix) —
   phone on a tripod, one GO HOME between takes
4. Bench photo for slide 10 / paper teaser while the phone is out

## Week 2 — hardening + mentor loop

**Day 6–7**: repeatability run (re-run v1_1 eval once, different day → run-to-run variance);
fill any bake-off gaps (Gemini leg needs the kernel's API key); strawberry pilot
(10 episodes, one afternoon) to scope V2 early — DeliGrasp force on a real strawberry,
bruise inspection by the judge. *Pilot only — informs the conference plan, not the paper.*

**Day 8–9**: full draft to mentor. While waiting: I update FULL_PIPELINE/DECISION_LEDGER
with all new measurements; regenerate every figure at camera-ready DPI; write the
reproducibility appendix (exact commit hashes, episode filter lists, seeds).

**Day 10**: mentor feedback in → revision.

## Week 3 — polish + submit

**Day 11–12**: revision round 2, abstract tightening, figure captions final.
**Day 13**: internal deadline — paper "done", only proofreading after.
**Day 14–15**: buffer (something WILL slip — the buffer is the plan working, not failing).

## What's prepped where (tomorrow's checklist)

| Item | Where | Status |
|---|---|---|
| V1.1 ledger (50 straight + 5 boundary) | `data/v1_1_coverage.json` | ✅ built |
| Sequencer ledger switch | `COVERAGE_LEDGER` global in v1_1_collect | ✅ wired |
| 0/90 canonicalization + Findings A/B/C | all collection notebooks | ✅ landed (15e195e) |
| Dual training script (V1.1 + subset) | `scripts/v1_1_train_prep.sh` | ✅ ready |
| V0 comparison eval | `notebooks/v0_eval.ipynb` (binary mapping + shim) | ✅ built |
| Live bake-off | `notebooks/appendix_bakeoff.ipynb` | ✅ ready since Fri |
| Judge sheet (50 blind images) | `data/judge_sheet/` + labels_template.csv | ✅ generated |
| Paper draft to iterate on | `docs/WORKSHOP_PAPER_DRAFT.md` | ✅ v0.1 |

## Hard rules for the whole plan

- `data/v1_eval_100.json`, `models/act_v1`, and the 229-episode training slice are
  **read-only history** — every new run writes to its own file.
- Any eval that dies mid-run: just re-run the cell (resume-safe). Any gripper weirdness:
  GO HOME (the reboot+verify handles it).
- Push to both repos at the end of every robot day (`git push origin ros && git push
  personal ros` after the remote-URL fix).
