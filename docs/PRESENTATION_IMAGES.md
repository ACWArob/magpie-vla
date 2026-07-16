# Presentation Image Library

*Every figure available for the talk, with its live URL, what it shows, and where it fits.
Pick from this menu when building slides (or hand this whole file to Claude/Gemini + the
deck source and say "use these"). All URLs are public raw-GitHub links, verified live.
Base: `https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/`*

Legend: ⭐ = strongest / bold enough to headline a slide · 🆕 = made 2026-07-16 for the talk

---

## 1. Headline / "money" visuals (⭐ big, one-message, slide-filling)

| Image | Shows | Best slide | URL (append to base) |
|---|---|---|---|
| ⭐🆕 **competence_map** | Green trained zone (97%) framed by red OOD ring (13%) — where it works vs stops | The OOD / competence-boundary slide | `talk/competence_map.png` |
| ⭐🆕 **v0_v1_rotated** | Two bars: V0 FROZE (0/75) vs V1 100% (75/75) on rotated blocks | The V0→V1 payoff / "only data changed" | `talk/v0_v1_rotated.png` |
| ⭐ **v1_eval_heatmap** | The 100-grasp result, 4 angle panels, colour = grade | The results slide (July 13) | `v1_eval_heatmap.png` |
| ⭐🆕 **tiny_data_stat** | Hero stat: "43 minutes → 97/100 · 442 MB · 0 labels" | Data-efficiency punch / transition | `talk/tiny_data_stat.png` |

## 2. The failure story (the talk's spine)

| Image | Shows | Best slide | URL |
|---|---|---|---|
| ⭐🆕 **aliasing_bimodal** | 15 episodes at 0° vs 47 at 90° — identical scenes, two demos | Root-cause slide (the bug in one picture) | `talk/aliasing_bimodal.png` |
| **v0/fig1_angle_collapse** | 71% of V0 grasps at ~90° regardless of block angle | Finding #1 (mode averaging) | `v0/fig1_angle_collapse.png` |
| **v1_eval_0deg_column** | The 0° column of the heat-map, labelled "V1.1 target" | The three-failures / next-steps slide | `v1_eval_0deg_column.png` |
| **v1_audit_angles** | Angle histogram with the "0° LOW" warning flagged | The audit slide (predicted the failure) | `v1_audit_angles.png` |

## 3. Method & decision evidence

| Image | Shows | Best slide | URL |
|---|---|---|---|
| ⭐🆕 **bakeoff_winner** | minAreaRect 4.5° vs PCA/GGX 11.5° vs depth-PCA 12.7° vs fixed-90° 22.5° | "why minAreaRect" / decision slide | `talk/bakeoff_winner.png` |
| ⭐ **gate_scoreboard** | 12-panel chosen-vs-rejected, every decision quantified | Backup / "everything is measured" | `appendix/gate_scoreboard.png` |
| **bakeoff_live_angles** | The full live bake-off (angles + detector latency) | Appendix / detail | `appendix/bakeoff_live_angles.png` |
| **offline_detectors** | SAM3 vs HSV: detection, latency, agreement | Detector-choice detail | `appendix/offline_detectors.png` |

## 4. How-it-works (process / pipeline)

| Image | Shows | Best slide | URL |
|---|---|---|---|
| ⭐ **fig1_pipeline** | Camera-ready pipeline loop with measured numbers in each box | The loop-at-a-glance slide | `appendix/fig1_pipeline.png` |
| 🆕 **timeline** | May→July milestone timeline, 5 dots | The timeline slide | `talk/timeline.png` |
| **sam3_wrist_overlay** | Live wrist frame with SAM3 mask (0.96 score) | The perception / expert slide | `sam3_wrist_overlay.png` |
| ⭐ **offline_aperture_staircase** | 12 episodes' gripper commands (open→prepos→squeeze) | The action-space slide | `appendix/offline_aperture_staircase.png` |
| **offline_actionspace** | V0 binary vs V1 7-level grip histograms | Action-space detail | `appendix/offline_actionspace.png` |

## 5. Data-design & supporting

| Image | Shows | Best slide | URL |
|---|---|---|---|
| **v0_v1/compare_angles_lengths** | V0 vs V1 angle spread + episode length | Data-design slide | `v0_v1/compare_angles_lengths.png` |
| **v0/fig2_support_vs_grid** | V0 competence stops at the data-support edge | Interpolation slide | `v0/fig2_support_vs_grid.png` |
| **offline_replay_v1** | Replay error 1.8mm/1.1° — "copies well ≠ succeeds" | The metric slide | `appendix/offline_replay_v1.png` |
| **offline_gate_coverage** | Reward-gate score distribution + designed-coverage scatter | Reward-gate slide | `appendix/offline_gate_coverage.png` |
| **v1_eval_summary** | 4-panel: success by angle, grade, yaw error, aperture | Results detail | `v1_eval_summary.png` |
| **ood_ring** | OOD ring grade grid + in-zone-vs-OOD bars | Alt to competence_map | `appendix/ood_ring.png` |

---

## Videos you're providing (slots to fill)

| Slot | Clip | Length |
|---|---|---|
| Data engine slide | one full autonomous collection cycle (pick → judge → place next) | 15–20 s |
| Eval slide | the 100-grasp eval running unattended (time-lapse) | 10–15 s |
| Closing slide | best rotated-block pick, full speed | ~10 s |
| Optional (first-policy slide) | V0's first-ever pick | 5 s |

---

## Notes for whoever builds the deck

- **Colour convention is consistent across the 🆕 talk figures**: green = V1 / works / chosen,
  red = V0 / fails / rejected, and every red/green is also labelled (never colour-alone).
- The `talk/` figures are sized and styled for slides (big fonts, one message). The
  `appendix/` figures are denser — better for the paper or backup slides than a 30-second slide.
- If you only take four images, take the four ⭐ in §1 — they carry the whole arc:
  tiny data → 97% → rotated-block win → the cliff.
- Deck source with these already placed: [SUMMER_TALK_SLIDES.md](SUMMER_TALK_SLIDES.md).
