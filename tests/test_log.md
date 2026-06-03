# MAGPIE Test Log

Add a new entry each time a test is run. Include date, object, result, and any notes.

---

## Floor / Reach Calibration
**Date:** 2026-06-02
**Notebook:** `floor_test.ipynb` / `magpie_demo.ipynb` Section 13
**Result:** PASS — 44 steps (3mm each), all succeeded. Fingertip confirmed at 30mm (HARD_FLOOR_Z).
**Notes:** Test run in free air (arm not above table). Confirms arm can physically reach HARD_FLOOR_Z without kinematic failure.

---

## Full Pickup — Tape Measure
**Date:** 2026-06-02
**Notebook:** `magpie_demo.ipynb` Section 12
**Object:** tape measure (~78mm diameter, ~45mm tall)
**Result:** FAIL — gripper closed past object (0N force), dropped during lift
**Notes:** Object detected correctly (score 0.962). Issue: DeliGrasp aperture_mm (19mm) smaller than object width (77mm) — gripper closed through object. Fixed by adding fallback to close_g() after set_pos() if no contact detected.

---

## Full Pickup — Tape Measure (after aperture fix)
**Date:** 2026-06-02
**Notebook:** `magpie_demo.ipynb` Section 12
**Object:** tape measure
**Result:** PARTIAL — contact detected (2.495N) but object not lifted successfully
**Notes:** Post-lift aperture check triggered (ap=67.6mm < obj_w*0.4=30.9mm). Grip too shallow. TABLE_Z measurement improved grasp_off to 22mm. Force policy using physics-based F_min.

---

## Full Pickup — Orange Electrical Component
**Date:** 2026-06-02
**Notebook:** `magpie_demo.ipynb` Section 12
**Object:** orange electrical component (~31mm width, ~15mm tall)
**Result:** DROPPED — 6 attempts all returned F=0N
**Notes:** Object thinner than DeliGrasp aperture target (19mm). Gripper closed to 18.9mm with zero force — fingers passed through object. Fixed with fallback close_g() when no contact after set_pos().

---

## Angle Sweep — Red Cube
**Date:** 2026-06-02
**Notebook:** `angle_sweep.ipynb`
**Object:** red cube (~40mm)
**Result:** 10/10 pickups HELD across angles 0°–90°
**Sweep angles:** 0, 10, 20, 30, 40, 50, 60, 70, 80, 90°
**Notes:** All pickups successful. Detected angle vs expected angle inconsistent due to Gemini non-deterministic classification (sometimes short_side instead of symmetric). Fixed: sweep now uses geometry-only angle detection. Cube correctly classified as symmetric → snaps to 0°.

---

*Add new entries below this line*
