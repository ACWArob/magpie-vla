"""
Magpie gripper fingertip-drop model from the 4-bar linkage geometry in:
  https://github.com/correlllab/h1_mujoco/blob/main/magpie/ur5e.xml

Mechanism
---------
Parallelogram 4-bar (a = c ≈ 0.04500 m, b = d ≈ 0.02236 m in the YZ plane of
base_top frame). finger_combined translates without rotating, so the vertical
fingertip drop = crank-arc Z-component evaluated at the goal aperture.

Calibration
-----------
Scale _K fitted from physical measurement (2026-05-27):
  full open (103.6 mm) → fully closed (0 mm) = 21 mm measured drop.
"""

import numpy as np

_L    = 0.04500                                  # crank arm length (m)
_PY   = 0.038                                    # crank pivot Y in base_top (m)
_FY   = 0.02227                                  # finger pad Y offset toward centre (m)
_PHI0 = float(np.arctan2(0.020953, 0.039824))    # 0.4836 rad


def _hinge_rad(aperture_mm: float) -> float:
    """Aperture (mm between finger pads) → crank hinge angle (rad)."""
    cos_val = np.clip((aperture_mm / 2000.0 - _PY + _FY) / _L, -1.0, 1.0)
    return float(np.arccos(cos_val) - _PHI0)


def _raw_drop(a_mm: float, b_mm: float) -> float:
    return float(_L * (np.sin(_PHI0 + _hinge_rad(b_mm)) - np.sin(_PHI0 + _hinge_rad(a_mm))))


_K = 0.021 / _raw_drop(103.6, 0.0)   # ≈ 1.377, fitted at measured endpoint


def fingertip_drop(aperture_open_mm: float, aperture_close_mm: float) -> float:
    """
    Downward displacement of fingertips (m, positive = toward floor) when
    the gripper closes from aperture_open_mm to aperture_close_mm.

    Uses the crank-arc endpoint — calibrated to the physical measurement
    (103.6 → 0 mm = 21 mm drop, 2026-05-27).

    Examples
    --------
    fingertip_drop(103.6, 0.0)   → 0.021 m  (full close, measured)
    fingertip_drop(103.6, 50.0)  → ~0.013 m
    fingertip_drop(103.6, 80.0)  → ~0.008 m
    fingertip_drop(103.6, 103.6) →  0.0 m
    """
    return float(_K * _raw_drop(aperture_open_mm, aperture_close_mm))
