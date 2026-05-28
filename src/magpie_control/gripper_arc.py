"""
Magpie gripper fingertip-drop model from the 4-bar linkage geometry in:
  https://github.com/correlllab/h1_mujoco/blob/main/magpie/ur5e.xml

Mechanism summary
-----------------
Left/right fingers driven by a planar parallelogram 4-bar (a = c ≈ 0.04500 m,
b = d ≈ 0.02236 m in the YZ plane of base_top frame). The finger_combined body
translates without rotating, so the vertical drop = crank-arc Z-component.

Arc peak
--------
The crank arc peaks (max Z drop) at PHI0+theta = pi/2, which corresponds to a
finger aperture of ~31.5 mm. For goal apertures BELOW 31.5 mm, the gripper
passes through the peak on the way to close, so the WORST-CASE drop is the
peak — not the endpoint. This is safety-relevant: grasping a narrow object still
incurs the full ~25 mm peak drop during the closing motion.

Public API
----------
fingertip_drop(aperture_open_mm, aperture_close_mm)
    Worst-case downward displacement (m) during closing — uses arc peak when
    the goal aperture is below the peak aperture.

safe_grasp_z(aperture_close_mm, floor_z=0.030)
    Minimum open-gripper fingertip Z (m) required at the grasp pose so the
    fingertips clear floor_z after closing to aperture_close_mm.

Calibration
-----------
Scale _K fitted from physical measurement (2026-05-27):
  full open (103.6 mm) → fully closed (0 mm) = 21 mm measured.
  Raw geometry gives 15.35 mm → _K ≈ 1.368.
  Extrapolated peak drop (31.5 mm aperture) ≈ 24.8 mm.
"""

import numpy as np

# Geometry from h1_mujoco/magpie/ur5e.xml (YZ-plane, base_top frame)
_L    = 0.04500                                  # crank arm length (m)
_PY   = 0.038                                    # crank pivot Y in base_top (m)
_FY   = 0.02227                                  # finger pad Y offset toward centre (m)
_PHI0 = float(np.arctan2(0.020953, 0.039824))    # 0.4836 rad — crank angle at hinge = 0

# Aperture at which the crank arc peaks (PHI0 + theta = pi/2 → cos = 0 → gap min Y):
PEAK_APERTURE_MM = float(2000.0 * (_PY - _FY))  # ≈ 31.46 mm


def _hinge_rad(aperture_mm: float) -> float:
    """Aperture (mm between finger pads) → crank hinge angle (rad)."""
    cos_val = np.clip((aperture_mm / 2000.0 - _PY + _FY) / _L, -1.0, 1.0)
    return float(np.arccos(cos_val) - _PHI0)


def _raw_drop(aperture_open_mm: float, aperture_close_mm: float) -> float:
    """Geometric Z-drop before calibration scale."""
    theta_a = _hinge_rad(aperture_open_mm)
    theta_b = _hinge_rad(aperture_close_mm)
    return float(_L * (np.sin(_PHI0 + theta_b) - np.sin(_PHI0 + theta_a)))


# Calibration: scale from measured 21 mm drop (103.6 → 0 mm), 2026-05-27
_K = 0.021 / _raw_drop(103.6, 0.0)   # ≈ 1.368


def fingertip_drop(aperture_open_mm: float, aperture_close_mm: float) -> float:
    """
    Worst-case downward displacement of fingertips (m, positive = toward floor)
    as the gripper closes from aperture_open_mm to aperture_close_mm.

    When goal aperture < PEAK_APERTURE_MM (~31.5 mm), the fingertips reach their
    lowest point at the arc peak rather than at the final position. This function
    returns the peak drop in that case (conservative but correct for safety).

    Examples
    --------
    fingertip_drop(103.6, 80.0)  → ~0.007 m  (large object, gripper barely closes)
    fingertip_drop(103.6, 30.0)  → ~0.025 m  (peak of arc, ~31.5 mm)
    fingertip_drop(103.6,  0.0)  → ~0.025 m  (passes through peak, peak is worst case)
    Calibrated endpoint: fingertip_drop(103.6, 0.0) reproduces ≈ 21 mm measured
      — but safety uses arc peak (≈25 mm) for goals below 31.5 mm.
    """
    if aperture_close_mm < PEAK_APERTURE_MM:
        # Worst case during motion is at the arc peak, not the endpoint
        return float(_K * _raw_drop(aperture_open_mm, PEAK_APERTURE_MM))
    else:
        return float(_K * _raw_drop(aperture_open_mm, aperture_close_mm))


def safe_grasp_z(aperture_close_mm: float, floor_z: float = 0.030) -> float:
    """
    Minimum open-gripper fingertip Z (m) required at the grasp pose so the
    fingertips clear floor_z after the gripper closes to aperture_close_mm.

    Usage in safety check:
        if fingertip_z_open < safe_grasp_z(goal_aperture_mm):
            abort  # closing would hit the floor

    The required Z scales with how much the gripper closes: a small object
    (aperture ≤ 31.5 mm) needs ~25 mm more clearance than a large one.
    """
    drop = fingertip_drop(103.6, aperture_close_mm)
    return float(floor_z + drop)
