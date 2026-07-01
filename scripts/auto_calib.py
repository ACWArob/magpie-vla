"""
Automatic hand-eye clocking calibration.

Moves the arm a known +X then +Y in WORLD frame (these moves do NOT use the camera
calibration), measures how the detected object slides in the image, and picks the
camera clocking Rz(theta) whose predicted image-shift best matches the measurement.

Two-stage search:
  1. Coarse: four 90° candidates (0 / 90 / 180 / 270°) — rules out axis flips.
  2. Fine  : ±45° sweep at 1° steps around the coarse winner — handles small off-axis
             mounts without any iterative solver or extra dependencies.

Returns the corrected tcp_to_cam and a diagnostic dict.

    from auto_calib import auto_calibrate
    tcp_to_cam, info = auto_calibrate(node, sam3_query, 'red cube')
"""

import time

import numpy as np


def auto_calibrate(node, sam3_query, object_name, step=0.03, z_off=0.120, verbose=True):
    """Calibrate camera clocking by moving the arm ±step m in world X and Y.

    Args:
        node        : Demo node with .tcp, .color, .depth, .caminfo, .move(), .spin()
        sam3_query  : callable(image_rgb, query) -> (boxes, scores, mask)
        object_name : SAM3 query string for the calibration target
        step        : arm jog distance in metres (default 30 mm)
        z_off       : camera optical centre distance from TCP along tool-Z (metres).
                      Physical measurement: GRIPPER_LEN_to_cam.  Start with 0.120;
                      if TABLE_Z reads ~24 mm too high use 0.144.
        verbose     : print per-candidate errors

    Returns:
        tcp_to_cam  : (4, 4) corrected extrinsic — drop-in replacement for _TCP_TO_CAM
        info        : dict with clocking_deg, match_err_px, trustworthy, measured_dX/dY
    """
    from pointcloud_utils import project_world_to_pixel
    from magpie_control.homog_utils import homog_xform, R_krot

    K = node.caminfo.k
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]

    def opx():
        node.spin(6)
        _, _, m = sam3_query(node.color.copy(), object_name)
        if m is None or not m.any():
            return None, None
        ys, xs = np.where(m)
        vd = node.depth.copy()[m].astype(float); vd = vd[vd > 0]
        # Depth under the mask anchors the whole clocking solve (Pcam below). If it's
        # dropped out (reflective surface, occlusion), DON'T silently assume 40cm —
        # that would solve the calibration against a wrong distance and corrupt
        # _TCP_TO_CAM for the rest of the session. Signal "no depth" so the caller bails.
        d = float(np.median(vd)) / 1000. if len(vd) else None
        return np.array([float(xs.mean()), float(ys.mean())]), d

    node.spin(20)
    if node.tcp is None:
        raise RuntimeError('auto_calibrate: no TCP pose — is the arm node (ur5_node) running?')
    node.unteach()
    home = node.tcp.copy()
    p0, d0 = opx()
    if p0 is None:
        raise RuntimeError(f'auto_calibrate: "{object_name}" not detected at start')
    if d0 is None:
        raise RuntimeError(f'auto_calibrate: no valid depth under "{object_name}" mask at '
                           'the calibration anchor — reposition the object or check for a '
                           'reflective surface. Refusing to calibrate against a guessed distance.')

    tX = home.copy(); tX[0, 3] += step
    node.move(tX, spd=0.05); time.sleep(0.3); pX, _ = opx(); node.move(home, spd=0.05)
    tY = home.copy(); tY[1, 3] += step
    node.move(tY, spd=0.05); time.sleep(0.3); pY, _ = opx(); node.move(home, spd=0.05)
    if pX is None or pY is None:
        raise RuntimeError('auto_calibrate: object left the view during a move — reduce step or reposition')

    mX, mY = pX - p0, pY - p0
    if verbose:
        print(f'  measured image shift: world +X -> ({mX[0]:+.0f},{mX[1]:+.0f})  '
              f'+Y -> ({mY[0]:+.0f},{mY[1]:+.0f}) px')

    Pcam = np.array([(p0[0] - cx) * d0 / fx, (p0[1] - cy) * d0 / fy, d0, 1.])

    def eval_theta(th_deg):
        Tc = homog_xform(R_krot([0, 0, 1], np.radians(th_deg)), [0, 0, z_off])
        P  = (home @ Tc @ Pcam)[:3]
        sX = project_world_to_pixel(P, tX, Tc, K)[0] - p0
        sY = project_world_to_pixel(P, tY, Tc, K)[0] - p0
        return float(np.linalg.norm(sX - mX) + np.linalg.norm(sY - mY)), Tc

    # ── Stage 1: coarse (4 quadrant candidates) ───────────────────────────────
    best_th, best_err, best_Tc = None, float('inf'), None
    for th in (0, 90, 180, 270):
        err, Tc = eval_theta(th)
        if verbose:
            print(f'    Rz({th:3d}deg): err={err:.0f}px')
        if err < best_err:
            best_th, best_err, best_Tc = th, err, Tc

    if verbose:
        print(f'  coarse best: Rz({best_th}deg) err={best_err:.1f}px — refining ±45° ...')

    # ── Stage 2: fine sweep ±45° at 1° resolution ─────────────────────────────
    for th in range(best_th - 45, best_th + 46):
        if th % 90 == 0:
            continue           # already evaluated in coarse pass
        err, Tc = eval_theta(th)
        if err < best_err:
            best_th, best_err, best_Tc = th, err, Tc

    info = dict(
        clocking_deg   = best_th,
        match_err_px   = round(best_err, 1),
        z_off_m        = z_off,
        measured_dX    = [round(x, 1) for x in mX],
        measured_dY    = [round(x, 1) for x in mY],
        trustworthy    = best_err < 30.,
    )
    if verbose:
        ok = '✓' if info['trustworthy'] else '⚠ high residual — try a different object spot or adjust z_off'
        print(f'  => Rz({best_th}deg), match err {best_err:.1f}px  {ok}')
    return best_Tc, info
