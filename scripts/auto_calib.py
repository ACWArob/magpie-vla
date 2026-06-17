"""
Automatic hand-eye clocking calibration.

Moves the arm a known +X then +Y in WORLD frame (these moves do NOT use the camera
calibration), measures how the detected object slides in the image, and picks the
camera clocking Rz(0/90/180/270) whose predicted image-shift matches the measurement.
Returns the corrected tcp_to_cam — no sign-guessing, no reflections.

This fixes the dominant error (axis flip / clocking). A small residual lateral offset
may remain (translation), but the grasp tolerates it; re-run on a 2nd object position
to confirm match_err stays low.

    from auto_calib import auto_calibrate
    tcp_to_cam, info = auto_calibrate(node, sam3_query, 'red cube')
"""

import time

import numpy as np


def auto_calibrate(node, sam3_query, object_name, step=0.03, z_off=0.120, verbose=True):
    from pointcloud_utils import project_world_to_pixel
    from magpie_control.homog_utils import homog_xform, R_krot

    K = node.caminfo.k
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]

    def opx():
        node.spin(6)
        b, s, m = sam3_query(node.color.copy(), object_name)
        if m is None or not m.any():
            return None, None
        ys, xs = np.where(m)
        vd = node.depth.copy()[m].astype(float); vd = vd[vd > 0]
        d = float(np.median(vd)) / 1000. if len(vd) else 0.4
        return np.array([float(xs.mean()), float(ys.mean())]), d

    node.unteach()
    home = node.tcp.copy()
    p0, d0 = opx()
    if p0 is None:
        raise RuntimeError(f'auto_calibrate: "{object_name}" not detected at start')

    tX = home.copy(); tX[0, 3] += step
    node.move(tX, spd=0.05); time.sleep(0.3); pX, _ = opx(); node.move(home, spd=0.05)
    tY = home.copy(); tY[1, 3] += step
    node.move(tY, spd=0.05); time.sleep(0.3); pY, _ = opx(); node.move(home, spd=0.05)
    if pX is None or pY is None:
        raise RuntimeError('auto_calibrate: object left the view during a move — reduce step')

    mX, mY = pX - p0, pY - p0
    if verbose:
        print(f'  measured image shift: world +X -> ({mX[0]:+.0f},{mX[1]:+.0f})  '
              f'+Y -> ({mY[0]:+.0f},{mY[1]:+.0f}) px')

    Pcam = np.array([(p0[0] - cx) * d0 / fx, (p0[1] - cy) * d0 / fy, d0, 1.])
    best = None
    for th in (0, 90, 180, 270):
        Tc = homog_xform(R_krot([0, 0, 1], np.radians(th)), [0, 0, z_off])
        P = (home @ Tc @ Pcam)[:3]
        sX = project_world_to_pixel(P, tX, Tc, K)[0] - p0
        sY = project_world_to_pixel(P, tY, Tc, K)[0] - p0
        err = float(np.linalg.norm(sX - mX) + np.linalg.norm(sY - mY))
        if verbose:
            print(f'    Rz({th:3d}deg): predicted +X ({sX[0]:+.0f},{sX[1]:+.0f}) '
                  f'+Y ({sY[0]:+.0f},{sY[1]:+.0f})  err={err:.0f}px')
        if best is None or err < best[1]:
            best = (th, err, Tc)

    info = dict(clocking_deg=best[0], match_err_px=round(best[1], 1),
                measured_dX=[round(x, 1) for x in mX],
                measured_dY=[round(x, 1) for x in mY],
                trustworthy=best[1] < 30.)
    if verbose:
        ok = '✓' if info['trustworthy'] else '⚠ high residual — try a different object spot'
        print(f'  => clocking Rz({best[0]}deg), match err {best[1]:.0f}px  {ok}')
    return best[2], info
