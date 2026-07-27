"""V2 offline self-test — exercise the ENTIRE decision pipeline with NO robot.

Runs the full per-object logic (perceive -> plan -> lift/weigh -> gate ->
record) on synthetic masks for a spread of object types, and prints a report.
This is what you run before touching hardware to confirm the software path is
sound. Embedded verbatim in notebooks/v2_collect.ipynb (the OFFLINE SELF-TEST
cell), and runnable directly:  python3 scripts/v2_selftest.py
"""
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from magpie_control.v2.grasp_planner import plan
from magpie_control.v2.state_builder import build_state, fz_is_live, STATE_DIM
from magpie_control.v2.deformation_check import assess as deform_assess
from magpie_control.v2.weight_estimate import assess as weigh, reconcile_force
from magpie_control.v2.episode_meta import EpisodeMeta

MM_PER_PX = 0.5
GRIPPER_MAX_MM = 105.0
DELICATE_CAP_N = 12.0


# ---- synthetic object gallery (shape, mm/px, true width, weight, fragile?) ----
def _rect(long, short, ang=0):
    m = np.zeros((400, 400), np.uint8)
    cv2.fillPoly(m, [cv2.boxPoints(((200, 200), (long, short), ang)).astype(np.int32)], 255)
    return m

def _blob(r):
    m = np.zeros((400, 400), np.uint8); cv2.circle(m, (200, 200), r, 255, -1); return m

def _crescent():
    m = np.zeros((400, 400), np.uint8)
    cv2.ellipse(m, (200, 230), (140, 110), 0, 200, 340, 255, -1)
    cv2.ellipse(m, (200, 190), (115, 85), 0, 180, 360, 0, -1)
    return cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))

def _strawberry():
    m = np.zeros((400, 400), np.uint8)
    cv2.circle(m, (200, 240), 60, 255, -1)
    cv2.ellipse(m, (200, 160), (26, 40), 0, 0, 360, 255, -1)
    return m

# name, mask, delicate?, true seated aperture mm, weight g, notes
OBJECTS = [
    ('red block',   _rect(90, 90),       False, 49, 120, 'rigid control'),
    ('banana',      _rect(230, 55, 20),  True,  27, 120, 'elongated'),
    ('strawberry',  _strawberry(),       True,  22,  25, 'body-not-leaves'),
    ('tomato',      _blob(70),           True,  62, 150, 'round + soft'),
    ('crescent',    _crescent(),         True,  30,  40, 'concave'),
    ('water bottle',_rect(80, 80),       True,  75, 1500, 'heavy + delicate'),
]


def run(verbose=True):
    results = []
    for name, mask, delicate, true_ap, weight_g, note in OBJECTS:
        row = {'object': name, 'checks': [], 'note': note}

        # 1) PLAN (perceive + priors-absent path)
        p = plan(mask, MM_PER_PX, object_name=name, max_width_mm=GRIPPER_MAX_MM,
                 delicate=delicate)
        row['method'] = p.method
        row['yaw'] = round(p.grasp_yaw_deg, 1)
        row['checks'].append(('grasp center on object',
                              mask[int(p.center_px[1]), int(p.center_px[0])] > 0))
        row['checks'].append(('prepos opens past object',
                              p.prepos_width_mm > p.expected_width_mm))
        row['checks'].append(('force seeded within delicate cap',
                              (not delicate) or p.seed_force_n <= DELICATE_CAP_N))
        # delicate objects must SEED gentle (<=2N, DeliGrasp ramps up) — a high
        # seed would crush produce before adaptive control kicks in
        row['checks'].append(('delicate seed is gentle (<=2N)',
                              (not delicate) or p.seed_force_n <= 2.0))

        # 2) WEIGH (simulate wrist_fz: baseline 0, holding = -weight)
        fz_hold = -(weight_g / 1000.0 * 9.81)
        w = weigh(0.0, fz_hold, delicate_force_cap_n=DELICATE_CAP_N)
        row['mass_g'] = w.mass_g
        force, force_ok = reconcile_force(p.seed_force_n, w, DELICATE_CAP_N)
        row['force_used'] = force
        row['weight_conflict'] = w.conflict
        # heavy+delicate SHOULD flag a conflict; others should not
        expect_conflict = (name == 'water bottle')
        row['checks'].append(('weight conflict flagged iff heavy+delicate',
                              w.conflict == expect_conflict))

        # 3) STATE assembly + fz-live guard over a fake episode
        frames = []
        for k in range(20):
            fz = 0.0 if k < 10 else fz_hold * (k - 9) / 10.0   # ramps as it lifts
            frames.append(build_state([0.1, -0.5, 0.3 + 0.004 * k, 3.1, 0, 0],
                                      grip_mm=true_ap, grip_force=force, wrist_fz=fz))
        frames = np.stack(frames)
        row['checks'].append(('state is 9-dim float32', frames.shape[1] == STATE_DIM))
        row['checks'].append(('wrist_fz reads LIVE (not V1 all-zero)', fz_is_live(frames)))

        # 4) DEFORMATION gate. The geometric term needs a TRUSTED expected
        # width — a GraspMemory prior measured from real seated grasps, not a
        # noisy single-frame perceived width (which spans a strawberry's whole
        # body, not the graspable neck). So: with a prior -> geometric+force;
        # cold-start (no prior) -> force-only (expected=0 skips geometry). Here
        # we simulate the warmed-prior case with the true seated width.
        prior_w = true_ap                     # stands in for a warmed prior
        gentle = deform_assess(prior_w, true_ap,
                               applied_force_n=force, target_force_n=force)
        crushed = deform_assess(prior_w, true_ap * 0.5,
                                applied_force_n=force * 2, target_force_n=force)
        row['checks'].append(('gentle grasp passes gate', gentle.ok))
        row['checks'].append(('crushed grasp fails gate', not crushed.ok))

        # 5) EPISODE META phase flow
        m = EpisodeMeta(task=f'pick up the {name}', object_name=name)
        for ph in ['approach', 'center', 'descend', 'squeeze', 'lift',
                   'transport', 'place', 'release']:
            m.set_phase(ph)
        summ = m.finish(success=gentle.ok)
        row['checks'].append(('episode meta phases logged',
                              summ['phases_visited'][-1] == 'release'))

        results.append(row)

    # ---- report ----
    total = passed = 0
    for r in results:
        cp = sum(1 for _, ok in r['checks'] if ok)
        total += len(r['checks']); passed += cp
        if verbose:
            flag = '' if cp == len(r['checks']) else '  <-- CHECK'
            conflict = '  [weight-conflict: skip/flag]' if r['weight_conflict'] else ''
            print(f"{r['object']:13s} {r['method']:12s} yaw={r['yaw']:5.1f} "
                  f"{r['mass_g']:5.0f}g force={r['force_used']:4.1f}N  "
                  f"{cp}/{len(r['checks'])} checks{conflict}{flag}")
            for label, ok in r['checks']:
                if not ok:
                    print(f"      FAIL: {label}")
    print(f"\n=== {passed}/{total} checks passed across {len(results)} objects ===")
    return passed == total


if __name__ == '__main__':
    ok = run()
    print('OFFLINE PIPELINE:', 'READY' if ok else 'NEEDS ATTENTION')
    raise SystemExit(0 if ok else 1)
