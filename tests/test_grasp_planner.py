import sys, os
import numpy as np, cv2
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from magpie_control.v2.grasp_planner import plan, GraspPlan


def _bar(angle=0, long=200, short=50):
    m = np.zeros((400, 400), np.uint8)
    cv2.fillPoly(m, [cv2.boxPoints(((200,200),(long,short),angle)).astype(np.int32)], 255)
    return m


def test_plan_uses_perceived_width_without_prior():
    p = plan(_bar(), mm_per_px=0.5, object_name='banana')
    assert isinstance(p, GraspPlan)
    assert not p.have_prior
    # short side 50px * 0.5 = 25mm perceived
    assert 20 <= p.expected_width_mm <= 30, p.expected_width_mm
    # prepos opens wider than the object
    assert p.prepos_width_mm > p.expected_width_mm


def test_prior_width_overrides_perception():
    p = plan(_bar(), mm_per_px=0.5, object_name='banana', width_prior_mm=33.0)
    assert p.have_prior and p.expected_width_mm == 33.0
    # aperture band centered on the prior
    lo, hi = p.aperture_band_mm
    assert lo < 33 < hi


def test_delicate_force_cap():
    # a high stale force prior must be capped for delicate objects
    p = plan(_bar(), 0.5, object_name='strawberry', force_prior_n=25.0, delicate=True)
    assert p.seed_force_n <= 12.0
    p2 = plan(_bar(), 0.5, object_name='block', force_prior_n=25.0, delicate=False)
    assert p2.seed_force_n == 25.0


def test_band_scales_with_object():
    small = plan(_bar(short=20), 0.5, width_prior_mm=10)
    big = plan(_bar(short=120), 0.5, width_prior_mm=60)
    sb = small.aperture_band_mm[1] - small.aperture_band_mm[0]
    bb = big.aperture_band_mm[1] - big.aperture_band_mm[0]
    assert bb > sb, (sb, bb)   # wider object -> wider tolerance band


def test_prepos_clamped_to_max_width():
    p = plan(_bar(short=100), 0.5, width_prior_mm=80, max_width_mm=90)
    assert p.prepos_width_mm <= 90


def test_yaw_present_and_rungs_logged():
    p = plan(_bar(30), 0.5)
    assert 0 <= p.grasp_yaw_deg < 180
    assert set(p.all_rungs) == {'minAreaRect', 'polygon', 'skeleton'}


if __name__ == '__main__':
    fails = 0
    for n, f in sorted(globals().items()):
        if n.startswith('test_'):
            try: f(); print(f'  PASS {n}')
            except AssertionError as e: fails += 1; print(f'  FAIL {n}: {e}')
    raise SystemExit(fails)
