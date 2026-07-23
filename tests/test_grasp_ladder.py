"""Unit tests for the V2 shape-gated grasp ladder (pytest or plain python)."""
import sys, os
import numpy as np
import cv2

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from magpie_control.v2.grasp_ladder import propose, RECT_GATE


def _ang_close(a, b, tol):
    d = abs((a - b) % 180.0)
    return min(d, 180.0 - d) <= tol


def _square(angle_deg, size=90):
    m = np.zeros((400, 400), np.uint8)
    rect = ((200, 200), (size, size), angle_deg)
    cv2.fillPoly(m, [cv2.boxPoints(rect).astype(np.int32)], 255)
    return m


def _bar(angle_deg, long=220, short=60):
    m = np.zeros((400, 400), np.uint8)
    rect = ((200, 200), (long, short), angle_deg)
    cv2.fillPoly(m, [cv2.boxPoints(rect).astype(np.int32)], 255)
    return m


def _crescent():
    m = np.zeros((400, 400), np.uint8)
    cv2.ellipse(m, (200, 230), (140, 110), 0, 200, 340, 255, -1)
    cv2.ellipse(m, (200, 190), (115, 85), 0, 180, 360, 0, -1)
    return cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))


def _strawberry():
    """Body blob + small leaf lobe offset upward: centroid != grasp point."""
    m = np.zeros((400, 400), np.uint8)
    cv2.circle(m, (200, 240), 70, 255, -1)               # body
    cv2.ellipse(m, (200, 150), (28, 42), 0, 0, 360, 255, -1)  # 'leaves'
    return m


def test_square_routes_to_min_area_rect():
    p = propose(_square(0))
    assert p.method == 'minAreaRect'
    assert p.rectangularity >= RECT_GATE


def test_rotated_square_angle():
    p = propose(_square(30))
    assert p.method == 'minAreaRect'
    assert _ang_close(p.axis_deg % 90, 30 % 90, 4), p.axis_deg  # square: mod-90


def test_bar_axis_and_width():
    p = propose(_bar(20))
    assert p.method == 'minAreaRect'          # a bar IS a rectangle
    assert _ang_close(p.axis_deg, 20, 4), p.axis_deg
    assert 50 <= p.width_px <= 70, p.width_px  # short side ~60

def test_crescent_leaves_min_area_rect():
    p = propose(_crescent())
    assert p.method != 'minAreaRect', (p.method, p.rectangularity)
    assert p.rectangularity < RECT_GATE
    # grasp center must lie INSIDE the mask (minAreaRect's center does not!)
    m = _crescent()
    x, y = map(int, p.center_px)
    assert m[y, x] > 0, 'grasp center not on the object'


def test_strawberry_center_is_body_not_centroid():
    m = _strawberry()
    p = propose(m)
    ys, xs = np.nonzero(m)
    centroid_y = ys.mean()
    # centroid is dragged up by the leaves; grasp center must sit lower (body)
    assert p.center_px[1] > centroid_y, (p.center_px, centroid_y)
    x, y = map(int, p.center_px)
    assert m[y, x] > 0


def test_all_rungs_logged():
    p = propose(_square(0))
    assert set(p.all_rungs) == {'minAreaRect', 'polygon', 'skeleton'}
    assert p.grasp_yaw_deg == (p.axis_deg + 90.0) % 180.0


if __name__ == '__main__':
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith('test_'):
            try:
                fn(); print(f'  PASS {name}')
            except AssertionError as e:
                fails += 1; print(f'  FAIL {name}: {e}')
    raise SystemExit(fails)
