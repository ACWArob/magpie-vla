"""Shape-gated grasp-angle ladder for AutoGrasp V2 (docs/V2_AUTOGRASP_PLAN.md, C1/C2).

V1 used cv2.minAreaRect on the SAM3 mask. That won the live bake-off on the
cube (4.5 deg vs mask-PCA 11.5 / depth-PCA 12.7 / fixed-90 22.5) because a
cube IS a rotated rectangle. On non-box masks the model mismatch produces
silent ~70 deg errors (measured on a crescent, 2026-07-22), so V2 gates by
shape and routes:

    rectangularity = mask_area / minAreaRect_area
      high (>= 0.88)             -> minAreaRect        (boxes: measured best)
      elongated + not box        -> skeleton-perp      (banana: local tangent
                                                        at the THICKEST ridge
                                                        point, not global PCA)
      otherwise                  -> polygon-antipodal  (blobby: opposing
                                                        near-parallel edges)

Also fixes V1's centering bug for irregular objects (plan C2): V1 centered on
the MASK CENTROID, which for a strawberry includes the leaves. Every rung here
returns the actual GRASP CENTER (pair midpoint / thickest ridge point / rect
center), which the expert should servo to instead of the centroid.

Angle convention (matches V1 usage): `axis_deg` is the object axis the fingers
should straddle, in image degrees [0, 180). The gripper yaw that places the
finger line PERPENDICULAR to that axis is `grasp_yaw_deg = (axis_deg + 90) % 180`.
Width is measured along the finger-close direction (perpendicular to axis).

Zero new dependencies: cv2 + numpy only (skeleton = distance-transform ridge).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np

RECT_GATE = 0.88          # >= this -> the mask really is a box; keep V1's winner
ELONG_GATE = 1.60         # aspect ratio >= this (and not a box) -> skeleton rung
_PAR_COS = 0.94           # polygon rung: |cos| between edges to count as parallel
_MIN_EDGE_PX = 8          # polygon rung: ignore tiny edges


@dataclass
class GraspProposal:
    method: str                     # 'minAreaRect' | 'polygon' | 'skeleton'
    axis_deg: float                 # object axis in [0,180) — see module docstring
    grasp_yaw_deg: float            # (axis_deg + 90) % 180
    center_px: tuple[float, float]  # GRASP point (not the mask centroid)
    width_px: float                 # object extent along the finger-close direction
    rectangularity: float
    aspect: float
    # every rung's raw answer, for bake-off logging / cross-checks:
    all_rungs: dict = field(default_factory=dict)

    def width_mm(self, mm_per_px: float) -> float:
        return self.width_px * mm_per_px


def _largest_contour(mask: np.ndarray):
    cnts, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL,
                               cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        raise ValueError('grasp_ladder: empty mask')
    return max(cnts, key=cv2.contourArea)


def _rung_min_area_rect(c):
    (cx, cy), (w, h), ang = cv2.minAreaRect(c)
    # normalize: axis_deg follows the LONG side
    axis = ang if w >= h else ang + 90.0
    return dict(axis_deg=axis % 180.0, center=(cx, cy),
                width_px=float(min(w, h)), long_px=float(max(w, h)))


def _rung_polygon(c, dt, max_width_px=None):
    """Antipodal near-parallel edge pairs on the simplified polygon.

    Pairs are scored by parallelism x edge support x MASK THICKNESS at the
    pair midpoint (distance transform). The thickness term is what routes the
    grasp onto the object's BODY: without it the narrowest pair wins, which on
    a strawberry is the leaves (caught by test_strawberry_center 2026-07-22).
    Width feasibility is a constraint, never a reward.
    """
    poly = cv2.approxPolyDP(c, 0.01 * cv2.arcLength(c, True), True)[:, 0, :].astype(np.float64)
    n = len(poly)
    h, w_img = dt.shape
    best = None
    for i in range(n):
        e1 = poly[(i + 1) % n] - poly[i]
        l1 = np.linalg.norm(e1)
        if l1 < _MIN_EDGE_PX:
            continue
        for j in range(i + 1, n):
            e2 = poly[(j + 1) % n] - poly[j]
            l2 = np.linalg.norm(e2)
            if l2 < _MIN_EDGE_PX:
                continue
            cosang = abs(float(np.dot(e1, e2)) / (l1 * l2))
            if cosang < _PAR_COS:
                continue
            m1 = (poly[i] + poly[(i + 1) % n]) / 2.0
            m2 = (poly[j] + poly[(j + 1) % n]) / 2.0
            wpx = float(np.linalg.norm(m2 - m1))
            if wpx < 5 or (max_width_px is not None and wpx > max_width_px):
                continue
            mid = (m1 + m2) / 2.0
            mx, my = int(round(mid[0])), int(round(mid[1]))
            thick = float(dt[my, mx]) if (0 <= my < h and 0 <= mx < w_img) else 0.0
            if thick <= 0:                     # midpoint off the object: not a grasp
                continue
            score = cosang * min(l1, l2) * thick
            if best is None or score > best['score']:
                close_dir = m2 - m1                    # fingers close along this
                axis = (np.degrees(np.arctan2(close_dir[1], close_dir[0])) + 90.0) % 180.0
                best = dict(score=score, axis_deg=axis,
                            center=tuple(mid.tolist()), width_px=wpx)
    return best  # may be None (no parallel pair found)


def _rung_skeleton(mask):
    """Grasp at the THICKEST ridge point, perpendicular to the LOCAL spine.

    Distance-transform ridge stands in for the medial axis (no skimage dep).
    Local (not global) tangent handles curved objects: a banana's global PCA
    axis is meaningless, its local spine direction is exactly right.
    """
    dt = cv2.distanceTransform(mask.astype(np.uint8), cv2.DIST_L2, 5)
    peak = np.unravel_index(int(np.argmax(dt)), dt.shape)   # (y, x) thickest point
    ridge = np.argwhere(dt > 0.60 * dt.max()).astype(np.float64)
    # local neighborhood of the peak on the ridge
    d = np.linalg.norm(ridge - np.array(peak, np.float64), axis=1)
    local = ridge[d < max(6.0, 2.5 * dt[peak])]
    if len(local) < 5:
        local = ridge
    local = local - local.mean(0)
    _, _, vt = np.linalg.svd(local, full_matrices=False)
    spine = np.degrees(np.arctan2(vt[0, 0], vt[0, 1])) % 180.0   # (y,x) -> deg
    return dict(axis_deg=spine, center=(float(peak[1]), float(peak[0])),
                width_px=float(2.0 * dt[peak]))


def propose(mask: np.ndarray, max_width_px: float | None = None) -> GraspProposal:
    """Run the gate, return the chosen rung's proposal (all rungs logged)."""
    c = _largest_contour(mask)
    area = cv2.contourArea(c)
    dt = cv2.distanceTransform(mask.astype(np.uint8), cv2.DIST_L2, 5)
    r_rect = _rung_min_area_rect(c)
    rectangularity = float(area / max(r_rect['width_px'] * r_rect['long_px'], 1.0))
    aspect = float(r_rect['long_px'] / max(r_rect['width_px'], 1.0))

    r_poly = _rung_polygon(c, dt, max_width_px)
    r_skel = _rung_skeleton(mask)
    rungs = dict(minAreaRect=r_rect, polygon=r_poly, skeleton=r_skel)

    if rectangularity >= RECT_GATE:
        method, r = 'minAreaRect', r_rect
    elif aspect >= ELONG_GATE:
        method, r = 'skeleton', r_skel
    elif r_poly is not None:
        method, r = 'polygon', r_poly
    else:                                   # blobby but no parallel pair: skeleton
        method, r = 'skeleton', r_skel

    return GraspProposal(
        method=method,
        axis_deg=float(r['axis_deg']) % 180.0,
        grasp_yaw_deg=(float(r['axis_deg']) + 90.0) % 180.0,
        center_px=tuple(map(float, r['center'])),
        width_px=float(r['width_px']),
        rectangularity=rectangularity,
        aspect=aspect,
        all_rungs=rungs,
    )
