"""
PCA baseline detector — wraps the CURRENT magpie pipeline (top-layer PCA grasp angle)
behind the GraspDetector interface, so it can be benchmarked against the learned
detectors. This is the "what we're doing now" reference line.

Works out of the box (no extra install) — uses pointcloud_utils already in the repo.
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from base import Grasp, GraspDetector          # noqa: E402  (when run from this dir)
try:
    from grasp_detectors.base import Grasp, GraspDetector  # noqa: F811
except Exception:
    pass


class PCABaseline(GraspDetector):
    name = 'pca_baseline'

    def __init__(self):
        from pointcloud_utils import (top_layer, analyse_pcd,
                                      grasp_rotation_matrix)
        self._top_layer = top_layer
        self._analyse = analyse_pcd
        self._rot = grasp_rotation_matrix

    def _detect(self, points, colors=None, mask=None, tcp=None, intrinsics=None):
        points = np.asarray(points, dtype=float)
        if len(points) < 10:
            return []
        pts_top = self._top_layer(points)
        pca = self._analyse(pts_top)
        cen = pca['centroid']
        ext = pca['extent_m']
        ang = pca['grasp_angle_deg']
        ratio = ext[0] / max(ext[1], 1e-6)

        pose = np.eye(4)
        pose[:3, :3] = self._rot(ang)
        pose[:3, 3] = cen

        # Heuristic confidence: well-defined major axis (elongated) → more confident;
        # near-square (PCA degenerate) → less. Bounded [0.3, 0.9].
        score = float(np.clip(0.3 + 0.3 * (ratio - 1.0), 0.3, 0.9))
        width = float(np.clip(ext[1], 0.005, 0.105))
        return [Grasp(pose=pose, width=width, score=score)]
