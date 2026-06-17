"""
Common interface for grasp detectors, so PCA / Contact-GraspNet / AnyGrasp / GraspGen
can be benchmarked head-to-head on the same input (see notebooks/grasp_compare.ipynb).

A detector takes a world-frame point cloud (+ optional colours / target mask) and
returns a ranked list of Grasp poses. The comparison harness then measures, per
detector: latency, number of grasps, top score, and (live, on the robot) the
execution success rate.

Why a socket/subprocess bridge for the learned detectors:
    The main env runs torch 2.10+cu128 (for SAM3 + lerobot). Contact-GraspNet (TF),
    AnyGrasp, and GraspGen pin OLDER torch/CUDA, so each lives in its own venv and is
    called over a Unix socket — same pattern as scripts/sam3_infer.py. The adapter
    classes here are thin clients; the heavy model runs in its bridge process.
"""

from __future__ import annotations

import abc
import time
from dataclasses import dataclass, field

import numpy as np


@dataclass
class Grasp:
    """A single 6-DOF parallel-jaw grasp in the WORLD frame."""
    pose: np.ndarray            # 4x4 homogeneous TCP target (gripper frame)
    width: float                # opening width at grasp (m)
    score: float                # detector confidence [0, 1] (higher = better)
    approach: np.ndarray = field(default_factory=lambda: np.array([0, 0, -1.]))  # unit approach dir (world)

    def angle_deg(self) -> float:
        """Top-down yaw of the closing axis (deg), for comparison with the PCA angle."""
        x = self.pose[:3, 0]
        return float(np.degrees(np.arctan2(x[1], x[0])) % 180.)


@dataclass
class DetectResult:
    grasps: list                # ranked list[Grasp], best first
    latency_s: float            # wall-clock inference time
    name: str                   # detector name

    @property
    def best(self):
        return self.grasps[0] if self.grasps else None

    @property
    def n(self):
        return len(self.grasps)


class GraspDetector(abc.ABC):
    """Implement detect(); the harness handles timing + metrics."""
    name: str = 'base'

    @abc.abstractmethod
    def _detect(self, points: np.ndarray, colors=None, mask=None,
                tcp=None, intrinsics=None) -> list:
        """Return a ranked list[Grasp] (best first). points: (N,3) world XYZ."""
        ...

    def detect(self, points, colors=None, mask=None, tcp=None,
               intrinsics=None) -> DetectResult:
        t0 = time.time()
        grasps = self._detect(points, colors=colors, mask=mask,
                              tcp=tcp, intrinsics=intrinsics)
        grasps = sorted(grasps, key=lambda g: g.score, reverse=True)
        return DetectResult(grasps=grasps, latency_s=time.time() - t0, name=self.name)

    def available(self) -> bool:
        """True if the detector's backend (model/bridge) is reachable."""
        return True


def benchmark(detectors, scene, repeats: int = 3) -> list:
    """Run each detector on one scene `repeats` times; return rows for a table.

    scene: dict with keys points (N,3), optional colors/mask/tcp/intrinsics.
    """
    rows = []
    for det in detectors:
        if not det.available():
            rows.append(dict(detector=det.name, available=False))
            continue
        lat = []
        res = None
        for _ in range(repeats):
            res = det.detect(scene['points'], colors=scene.get('colors'),
                             mask=scene.get('mask'), tcp=scene.get('tcp'),
                             intrinsics=scene.get('intrinsics'))
            lat.append(res.latency_s)
        rows.append(dict(
            detector=det.name, available=True,
            n_grasps=res.n,
            top_score=round(res.best.score, 3) if res.best else None,
            top_angle_deg=round(res.best.angle_deg(), 1) if res.best else None,
            latency_ms=round(1000 * float(np.median(lat)), 1),
        ))
    return rows
