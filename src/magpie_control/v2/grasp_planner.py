"""V2 grasp planner — fuse the shape-gated ladder with per-object priors into
one plan the scripted expert executes (docs/V2_AUTOGRASP_PLAN.md C1-C5, D1, D5).

The ladder (perception) answers WHERE and WHICH ORIENTATION from this frame.
GraspMemory (history) answers HOW HARD and roughly HOW WIDE from past grasps of
this object. This module combines them and emits the numbers every downstream
stage needs, so the collection loop calls one function instead of stitching
perception + priors + gate thresholds inline (which is how V1 drifted).

Design: the planner takes PLAIN NUMBERS for the priors (force_prior_n,
width_prior_mm), not a GraspMemory handle, so it is unit-testable off-robot and
never imports DINOv2. `priors_from_memory()` is the thin adapter that pulls
those numbers from a live GraspMemory; the loop calls it, then calls `plan()`.
"""
from __future__ import annotations

from dataclasses import dataclass

from .grasp_ladder import propose, GraspProposal

# Sensible fallbacks for a brand-new object with no prior yet.
_DEFAULT_FORCE_N = 8.0        # DeliGrasp will refine; conservative seed
_DELICATE_FORCE_CAP_N = 12.0  # never seed above this on an unknown object


@dataclass
class GraspPlan:
    # geometry (from the ladder)
    method: str
    grasp_yaw_deg: float
    center_px: tuple[float, float]
    width_mm: float
    # force (from prior, DeliGrasp refines live)
    prepos_width_mm: float       # open to a bit wider than the object
    seed_force_n: float
    # gate params handed to the deformation check (D2) + physical verify (D1)
    expected_width_mm: float     # for deform_ratio
    aperture_band_mm: tuple[float, float]   # per-object seated band (D1)
    # bookkeeping / bake-off
    rectangularity: float
    have_prior: bool
    all_rungs: dict


def plan(mask,
         mm_per_px: float,
         *,
         object_name: str = 'object',
         force_prior_n: float | None = None,
         width_prior_mm: float | None = None,
         max_width_mm: float | None = None,
         prepos_margin_mm: float = 8.0,
         delicate: bool = True) -> GraspPlan:
    """Produce a full grasp plan from a SAM3 mask + optional priors.

    mask         : binary object (or PART) mask — same interface for
                   "strawberry" and "cup handle"
    mm_per_px    : depth-derived scale at the object
    force_prior_n / width_prior_mm : from GraspMemory, or None for a new object
    max_width_mm : gripper max aperture (feasibility clamp for the polygon rung)
    """
    max_w_px = (max_width_mm / mm_per_px) if (max_width_mm and mm_per_px > 0) else None
    prop: GraspProposal = propose(mask, max_width_px=max_w_px)
    perceived_w = prop.width_mm(mm_per_px)

    # expected width: trust the PRIOR once we have enough history (it's measured
    # from real seated grasps), else the perceived width from this frame.
    have_prior = width_prior_mm is not None and width_prior_mm > 1.0
    expected_w = float(width_prior_mm) if have_prior else perceived_w

    # force seed
    if force_prior_n is not None and force_prior_n > 0:
        seed_force = float(force_prior_n)
    else:
        seed_force = _DEFAULT_FORCE_N
    if delicate:
        seed_force = min(seed_force, _DELICATE_FORCE_CAP_N)

    # seated aperture band for the physical verify (D1): centered on expected
    # width, +/- a tolerance that scales with the object (wider objects, wider
    # band). Replaces V1's hardcoded cube-specific 20-40mm.
    tol = max(4.0, 0.20 * expected_w)
    band = (max(0.0, expected_w - tol), expected_w + tol)

    prepos = expected_w + prepos_margin_mm
    if max_width_mm:
        prepos = min(prepos, max_width_mm)

    return GraspPlan(
        method=prop.method,
        grasp_yaw_deg=prop.grasp_yaw_deg,
        center_px=prop.center_px,
        width_mm=round(perceived_w, 1),
        prepos_width_mm=round(prepos, 1),
        seed_force_n=round(seed_force, 2),
        expected_width_mm=round(expected_w, 1),
        aperture_band_mm=(round(band[0], 1), round(band[1], 1)),
        rectangularity=round(prop.rectangularity, 3),
        have_prior=have_prior,
        all_rungs=prop.all_rungs,
    )


def priors_from_memory(gm, object_name: str):
    """Adapter: pull (force_prior_n, width_prior_mm) from a live GraspMemory.

    Kept separate + defensive so the planner stays testable and a memory miss
    just yields (None, None) -> the plan falls back to perception + safe seeds.
    """
    force = width = None
    try:
        p = gm.get_prior(object_name)
        if isinstance(p, dict):
            force = p.get('x')
        elif p is not None:
            force = float(p)
    except Exception:
        pass
    try:
        shapes = gm._load_shapes() if hasattr(gm, '_load_shapes') else {}
        s = shapes.get(object_name) or shapes.get(object_name.lower())
        if isinstance(s, dict):
            # shape model stores a minor axis (grasp-width) in mm if present
            width = s.get('width_mm') or s.get('minor_mm') or s.get('minor')
    except Exception:
        pass
    return force, width
