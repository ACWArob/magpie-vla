"""Deformation / squish grading term for V2 (docs/V2_AUTOGRASP_PLAN.md D2).

V1's reward gate was: held-through-lift AND Gemini grade >= 0.6. That misses
the failure mode fruit introduces — held but CRUSHED. "Delicately" has to be
graded, not assumed. This adds a deterministic, physical term alongside (never
replacing) the Gemini judge.

Two independent signals, either can flag a squish:

1. Geometric — the grasp aperture closed well past the object's own width.
   A rigid block stops the fingers at its width; a strawberry keeps yielding.
   deform_ratio = (expected_width - seated_aperture) / expected_width
   where expected_width comes from perception (the ladder's width_px -> mm) or
   the object's GraspMemory prior.

2. Force — DeliGrasp already ramps to a target compliance-aware force. Applied
   force well above that target means the object gave way and we kept squeezing.

Output is a 0..1 gentleness score (1 = pristine, 0 = mush) plus a boolean
`ok` at a configurable threshold, so it slots straight into the reward gate:
    keep = held and gemini_grade >= 0.6 and deform.ok
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class DeformResult:
    gentleness: float          # 0..1, higher is gentler
    deform_ratio: float        # fraction of expected width the fingers overran
    force_overrun: float       # applied/target force ratio - 1, clipped >=0
    ok: bool
    reason: str


def assess(expected_width_mm: float,
           seated_aperture_mm: float,
           applied_force_n: float | None = None,
           target_force_n: float | None = None,
           max_deform_ratio: float = 0.25,
           max_force_overrun: float = 0.75,
           min_gentleness: float = 0.6) -> DeformResult:
    """Grade how gently an object was grasped.

    expected_width_mm : object width from perception/prior (finger-close axis)
    seated_aperture_mm: gripper aperture once seated on the object
    applied/target_force_n : optional DeliGrasp forces for the force signal

    A NEGATIVE-or-tiny expected width is treated as unknown -> geometric term
    is skipped (falls back to force-only, or passes if neither is available,
    so a missing measurement never fabricates a squish).
    """
    reasons = []

    # --- geometric term ---
    if expected_width_mm and expected_width_mm > 1.0:
        deform_ratio = max(0.0, (expected_width_mm - seated_aperture_mm) / expected_width_mm)
    else:
        deform_ratio = 0.0
        reasons.append('no-width')
    geo_gentle = 1.0 - min(1.0, deform_ratio / max_deform_ratio) if max_deform_ratio > 0 else 1.0

    # --- force term ---
    if applied_force_n is not None and target_force_n and target_force_n > 1e-6:
        force_overrun = max(0.0, applied_force_n / target_force_n - 1.0)
        force_gentle = 1.0 - min(1.0, force_overrun / max_force_overrun) if max_force_overrun > 0 else 1.0
    else:
        force_overrun = 0.0
        force_gentle = 1.0
        reasons.append('no-force')

    # combined gentleness = the WORSE of the two signals (a squish on either
    # axis is a squish); missing signals default to 1.0 so they don't dominate.
    gentleness = min(geo_gentle, force_gentle)
    ok = gentleness >= min_gentleness
    if not ok:
        if geo_gentle <= force_gentle:
            reasons.append(f'over-closed {deform_ratio*100:.0f}% past width')
        else:
            reasons.append(f'over-forced {force_overrun*100:.0f}% past target')
    return DeformResult(
        gentleness=round(gentleness, 3),
        deform_ratio=round(deform_ratio, 3),
        force_overrun=round(force_overrun, 3),
        ok=ok,
        reason=('ok' if ok else '; '.join(r for r in reasons if not r.startswith('no-'))
                or 'insufficient-signal'),
    )
