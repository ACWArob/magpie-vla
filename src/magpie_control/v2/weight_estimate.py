"""Object weight from the wrist FT sensor, and the grip force it implies
(docs/V2_AUTOGRASP_PLAN.md — the "account for weight" requirement).

Two jobs:

1. MEASURE weight. Once the object is lifted clear, the wrist z-force shifts by
   the object's weight relative to the empty-gripper baseline. Because V2
   finally records wrist_fz for real (state_builder), this is free per episode:
       weight_N = fz_baseline - fz_holding      (holding is more negative)

2. IMPLY the grip force needed so it does not slip. For a pinch grasp the
   friction condition is  2 * mu * F_grip >= weight, i.e.
       F_grip_min = weight / (2 * mu)  * safety
   Heavier -> firmer. This FIGHTS the delicate cap, and that tension is a real
   physical fact, not a bug: a heavy AND delicate object (a full water bottle,
   a large tomato) may need more force than is safe. We surface that as a
   `conflict` flag so the collection loop can skip/flag rather than crush or
   drop silently.
"""
from __future__ import annotations

from dataclasses import dataclass

G = 9.81
_DEFAULT_MU = 0.7          # silicone-ish fingertip on typical produce; conservative
_SAFETY = 1.5             # grip-force margin over the slip threshold


@dataclass
class WeightEstimate:
    weight_n: float           # object weight in newtons (>=0)
    mass_g: float             # grams, for readability
    grip_force_min_n: float   # to not slip under its own weight
    conflict: bool            # needed force exceeds the delicate cap
    note: str


def weight_from_fz(fz_baseline: float, fz_holding: float) -> float:
    """Object weight (N) from empty vs holding wrist-z force.

    Sign convention: downward load is NEGATIVE (state_builder keeps it raw), so
    holding is more negative than baseline and weight = baseline - holding.
    Clipped at 0 (noise / mis-measure never yields a negative weight)."""
    return max(0.0, float(fz_baseline) - float(fz_holding))


def assess(fz_baseline: float,
           fz_holding: float,
           *,
           mu: float = _DEFAULT_MU,
           delicate_force_cap_n: float | None = None) -> WeightEstimate:
    w = weight_from_fz(fz_baseline, fz_holding)
    grip_min = (w / (2.0 * mu)) * _SAFETY if mu > 1e-6 else float('inf')
    conflict = (delicate_force_cap_n is not None and grip_min > delicate_force_cap_n)
    if conflict:
        note = (f'needs {grip_min:.1f}N to hold {w/G*1000:.0f}g but delicate cap '
                f'is {delicate_force_cap_n:.1f}N — heavy+delicate, flag')
    elif w < 1e-3:
        note = 'no measurable weight (not lifted clear, or FT not live)'
    else:
        note = f'{w/G*1000:.0f}g -> >= {grip_min:.1f}N to hold'
    return WeightEstimate(
        weight_n=round(w, 3),
        mass_g=round(w / G * 1000.0, 1),
        grip_force_min_n=round(grip_min, 2),
        conflict=conflict,
        note=note,
    )


def reconcile_force(seed_force_n: float,
                    weight: WeightEstimate,
                    delicate_force_cap_n: float) -> tuple[float, bool]:
    """Combine the delicate seed with the anti-slip minimum.

    Returns (force_to_use, ok). The grip must be at least enough to hold the
    weight; if that exceeds the delicate cap, we clamp to the cap and report
    ok=False so the loop knows this object is at the edge of what is safe (the
    grasp may slip — better a logged slip than a crush)."""
    need = weight.grip_force_min_n
    force = max(seed_force_n, min(need, delicate_force_cap_n))
    ok = need <= delicate_force_cap_n
    return round(force, 2), ok
