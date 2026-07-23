"""Per-frame phase labels + per-episode task strings for V2 recording.

Why (docs/V2_AUTOGRASP_PLAN.md D3 + top-lab table):
- The scripted expert always KNOWS which phase it is in; V1 threw that away.
  Recording it costs nothing and our own ablation showed phase information is
  load-bearing (the force-as-phase-clock finding: removing the only phase-
  bearing input cost 20 points). SkillGen-style segment labels also enable
  phase-wise training/analysis later.
- A language task string per episode (BridgeData V2 lesson) future-proofs the
  dataset for language/part-conditioning ("pick up the strawberry" /
  "pick up the cup by the handle") without re-collection.

Usage in the collection loop:
    meta = EpisodeMeta(task='pick up the strawberry', object_name='strawberry')
    meta.set_phase('approach')            # expert calls at each transition
    ...
    frame_info = meta.frame()             # merge into the recorder's per-frame dict
    ...
    summary = meta.finish(success=True)   # -> episode-level dict for attempts.jsonl
"""
from __future__ import annotations

import time

# Canonical phase vocabulary — keep STABLE across V2; downstream training
# depends on these exact strings. Order documents the nominal sequence.
PHASES = (
    'idle',        # before the first manipulation move
    'approach',    # move-above + travel toward the object
    'center',      # visual centering / re-localise at grasp angle
    'descend',     # lowering to grasp height
    'squeeze',     # DeliGrasp force ramp / close
    'lift',        # incremental lift + verify
    'transport',   # carrying to the place target
    'place',       # guarded-Fz set-down
    'release',     # open + retreat
    'recover',     # any recovery/retry path
)
_PHASE_IDX = {p: i for i, p in enumerate(PHASES)}


class EpisodeMeta:
    def __init__(self, task: str, object_name: str, instance_id: str = ''):
        self.task = task
        self.object_name = object_name
        self.instance_id = instance_id      # e.g. 'strawberry_2' for E2 rotation
        self._phase = 'idle'
        self._t0 = time.time()
        self._transitions: list[tuple[float, str]] = [(0.0, 'idle')]

    @property
    def phase(self) -> str:
        return self._phase

    def set_phase(self, phase: str) -> None:
        if phase not in _PHASE_IDX:
            raise ValueError(f'unknown phase {phase!r}; add to PHASES deliberately, '
                             'not ad hoc — downstream training keys on these strings')
        if phase != self._phase:
            self._phase = phase
            self._transitions.append((round(time.time() - self._t0, 3), phase))

    def frame(self) -> dict:
        """Per-frame dict to merge into the recorder's frame metadata."""
        return {'phase': self._phase, 'phase_idx': _PHASE_IDX[self._phase]}

    def finish(self, success: bool) -> dict:
        """Episode-level record for the attempts ledger."""
        return {
            'task': self.task,
            'object': self.object_name,
            'instance_id': self.instance_id,
            'success': bool(success),
            'phase_transitions': self._transitions,
            'phases_visited': [p for _, p in self._transitions],
        }
