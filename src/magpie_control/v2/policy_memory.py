"""Memory token — turn GraspMemory retrieval into a POLICY input
(research thrust: recurrent/live memory through the policy; see project
perpetual-learning + conference P3 "frozen policy improves during deployment").

Today GraspMemory (DINO RAG) feeds only the SCRIPTED expert. This module packs
the retrieved k-nearest past grasps into a fixed-length vector that appends to
the policy's observation, so the LEARNED policy conditions on "what worked for
similar objects before" — physical priors (force, angle, compliance) that a
single wrist frame cannot reveal.

The elegant property: the policy is trained to CONSUME this vector; the memory
store updates live at deploy, so a FROZEN policy improves as memory grows — no
retraining. Retrieval is external; the network just learns to use whatever it
retrieves.

IMPORTANT prerequisite (measured 2026-07-27): this only helps once the memory
holds DIVERSE objects. The current store is 99.6% red block — nothing to
retrieve across. So memory-conditioning is gated on V2 multi-object collection;
this module + its harness are built now so the experiment runs the moment that
data exists.

Design: `pack()` is a PURE function of the retrieved neighbors (list of dicts)
so it is unit-testable off-robot; `from_memory()` is the adapter that pulls
neighbors from a live GraspMemory given the current view.
"""
from __future__ import annotations

import numpy as np

# Fixed memory-token layout (keep STABLE — the policy is trained on these slots).
MEMORY_NAMES = [
    'mem_force_mean',    # N — weighted-mean grip force that held for neighbors
    'mem_force_std',     # N — spread => how consistent the physical prior is
    'mem_angle_sin',     # circular grasp-angle summary (sin)
    'mem_angle_cos',     # circular grasp-angle summary (cos)
    'mem_success_rate',  # fraction of neighbors that held
    'mem_top_sim',       # cosine sim of the best match => retrieval confidence
    'mem_coverage',      # effective neighbor count / k  => is memory populated?
    'mem_width_mean',    # mm — typical seated aperture of neighbors
]
MEMORY_DIM = len(MEMORY_NAMES)

# When retrieval finds nothing similar (new object, empty store), the token is
# all zeros EXCEPT a coverage/confidence of 0 — the policy learns "no memory ->
# fall back to vision". A zero token must be an unambiguous "no signal".
_EMPTY = np.zeros(MEMORY_DIM, dtype=np.float32)


def pack(neighbors, sim_floor: float = 0.0) -> np.ndarray:
    """Summarize retrieved neighbors into the fixed memory vector.

    neighbors : list of dicts, each with keys:
        sim     (cosine similarity, 0..1)   — required
        force   (N)        angle (deg)       held (bool)      width_mm
      any missing field is treated as absent and skipped for that stat.
    sim_floor : drop neighbors below this similarity (different object).
    """
    ns = [n for n in neighbors if float(n.get('sim', 0.0)) >= sim_floor]
    if not ns:
        return _EMPTY.copy()
    sims = np.array([float(n['sim']) for n in ns], dtype=np.float64)
    w = sims / sims.sum() if sims.sum() > 1e-9 else np.ones_like(sims) / len(sims)

    def wmean(key):
        vals = [(i, float(n[key])) for i, n in enumerate(ns) if n.get(key) is not None]
        if not vals:
            return None
        idx = [i for i, _ in vals]; v = np.array([x for _, x in vals])
        ww = w[idx]; ww = ww / ww.sum()
        return float((v * ww).sum())

    force_mean = wmean('force')
    forces = [float(n['force']) for n in ns if n.get('force') is not None]
    force_std = float(np.std(forces)) if len(forces) > 1 else 0.0

    # circular mean of grasp angle (mod 180 -> double-angle, standard trick)
    angs = [np.radians(2.0 * float(n['angle'])) for n in ns if n.get('angle') is not None]
    if angs:
        aw = w[:len(angs)]; aw = aw / aw.sum()
        s = float((np.sin(angs) * aw).sum()); c = float((np.cos(angs) * aw).sum())
    else:
        s = c = 0.0

    helds = [bool(n['held']) for n in ns if n.get('held') is not None]
    success_rate = float(np.mean(helds)) if helds else 0.0
    width_mean = wmean('width_mm') or 0.0
    top_sim = float(sims.max())
    # coverage: effective number of confident neighbors, normalized by a nominal k=5
    coverage = min(1.0, float((sims >= 0.5).sum()) / 5.0)

    return np.array([
        force_mean if force_mean is not None else 0.0,
        force_std, s, c, success_rate, top_sim, coverage, width_mean,
    ], dtype=np.float32)


def from_memory(gm, image_rgb, object_name: str, k: int = 5) -> np.ndarray:
    """Adapter: retrieve k-nearest past grasps from a live GraspMemory and pack.

    Defensive — any retrieval failure yields the empty token, so a cold/broken
    memory degrades to vision-only rather than crashing the control loop.
    """
    try:
        neighbors = _retrieve(gm, image_rgb, object_name, k)
    except Exception:
        neighbors = []
    return pack(neighbors)


def _retrieve(gm, image_rgb, object_name, k):
    """Pull k-nearest (sim, force, angle, held, width) from GraspMemory's store.

    Uses the same DINOv2 embedding + cosine retrieval GraspMemory already does
    for the scripted expert, so train/deploy retrieval is identical.
    """
    if getattr(gm, '_emb_store', None) is None and hasattr(gm, '_load_embeddings'):
        gm._load_embeddings()
    store = getattr(gm, '_emb_store', None)
    if not store or 'embeddings' not in store or len(store['embeddings']) == 0:
        return []
    q = gm._embed_dino(image_rgb) if hasattr(gm, '_embed_dino') else None
    if q is None:
        return []
    E = np.asarray(store['embeddings'], dtype=np.float32)
    sims = E @ np.asarray(q, dtype=np.float32)          # cosine (both L2-normed)
    top = np.argsort(-sims)[:k]
    out = []
    names = store.get('names'); forces = store.get('labels')
    for i in top:
        out.append(dict(
            sim=float(sims[i]),
            force=float(forces[i]) if forces is not None else None,
            angle=None, held=None, width_mm=None,   # enriched once V2 logs them per-embedding
            name=str(names[i]) if names is not None else None,
        ))
    return out
