"""
GraspMemory — persistent force priors for VLA training.

Two-layer lookup:
  1. DINO RAG   — embed detection crop with DINOv2 ViT-S, retrieve k nearest
                  past grasps by cosine similarity, weighted-mean force.
  2. String     — fallback when no embeddings stored yet; exact object-name match.

Force estimate is tracked per object with a 1-D Kalman filter:
  state  x = force needed (N)
  P      = uncertainty (N²)
  Q      = 0.25  process noise  (force can vary between placements)
  R      = 0.50  measurement noise (slip guard + pose variability)

Usage (notebook):
    from grasp_memory import GraspMemory
    gm = GraspMemory('~/magpie_control/data/grasp_log')

    prior = gm.get_prior(ACTIVE, crop_rgb)      # before grasp
    gm.update(ACTIVE, result['true_force_label'], crop_rgb)  # after grasp
"""

import json
import os
import pathlib
import time

import numpy as np

# ── Kalman constants ──────────────────────────────────────────────────────────
_Q = 0.25   # process noise  (N²) — how much force varies between placements
_R = 0.50   # measurement noise (N²) — slip-guard + placement variability
_P0 = 4.0   # initial uncertainty (N²) — wide prior on first grasp
# Novelty guard: a named prior is keyed on the object NAME only. Before trusting it,
# check the object we're actually looking at matches what the prior was learned from
# (cosine sim of DINO embeddings). Below this, treat it as a different item — same
# name, different object (e.g. a much bigger/smaller "red block"). Same object across
# poses/lighting usually scores 0.6–0.9; different objects 0.2–0.5. Tunable.
_NAMED_MATCH_MIN = 0.5

# ── DINO ─────────────────────────────────────────────────────────────────────
_DINO_MODEL  = None   # lazy-loaded
_DINO_DEVICE = 'cpu'
_DINO_DIM    = 384    # ViT-S output dim


def _load_dino():
    global _DINO_MODEL
    if _DINO_MODEL is not None:
        return True
    try:
        import torch
        from torchvision import transforms
        _DINO_MODEL = torch.hub.load(
            'facebookresearch/dinov2', 'dinov2_vits14',
            pretrained=True, verbose=False)
        _DINO_MODEL.eval()
        _DINO_MODEL._transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                 std=[0.229, 0.224, 0.225]),
        ])
        print('[GraspMemory] DINOv2 ViT-S loaded (CPU)')
        return True
    except Exception as e:
        print(f'[GraspMemory] DINOv2 unavailable ({e}) — using colour histogram fallback')
        return False


def _embed_dino(image_rgb: np.ndarray) -> np.ndarray:
    import torch
    x = _DINO_MODEL._transform(image_rgb).unsqueeze(0)
    with torch.no_grad():
        feat = _DINO_MODEL(x).squeeze().numpy()
    return feat / (np.linalg.norm(feat) + 1e-8)


def _embed_histogram(image_rgb: np.ndarray) -> np.ndarray:
    """64-bin RGB histogram — fast fallback when DINO is unavailable.
    L2-normalised so dot product == cosine similarity, matching DINO embeddings."""
    h = np.concatenate([
        np.histogram(image_rgb[:, :, c], bins=64, range=(0, 256))[0]
        for c in range(3)
    ]).astype(float)
    return h / (np.linalg.norm(h) + 1e-8)


class GraspMemory:
    def __init__(self, data_dir: str = '~/magpie_control/data/grasp_log'):
        self._dir = pathlib.Path(data_dir).expanduser()
        self._dir.mkdir(parents=True, exist_ok=True)

        self._priors_path = self._dir / 'force_priors.json'
        self._emb_path    = self._dir / 'embeddings.npz'
        self._orient_path = self._dir / 'orientation_log.json'
        self._shape_path  = self._dir / 'shape_log.json'

        # {object_name: {x, P, n}}
        self._priors: dict = self._load_priors()

        # {object_name: [ {angle, strategy, held, quality, ts}, ... ]}
        # Orientation outcome history — drives get_angle_advice (VLA feedback loop)
        self._orient: dict = self._load_orient()

        # {object_name: [ {major, minor, height, ts}, ... ]}
        # Per-object shape observations (mm). Same object grasped repeatedly → builds a
        # persistent geometry "model" that's more robust than any single noisy top-down
        # scan. Drives get_shape (sanity-check / fallback for grasp width + strategy).
        self._shapes: dict = self._load_shapes()

        # Embedding store — loaded lazily
        self._emb_store: dict | None = None   # {embeddings, labels, names}

        # Try to load DINO now so first grasp isn't slow
        self._dino_ok = _load_dino()

    # ── Public API ────────────────────────────────────────────────────────────

    def get_prior(self, object_name: str,
                  image_crop: np.ndarray | None = None) -> dict:
        """Return force prior for this object.

        Returns a dict with:
            force_mean  — best estimate of force needed (N)
            force_std   — 1-sigma uncertainty (N)
            n           — number of past grasps of THIS specific object
            source      — 'string' | 'dino_rag' | 'none'

        Priority: named Kalman state first, DINO RAG for truly new objects only.
        n always reflects per-object count so skip-DeliGrasp triggers correctly.
        """
        key = object_name.lower().strip()

        # 1. Named Kalman state — most accurate for known objects
        if key in self._priors:
            p = self._priors[key]
            out = {
                'force_mean': p['x'],
                'force_std':  float(np.sqrt(p['P'])),
                'n':          p['n'],
                'source':     'string',
                'match_sim':  None,
                'mismatch':   False,
            }
            # Novelty guard: the prior is keyed on NAME only. Verify the object in
            # front of us actually matches what this prior was learned from — a
            # different "red block" (much bigger/smaller, different material) must NOT
            # inherit a confident force prior. If the best visual match to this name's
            # own stored grasps is weak, distrust it: report n=0 so the caller treats
            # it as a new item (DeliGrasp runs, no fast-skip, no heavy memory blend).
            if image_crop is not None and self._dino_ok:
                sim = self._named_match(image_crop, key)
                if sim is not None:
                    out['match_sim'] = sim
                    if sim < _NAMED_MATCH_MIN:
                        out['mismatch'] = True
                        out['n'] = 0
            return out

        # 2. DINO RAG — bootstrap for truly new objects (no named prior yet)
        #    n=0 so skip-DeliGrasp never fires on a new object's first grasp
        if image_crop is not None and self._dino_ok:
            store = self._load_emb_store()
            if store is not None and len(store['labels']) >= 3:
                result = self._rag_lookup(image_crop, store)
                if result is not None:
                    result['n'] = 0   # new object — no per-object history yet
                    return result

        return {'force_mean': None, 'force_std': None, 'n': 0, 'source': 'none'}

    def update(self, object_name: str, true_force: float,
               image_crop: np.ndarray | None = None) -> None:
        """Kalman update after a grasp. Saves priors + embedding to disk."""
        key = object_name.lower().strip()

        # Kalman update
        if key not in self._priors:
            self._priors[key] = {'x': true_force, 'P': _P0, 'n': 1}
        else:
            p = self._priors[key]
            P_pred = p['P'] + _Q
            K      = P_pred / (P_pred + _R)
            p['x'] = p['x'] + K * (true_force - p['x'])
            p['P'] = (1.0 - K) * P_pred
            p['n'] += 1

        self._save_priors()

        # Store DINO/histogram embedding
        if image_crop is not None:
            emb = self._embed(image_crop)
            self._append_embedding(emb, true_force, key)

    # ── Orientation feedback loop (VLA) ─────────────────────────────────────────

    @staticmethod
    def _circ180(a: float, b: float) -> float:
        """Angular distance on a 180° period — a gripper line is symmetric, so
        an angle θ and θ+180° produce the same grasp."""
        d = abs((a % 180.) - (b % 180.))
        return min(d, 180. - d)

    def record_outcome(self, object_name: str, angle_deg: float, strategy: str,
                       held: bool, quality: float | None = None,
                       pca_angle: float | None = None) -> None:
        """Log a grasp outcome at a given orientation. Feeds get_angle_advice and
        get_angle_correction so the system learns which orientations slip AND the
        systematic PCA→good-angle correction for THIS object.

        pca_angle: the RAW PCA angle before any correction. Storing it lets us learn
        the pose-invariant correction delta (final_angle - pca_angle)."""
        key = object_name.lower().strip()
        self._orient.setdefault(key, []).append({
            'angle':     float(angle_deg) % 180.,
            'pca_angle': float(pca_angle) % 180. if pca_angle is not None else None,
            'strategy':  strategy,
            'held':      bool(held),
            'quality':   float(quality) if quality is not None else None,
            'ts':        time.strftime('%Y-%m-%dT%H:%M:%S'),
        })
        self._orient[key] = self._orient[key][-50:]   # keep last 50 per object
        self._save_orient()

    @staticmethod
    def _wrap90(d: float) -> float:
        """Wrap an angle delta into (-90, 90] — the gripper line is symmetric mod
        180°, so the meaningful correction is the smallest signed rotation."""
        return (d + 90.) % 180. - 90.

    def get_angle_correction(self, object_name: str, min_n: int = 3) -> dict | None:
        """Learn the systematic correction between the raw PCA grasp angle and the
        angle that actually produced HELD grasps. This is a delta, so it is
        pose-invariant and generalises across placements (a cube placed at any
        rotation gets the same 'PCA reads diagonal, rotate to a face' fix).

        Returns {delta, n, mean_quality, spread_deg, confident} or None.
        'confident' → trust it and skip the per-grasp Gemini visual check.
        """
        key  = object_name.lower().strip()
        hist = self._orient.get(key, [])
        # Learn ONLY from good grasps: held AND not scored poorly. A held-but-bad-angle
        # grasp gets a low quality score (issue=angle) from the reward — including it
        # would teach the wrong correction, so exclude quality < 0.6. Unscored (None)
        # held grasps are kept (benefit of the doubt until the reward runs).
        samples = [h for h in hist
                   if h['held'] and h.get('pca_angle') is not None
                   and (h.get('quality') is None or h['quality'] >= 0.6)]
        if len(samples) < min_n:
            return None
        deltas = np.array([self._wrap90(h['angle'] - h['pca_angle'])
                           for h in samples])
        # Circular mean on a 180° period (double-angle trick), then halve back.
        ang2       = np.radians(deltas * 2.)
        mean_delta = float(np.degrees(np.arctan2(np.sin(ang2).mean(),
                                                 np.cos(ang2).mean())) / 2.)
        spread = float(np.std(deltas))
        quals  = [h['quality'] for h in samples if h.get('quality') is not None]
        mq     = float(np.mean(quals)) if quals else 0.6
        return {
            'delta':        mean_delta,
            'n':            len(samples),
            'mean_quality': mq,
            'spread_deg':   spread,
            'confident':    len(samples) >= 5 and spread < 15. and mq >= 0.65,
        }

    def set_last_quality(self, object_name: str, quality: float) -> None:
        """Patch the quality of the most recent outcome (dg_summary runs after
        record_outcome, once the reward score is known)."""
        key = object_name.lower().strip()
        if self._orient.get(key):
            self._orient[key][-1]['quality'] = float(quality)
            self._save_orient()

    def get_angle_advice(self, object_name: str, proposed_angle_deg: float,
                         symmetric: bool = False, tol_deg: float = 25.) -> dict:
        """Has this orientation failed before for this object? If so, recommend a
        better angle. Core of the orientation feedback loop.

        Returns dict with:
            recommend_angle      — angle to actually use (may differ from proposed)
            overridden           — True if we changed it from the proposed angle
            n_at_proposed        — past attempts near the proposed angle
            success_at_proposed  — how many of those held
            rate_at_proposed     — success fraction (None if no history)
            reason               — human-readable explanation (also a VLA label)
        """
        key  = object_name.lower().strip()
        hist = self._orient.get(key, [])
        pa   = float(proposed_angle_deg) % 180.

        near      = [h for h in hist if self._circ180(h['angle'], pa) <= tol_deg]
        n_near    = len(near)
        succ_near = sum(1 for h in near if h['held'])
        rate      = (succ_near / n_near) if n_near else None

        out = {
            'proposed_angle':      float(proposed_angle_deg),
            'recommend_angle':     float(proposed_angle_deg),
            'n_at_proposed':       n_near,
            'success_at_proposed': succ_near,
            'rate_at_proposed':    rate,
            'overridden':          False,
            'reason': ('no orientation history' if n_near == 0
                       else f'{succ_near}/{n_near} held near {pa:.0f}°'),
        }

        # Intervene only with ≥2 attempts AND a majority that slipped.
        if n_near >= 2 and rate is not None and rate < 0.5:
            fails = n_near - succ_near
            alt = self._best_alt_angle(hist, avoid=pa, tol=tol_deg)
            if alt is not None:
                out['recommend_angle'] = alt
                out['overridden'] = True
                out['reason'] = (f'orientation ~{pa:.0f}° slipped {fails}/{n_near} times; '
                                 f'using historically better ~{alt:.0f}°')
            elif symmetric:
                out['recommend_angle'] = (float(proposed_angle_deg) + 90.) % 360.
                out['overridden'] = True
                out['reason'] = (f'orientation ~{pa:.0f}° slipped {fails}/{n_near} times; '
                                 f'object symmetric → rotating 90° to grip the other axis')
            else:
                out['reason'] = (f'orientation ~{pa:.0f}° slipped {fails}/{n_near} times; '
                                 f'no better angle known yet (not symmetric)')
        return out

    def _best_alt_angle(self, hist: list, avoid: float, tol: float):
        """Best historically-successful angle bucket that is far from `avoid`.
        Returns None if no alternate holds >50% of the time."""
        buckets: dict = {}
        for h in hist:
            b = (round((h['angle'] % 180.) / tol) * tol) % 180.
            n_s = buckets.setdefault(b, [0, 0])
            n_s[0] += 1
            n_s[1] += 1 if h['held'] else 0
        best, best_rate = None, 0.5   # alternate must beat 50% historical hold
        for ang, (n, s) in buckets.items():
            if self._circ180(ang, avoid) <= tol:
                continue
            rate = s / n
            if rate > best_rate:
                best, best_rate = ang, rate
        return best

    def _load_orient(self) -> dict:
        if self._orient_path.exists():
            with open(self._orient_path) as f:
                return json.load(f)
        return {}

    def _save_orient(self) -> None:
        with open(self._orient_path, 'w') as f:
            json.dump(self._orient, f, indent=2)

    def _load_shapes(self) -> dict:
        if self._shape_path.exists():
            with open(self._shape_path) as f:
                return json.load(f)
        return {}

    def _save_shapes(self) -> None:
        with open(self._shape_path, 'w') as f:
            json.dump(self._shapes, f, indent=2)

    # ── Shape model (accumulated geometry per object) ───────────────────────────
    def record_shape(self, object_name: str, major_mm: float, minor_mm: float,
                     height_mm: float | None = None) -> None:
        """Log one shape observation (top-down PCA extents + height, in mm). Over many
        grasps of the same object this accumulates into a persistent geometry model."""
        key = object_name.lower().strip()
        # store major >= minor so orientation in-plane doesn't scramble the stats
        _maj, _min = (float(major_mm), float(minor_mm))
        if _min > _maj:
            _maj, _min = _min, _maj
        self._shapes.setdefault(key, []).append({
            'major':  _maj,
            'minor':  _min,
            'height': float(height_mm) if height_mm is not None else None,
            'ts':     time.strftime('%Y-%m-%dT%H:%M:%S'),
        })
        self._shapes[key] = self._shapes[key][-50:]   # keep last 50
        self._save_shapes()

    def record_cloud(self, object_name: str, pts, held: bool | None = None,
                     angle_deg: float | None = None) -> str | None:
        """Persist one denoised object cloud (world frame) for later 3D-model fitting.

        Stored centroid-subtracted (position factored out) but with orientation AS
        OBSERVED — the offline fitter aligns the partial views across orientations into
        one canonical model. One compressed .npz per grasp under
        data/grasp_log/clouds/<object>/. Pure data hoarding; nothing reads it live yet."""
        key = object_name.lower().strip()
        pts = np.asarray(pts, dtype=np.float64)
        if len(pts) < 20:
            return None
        ctr = pts.mean(axis=0)
        rel = (pts - ctr).astype(np.float32)
        d = self._dir / 'clouds' / key
        d.mkdir(parents=True, exist_ok=True)
        fn = d / (time.strftime('%Y%m%d_%H%M%S_%f')[:-3] + '.npz')
        np.savez_compressed(
            fn, points=rel, centroid=ctr.astype(np.float32),
            held=bool(held) if held is not None else False,
            angle_deg=float(angle_deg) if angle_deg is not None else 0.,
            object=key)
        return str(fn)

    def count_clouds(self, object_name: str) -> int:
        """How many partial-view clouds have been saved for this object."""
        key = object_name.lower().strip()
        d = self._dir / 'clouds' / key
        return len(list(d.glob('*.npz'))) if d.exists() else 0

    def record_image(self, object_name: str, image_crop, held: bool | None = None) -> str | None:
        """Keep at least one reference image of the object in its record, so it can be
        pulled later (Gemini comparison, human inspection) instead of re-analysing a
        fresh frame every time. Ensures one exists ASAP; refreshes from held grasps."""
        key = object_name.lower().strip()
        arr = np.asarray(image_crop)
        if arr.size == 0 or arr.ndim != 3:
            return None
        d = self._dir / 'object_images'
        d.mkdir(parents=True, exist_ok=True)
        fn = d / f'{key}.jpg'
        if fn.exists() and not held:
            return str(fn)            # keep existing unless we have a fresh held view
        try:
            import cv2
            cv2.imwrite(str(fn), cv2.cvtColor(arr, cv2.COLOR_RGB2BGR))
            return str(fn)
        except Exception:
            return None

    def get_image_path(self, object_name: str) -> str | None:
        """Path to the stored reference image for this object, or None."""
        key = object_name.lower().strip()
        fn = self._dir / 'object_images' / f'{key}.jpg'
        return str(fn) if fn.exists() else None

    def get_shape(self, object_name: str, min_n: int = 2) -> dict | None:
        """Robust accumulated geometry for this object, or None if too few records.

        Returns {major_mm, minor_mm, height_mm, major_std, minor_std, ratio, n}.
        Uses medians so a single bad scan can't move the model much."""
        key  = object_name.lower().strip()
        hist = self._shapes.get(key, [])
        if len(hist) < min_n:
            return None
        majors = np.array([h['major'] for h in hist], float)
        minors = np.array([h['minor'] for h in hist], float)
        heights = [h['height'] for h in hist if h.get('height') is not None]
        maj_med, min_med = float(np.median(majors)), float(np.median(minors))
        return {
            'major_mm':  maj_med,
            'minor_mm':  min_med,
            'height_mm': float(np.median(heights)) if heights else None,
            'major_std': float(np.std(majors)),
            'minor_std': float(np.std(minors)),
            'ratio':     maj_med / max(min_med, 1e-6),
            'n':         len(hist),
        }

    # ── Internal ──────────────────────────────────────────────────────────────

    def _embed(self, image_crop: np.ndarray) -> np.ndarray:
        if self._dino_ok:
            try:
                return _embed_dino(image_crop)
            except Exception:
                pass
        return _embed_histogram(image_crop)

    def _named_match(self, image_crop: np.ndarray, key: str) -> float | None:
        """Max cosine similarity between the current crop and the grasp embeddings
        stored under `key`. Used to sanity-check a named prior against the object we
        actually see. None if no embeddings exist for this name yet."""
        store = self._load_emb_store()
        if store is None or len(store['labels']) == 0:
            return None
        sel = store['names'] == key
        if not sel.any():
            return None
        emb  = self._embed(image_crop)
        sims = store['embeddings'][sel] @ emb   # embeddings are L2-normed → cosine
        return float(sims.max())

    def _rag_lookup(self, image_crop: np.ndarray, store: dict,
                    k: int = 5) -> dict | None:
        emb  = self._embed(image_crop)
        sims = store['embeddings'] @ emb           # cosine (embeddings are L2-normed)
        if sims.max() < 0.5:
            return None                            # no confident match
        top_k   = np.argsort(sims)[-k:]
        weights = sims[top_k].clip(0)
        if weights.sum() < 1e-8:
            return None
        weights /= weights.sum()
        force_mean = float(np.dot(weights, store['labels'][top_k]))
        # Variance of retrieved samples as uncertainty estimate
        force_var  = float(np.dot(weights, (store['labels'][top_k] - force_mean) ** 2))
        return {
            'force_mean': force_mean,
            'force_std':  float(np.sqrt(force_var + _R)),
            'n':          int(len(store['labels'])),
            'source':     'dino_rag',
        }

    def _load_priors(self) -> dict:
        if self._priors_path.exists():
            with open(self._priors_path) as f:
                return json.load(f)
        return {}

    def _save_priors(self) -> None:
        with open(self._priors_path, 'w') as f:
            json.dump(self._priors, f, indent=2)

    def _load_emb_store(self) -> dict | None:
        if self._emb_store is not None:
            return self._emb_store
        if not self._emb_path.exists():
            return None
        d = np.load(self._emb_path, allow_pickle=True)
        self._emb_store = {
            'embeddings': d['embeddings'],
            'labels':     d['labels'],
            'names':      d['names'],
        }
        return self._emb_store

    def _append_embedding(self, emb: np.ndarray, label: float, name: str) -> None:
        store = self._load_emb_store()
        if store is None:
            store = {
                'embeddings': emb[np.newaxis],
                'labels':     np.array([label]),
                'names':      np.array([name]),
            }
        else:
            store['embeddings'] = np.vstack([store['embeddings'], emb[np.newaxis]])
            store['labels']     = np.append(store['labels'], label)
            store['names']      = np.append(store['names'], name)
        self._emb_store = store
        np.savez(self._emb_path,
                 embeddings=store['embeddings'],
                 labels=store['labels'],
                 names=store['names'])
