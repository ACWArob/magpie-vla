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

        # {object_name: {x, P, n}}
        self._priors: dict = self._load_priors()

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
            return {
                'force_mean': p['x'],
                'force_std':  float(np.sqrt(p['P'])),
                'n':          p['n'],
                'source':     'string',
            }

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

    # ── Internal ──────────────────────────────────────────────────────────────

    def _embed(self, image_crop: np.ndarray) -> np.ndarray:
        if self._dino_ok:
            try:
                return _embed_dino(image_crop)
            except Exception:
                pass
        return _embed_histogram(image_crop)

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
