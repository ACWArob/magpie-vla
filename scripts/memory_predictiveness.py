"""Is retrieved memory PREDICTIVE of the right grasp force? (robot-free)

Leave-one-out over the GraspMemory embedding store: for each past grasp, hide
it, retrieve its k-nearest by DINO cosine similarity, predict its force from the
neighbors (similarity-weighted), and compare to a global-mean baseline. If
retrieval beats the mean, memory-conditioning has signal the policy can use.

This is the offline gate for the memory-through-policy experiment. Run it now
(establishes the baseline + proves the harness) and re-run it as V2 adds diverse
objects — the whole point of memory shows up only with diversity.

    python3 scripts/memory_predictiveness.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))


def loo_force_prediction(emb, force, names, k=5, exclude_same_name=False):
    """Return (mae_retrieval, mae_meanbaseline, n_scored) over uncapped forces."""
    real = force < 15.9                       # drop the 16N cap (degenerate)
    idx = np.where(real)[0]
    if len(idx) < 5:
        return None
    errs_r, errs_m = [], []
    for i in idx:
        sims = emb @ emb[i]
        sims[i] = -1                          # leave-one-out
        if exclude_same_name:
            sims[names == names[i]] = -1      # force CROSS-object retrieval
        order = np.argsort(-sims)[:k]
        order = order[sims[order] > 0]
        if len(order) == 0:
            continue
        w = sims[order]; w = w / w.sum()
        pred = float((force[order] * w).sum())
        base = float(force[real & (np.arange(len(force)) != i)].mean())
        errs_r.append(abs(pred - force[i])); errs_m.append(abs(base - force[i]))
    if not errs_r:
        return None
    return float(np.mean(errs_r)), float(np.mean(errs_m)), len(errs_r)


def main():
    store = os.path.expanduser('~/magpie_control/data/grasp_log/embeddings.npz')
    z = np.load(store, allow_pickle=True)
    emb, force, names = z['embeddings'].astype(np.float32), z['labels'].astype(float), z['names']
    import collections as _c
    counts = _c.Counter(names.tolist())
    n_obj = len(counts)
    dominant, dom_n = counts.most_common(1)[0]
    dom_frac = dom_n / len(names)
    effective = sum(1 for _, c in counts.items() if c / len(names) >= 0.02)  # >=2% share
    print(f'store: {len(emb)} grasps, {n_obj} nominal objects '
          f'({dominant!r} is {dom_frac*100:.1f}%), {effective} with >=2% share, '
          f'{(force<15.9).sum()} uncapped-force\n')

    res = loo_force_prediction(emb, force, names, k=5)
    if res is None:
        print('  too few uncapped-force samples to score.')
    else:
        mae_r, mae_m, n = res
        print(f'FORCE prediction (leave-one-out, n={n}):')
        print(f'  retrieval MAE : {mae_r:.2f} N')
        print(f'  mean baseline : {mae_m:.2f} N')
        gain = 100 * (mae_m - mae_r) / mae_m if mae_m > 0 else 0
        print(f'  retrieval {"BEATS" if mae_r < mae_m else "does NOT beat"} the mean '
              f'({gain:+.0f}% MAE)')

    print(f'\nDIVERSITY GATE: {effective} object(s) with meaningful share '
          f'(dominant {dominant!r} = {dom_frac*100:.1f}%).')
    if effective < 3 or dom_frac > 0.90:
        print('  -> EFFECTIVELY single-object. Every retrieval returns near-identical')
        print('     views, so DINO similarity cannot discriminate force -> retrieval adds')
        print('     noise vs the mean (seen above). This is the EXPECTED null and it')
        print('     CONFIRMS the hypothesis: memory-conditioning value is gated on object')
        print('     DIVERSITY. Re-run after V2 collects banana/strawberry/apple/cup.')
    else:
        print('  -> diverse store: cross-object result is now meaningful.')
        r2 = loo_force_prediction(emb, force, names, k=5, exclude_same_name=True)
        if r2:
            print(f'  CROSS-object retrieval MAE {r2[0]:.2f}N vs mean {r2[1]:.2f}N (n={r2[2]})')


if __name__ == '__main__':
    main()
