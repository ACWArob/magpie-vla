#!/usr/bin/env python3
"""GSNet cold-start smoke test — no arm motion.

Runs the GSNetDocker adapter (one-shot `docker run --rm` → load → infer → exit) on a
saved scene cloud or a synthetic cube. Confirms: image present → container runs →
graspness infers → grasps parsed → VRAM is released after (cold-start working).

Expect HIGH latency here (~20-40s): container start + MinkowskiEngine import + model
load dominate. That's the intended trade for zero VRAM coexistence with SAM3.

    python3 scripts/grasp_detectors/smoke_test_gsnet.py [optional_scene.npy]
"""
import glob
import os
import subprocess
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from grasp_detectors.gsnet_docker import GSNetDocker


def vram():
    try:
        return subprocess.check_output(
            ['nvidia-smi', '--query-gpu=memory.used,memory.total', '--format=csv,noheader'],
            text=True).strip()
    except Exception:
        return 'n/a'


def get_scene():
    arg = sys.argv[1] if len(sys.argv) > 1 else None
    npys = sorted(glob.glob(os.path.expanduser('~/magpie_control/data/scenes/*.npy')))
    if arg and os.path.exists(arg):
        return np.load(arg), arg
    if npys:
        return np.load(npys[-1]), npys[-1]
    n = 3000
    pts = np.stack([
        np.random.uniform(-0.025, 0.025, n) - 0.21,
        np.random.uniform(-0.025, 0.025, n) - 0.56,
        np.random.uniform(0.0, 0.05, n) + 0.05], axis=1).astype(np.float32)
    return pts, 'synthetic 5cm cube'


def main():
    det = GSNetDocker()
    print(f'image available : {det.available()}')
    if not det.available():
        print('❌ gsnet:latest image not found — build it first.')
        return 1

    pts, src = get_scene()
    print(f'scene           : {src}  ({len(pts)} pts)')
    print(f'VRAM before     : {vram()}')

    t0 = time.time()
    res = det.detect(pts)
    dt = time.time() - t0
    print(f'cold-start run  : {res.n} grasps in {dt:.1f}s  (load+infer+exit)')
    print(f'VRAM after      : {vram()}   ← should be ~back to before (container exited)')

    if res.best:
        g = res.best
        print(f'\nbest grasp: score={g.score:.3f}  angle={g.angle_deg():.1f}deg  '
              f'pos=({g.pose[0,3]:.3f},{g.pose[1,3]:.3f},{g.pose[2,3]:.3f})  width={g.width*1000:.0f}mm')
        print('\n✅ GSNet cold-start smoke test PASSED — adapter + docker one-shot working.')
        return 0
    print('\n⚠ 0 grasps returned — check the input cloud / checkpoint.')
    return 2


if __name__ == '__main__':
    sys.exit(main())
