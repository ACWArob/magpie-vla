#!/usr/bin/env python3
"""
GraspGen smoke test — lowest-stakes check before any arm motion.

Run AFTER starting the GraspGen server (in its own venv/terminal):
    python ~/GraspGen/client-server/graspgen_server.py \
        --gripper_config ~/GraspGenModels/checkpoints/graspgen_robotiq_2f_140.yml --port 5556

Then (from the magpie env):
    python scripts/grasp_detectors/smoke_test_graspgen.py [optional_scene.npy]

It confirms: server reachable → adapter protocol works → grasps returned → latency,
and prints VRAM before/after so you can see if GraspGen fits alongside SAM3.
"""
import glob
import os
import subprocess
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from grasp_detectors.graspgen_zmq import GraspGenZMQ


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
    n = 2000
    pts = np.stack([
        np.random.uniform(-0.025, 0.025, n) - 0.18,
        np.random.uniform(-0.025, 0.025, n) - 0.53,
        np.random.uniform(0.0, 0.05, n) + 0.05], axis=1).astype(np.float32)
    return pts, 'synthetic 5cm cube'


def main():
    host = os.environ.get('GRASPGEN_HOST', 'localhost')
    port = int(os.environ.get('GRASPGEN_PORT', '5556'))
    det = GraspGenZMQ(host=host, port=port)

    print(f'VRAM before infer : {vram()}')
    if not det.available():
        print(f'\n❌ GraspGen server not reachable at {host}:{port}')
        print('   Start it first:')
        print('   python ~/GraspGen/client-server/graspgen_server.py \\')
        print('       --gripper_config ~/GraspGenModels/checkpoints/graspgen_robotiq_2f_140.yml --port 5556')
        return 1
    print('✓ server reachable')

    pts, src = get_scene()
    print(f'scene             : {src}  ({len(pts)} pts)')

    lat, res = [], None
    for i in range(3):
        res = det.detect(pts)
        lat.append(res.latency_s)
        print(f'  run {i+1}: {res.n} grasps in {1000*res.latency_s:.0f}ms')
    print(f'VRAM after infer  : {vram()}   ← compare to "before" to see GraspGen footprint')

    if res.best:
        g = res.best
        print(f'\nbest grasp: score={g.score:.3f}  angle={g.angle_deg():.1f}deg  '
              f'pos=({g.pose[0,3]:.3f},{g.pose[1,3]:.3f},{g.pose[2,3]:.3f})')
        print(f'median latency: {1000*np.median(lat):.0f}ms')
        print('\n✅ GraspGen smoke test PASSED — server + adapter working, no arm needed.')
        return 0
    print('\n⚠ server returned 0 grasps — check the input cloud / gripper config.')
    return 2


if __name__ == '__main__':
    sys.exit(main())
