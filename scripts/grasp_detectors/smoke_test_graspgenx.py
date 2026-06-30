#!/usr/bin/env python3
"""GraspGenX smoke test — lowest-stakes check before any arm motion.

Run AFTER starting the GraspGenX server (in its own uv venv / terminal):
    cd ~/GraspGenX && uv run python client-server/graspgenx_server.py \
        --config ext/graspgenx_checkpoints/release --assets_dir assets \
        --default_gripper magpie --port 5557

Then (from the magpie env):
    python3 scripts/grasp_detectors/smoke_test_graspgenx.py [optional_scene.npy]

Confirms: server reachable → magpie-env bridge works → grasps returned for the MAGPIE
gripper → latency + VRAM. No arm needed.
"""
import glob
import os
import subprocess
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from grasp_detectors.graspgenx_zmq import GraspGenXZMQ


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
    n = 4000
    rng = np.random.default_rng(0)
    pts = np.column_stack([rng.uniform(-0.03, 0.03, n),
                           rng.uniform(-0.02, 0.02, n),
                           rng.uniform(-0.02, 0.02, n)]).astype(np.float32)
    pts += np.array([-0.18, -0.48, 0.06], np.float32)        # off-origin like a real scene
    return pts, 'synthetic 6x4x4cm block'


def main():
    host = os.environ.get('GRASPGENX_HOST', 'localhost')
    port = int(os.environ.get('GRASPGENX_PORT', '5557'))
    det = GraspGenXZMQ(host=host, port=port)

    print(f'VRAM before infer : {vram()}')
    if not det.available():
        print(f'\n❌ GraspGenX server not reachable at {host}:{port}')
        print('   Start it first:')
        print('   cd ~/GraspGenX && uv run python client-server/graspgenx_server.py \\')
        print('       --config ext/graspgenx_checkpoints/release --assets_dir assets \\')
        print('       --default_gripper magpie --port 5557')
        return 1
    print('✓ server reachable (MAGPIE gripper)')

    pts, src = get_scene()
    print(f'scene             : {src}  ({len(pts)} pts)')

    lat, res = [], None
    for i in range(3):
        res = det.detect(pts)
        lat.append(res.latency_s)
        print(f'  run {i+1}: {res.n} grasps in {1000*res.latency_s:.0f}ms')
    print(f'VRAM after infer  : {vram()}')

    if res.best:
        g = res.best
        print(f'\nbest grasp: score={g.score:.3f}  angle={g.angle_deg():.1f}deg  '
              f'pos=({g.pose[0,3]:.3f},{g.pose[1,3]:.3f},{g.pose[2,3]:.3f})')
        print(f'median latency: {1000*np.median(lat):.0f}ms')
        print('\n✅ GraspGenX smoke test PASSED — server + magpie bridge working, MAGPIE gripper.')
        return 0
    print('\n⚠ server returned 0 grasps — check the input cloud / gripper config.')
    return 2


if __name__ == '__main__':
    sys.exit(main())
