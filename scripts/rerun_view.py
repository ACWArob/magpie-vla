#!/usr/bin/env python3
"""Rerun QA viewer for the magpie LeRobot dataset.

Purpose: eyeball recorded episodes BEFORE training — confirm trajectories are smooth
(not jumpy waypoints), the wrist video lines up with the state/action signals, and the
grasp actually closes/holds. This is the "visualize/QA" gate in the data→viz→train plan.

Shows, time-synced and scrubbable:
  • the wrist camera video
  • all 9 observation.state channels   (x,y,z,rx,ry,rz,grip_mm,grip_force,wrist_fz)
  • all 7 action channels              (x,y,z,rx,ry,rz,grip_cmd)
  • a 3D TCP path coloured by gripper aperture (green=open → red=closed)

Usage (from the magpie env):
  python3 scripts/rerun_view.py                 # all episodes → save .rrd
  python3 scripts/rerun_view.py --episode 0     # one episode
  python3 scripts/rerun_view.py --mode spawn    # local window
  python3 scripts/rerun_view.py --mode web      # browser viewer (remote robot)
Then open the printed .rrd with `rerun <file>` (save mode).
"""
import argparse
import os
import sys

import numpy as np

STATE_NAMES = ['x', 'y', 'z', 'rx', 'ry', 'rz', 'grip_mm', 'grip_force', 'wrist_fz']
ACTION_NAMES = ['x', 'y', 'z', 'rx', 'ry', 'rz', 'grip_cmd']


def _to_hwc_uint8(img):
    """LeRobot image tensor [3,H,W] (float 0-1 or uint8) → HxWx3 uint8."""
    a = img.detach().cpu().numpy() if hasattr(img, 'detach') else np.asarray(img)
    if a.ndim == 3 and a.shape[0] in (1, 3):      # CHW → HWC
        a = np.transpose(a, (1, 2, 0))
    if a.dtype != np.uint8:                       # float 0-1 → 0-255
        a = (np.clip(a, 0.0, 1.0) * 255).astype(np.uint8)
    return a


def _aperture_color(grip_mm, lo, hi):
    """green (open) → red (closed) by aperture."""
    t = 0.0 if hi <= lo else float(np.clip((grip_mm - lo) / (hi - lo), 0, 1))
    return [int(255 * (1 - t)), int(255 * t), 40]   # closed→red, open→green


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', default='~/magpie_control/data/lerobot_magpie')
    ap.add_argument('--repo_id', default='magpie/grasp')
    ap.add_argument('--episode', type=int, default=None, help='single episode (default: all)')
    ap.add_argument('--mode', choices=['save', 'spawn', 'web'], default='save')
    ap.add_argument('--cam', default='observation.images.wrist')
    args = ap.parse_args()

    root = os.path.expanduser(args.root)
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    import rerun as rr

    eps = [args.episode] if args.episode is not None else None
    ds = LeRobotDataset(args.repo_id, root=root, episodes=eps)
    print(f'loaded {ds.num_episodes} episode(s), {ds.num_frames} frames @ {ds.fps}Hz')

    # aperture range across the loaded data → stable colour scale
    grip_all = np.array([float(ds[i]['observation.state'][6]) for i in range(len(ds))])
    glo, ghi = (float(grip_all.min()), float(grip_all.max())) if len(grip_all) else (0., 40.)

    rr.init('magpie_grasp_qa', spawn=(args.mode == 'spawn'))
    if args.mode == 'web':
        rr.serve_web()
        print('  Rerun web viewer started — open the printed URL')
    elif args.mode == 'save':
        out = os.path.join(os.path.dirname(root.rstrip('/')),
                           f'{os.path.basename(root.rstrip("/"))}_qa.rrd')
        rr.save(out)
        print(f'  recording → {out}   (view: `rerun {out}`)')

    path_xyz, path_cols = [], []
    last_ep = None
    for i in range(len(ds)):
        it = ds[i]
        st = it['observation.state'].numpy().astype(float)
        acn = it['action'].numpy().astype(float)
        ep = int(it['episode_index'])
        fr = int(it['frame_index'])
        ts = float(it['timestamp'])

        # reset the 3D path at each episode boundary
        if ep != last_ep:
            path_xyz, path_cols = [], []
            last_ep = ep
            rr.log('task', rr.TextLog(it.get('task') or '(no task)'))

        rr.set_time('frame', sequence=i)
        try:
            rr.set_time('time_s', duration=ts)      # seconds timeline (rerun >=0.23)
        except Exception:
            pass

        rr.log(f'wrist/{args.cam}', rr.Image(_to_hwc_uint8(it[args.cam])))
        for n, v in zip(STATE_NAMES, st):
            rr.log(f'state/{n}', rr.Scalars(float(v)))
        for n, v in zip(ACTION_NAMES, acn):
            rr.log(f'action/{n}', rr.Scalars(float(v)))

        col = _aperture_color(st[6], glo, ghi)
        path_xyz.append([st[0], st[1], st[2]])
        path_cols.append(col)
        # moving TCP marker + the path traced so far (scrubbable)
        rr.log('world/tcp', rr.Points3D([path_xyz[-1]], colors=[col], radii=0.004))
        if len(path_xyz) > 1:
            rr.log('world/path', rr.LineStrips3D([path_xyz], colors=[[120, 120, 255]]))
            rr.log('world/path_pts', rr.Points3D(path_xyz, colors=path_cols, radii=0.002))

        if i % 50 == 0:
            print(f'  frame {i}/{len(ds)}  ep{ep} f{fr}  z={st[2]:.3f} grip={st[6]:.1f}mm '
                  f'force={st[7]:.2f} fz={st[8]:.2f}')

    print('done. ' + (f'open with `rerun {out}`' if args.mode == 'save' else 'viewer live'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
