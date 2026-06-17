"""
Reset grasp memory + logs to a clean, consistent baseline.

Archives the current data/grasp_log to a timestamped backup, clears it, and
re-seeds the YCB force priors. Use this before a fresh data-collection run so
every session starts from the same state.

    python3 scripts/reset_grasp_data.py            # backup + clear + reseed
    python3 scripts/reset_grasp_data.py --no-seed  # leave priors empty (cold)
    python3 scripts/reset_grasp_data.py --no-backup # discard old data (careful)

IMPORTANT: restart the notebook kernel afterwards so GraspMemory reloads the
clean state from disk (the in-memory gm would otherwise overwrite the reset).
"""

import argparse
import os
import pathlib
import shutil
import time

DATA_DIR = pathlib.Path('~/magpie_control/data/grasp_log').expanduser()


def reset(do_backup=True, do_seed=True):
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    files = [f for f in DATA_DIR.iterdir() if f.is_file()]

    if files and do_backup:
        ts  = time.strftime('%Y%m%d_%H%M%S')
        bak = DATA_DIR.parent / f'grasp_log_backup_{ts}'
        bak.mkdir(parents=True, exist_ok=True)
        for f in files:
            shutil.move(str(f), str(bak / f.name))
        print(f'Archived {len(files)} files → {bak}')
    else:
        for f in files:
            f.unlink()
        if files:
            print(f'Deleted {len(files)} files (no backup)')

    print('Cleared: force_priors.json, embeddings.npz, orientation_log.json, '
          'grasps_*.jsonl, images')

    if do_seed:
        import sys
        sys.path.insert(0, os.path.dirname(__file__))
        from seed_ycb_priors import seed
        seed(str(DATA_DIR))
    else:
        print('Skipped seeding — priors start empty (cold).')

    print('\nDONE. Restart the notebook kernel so GraspMemory reloads the clean state.')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--no-backup', action='store_true', help='delete instead of archive')
    ap.add_argument('--no-seed',   action='store_true', help='leave force priors empty')
    args = ap.parse_args()
    reset(do_backup=not args.no_backup, do_seed=not args.no_seed)
