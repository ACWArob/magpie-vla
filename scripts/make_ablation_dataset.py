#!/usr/bin/env python3
"""Make an input-ablation COPY of lerobot_v1 with chosen observation.state columns dropped.
Videos are symlinked (identical, 449MB — no triplication). Action is untouched (7-dim).
state names: 0=x 1=y 2=z 3=rx 4=ry 5=rz 6=grip_mm 7=grip_force 8=wrist_fz
"""
import sys, json, shutil, os, glob
import numpy as np, pandas as pd
SRC = os.path.expanduser('~/magpie_control/data/lerobot_v1')

def make(out_name, drop_idx):
    keep = [i for i in range(9) if i not in drop_idx]
    OUT = os.path.expanduser(f'~/magpie_control/data/{out_name}')
    if os.path.exists(OUT): shutil.rmtree(OUT)
    os.makedirs(f'{OUT}/meta', exist_ok=True)
    # 1. data parquets — slice observation.state
    for f in glob.glob(f'{SRC}/data/**/*.parquet', recursive=True):
        dst = f'{OUT}/{os.path.relpath(f, SRC)}'; os.makedirs(os.path.dirname(dst), exist_ok=True)
        df = pd.read_parquet(f)
        df['observation.state'] = df['observation.state'].apply(lambda a: np.asarray(a, np.float32)[keep])
        df.to_parquet(dst)
    # 2. videos — symlink (identical content)
    os.symlink(f'{SRC}/videos', f'{OUT}/videos')
    # 3. info.json — new state shape + names
    info = json.load(open(f'{SRC}/meta/info.json'))
    nm = info['features']['observation.state']['names']
    info['features']['observation.state']['names'] = [nm[i] for i in keep]
    info['features']['observation.state']['shape'] = [len(keep)]
    json.dump(info, open(f'{OUT}/meta/info.json','w'), indent=2)
    # 4. stats.json — slice observation.state arrays
    stats = json.load(open(f'{SRC}/meta/stats.json'))
    for k, v in stats['observation.state'].items():
        if isinstance(v, list) and len(v) == 9:
            stats['observation.state'][k] = [v[i] for i in keep]
    json.dump(stats, open(f'{OUT}/meta/stats.json','w'), indent=2)
    shutil.copy(f'{SRC}/meta/tasks.parquet', f'{OUT}/meta/tasks.parquet')
    # 5. per-episode meta stats — slice stats/observation.state/*
    for f in glob.glob(f'{SRC}/meta/episodes/**/*.parquet', recursive=True):
        dst = f'{OUT}/{os.path.relpath(f, SRC)}'; os.makedirs(os.path.dirname(dst), exist_ok=True)
        df = pd.read_parquet(f)
        for c in df.columns:
            if c.startswith('stats/observation.state/'):
                df[c] = df[c].apply(lambda a: np.asarray(a)[keep] if np.asarray(a).ndim==1 and np.asarray(a).shape[0]==9 else a)
        df.to_parquet(dst)
    print(f'{out_name}: state 9 -> {len(keep)}  keep={keep} ({[nm[i] for i in keep]})')

if __name__ == '__main__':
    make('lerobot_v1_noXTheta', [0, 5])      # drop x, rz(yaw="theta")
    make('lerobot_v1_noForce',  [7])         # drop grip_force
    make('lerobot_v1_noBoth',   [0, 5, 7])   # drop x, rz, grip_force
