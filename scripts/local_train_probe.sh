#!/usr/bin/env bash
# Gate 12 probe: measure the lab 2070's actual training rate (run with the robot stack
# DOWN — needs the GPU). 300 steps -> s/step -> extrapolated 150k time. ~10-15 min.
set -euo pipefail
cd ~/magpie_control
python3 - <<'PY'
import sys, types, time
for m in ['lerobot.policies.groot','lerobot.policies.groot.configuration_groot','lerobot.policies.groot.modeling_groot']:
    sys.modules[m] = types.ModuleType(m)
sys.modules['lerobot.policies.groot.configuration_groot'].GrootConfig = type('GrootConfig', (), {})
sys.modules['lerobot.policies.groot.modeling_groot'].GrootPolicy = type('GrootPolicy', (), {})
import subprocess
t0 = time.time()
subprocess.run([sys.executable, '-m', 'lerobot.scripts.lerobot_train',
    '--dataset.repo_id=magpie/v1', '--dataset.root=data/lerobot_v1',
    '--policy.type=act', '--policy.push_to_hub=false', '--policy.device=cuda',
    '--output_dir=/tmp/claude-1001/train_probe', '--batch_size=8',
    '--steps=300', '--save_freq=100000', '--log_freq=50', '--wandb.enable=false'], check=True)
dt = time.time() - t0
sps = dt / 300.
print(f'\n[gate 12] measured: {sps:.3f} s/step on the RTX 2070 (incl. startup amortized)')
print(f'[gate 12] 150k steps locally ≈ {sps*150000/3600:.1f} h  vs  GH200 measured 1.5 h')
PY
