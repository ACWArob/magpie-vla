# Training a V0 Policy on NSF ACCESS (DeltaAI) — Standalone Guide

End-to-end: collected episodes on the lab PC → trained ACT policy from DeltaAI.
No Claude required. Assumes the dataset already exists at `~/magpie_control/data/lerobot_v0`
(created by `magpie_collect.ipynb` — cells 7/8 or the AUTO-COLLECT cell).

**Key facts**
| thing | value |
|---|---|
| Cluster | NCSA **DeltaAI** (GH200 nodes: ARM CPU + H100-class GPU) |
| Login host | `dtai-login.delta.ncsa.illinois.edu` |
| Username | `aabid` |
| Partition | `ghx4` |
| Account string | run `accounts` on DeltaAI (looks like `abcd-dtai-gh`) |
| Dataset (local) | `~/magpie_control/data/lerobot_v0` (LeRobot v0.4.4 format) |
| lerobot version | **must be `0.4.4` on the cluster** — same version that wrote the dataset |

---

## 0. Sanity-check the dataset before pushing

```bash
python3 -c "
import json
info = json.load(open('/home/user/magpie_control/data/lerobot_v0/meta/info.json'))
print('episodes:', info['total_episodes'], '| frames:', info['total_frames'])"
```
You want ≥50 episodes. If this errors, the dataset is broken — don't bother pushing.

## 1. Push the dataset (from the lab PC, in a normal terminal)

```bash
bash ~/magpie_control/scripts/nsf_push_v0.sh
```
- First time: answer `yes` to the host-key prompt, then password + Duo.
- Re-running is safe — rsync only sends what changed (so you can push more episodes later).

Verify it landed:
```bash
ssh aabid@dtai-login.delta.ncsa.illinois.edu 'head -5 ~/lerobot_v0/meta/info.json'
```

## 2. One-time environment setup (on DeltaAI login node)

**This is the recipe that actually works (validated 2026-07-06).** DeltaAI is ARM (GH200):
plain `pip install` gives a CPU-only torch, and the newest pytorch modules (2.10+) ship a
torchvision with `VideoReader` REMOVED (breaks lerobot video decode; torchcodec is unusable
there — no FFmpeg shared libs on the system). The 2.7.0 module has both CUDA torch and a
torchvision with VideoReader.

```bash
ssh aabid@dtai-login.delta.ncsa.illinois.edu
module purge
module load python/miniforge3_pytorch/2.7.0   # CUDA torch 2.7 + torchvision 0.22 (has VideoReader)
module load cuda                               # provides libcufile.so.0 that torch needs
python -m venv --system-site-packages ~/lerobot_env   # --system-site-packages = reuse module torch
source ~/lerobot_env/bin/activate
pip install --upgrade pip
pip install lerobot==0.4.4
pip uninstall -y torch torchvision torchcodec  # remove any CPU torch pip may have added
python -c "import torch, torchvision; print(torch.version.cuda, hasattr(torchvision.io,'VideoReader'))"
# must print: 12.6 True
python -c "
from lerobot.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset('magpie/v0', root='$HOME/lerobot_v0')
s = ds[0]; print('sample OK')"
# must print: sample OK   (this catches video-decode failures BEFORE the job runs)
```

## 3. Get your account string + copy the job script

```bash
accounts                      # note the slurm account, e.g. abcd-dtai-gh
```
Copy `~/magpie_control/scripts/nsf_train_v0.slurm` from the lab PC to DeltaAI
(`scp` it, or nano + paste). Edit its `#SBATCH --account=` line with your string.

## 4. Submit + confirm it is actually training

```bash
sbatch nsf_train_v0.slurm
squeue -u $USER               # want state "R" (running). "PD" = queued.
tail -f act_v0_*.log          # healthy = loss lines every 200 steps
```
**Do not walk away on a 10-second crash log.** If the tail shows a traceback, fix before bed
(most common: wrong account string, dataset path typo, torch without CUDA — see step 2).

If stuck `PD` for >1h: lower `--time=` to `06:00:00` (shorter jobs schedule sooner) and resubmit.

This is the **exact job that worked** (2026-07-07, job 2621694 — 100k steps in ~62 min
on one GH200, loss converged to ~0.043). Write it on DeltaAI as `~/nsf_train_v0.slurm`:

```bash
#!/usr/bin/env bash
#SBATCH --account=bgcd-dtai-gh
#SBATCH --partition=ghx4
#SBATCH --gpus-per-node=1
#SBATCH --job-name=act_v0
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=8
#SBATCH --mem=64G
#SBATCH --time=10:00:00
#SBATCH --output=act_v0_%j.log
set -euo pipefail
module purge
module load python/miniforge3_pytorch/2.7.0
module load cuda
source ~/lerobot_env/bin/activate
python -m lerobot.scripts.lerobot_train \
  --dataset.repo_id=magpie/v0 \
  --dataset.root=$HOME/lerobot_v0 \
  --policy.type=act \
  --policy.push_to_hub=false \
  --policy.device=cuda \
  --output_dir=$HOME/act_v0_run \
  --batch_size=8 \
  --steps=100000 \
  --save_freq=10000 \
  --log_freq=200 \
  --wandb.enable=false
```

Notes from the run that failed first:
- **`--policy.push_to_hub=false` is REQUIRED** — without it the job dies in 33 s with
  `ValueError: 'policy.repo_id' argument missing` (lerobot tries to upload to HuggingFace Hub).
- The module loads must match the env-setup stack exactly (2.7.0 + cuda).
- Actual timing on GH200: ~0.037 s/step → 100k ≈ 1 h, ~33 epochs over 60 episodes.
  Cost: ~1 GPU-hour of the 100-hour allocation.

## 5. Next morning — check + pull the policy back

On DeltaAI:
```bash
tail -3 act_v0_*.log                  # last loss / "DONE"
ls ~/act_v0_run/checkpoints/          # e.g. 010000 ... 100000
```
On the lab PC (note the `pretrained_model/` subfolder — that's the loadable policy;
`last` is a symlink to the newest step):
```bash
mkdir -p ~/magpie_control/models
rsync -avz aabid@dtai-login.delta.ncsa.illinois.edu:'act_v0_run/checkpoints/last/pretrained_model/' \
      ~/magpie_control/models/act_v0/
```
Expected files: `config.json`, `model.safetensors`, `policy_preprocessor*`, `policy_postprocessor*`,
`train_config.json` (~200 MB total).

## 6. Quick smoke-test the checkpoint loads (lab PC)

A plain `from lerobot.policies... import ACTPolicy` CRASHES on the lab PC
(`TypeError: non-default argument 'backbone_cfg' ...`) — lerobot's package init imports EVERY
policy and the unrelated `groot` policy is broken under this Python. Stub it first:

```bash
python3 - <<'EOF'
import sys, types
for m in ['lerobot.policies.groot', 'lerobot.policies.groot.configuration_groot',
          'lerobot.policies.groot.modeling_groot']:
    sys.modules[m] = types.ModuleType(m)
sys.modules['lerobot.policies.groot.configuration_groot'].GrootConfig = type('GrootConfig', (), {})
sys.modules['lerobot.policies.groot.modeling_groot'].GrootPolicy = type('GrootPolicy', (), {})
from lerobot.policies.act.modeling_act import ACTPolicy
p = ACTPolicy.from_pretrained('/home/user/magpie_control/models/act_v0')
print('policy loaded OK:', sum(x.numel() for x in p.parameters())/1e6, 'M params')
EOF
```
Expected: `policy loaded OK: 51.6M params`. The same stub is baked into the deploy notebook.

## 7. Deploy on the arm

`notebooks/v0_test.ipynb` — standalone (no SAM3/GraspGenX; the policy is end-to-end).
Run its 3 setup cells, place the block centrally, arm at the overhead start pose, run the
DEPLOY cell **with a hand on the e-stop**. Safety envelope: XY boxed ±10 cm around the start
pose, TCP z floored at table + GRIPPER_LEN + 3 mm (fingertips can't hit the table), ≤3 cm
motion/tick, orientation within 35° of start, gripper edge-triggered. Interrupt (■) stops it.

---

## Troubleshooting quick table

| symptom | cause | fix |
|---|---|---|
| `Could not resolve hostname` | wrong host | use `dtai-login.delta.ncsa.illinois.edu` |
| `Host key verification failed` | first connect from a script | ssh once by hand, answer `yes` |
| `Invalid account` on sbatch | wrong account string | run `accounts`, copy exactly |
| job stuck `PD` | queue busy | shorter `--time=`, resubmit; or try again in the morning |
| `torch.version.cuda` = None | pip gave a CPU wheel (ARM) | `pip uninstall -y torch torchvision` → module torch takes over |
| `AttributeError: VideoReader` | pytorch module ≥2.10 (torchvision removed it) | use `python/miniforge3_pytorch/2.7.0` |
| `libcufile.so.0` missing | CUDA toolkit module not loaded | `module load cuda` |
| torchcodec `libavutil.so.*` errors | no FFmpeg shared libs on DeltaAI | don't use torchcodec; pyav path works on the 2.7.0 module |
| train crashes on dataset load | lerobot version mismatch | cluster must run **0.4.4** exactly |
| out-of-memory in training | batch too big | halve `--batch_size` (8 → 4) in the slurm file |
| dies in ~30 s: `'policy.repo_id' argument missing` | lerobot defaults to hub upload | add `--policy.push_to_hub=false` |
| lab-PC import crash: `non-default argument 'backbone_cfg'` | broken groot policy in lerobot init | use the groot stub (step 6) |

## Adding more data later
Collect more episodes locally (same notebook — they append to `lerobot_v0`), re-run step 1
(rsync sends only the new files), then resubmit the training job with a fresh `--output_dir`
(e.g. `~/act_v0_run2`) so runs don't overwrite each other.
