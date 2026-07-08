#!/usr/bin/env bash
# Baby ACT training on the V0 dataset — THE VERSION THAT WORKED (job 2621694, 2026-07-07:
# 100k steps in ~62 min on one DeltaAI GH200, loss ~0.043).
# Copy to DeltaAI as ~/nsf_train_v0.slurm and run:  sbatch ~/nsf_train_v0.slurm
# Env setup first — see docs/NSF_TRAINING.md step 2 (module 2.7.0 + cuda, venv, lerobot==0.4.4).
#SBATCH --account=bgcd-dtai-gh
#SBATCH --partition=ghx4
#SBATCH --gpus-per-node=1
#SBATCH --job-name=act_v1
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=8
#SBATCH --mem=64G
#SBATCH --time=10:00:00
#SBATCH --output=act_v1_%j.log
set -euo pipefail
module purge
module load python/miniforge3_pytorch/2.7.0
module load cuda
source ~/lerobot_env/bin/activate
python -m lerobot.scripts.lerobot_train \
  --dataset.repo_id=magpie/v1 \
  --dataset.root=$HOME/lerobot_v1 \
  --policy.type=act \
  --policy.push_to_hub=false \
  --policy.device=cuda \
  --output_dir=$HOME/act_v1_run \
  --batch_size=8 \
  --steps=150000 \
  --save_freq=10000 \
  --log_freq=200 \
  --wandb.enable=false
