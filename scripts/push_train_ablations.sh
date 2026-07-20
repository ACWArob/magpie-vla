#!/usr/bin/env bash
# Push the 3 input-ablation datasets to DeltaAI and submit 3 training jobs.
# Prereq: ssh deltaai exit  (Duo once — ControlMaster socket). Run from ~/magpie_control.
set -euo pipefail
R=deltaai
say(){ echo "[ablations $(date +%H:%M)] $*"; }

ssh -o BatchMode=yes "$R" 'echo ok' >/dev/null || { echo "No SSH socket. Run: ssh deltaai exit"; exit 1; }

# 1. make sure the video-bearing base dataset is current on NSF (incremental)
say "syncing base videos (incremental)..."
rsync -az --copy-unsafe-links data/lerobot_v1/ "$R:~/lerobot_v1/"

# 2. push each ablation's data+meta (small) and symlink its videos to the base
for D in noXTheta noForce noBoth; do
  say "pushing lerobot_v1_$D (data+meta)..."
  rsync -az data/lerobot_v1_$D/data data/lerobot_v1_$D/meta "$R:~/lerobot_v1_$D/"
  ssh "$R" "ln -sfn ~/lerobot_v1/videos ~/lerobot_v1_$D/videos"
done

# 3. smoke-test one sample loads on NSF (catches video-decode issues before the queue)
say "smoke-testing loads on NSF..."
ssh "$R" "source ~/lerobot_env/bin/activate && python -c \"
from lerobot.datasets.lerobot_dataset import LeRobotDataset
for n,e in [('lerobot_v1_noXTheta',7),('lerobot_v1_noForce',8),('lerobot_v1_noBoth',6)]:
    d=LeRobotDataset('magpie/v1',root='\$HOME/'+n); s=d[0]
    print(n,'state',s['observation.state'].shape[0],'expected',e)\""

# 4. write + submit a slurm job per ablation
for D in noXTheta noForce noBoth; do
  ssh "$R" "cat > ~/train_$D.slurm <<EOF
#!/usr/bin/env bash
#SBATCH --account=bgcd-dtai-gh
#SBATCH --partition=ghx4
#SBATCH --gpus-per-node=1
#SBATCH --job-name=act_$D
#SBATCH --cpus-per-task=8
#SBATCH --mem=64G
#SBATCH --time=03:00:00
#SBATCH --output=act_${D}_%j.log
set -euo pipefail
module purge && module load python/miniforge3_pytorch/2.7.0 cuda
source ~/lerobot_env/bin/activate
python -m lerobot.scripts.lerobot_train \\
  --dataset.repo_id=magpie/v1 --dataset.root=\\\$HOME/lerobot_v1_$D \\
  --policy.type=act --policy.push_to_hub=false --policy.device=cuda \\
  --output_dir=\\\$HOME/act_${D}_run \\
  --batch_size=8 --steps=150000 --save_freq=10000 --log_freq=200 --wandb.enable=false
EOF
sbatch ~/train_$D.slurm"
done
say "submitted. check: ssh deltaai squeue -u \$USER"
