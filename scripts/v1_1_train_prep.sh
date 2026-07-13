#!/usr/bin/env bash
# V1.1 + subset-ablation training prep. Run AFTER the V1.1 collection finishes.
# Emits the episode filter lists, pushes the dataset delta, and submits BOTH jobs:
#   act_v1_1  : 182 kept old episodes + all new (>=233) episodes, 150k steps
#   act_v1s60 : 60-episode random subset of the KEPT V1 episodes (seed 7) — the
#               "design beats volume" ablation at V0's exact budget
# Requires: ssh deltaai socket alive (ssh deltaai exit — Duo once).
set -euo pipefail
cd ~/magpie_control

python3 - <<'PY'
import json
info = json.load(open('data/lerobot_v1/meta/info.json'))
filt = json.load(open('data/v1_1_episode_filter.json'))
keep = [e for e in filt['keep'] if e <= 228]              # kept originals only
new  = list(range(232, info['total_episodes']))            # V1.1 replacements (post-stowaway)
v11  = sorted(keep + new)
import random; random.seed(7)
sub60 = sorted(random.sample(keep, 60))
json.dump(v11,  open('/tmp/eps_v1_1.json','w'))
json.dump(sub60,open('/tmp/eps_sub60.json','w'))
print(f'v1.1 episodes: {len(v11)} ({len(keep)} kept + {len(new)} new) | subset: 60 (seed 7)')
assert len(new) >= 40, f'only {len(new)} new episodes — did the V1.1 collection run?'
PY

echo '[prep] pushing dataset delta...'
rsync -az data/lerobot_v1/ deltaai:~/lerobot_v1/
scp -q /tmp/eps_v1_1.json /tmp/eps_sub60.json deltaai:~/

for TAG in v1_1 sub60; do
  [ "$TAG" = v1_1 ] && OUT=act_v1_1_run || OUT=act_v1s60_run
  ssh deltaai "cat > ~/nsf_train_${TAG}.slurm <<EOF
#!/usr/bin/env bash
#SBATCH --account=bgcd-dtai-gh
#SBATCH --partition=ghx4
#SBATCH --gpus-per-node=1
#SBATCH --job-name=act_${TAG}
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=8
#SBATCH --mem=64G
#SBATCH --time=10:00:00
#SBATCH --output=act_${TAG}_%j.log
set -euo pipefail
module purge
module load python/miniforge3_pytorch/2.7.0
module load cuda
source ~/lerobot_env/bin/activate
EPS=\\\$(cat ~/eps_${TAG}.json)
python -m lerobot.scripts.lerobot_train \\
  --dataset.repo_id=magpie/v1 --dataset.root=\\\$HOME/lerobot_v1 \\
  --dataset.episodes=\"\\\$EPS\" \\
  --policy.type=act --policy.push_to_hub=false --policy.device=cuda \\
  --output_dir=\\\$HOME/${OUT} \\
  --batch_size=8 --steps=150000 --save_freq=10000 --log_freq=200 --wandb.enable=false
EOF
sbatch ~/nsf_train_${TAG}.slurm"
done
ssh deltaai 'squeue -u $USER'
echo '[prep] both jobs submitted. Pull later with:'
echo '  rsync -az deltaai:act_v1_1_run/checkpoints/last/pretrained_model/ models/act_v1_1/'
echo '  rsync -az deltaai:act_v1s60_run/checkpoints/last/pretrained_model/ models/act_v1s60/'
