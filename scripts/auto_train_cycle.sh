#!/usr/bin/env bash
# Fully automated: audit -> push -> verify -> train -> pull. Run overnight, wake to a policy.
#
# ONE-TIME SETUP (enables no-reauth automation through a single Duo login):
#   1. Add to ~/.ssh/config:
#        Host deltaai
#          HostName dtai-login.delta.ncsa.illinois.edu
#          User aabid
#          ControlMaster auto
#          ControlPath ~/.ssh/cm-%r@%h-%p
#          ControlPersist 12h
#          ServerAliveInterval 60
#   2. Before leaving: `ssh deltaai exit`  (password + Duo ONCE — socket then persists 12h)
#   3. Then:  nohup bash scripts/auto_train_cycle.sh v1 150000 > /tmp/auto_train.log 2>&1 &
#
# Usage: auto_train_cycle.sh <tag> [steps]     e.g.  auto_train_cycle.sh v1 150000
set -euo pipefail
TAG="${1:?usage: auto_train_cycle.sh <tag> [steps]}"
STEPS="${2:-150000}"
DATA="$HOME/magpie_control/data/lerobot_${TAG}"
REMOTE="deltaai"   # ssh-config alias with ControlMaster (see header)

say() { echo "[auto-train $(date +%H:%M)] $*"; }

# ── 0. connection check (fails fast if the ControlMaster socket isn't up) ──
ssh -o BatchMode=yes "$REMOTE" 'echo ok' >/dev/null 2>&1 \
  || { say "NO live SSH session. Run:  ssh deltaai exit   (Duo once), then retry."; exit 1; }
say "SSH socket alive ✓"

# ── 1. local audit (hard gate — refuses to ship a bad dataset) ──
python3 - "$DATA" <<'PY'
import json, sys, glob, collections
import numpy as np
data = sys.argv[1]
info = json.load(open(f'{data}/meta/info.json'))
assert info['total_episodes'] >= 50, f"only {info['total_episodes']} episodes"
recs = [json.loads(l) for l in open(data + '_attempts.jsonl') if l.strip()]
kept = [r for r in recs if r.get('kept')]
angs = np.array([r['grasp_angle_deg'] for r in kept if r.get('grasp_angle_deg') is not None])
h, _ = np.histogram(angs, bins=[0,7.5,22.5,37.5,52.5,67.5,82.5,90.001])
worst = h.max() / max(h.sum(), 1)
assert worst < 0.45, f"angle collapse: {worst*100:.0f}% in one bin"
print(f"[audit] {info['total_episodes']} eps, worst angle bin {worst*100:.0f}% — PASS")
PY

# ── 2. push (incremental) + remote smoke test ──
say "pushing dataset..."
rsync -az "$DATA/" "$REMOTE:~/lerobot_${TAG}/"
ssh "$REMOTE" "source ~/lerobot_env/bin/activate && python -c \"
from lerobot.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset('magpie/${TAG}', root='\$HOME/lerobot_${TAG}')
s = ds[0]; print('[remote] episodes:', ds.num_episodes, '- sample OK')\""

# ── 3. write + submit the job ──
say "submitting training (${STEPS} steps)..."
ssh "$REMOTE" "cat > ~/nsf_train_${TAG}.slurm <<EOF
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
python -m lerobot.scripts.lerobot_train \\
  --dataset.repo_id=magpie/${TAG} --dataset.root=\\\$HOME/lerobot_${TAG} \\
  --policy.type=act --policy.push_to_hub=false --policy.device=cuda \\
  --output_dir=\\\$HOME/act_${TAG}_run \\
  --batch_size=8 --steps=${STEPS} --save_freq=10000 --log_freq=200 --wandb.enable=false
EOF
sbatch ~/nsf_train_${TAG}.slurm"

# ── 4. wait for the job (poll every 5 min; catch instant crashes first) ──
sleep 120
ssh "$REMOTE" "tail -5 act_${TAG}_*.log 2>/dev/null" | grep -qE "loss:|PD|Submitted" \
  || { say "no training progress after 2 min — check the log:"; ssh "$REMOTE" "tail -20 act_${TAG}_*.log"; exit 1; }
say "training confirmed — polling every 5 min..."
while ssh "$REMOTE" "squeue -u \$USER -h -n act_${TAG}" | grep -q .; do sleep 300; done
ssh "$REMOTE" "tail -3 act_${TAG}_*.log"

# ── 5. pull the policy home + local load test ──
say "pulling policy..."
mkdir -p "$HOME/magpie_control/models/act_${TAG}"
rsync -az "$REMOTE:~/act_${TAG}_run/checkpoints/last/pretrained_model/" \
      "$HOME/magpie_control/models/act_${TAG}/"
python3 - "$TAG" <<'PY'
import sys, types
for m in ['lerobot.policies.groot', 'lerobot.policies.groot.configuration_groot',
          'lerobot.policies.groot.modeling_groot']:
    sys.modules[m] = types.ModuleType(m)
sys.modules['lerobot.policies.groot.configuration_groot'].GrootConfig = type('GrootConfig', (), {})
sys.modules['lerobot.policies.groot.modeling_groot'].GrootPolicy = type('GrootPolicy', (), {})
from lerobot.policies.act.modeling_act import ACTPolicy
p = ACTPolicy.from_pretrained(f'/home/user/magpie_control/models/act_{sys.argv[1]}')
print(f'[local] policy loads OK — {sum(x.numel() for x in p.parameters())/1e6:.1f}M params')
PY
say "DONE — models/act_${TAG} ready to deploy. 🎉"
