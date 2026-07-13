# Claude prompts — two matching PowerPoint decks (total ≤15 min)

Generated 2026-07-13. Paste each into claude.ai separately. Images are raw GitHub links
(public repo ACWArob/magpie-vla, branch ros) — all verified reachable.

## Prompt 1 — Pipeline deck (~9 min, 9 slides)

```
Make me a Microsoft PowerPoint (.pptx) presentation, 9 slides, to present in ~9 minutes.

DESIGN (use exactly this, it must match a second deck I'm making):
- Clean white background, dark navy text (#1B2430), safety-orange accent (#D9560B) for titles/highlights
- One idea per slide, max 4 bullets, big readable text (min 18pt body)
- Numbers and metrics in a monospace font, bold
- No clipart, no emoji, no gradients

CONTENT — "AutoGrasp → Data → Policy: a robot that trains itself":

Slide 1 (title): "A Robot That Generates Its Own Training Data" — scripted expert → self-judged dataset → ACT policy trained on NSF ACCESS → 97/100 picks. Zero teleoperation, zero human labels.

Slide 2 (the loop): Scripted expert picks blocks (real UR5e arm) → reward gate (VLM judge ≥0.6 AND physically held) → LeRobot dataset → train on DeltaAI supercomputer (~1.5h) → deploy at 10Hz → evaluate on the same grid. Draw this as a simple cycle diagram with boxes and arrows.

Slide 3 (the expert): SAM3 segmentation → center over block → pick angle from block geometry → grasp with force control → episode auto-saved only if it passes the quality gate. 99.1% save rate over 231 attempts, fully hands-free.
Image: https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/sam3_wrist_overlay.png

Slide 4 (V0, the honest failure): 60 random-placement episodes trained fine (replay error 1.9mm) but deployment failed 3 measurable ways: only works inside the training zone; 42/62 episodes grasped at 90° regardless of block angle → policy FROZE on rotated blocks; 40% wasted footage.
Image: https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/v0/fig1_angle_collapse.png

Slide 5 (V1 data design): every V0 failure got a designed answer — 5×5 grid × 7 angles (designed coverage), deterministic angle labels (worst bin 71%→19.7%), gripper aperture as a continuous action, episodes end at lift (39.6s→11.2s). 229 episodes at 66.7/hour, 2.1× faster.
Image: https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/v0_v1/compare_angles_lengths.png

Slide 6 (the audit): data ships to the cluster only if it passes checks (count, angle histogram, coverage holes). One warning — "0° bin LOW" — we misread; it predicted the exact deployment failure. Lesson: audit warnings get investigated.
Image: https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/v1_audit_angles.png

Slide 7 (RESULTS — the money slide): 100-grasp evaluation, fully unattended. 97/100 picks. Rotated blocks: 100% (75/75) — the exact case V0 froze on. All 3 failures at 0° — exactly where the audit warned. No spatial falloff (97% at the grid edge).
Image (full width): https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/v1_eval_heatmap.png

Slide 8 (evidence detail): success by angle, quality grades, alignment error — the grading separates "picked it up" from "picked it up well" (0° succeeds 88% but at 0.58 quality — the aliasing shows in quality first).
Image: https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/v1_eval_summary.png

Slide 9 (next): V1.1 = fix the 0° label aliasing, ~35 replacement episodes, retrain, re-run the SAME eval → before/after heat-maps. Then V2 multi-object. Long-term: simulation twin + continual-learning memory.
Image: https://raw.githubusercontent.com/ACWArob/magpie-vla/ros/docs/figures/v1_eval_0deg_column.png

Add one-sentence speaker notes per slide.
```

## Prompt 2 — NSF deck (lab runbook edition, ~10 min, 12 slides — framework-agnostic)

```
Make me a Microsoft PowerPoint (.pptx) presentation, 12 slides. This is a practical
runbook talk for my robotics lab: by the end, anyone with an NSF ACCESS allocation should
be able to run THEIR OWN training job this week by copying my commands. Show real commands
in code blocks on the slides — that is the point of this deck. Anything in <ANGLE_BRACKETS>
is where they substitute their own project's values.

DESIGN (must match my other deck EXACTLY):
- Clean white background, dark navy text (#1B2430), safety-orange accent (#D9560B) for titles/highlights
- Code blocks in monospace on a very light gray panel, minimum 14pt
- Max 4 bullets or 1 code block per slide, big readable text
- No clipart, no emoji, no gradients

CONTENT — "Training on NSF ACCESS: the copy-paste runbook":

Slide 1 (title + promise): "From Your Lab to a Supercomputer and Back — every command included."
NSF ACCESS = free, proposal-based GPU time on national supercomputers (DeltaAI, Bridges-2, Anvil, Expanse...).
Worked example throughout: our robot policy — 229 episodes up, trained policy back, ~1.5 GPU-hours.
Rule: local machine = develop + deploy; cluster = train.

Slide 2 (know your machine — fill this table FIRST):
| Login hostname | from the resource's user guide (NOT guessable) | dtai-login.delta.ncsa.illinois.edu |
| Username | your ACCESS mapping | <YOUR_USERNAME> |
| Slurm account | run `accounts` after login | looks like abcd-dtai-gh |
| GPU partition | user guide or `sinfo` | ghx4 |
| CPU architecture | `uname -m` | aarch64 — ARM! changes all of Python packaging |

Slide 3 (one-time: no-hassle login). Add to ~/.ssh/config:
```
Host deltaai
  HostName dtai-login.delta.ncsa.illinois.edu
  User <YOUR_USERNAME>
  ControlMaster auto
  ControlPath ~/.ssh/cm-%r@%h-%p
  ControlPersist 12h
```
Then `ssh deltaai exit` = password + Duo ONCE; every later ssh/rsync/sbatch reuses the
socket for 12 hours with no re-auth. This one block is what makes overnight automation possible.

Slide 4 (push your data up):
```
rsync -avz --progress <LOCAL_DATA_DIR>/ deltaai:~/<REMOTE_DATA_DIR>/
```
rsync is incremental — rerun anytime, only new files transfer. Sanity-check the dataset
locally FIRST: never debug a broken dataset through a Slurm queue.

Slide 5 (one-time environment — the pattern that works on ARM):
```
module avail python pytorch cuda     # ALWAYS look first — use what the site maintains
module purge
module load python/miniforge3_pytorch/2.7.0   # site module = compiled GPU stack
module load cuda
python -m venv --system-site-packages ~/<PROJECT>_env   # KEY FLAG: layer on module torch
source ~/<PROJECT>_env/bin/activate
pip install <YOUR_PACKAGES>          # pure-Python deps only; PIN data-format versions
python -c "import torch; print(torch.version.cuda)"     # must NOT print None
```
Pattern: the site module supplies torch+CUDA; your venv layers project packages on top.

Slide 6 (the 4 pitfalls we hit — so you don't):
| torch.version.cuda = None | pip installed a CPU wheel (ARM) | use the module's torch |
| API missing (VideoReader) | NEWEST module too new — API removed | load an OLDER version |
| lib*.so not found | CUDA toolkit not loaded | module load cuda |
| job dies in 33 s | framework tried to upload to a public hub | our flag: --policy.push_to_hub=false — read YOUR framework's defaults |

Slide 7 (the 5-second test that saves the night). On the login node, BEFORE submitting:
```
python -c "<load ONE sample of YOUR data through YOUR framework>"
# robot-policy example (ours):  LeRobotDataset(...)[0]      # forces real video decode
# LLM example:                  AutoModel.from_pretrained() # forces weight download+load
# vision example:               next(iter(DataLoader(ds)))  # forces decode+transform
```
Exercises the exact code path the job will run. Caught a failure that would have crashed at 2 a.m.
Catches ~90% of first-job failures. Never trust an overnight job you haven't smoke-tested.

Slide 8 (WHAT are you training? Same skeleton, swap 3 lines). The whole recipe is
framework-agnostic — only pip line, train command, and resources change. Table:
| You want to train... | pip install | train command looks like | typical ask |
| Robot policy / imitation (our case) | lerobot==0.4.4 (PIN the version that wrote your data) | python -m lerobot.scripts.lerobot_train --dataset.root=... | 1 GPU, 1-2 h |
| Fine-tune an LLM / VLM | transformers peft trl accelerate | accelerate launch train.py or trl sft ... | 1-4 GPUs, hours |
| Vision model (detector/classifier) | timm / ultralytics / torchvision | python train.py --data ... --epochs ... | 1 GPU, minutes-hours |
| RL in simulation | your sim (mujoco, isaac) + sb3/rsl_rl | python train_rl.py --headless | 1 GPU + MANY CPUs (--cpus-per-task=32) |
| Anything embarrassingly parallel (sweeps) | same as above | sbatch --array=0-15 sweep.slurm | many small jobs beat one big one |
Two universal rules: PIN the version of whatever WROTE your data format, and ask `accounts`/docs which partition fits (GPU partitions for training, CPU partitions exist too and don't burn GPU credits).

Slide 9 (the Slurm job — our actual working file, adapt the last line to your project):
```
#!/usr/bin/env bash
#SBATCH --account=<YOUR_ACCOUNT>      # from `accounts`
#SBATCH --partition=ghx4
#SBATCH --gpus-per-node=1
#SBATCH --time=10:00:00               # generous but honest; shorter queues faster
#SBATCH --output=train_%j.log
set -euo pipefail
module purge && module load python/miniforge3_pytorch/2.7.0 cuda   # SAME stack as setup
source ~/<PROJECT>_env/bin/activate
python -m <YOUR_TRAINING_COMMAND> --checkpoint_every <N>
```
Jobs start from a clean shell — the module loads MUST be repeated inside the script.
Checkpoint often: a killed job with checkpoints is still a result.

Slide 10 (submit + babysit for exactly 2 minutes):
```
sbatch train.slurm
squeue -u $USER          # PD = queued, R = running (1-GPU jobs start in minutes)
tail -f train_*.log      # WAIT until real progress lines tick, then log out
```
Healthy = loss lines ticking. Instant traceback = fix NOW, not tomorrow. sbatch jobs
survive logout. Our numbers: 0.037 s/step on a GH200 → 150k steps ≈ 1.5 h.

Slide 11 (pull results home + verify). From YOUR machine (not inside the ssh session):
```
rsync -avz deltaai:<RUN_DIR>/checkpoints/last/ ~/models/<NAME>/
python -c "<load the checkpoint and print param count>"   # verify BEFORE deploying
```
Then verify offline against held-out data before touching hardware — our replay test
measured 1.9 mm error and told us the model was fine before the robot ever moved.

Slide 12 (the end state — one command, overnight):
```
ssh deltaai exit                      # Duo once, before leaving the lab
nohup bash auto_train_cycle.sh v1 150000 &
```
Our script chains: data audit gate → rsync push → remote smoke test → sbatch → poll →
pull → local load test. Leave at 6 pm, arrive to a trained, verified policy.
Adapt point for YOUR project: replace two things only — the audit check and the train command.
The policies this trained scored 97/100 on hardware (see my other deck).

Add one-sentence speaker notes per slide.
```
