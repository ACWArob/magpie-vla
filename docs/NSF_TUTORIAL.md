# Training on NSF ACCESS (DeltaAI) — Step-by-Step Tutorial

*Format: every step = one goal, one command block, then a table decoding every component
of that command. No narrative. `[SCREENSHOT]` marks where a screen capture goes.
Placeholders in `<ANGLE_BRACKETS>` are where you substitute your own values.*

---

## Step 0 — Understand the machine you're about to use

| Term | What it actually means |
|---|---|
| **NSF ACCESS** | An NSF program that gives US researchers free time on national supercomputers. You apply with a short proposal and receive **credits** on a specific machine. |
| **DeltaAI** | The machine we use, at NCSA (Illinois). It is a *cluster*: many computers ("nodes") connected together, shared by hundreds of users. |
| **Login node** | The one computer you SSH into. It is for editing files and submitting jobs — **never for heavy compute** (everyone shares it). |
| **Compute node** | Where jobs actually run. You never SSH there; a scheduler assigns them. |
| **GH200** | The node type on DeltaAI: an NVIDIA "superchip" = a **72-core ARM CPU + an H100-class GPU** glued together. The GPU is why we're here. |
| **ARM (aarch64)** | The CPU architecture. Most Python packages are pre-built for Intel/AMD (`x86_64`). On ARM, `pip install torch` silently gives you a **CPU-only** build — this single fact causes most first-day failures (Step 4 fixes it). |
| **Slurm** | The job scheduler. You write a small script describing what you need (1 GPU, 2 hours), submit it, and Slurm runs it on a compute node when one is free. |

[SCREENSHOT: ACCESS allocations dashboard showing the DeltaAI allocation]

---

## Step 1 — Log in

```bash
ssh <username>@dtai-login.delta.ncsa.illinois.edu
```

| Component | Meaning |
|---|---|
| `ssh` | Opens an encrypted terminal session on a remote machine. |
| `<username>` | Your DeltaAI username — comes from your ACCESS account mapping (not necessarily your university ID). |
| `dtai-login.delta...` | The **login-node** hostname. It is NOT guessable — copy it from the DeltaAI user guide. (`deltaai.ncsa...` does not resolve.) |
| Password + Duo | The cluster uses two-factor auth: your password, then approve a Duo push. Every new connection asks again — Step 2 removes that pain. |

First thing after logging in, run these two and write down the answers:

```bash
accounts        # -> your Slurm account string, e.g. bgcd-dtai-gh
uname -m        # -> aarch64  (confirms: this machine is ARM)
```

| Component | Meaning |
|---|---|
| `accounts` | DeltaAI command that prints which **allocation account** your jobs must be charged to. You need this string in Step 6. |
| `uname -m` | Prints the CPU architecture. `aarch64` = ARM (see Step 0 — it changes how Python is installed). |

[SCREENSHOT: terminal after first login showing the accounts output]

---

## Step 2 — Log in ONCE per day (ControlMaster)

Add this to `~/.ssh/config` **on your lab machine**:

```
Host deltaai
  HostName dtai-login.delta.ncsa.illinois.edu
  User <username>
  ControlMaster auto
  ControlPath ~/.ssh/cm-%r@%h-%p
  ControlPersist 12h
```

| Component | Meaning |
|---|---|
| `Host deltaai` | Creates a nickname: from now on `ssh deltaai` works everywhere. |
| `ControlMaster auto` | The first connection opens a tunnel that later connections **reuse**. |
| `ControlPath` | Where the reusable tunnel's socket file lives (`%r`=user, `%h`=host, `%p`=port — just makes the filename unique). |
| `ControlPersist 12h` | Keep the tunnel alive 12 hours after the last use. **This is the trick**: password + Duo ONCE in the morning, then every `ssh`/`rsync`/`scp` that day reuses the tunnel with no re-authentication. Scripted/overnight workflows depend on this. |

Then each morning: `ssh deltaai exit` (authenticate once; the `exit` just closes the visible session — the tunnel stays).

---

## Step 3 — Upload your dataset

Run **on your lab machine** (always transfer from the machine that owns the data):

```bash
rsync -avz --progress ~/my_project/data/my_dataset/  deltaai:~/my_dataset/
```

| Component | Meaning |
|---|---|
| `rsync` | File-copy tool that only transfers what changed — re-running it later sends just the new files. |
| `-a` | "Archive": keep permissions, timestamps, subfolders. |
| `-v` | Verbose: print each file. |
| `-z` | Compress during transfer (faster over the internet). |
| `--progress` | Show a progress bar per file. |
| trailing `/` on the source | Means "the *contents* of this folder" — without it you get an extra nested folder on the other side. |
| `deltaai:` | The Step-2 nickname; rsync rides the existing tunnel (no Duo). |

Rule: sanity-check the dataset locally BEFORE uploading. Never debug a broken dataset
through a job queue.

---

## Step 4 — Build the Python environment (once)

```bash
module avail python pytorch          # 1. see what the site provides
module purge                         # 2. start from a clean slate
module load python/miniforge3_pytorch/2.7.0   # 3. site-built Python + GPU torch
module load cuda                     # 4. the GPU libraries torch needs
python -m venv --system-site-packages ~/myproj_env   # 5. your own env ON TOP
source ~/myproj_env/bin/activate     # 6. turn it on
pip install "lerobot==0.4.4"         # 7. your project packages (pinned!)
python -c "import torch; print(torch.version.cuda)"  # 8. must NOT print None
```

| Component | Meaning |
|---|---|
| **module system** | Clusters don't let you `apt install`. Instead, admins pre-build software as loadable "modules". `module load X` puts X on your path. |
| `module avail` | Lists what exists. **Always look first** — the site's PyTorch is compiled correctly for this ARM+GPU hardware; yours from pip is not. |
| `module purge` | Unloads everything — protects you from leftover modules from a previous session interfering. |
| `python/miniforge3_pytorch/2.7.0` | A module containing Python + a **GPU-enabled torch built for ARM**. We use 2.7.0 specifically: the newest one removed a video-decoding API our data loader needs. Lesson: **newest ≠ best**. |
| `module load cuda` | Torch needs NVIDIA's CUDA libraries at run time; this puts them on the path. Forgetting it = `libcufile.so not found`. |
| `venv` | A private Python environment so your packages don't fight other users' or the system's. |
| `--system-site-packages` | **The key flag.** Your venv can *see through* to the module's packages — so you keep the site's GPU torch and add only your own packages on top. Without it, pip would install its own (CPU-only, ARM-broken) torch. |
| `pip install "lerobot==0.4.4"` | `==` **pins** the exact version. Pin anything that reads your data format: the library that READS the dataset must match the one that WROTE it. |
| `torch.version.cuda` check | If it prints `None`, torch can't see the GPU — you have a CPU build; redo this step. If it prints `12.6`, you're good. |

[SCREENSHOT: the verification line printing 12.6]

---

## Step 5 — The 5-second test that saves the night

On the login node, BEFORE submitting anything:

```bash
python -c "
from lerobot.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset('myproj/v1', root='$HOME/my_dataset')
print(ds[0] is not None, '- sample OK')"
```

| Component | Meaning |
|---|---|
| Loading **one sample** | Forces the full real code path — file access, video decode, format parsing. This is exactly what your job will do at minute 0. |
| Why on the login node | It takes 5 seconds and needs no GPU. If it fails here, it would have failed at 2 a.m. inside the queue instead. Catches ~90% of first-job failures. |
| Adapt to your framework | LLM person: `AutoModel.from_pretrained(...)`. Vision person: pull one batch from your DataLoader. Same idea: *touch your data through your library once*. |

---

## Step 6 — Write the job script

Save as `~/train.slurm` on DeltaAI:

```bash
#!/usr/bin/env bash
#SBATCH --account=bgcd-dtai-gh
#SBATCH --partition=ghx4
#SBATCH --gpus-per-node=1
#SBATCH --time=04:00:00
#SBATCH --output=train_%j.log
set -euo pipefail
module purge
module load python/miniforge3_pytorch/2.7.0
module load cuda
source ~/myproj_env/bin/activate
python -m lerobot.scripts.lerobot_train \
  --dataset.repo_id=myproj/v1 --dataset.root=$HOME/my_dataset \
  --policy.type=act --policy.push_to_hub=false --policy.device=cuda \
  --output_dir=$HOME/run1 --batch_size=8 --steps=150000 --save_freq=10000
```

| Component | Meaning |
|---|---|
| `#SBATCH` lines | Instructions **to Slurm**, not to bash — they describe what resources you're requesting. |
| `--account` | Who pays (in credits): the string from Step 1's `accounts`. Wrong string = instant "Invalid account" rejection. |
| `--partition=ghx4` | Which *group of nodes* to use. `ghx4` = DeltaAI's GH200 GPU nodes. (Clusters also have CPU partitions — those don't burn GPU credits.) |
| `--gpus-per-node=1` | Request exactly one GPU. Ask for what you use — bigger requests queue longer AND cost more. |
| `--time=04:00:00` | Maximum run time (hh:mm:ss). Generous but honest: too short = job killed mid-training; too long = queues slower. |
| `--output=train_%j.log` | Where stdout goes. `%j` = the job number, so runs never overwrite each other. |
| `set -euo pipefail` | Bash safety: die immediately on any error instead of continuing blind. |
| `module purge` + loads **inside the script** | Jobs start from a **clean shell** — nothing from your login session carries over. The script must rebuild the environment itself, with the SAME modules as Step 4. |
| `--policy.push_to_hub=false` | Framework-specific gotcha: lerobot tries to upload your model to HuggingFace by default and dies in 33 s if it can't. Every framework has a default like this — read yours. |
| `--save_freq=10000` | Write a checkpoint every 10k steps. **A killed job with checkpoints is still a result.** |

---

## Step 7 — Submit, then babysit for exactly 2 minutes

```bash
sbatch train.slurm
squeue -u $USER
tail -f train_*.log
```

| Component | Meaning |
|---|---|
| `sbatch` | Hands the script to Slurm. Prints `Submitted batch job 2631116` — that number is `%j`. |
| `squeue -u $USER` | Shows *your* jobs. State `PD` = pending (waiting for a node), `R` = running. Small 1-GPU jobs usually start within minutes. |
| `tail -f` | Live-follows the log file. **Watch until you see real training lines** (loss values ticking). An instant Python traceback = fix it NOW, not tomorrow. |
| Then log out | Slurm jobs do not need your SSH session. Close the laptop; the job runs. |

Reference numbers from our runs: ~0.037 s/step on one GH200 → 150k steps ≈ 1.5 h.

[SCREENSHOT: squeue showing the job in R state + first loss lines in the log]

---

## Step 8 — Pull the result home and verify it

Run **on your lab machine**:

```bash
rsync -avz deltaai:~/run1/checkpoints/last/pretrained_model/  ~/models/my_policy/
python3 -c "<load the checkpoint; print parameter count>"
```

| Component | Meaning |
|---|---|
| `checkpoints/last/` | `last` is a symlink to the newest checkpoint — you don't need to know the step number. |
| `pretrained_model/` | The loadable artifact: weights + config + **normalization statistics**. The stats files MUST travel with the weights — they are part of the model. |
| Local load test | Prove the checkpoint opens on YOUR machine before anything depends on it. Ours also replays held-out data through the model (measured 1.9 mm error) before the robot ever moves. |

---

## Step 9 — The end state: one command, overnight

```bash
ssh deltaai exit          # morning: Duo once (Step 2 tunnel now alive 12h)
nohup bash scripts/auto_train_cycle.sh v1 150000 > /tmp/train.log 2>&1 &
```

| Component | Meaning |
|---|---|
| `nohup ... &` | Run in the background, immune to the terminal closing. |
| The script chains | audit the dataset → rsync push → login-node smoke test (Step 5) → sbatch (Step 6) → poll squeue → rsync pull (Step 8) → local load test. |
| To adapt it | Replace exactly two things: the audit check (what makes YOUR data shippable) and the training command. Everything else is generic. |
| Bonus | Slurm loves **parallel jobs**: our script submits the main training AND an ablation simultaneously — a second experiment costs zero extra wall-clock. |

---

## Troubleshooting — symptom → cause → fix

| Symptom | Cause | Fix |
|---|---|---|
| `torch.version.cuda = None` | pip installed a CPU wheel (ARM) | Step 4: use the module's torch, `--system-site-packages` |
| `VideoReader` / API missing | newest module removed it | load the **older** module (2.7.0) |
| `libcufile.so not found` | CUDA module not loaded | `module load cuda` (in the JOB script too) |
| job dies in ~30 s | framework's upload-to-hub default | `--policy.push_to_hub=false` (or your framework's equivalent) |
| `Invalid account` | wrong account string | run `accounts`, copy exactly |
| stuck `PD` > 1 h | asking for too much time/resources | lower `--time`, ask for 1 GPU |
| `Host key verification failed` in a script | first-ever connection | ssh once by hand, answer `yes` |
