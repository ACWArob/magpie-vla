# Using NSF ACCESS for Your Research — Slide Deck Source (General Template)

> Format: one `## Slide N` block per slide — title, bullets, speaker notes. Paste into
> Google Slides / Gemini as-is.
> Placeholders: anything in `<ANGLE_BRACKETS>` is where YOU insert your project's values.
> A robot-learning project (60 demos → trained policy in 1 day) runs through the deck as
> the concrete worked example — swap in your own domain freely.

---

## Slide 1 — Title
**From Your Lab to a National Supercomputer and Back: A Practical Guide to NSF ACCESS**
- What ACCESS is, how to get on it, and how to run `<YOUR PROJECT>`'s compute there
- Everything shown is copy-paste reproducible — placeholders marked like `<THIS>`
- Worked example throughout: training a robot manipulation policy (1 GPU-hour, same-day turnaround)

*Speaker notes: Goal of this talk — by the end, anyone with an ACCESS allocation should be able to run their first job this week. Every command shown was actually run; every pitfall was actually hit.*

---

## Slide 2 — What is NSF ACCESS?
- NSF program allocating time on national supercomputers to US researchers — **free, proposal-based**
- Resources include: Delta & DeltaAI (NCSA), Bridges-2 (PSC), Anvil (Purdue), Expanse (SDSC), and more
- You request an allocation → receive **credits/GPU-hours on a specific machine** → log in with institutional credentials + Duo
- Tiers: Explore (~easy, small) → Discover → Accelerate → Maximize (bigger, more review)

*Speaker notes: For most lab-scale ML, an Explore/Discover allocation is plenty — our whole case study used ~2% of a small allocation.*

---

## Slide 3 — When should you use it?
- Your local GPU is **too small, too slow, or too busy** (ours was already running the robot's perception stack)
- Training jobs are **batchable**: prepare data locally → train remotely → bring results back
- You do NOT need it for: interactive debugging, small inference, anything latency-sensitive
- Rule of thumb: local machine = develop + deploy; cluster = train

*Speaker notes: Case study framing: robot collects demonstrations locally, the cluster trains the policy, the policy runs back on the lab machine at 10 Hz. Clean division.*

---

## Slide 4 — The pipeline (works for any project)
```
your machine:  prepare  <YOUR_DATASET>
     │   rsync (+ Duo)
     ▼
cluster login node:  one-time environment setup
     │   sbatch  <YOUR_JOB>.slurm
     ▼
compute node (GPU):  training / simulation / analysis
     │   rsync results back
     ▼
your machine:  use  <YOUR_TRAINED_MODEL / RESULTS>
```
- First time (incl. debugging): an evening. Every time after: **~20 minutes of your attention + queue/runtime**

*Speaker notes: The first trip is the expensive one — write a runbook as you go so trip #2 is copy-paste. Ours is 1 page.*

---

## Slide 5 — Step 0: Know your machine's identity
| Item | How to find it | Our example |
|---|---|---|
| Login hostname | resource's user guide (NOT guessable!) | `dtai-login.delta.ncsa.illinois.edu` |
| Username | your ACCESS/institution mapping | `<YOUR_USERNAME>` |
| Slurm account | run `accounts` after login | `<abcd-xxxx-gh>` |
| GPU partition | user guide / `sinfo` | `ghx4` (DeltaAI GH200) |
| CPU architecture | `uname -m` after login | **aarch64 (ARM!)** on DeltaAI |

*Speaker notes: Two gotchas already: (1) hostnames are non-obvious — the "logical" spelling of DeltaAI's hostname doesn't resolve; (2) some new machines are ARM, which changes EVERYTHING about Python packaging — slide 8.*

---

## Slide 6 — Step 1: Move your data up
```bash
rsync -avz --progress  <LOCAL_DATA_DIR>/ \
    <YOUR_USERNAME>@<LOGIN_HOST>:~/<REMOTE_DATA_DIR>/
```
- Duo prompt on every connection — but rsync is **incremental**: re-run it anytime, only new files transfer
- Sanity-check the dataset locally FIRST (don't debug a broken dataset through a queue)
- Case study: 387 MB / 60 robot episodes → ~90 s

*Speaker notes: Incremental rsync is a workflow feature: as our dataset grows, the same command ships only the delta.*

---

## Slide 7 — Step 2: One-time environment setup
```bash
module avail <python|pytorch|cuda>      # ALWAYS start here — see what the site maintains
module purge
module load <SITE_PYTORCH_MODULE>       # prefer site modules for the GPU stack
module load cuda
python -m venv --system-site-packages ~/<PROJECT>_env
source ~/<PROJECT>_env/bin/activate
pip install <YOUR_PACKAGES>             # pure-Python deps only
```
- Pattern: **site module supplies the compiled GPU stack** (torch/CUDA), your venv layers project packages on top
- `--system-site-packages` is the key flag that makes the layering work
- **Pin versions that touch your data format** (our dataset library had to match exactly)

*Speaker notes: Resist `pip install torch` — on this ARM machine it silently installs a CPU-only build. Next slide is the pitfall list.*

---

## Slide 8 — The 4 environment pitfalls we hit (so you don't)
| Symptom | Cause | Fix |
|---|---|---|
| `no matching distribution` | system Python ancient (3.6) | `module load` a modern python |
| `torch.version.cuda = None` | pip installed a **CPU wheel** (ARM) | uninstall it; use the module's torch |
| API missing (`VideoReader`) | *newest* module too new — API removed | use an **older** module version |
| `libXXX.so not found` | CUDA toolkit not loaded | `module load cuda` |

- Meta-lessons: **newest ≠ best**; on ARM verify everything; `python -c "import torch; print(torch.version.cuda)"` is your friend

*Speaker notes: Each of these cost 10–30 minutes at night. With this table, they cost zero.*

---

## Slide 9 — Step 3: The 5-second test that saves the night
```python
# run INTERACTIVELY on the login node, BEFORE submitting:
<load one sample of YOUR data through YOUR library>
# e.g. ours:  ds = LeRobotDataset(...); sample = ds[0]   # forces real video decode
```
- Exercise the **exact code path your job will run** — data loading is where jobs die
- Caught our third pitfall interactively; the overnight job would have crashed at 2 a.m. instead
- **Never trust an overnight job you haven't smoke-tested**

*Speaker notes: If your data loads and your framework sees the GPU, ~90% of first-job failures are already eliminated.*

---

## Slide 10 — Step 4: Write the Slurm job
```bash
#!/usr/bin/env bash
#SBATCH --account=<YOUR_ACCOUNT>        # from `accounts`
#SBATCH --partition=<GPU_PARTITION>
#SBATCH --gpus-per-node=1
#SBATCH --time=<HH:MM:SS>               # generous but honest — shorter queues faster
#SBATCH --output=<JOB>_%j.log
set -euo pipefail
module purge && module load <SITE_PYTORCH_MODULE> cuda   # SAME stack as setup
source ~/<PROJECT>_env/bin/activate
python <YOUR_TRAINING_COMMAND> --checkpoint_every <N>
```
- The job script must repeat your module loads — jobs start from a clean shell
- **Checkpoint frequently** — a killed job with checkpoints is still a result

*Speaker notes: Our first job died in 33 seconds — a library default tried to upload the model to a hub and demanded an account flag. Read your framework's defaults. A fast failure is a GOOD failure — you only catch it if you watch the first minutes (next slide).*

---

## Slide 11 — Step 5: Submit and babysit (for 2 minutes)
```bash
sbatch <JOB>.slurm
squeue -u $USER            # PD = queued, R = running
tail -f <JOB>_*.log        # WAIT until you see real progress lines
```
- Small 1-GPU jobs typically start in minutes even on busy machines
- Healthy = metrics ticking; unhealthy = instant traceback → fix now, not tomorrow
- Then log out — sbatch jobs don't need your session

*Speaker notes: The discipline: never leave until the log shows real training progress. Two minutes of watching bought us the confidence to sleep.*

---

## Slide 12 — Step 6: Bring results home
```bash
rsync -avz <YOUR_USERNAME>@<LOGIN_HOST>:~/<RESULTS_DIR>/ <LOCAL_DIR>/
```
- Verify the artifact loads on YOUR machine before celebrating (library version mismatches bite here too)
- Case study: 200 MB trained policy back on the lab PC, loading + running inference at 11 ms/step on a 5-year-old GPU

*Speaker notes: Small models mean pull-back is trivial. For big artifacts, consider Globus — ACCESS sites support it.*

---

## Slide 13 — Case study results (what 1 GPU-hour bought)
- Robot manipulation policy (ACT, 51.6M params), 60 self-collected demonstrations
- **100,000 training steps in 62 minutes** on one GH200 (would be many hours locally, blocking the robot)
- Deployed same day → autonomous visual pick-and-place on the real arm
- Offline verification: policy reproduces training trajectories to **1.9 mm**

*Speaker notes: The result isn't just the policy — it's the demonstrated LOOP: collect → train remotely → deploy → measure → recollect. That loop runs weekly now.*

---

## Slide 14 — Cost accounting (compute is not your bottleneck)
- Training run: **~1 GPU-hour** of a 100-hour allocation
- Our data collection: ~30–60 samples(episodes)/hour on the physical system → the ROBOT is the bottleneck, not the cluster
- Even a 10× bigger dataset trains in an afternoon of allocation time
- Budget rule: reserve allocation for **runs you'll actually analyze**; iterate on data quality locally

*Speaker notes: This inverts the instinct to hoard GPU-hours. At lab scale, allocation hours are abundant; instrument time and data design are scarce.*

---

## Slide 15 — Best-practices checklist (each maps to a real failure)
1. Find the real login hostname + account string first (`accounts`, user guide)
2. Site module for GPU stack, venv on top (`--system-site-packages`)
3. **Pin data-format library versions** to match your local machine
4. Verify `torch.version.cuda` (or your framework's GPU check) before anything else
5. Smoke-test your **data-loading path interactively** before sbatch
6. Watch the first minutes of the first job (`tail -f`)
7. Checkpoint frequently; treat partial runs as results
8. Write the runbook as you go — future-you reproduces in 20 minutes

*Speaker notes: This slide is the takeaway photo. Everything else is supporting evidence.*

---

## Slide 16 — Template: your project on ACCESS
Fill in one line each and you're operational:
```
Machine/partition:   <e.g. DeltaAI / ghx4>
Login:               <USERNAME>@<LOGIN_HOST>
Account:             <from `accounts`>
Data up:             rsync <DATA>/  →  ~/<DATA>/
Environment:         module load <PYTORCH_MODULE> cuda  +  venv  +  pip <PKGS>
Smoke test:          <one-sample load of YOUR data>
Job:                 sbatch <JOB>.slurm   (account, partition, checkpoints)
Results back:        rsync ~/<RESULTS>/  →  local
```

*Speaker notes: Offer: our full runbook and slurm template are in the repo (docs/NSF_TRAINING.md) — clone, find-and-replace the placeholders, run.*

---

## Slide 17 — Q&A / pointers
- ACCESS portal: allocations.access-ci.org (requests, usage tracking)
- Machine user guides: docs.ncsa.illinois.edu (Delta/DeltaAI), PSC/Purdue/SDSC equivalents
- Our artifacts: 1-page runbook, validated slurm script, this deck — all placeholder-ready
- Contact: `<YOUR NAME / EMAIL>`

*Speaker notes: Close with the one-sentence pitch: "If your training fits in a file and your data fits in an rsync, ACCESS turns your laptop-bound project into a supercomputer project in an afternoon."*

---

# 🎨 PROMPT — paste this + the deck above into Gemini Canvas / Claude / Google Slides AI

> Create a polished, professional slide presentation from the slide-deck source above.
>
> Instructions:
> - Each "## Slide N" block is exactly one slide: use its bold line as the slide title,
>   the bullets as the slide body, and the *Speaker notes* line as that slide's speaker notes.
> - Render the code blocks as monospaced code panels on the slide — do not paraphrase or
>   reflow the commands; they must stay copy-pasteable.
> - Render the markdown tables as clean slide tables.
> - Keep every `<ANGLE_BRACKET>` placeholder visibly highlighted (e.g. colored text) so the
>   audience can see where their own values go.
> - Visual style: clean academic/tech deck — dark title slide, light content slides, one
>   accent color, generous whitespace, no stock photos. A small footer with the deck title.
> - For Slide 4 and the pipeline diagram: render as a simple vertical flow diagram
>   (boxes + arrows), not as plain code text.
> - Do not add, remove, or reorder slides. Do not summarize away details — this is a
>   hands-on tutorial deck, the specifics ARE the content.

---
---

# 📋 APPENDIX — General Runbook Template (handout, not part of the slide deck)

> Copy this section, find-and-replace the `<PLACEHOLDERS>`, and it becomes YOUR project's
> cluster runbook. The V0 robot-project instance of this exact template lives at
> `docs/NSF_TRAINING.md` — use it as a filled-in reference.

**Fill these in once:**

| Placeholder | Yours | Example (our project) |
|---|---|---|
| `<LOGIN_HOST>` | | `dtai-login.delta.ncsa.illinois.edu` |
| `<USERNAME>` | | `aabid` |
| `<ACCOUNT>` (run `accounts`) | | `bgcd-dtai-gh` |
| `<PARTITION>` | | `ghx4` |
| `<PYTORCH_MODULE>` | | `python/miniforge3_pytorch/2.7.0` |
| `<DATA_DIR>` | | `lerobot_v0` (387 MB) |
| `<PKGS>` | | `lerobot==0.4.4` |
| `<TRAIN_CMD>` | | `python -m lerobot.scripts.lerobot_train ...` |
| `<JOB_NAME>` | | `act_v0` |

## 0. Sanity-check your data locally
```bash
<one command that proves your dataset is complete and loadable>
```
Never debug a broken dataset through a queue.

## 1. Push the data
```bash
rsync -avz --progress <LOCAL_DATA_PATH>/ <USERNAME>@<LOGIN_HOST>:~/<DATA_DIR>/
```
First connection: accept the host key, then password + 2FA. Re-runs are incremental.

## 2. One-time environment (on the login node)
```bash
ssh <USERNAME>@<LOGIN_HOST>
module avail python pytorch cuda        # discover what the site maintains
module purge
module load <PYTORCH_MODULE>            # site module = the compiled GPU stack
module load cuda                        # CUDA runtime libs (e.g. libcufile)
python -m venv --system-site-packages ~/<JOB_NAME>_env
source ~/<JOB_NAME>_env/bin/activate
pip install --upgrade pip
pip install <PKGS>                      # PIN versions that touch your data format
pip uninstall -y torch torchvision      # evict any CPU wheel pip snuck in (ARM machines!)
python -c "import torch; print(torch.version.cuda)"   # must NOT print None
```

## 3. Smoke-test the exact job code path (5 seconds, saves a night)
```bash
python -c "<load ONE sample of your data through YOUR library>"
```
If this passes and CUDA is visible, ~90% of first-job failures are eliminated.

## 4. The job script (`~/<JOB_NAME>.slurm`)
```bash
#!/usr/bin/env bash
#SBATCH --account=<ACCOUNT>
#SBATCH --partition=<PARTITION>
#SBATCH --gpus-per-node=1
#SBATCH --job-name=<JOB_NAME>
#SBATCH --cpus-per-task=8
#SBATCH --mem=64G
#SBATCH --time=<HH:MM:SS>
#SBATCH --output=<JOB_NAME>_%j.log
set -euo pipefail
module purge
module load <PYTORCH_MODULE>
module load cuda
source ~/<JOB_NAME>_env/bin/activate
<TRAIN_CMD>            # include: checkpoint-every-N + disable any hub/cloud upload defaults
```

## 5. Submit + confirm it's actually running
```bash
sbatch ~/<JOB_NAME>.slurm
squeue -u $USER                 # want "R"; "PD" = queued (fine, it starts without you)
tail -f <JOB_NAME>_*.log        # WAIT for real progress lines before walking away
```

## 6. Bring results home (from your machine)
```bash
rsync -avz <USERNAME>@<LOGIN_HOST>:~/<RESULTS_DIR>/ <LOCAL_RESULTS>/
<one command that proves the artifact loads locally>
```

## Troubleshooting (general — each row was a real failure)
| Symptom | Cause | Fix |
|---|---|---|
| `Could not resolve hostname` | guessed login host | user guide has the real one |
| `Host key verification failed` | first connect from a script | ssh once by hand, answer `yes` |
| `no matching distribution` | ancient system Python | `module load` a modern python |
| GPU check prints `None` | pip installed a CPU wheel (common on ARM) | uninstall it; module torch takes over |
| missing API/attribute errors | newest module too new | try an OLDER module version |
| `libXXX.so not found` | toolkit module not loaded | `module load cuda` (or equivalent) |
| `Invalid account` on sbatch | wrong account string | run `accounts`, copy exactly |
| job dies in seconds | framework default needs a flag (e.g. hub upload) | read the traceback, add the flag |
| stuck `PD` for hours | queue busy / walltime too long | shorter `--time`, resubmit |
| OOM during run | batch too big | halve the batch size |

## Iterating
More data later → re-run step 1 (rsync sends only new files) → resubmit with a fresh
`--output_dir` so runs never overwrite each other.
