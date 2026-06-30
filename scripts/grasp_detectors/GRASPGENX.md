# GraspGenX — grip-anything for the MAGPIE gripper

**GraspGenX** (NVlabs, ICRA'26 — https://github.com/NVlabs/GraspGenX) is a *cross-embodiment*
6-DOF grasp model: a **single** model that generalises to **any** gripper by conditioning on the
gripper's **swept-volume** representation (vs. GraspGen, which trains one model per gripper).
This is why it's the right model for our custom **MAGPIE** gripper — no per-gripper training.

This is what the Correll lab uses ("GraspGenX"). Status: **installed + working** with the lab's
MAGPIE gripper, producing high-confidence grasps. Remaining before live grasping: **camera
recalibration** (independent of the grasp model — see bottom).

## What's on the machine
| Path | What |
|---|---|
| `~/GraspGenX/` | NVlabs GraspGenX repo, installed via `uv` (`.venv`, torch 2.6+cu124) |
| `~/GraspGenX/ext/graspgenx_checkpoints/release/{gen,dis}/` | model weights (1.2 G + 462 M, git-LFS) |
| `~/GraspGenX/ext/gripper_descriptions/` | stock gripper metadata (auto-cloned) |
| `~/GraspGenX/assets/x_grippers/magpie/` | **the lab's MAGPIE gripper def** — `config.json` (swept-volume), `gripper.urdf`, `coll_mesh.obj`, `vis_mesh.obj` (from `correlllab/CL_Assets/graspgenx/assets/magpie`) |
| `~/CL_Assets/graspgenx/assets/magpie/` | source of the above (cloned) |

The model conditions on `sweep_volume_v2` (the boxes in `config.json`). The
`points.json / tsdf.npy / *_vae_repr.json "dummy values"` warnings at load are **harmless** —
those caches feed an alternate conditioning path this checkpoint doesn't use.

## Run it
1. **Start the server** (its own uv venv, GPU, ~3 GB VRAM):
   ```bash
   bash scripts/run_graspgenx_server.sh          # gripper=magpie, port=5557
   ```
2. **Smoke-test the bridge** (from the magpie env, no arm):
   ```bash
   python3 scripts/grasp_detectors/smoke_test_graspgenx.py
   ```
3. **Use it in the pickup**: set `GRASP_METHOD = 'graspgenx'` then run the Full Pickup cell.
   The dispatch in the pickup maps it to `socket_detector.graspgenx()` → `graspgenx_zmq.py`.

## How the integration is wired
```
magpie pickup (GRASP_METHOD='graspgenx')
   → socket_detector.graspgenx()
   → graspgenx_zmq.GraspGenXZMQ  (thin ZMQ client; only needs pyzmq+msgpack, no torch)
   → tcp://localhost:5557  ──▶  GraspGenX server (uv venv, GPU, MAGPIE swept volume)
   ◀── grasps (K,4,4) + confidences
```
The bridge centres the object cloud before sending and adds the centroid back to the returned
grasp translations (→ world frame), exactly like the GraspGen bridge.

## VRAM (8 GB card)
GraspGenX ≈ 3 GB, SAM3 ≈ 4 GB. They don't both fit alongside everything else, so the design is
**one heavy model in VRAM at a time** (SAM3 segments → free → GraspGenX plans). "Better data,
not faster." A stuck root-owned `graspgen_server` (old NVIDIA GraspGen, port 5556, ~3 GB) may
still be hogging VRAM — `sudo docker kill graspgen_server` to reclaim it.

## ⚠️ Still blocking live grasps (NOT the model)
**Camera calibration** (`_TCP_TO_CAM`) is off — the arm doesn't position above the object
(the two scans localise the same object ~5 cm apart in Z, 7 cm in XY). Fix = run the **calib
probe (notebook cell 7)** to re-measure `_TCP_TO_CAM`. Until then, *any* grasp model lands
beside the object. This is the first thing to do at the robot.
