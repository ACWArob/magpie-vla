# Learned grasp detectors — setup

Each model pins **older torch/CUDA** than the main env (torch 2.10+cu128), so each
runs in its **own venv** and is called over a Unix socket — same pattern as SAM3
(`~/sam3_env` + `scripts/sam3_infer.py`). The adapters in `socket_detector.py` are
thin clients; the model runs in a bridge process.

> ⚠️ **Calibration first.** Every detector outputs a grasp *pose* the arm must reach.
> If `_TCP_TO_CAM` is wrong, the best detector still grasps the wrong spot. Fix
> calibration (`calib_probe` cell) before trusting any benchmark's success rate.

> ⚠️ **VRAM.** SAM3 uses ~4.5 GB of the 8 GB RTX 2070 SUPER. For the **offline**
> benchmark (saved point clouds) you can stop SAM3 to free VRAM. For **live** runs,
> only load one detector at a time.

---

## 1. Contact-GraspNet (open, proven, recommended first)
Repo: <https://github.com/NVlabs/contact_graspnet> (TensorFlow). >90% success on
unseen objects in clutter.

```bash
python3.10 -m venv ~/cgn_env && source ~/cgn_env/bin/activate
pip install tensorflow==2.5 numpy==1.23 opencv-python trimesh pyyaml
# CUDA pointnet2 ops + checkpoints per the repo README, then:
# python scripts/grasp_detectors/bridges/contact_graspnet_bridge.py --socket /tmp/contact_graspnet.sock
```

## 2. GraspGen (NVIDIA, 2025, PyTorch — best modern-stack fit)
Repo: <https://github.com/NVlabs/GraspGen>. Diffusion-based 6-DOF.

```bash
python3.10 -m venv ~/graspgen_env && source ~/graspgen_env/bin/activate
# follow repo install (its own torch pin), download weights, then:
# python scripts/grasp_detectors/bridges/graspgen_bridge.py --socket /tmp/graspgen.sock
```

## 3. AnyGrasp (SOTA, commercial — license-gated)
Get a license + the SDK from <https://github.com/graspnet/anygrasp_sdk>
(registers your machine ID; they email a license file).

```bash
python3.10 -m venv ~/anygrasp_env && source ~/anygrasp_env/bin/activate
# install per SDK (MinkowskiEngine etc.), place license, then:
# python scripts/grasp_detectors/bridges/anygrasp_bridge.py --socket /tmp/anygrasp.sock
```

---

## Bridge contract
A bridge listens on its socket and, per request, returns ranked grasps **in the
camera/world frame matching the input points**:

```
request : {"points": [[x,y,z],...], "colors": [[r,g,b],...]|null}
reply   : {"grasps": [{"pose": <16 floats row-major 4x4>, "width": <m>,
                       "score": <0..1>, "approach": [x,y,z]}, ...]}
```

Once a bridge is up, the comparison notebook auto-detects it (`available()` checks the
socket) and includes it. No bridge running → that detector is skipped, baseline still runs.
