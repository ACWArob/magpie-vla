"""GSNet / graspness detector via COLD-START docker run (no resident server).

GSNet (ICCV'21 graspness) is the open core of AnyGrasp — same SJTU lineage — so it
gives us that family in the comparison now, before the AnyGrasp license lands.

It needs MinkowskiEngine + a compiled CUDA stack that doesn't exist on the host
(no nvcc), so the model lives in a Docker image. Rather than a resident server, each
detect() does a one-shot `docker run --rm`: the container loads the model, infers on
the cloud, writes grasps JSON, and exits — freeing ALL its VRAM before the arm moves.
That trades ~model-load latency per grasp for zero VRAM coexistence with SAM3, which is
the right call on an 8GB GPU ("best auto data, not speed").

Returned grasp poses are in the SAME frame as the input cloud (the magpie pipeline
passes a WORLD-frame segmented cloud), like GraspGenZMQ — no extra transform here.
"""

import json
import os
import subprocess
import time

import numpy as np

try:
    from grasp_detectors.base import Grasp, GraspDetector
except Exception:
    from base import Grasp, GraspDetector


class GSNetDocker(GraspDetector):
    name = 'gsnet'

    def __init__(self, image='gsnet:latest',
                 io_dir='~/magpie_control/data/gsnet_io',
                 weights='~/GSNetModels/graspness_realsense.tar',
                 topk=20, num_point=15000,
                 docker_wrap='sg docker -c', timeout=300):
        # docker_wrap: the magpie process isn't in the docker group at process-tree
        # level, so we run docker through `sg docker -c "..."` (same as the build). Set
        # to '' if the launching shell already has docker group membership.
        self.image = image
        self.io_dir = os.path.expanduser(io_dir)
        self.weights = os.path.expanduser(weights)
        self.topk = topk
        self.num_point = num_point
        self.docker_wrap = docker_wrap
        self.timeout = timeout
        os.makedirs(self.io_dir, exist_ok=True)

    def _wrap(self, inner):
        return f'{self.docker_wrap} "{inner}"' if self.docker_wrap else inner

    def available(self) -> bool:
        try:
            out = subprocess.check_output(
                self._wrap(f'docker images -q {self.image}'),
                shell=True, text=True, timeout=20).strip()
            return bool(out)
        except Exception:
            return False

    def _detect(self, points, colors=None, mask=None, tcp=None, intrinsics=None):
        pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        if len(pts) < 100:
            return []
        stamp = str(int(time.time() * 1000))
        in_npy = os.path.join(self.io_dir, f'in_{stamp}.npy')
        out_json = os.path.join(self.io_dir, f'out_{stamp}.json')
        np.save(in_npy, pts)

        wdir = os.path.dirname(self.weights)
        wname = os.path.basename(self.weights)
        inner = (
            # legacy nvidia runtime (this host's CDI spec mounts a missing MPS binary,
            # so `--gpus all` fails; --runtime=nvidia + NVIDIA_VISIBLE_DEVICES is the path)
            f'docker run --rm --runtime=nvidia -e NVIDIA_VISIBLE_DEVICES=all --ipc=host '
            # pointnet2_utils.py does `import pytorch_utils` (a sibling in /code/pointnet2),
            # so put that dir on PYTHONPATH (more robust than baking a sys.path line).
            f'-e PYTHONPATH=/code/pointnet2:/code/utils:/code '
            f'-v {self.io_dir}:/io -v {wdir}:/weights '
            f'{self.image} '
            f'python /code/infer_oneshot.py '
            f'--in_npy /io/{os.path.basename(in_npy)} '
            f'--out_json /io/{os.path.basename(out_json)} '
            f'--checkpoint /weights/{wname} '
            f'--topk {self.topk} --num_point {self.num_point}'
        )
        try:
            subprocess.run(self._wrap(inner), shell=True, timeout=self.timeout,
                           check=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            with open(out_json) as f:
                d = json.load(f)
        except subprocess.CalledProcessError as e:
            tail = (e.output.decode()[-800:] if e.output else str(e))
            raise RuntimeError(f'gsnet docker run failed:\n{tail}')
        except FileNotFoundError:
            raise RuntimeError('gsnet produced no output (container crashed before writing JSON)')
        finally:
            for f in (in_npy, out_json):
                try:
                    os.remove(f)
                except OSError:
                    pass

        if 'error' in d:
            raise RuntimeError(f"gsnet: {d['error']}")
        out = []
        for g in d.get('grasps', []):
            pose = np.array(g['pose'], dtype=float).reshape(4, 4)
            out.append(Grasp(pose=pose, width=float(g.get('width', 0.05)),
                             score=float(g.get('score', 0.0)),
                             approach=np.array(g.get('approach', [0, 0, -1.]), dtype=float)))
        return out
