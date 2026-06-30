"""
Generic socket-bridge client for learned grasp detectors that run in their OWN venv
(Contact-GraspNet / AnyGrasp / GraspGen). Mirrors how SAM3 is integrated
(scripts/sam3_infer.py + a Unix socket), so the heavy model never has to share the
main env's torch/CUDA.

Protocol (one JSON line each way over a Unix socket):
    request : {"points": [[x,y,z],...], "colors": [[r,g,b],...]|null}
    reply   : {"grasps": [{"pose": 4x4 row-major, "width": m, "score": s,
                           "approach": [x,y,z]}, ...]}
              or {"error": "..."}

Each detector's bridge process (e.g. scripts/grasp_detectors/bridges/contact_graspnet_bridge.py,
run inside its venv) implements the model side. See INSTALL.md.
"""

import json
import os
import socket

import numpy as np

try:
    from grasp_detectors.base import Grasp, GraspDetector
except Exception:
    from base import Grasp, GraspDetector


class SocketDetector(GraspDetector):
    def __init__(self, name, sock_path, timeout=30.):
        self.name = name
        self.sock_path = sock_path
        self.timeout = timeout

    def available(self) -> bool:
        return os.path.exists(self.sock_path)

    def _detect(self, points, colors=None, mask=None, tcp=None, intrinsics=None):
        req = {'points': np.asarray(points, float).tolist(),
               'colors': (np.asarray(colors).tolist() if colors is not None else None)}
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
            s.settimeout(self.timeout)
            s.connect(self.sock_path)
            s.sendall((json.dumps(req) + '\n').encode())
            raw = b''
            while True:
                chunk = s.recv(65536)
                if not chunk:
                    break
                raw += chunk
        d = json.loads(raw.decode().strip())
        if 'error' in d:
            raise RuntimeError(f'{self.name} bridge: {d["error"]}')
        out = []
        for g in d.get('grasps', []):
            out.append(Grasp(
                pose=np.array(g['pose'], dtype=float).reshape(4, 4),
                width=float(g.get('width', 0.05)),
                score=float(g.get('score', 0.0)),
                approach=np.array(g.get('approach', [0, 0, -1.]), dtype=float)))
        return out


# Convenience factories — point at the bridge sockets (created by the install scripts)
def contact_graspnet(sock='/tmp/contact_graspnet.sock'):
    return SocketDetector('contact_graspnet', sock)


def graspgen(host='localhost', port=5556):
    """GraspGen talks ZMQ (not a Unix socket) — its server ships a ZMQ interface."""
    try:
        from grasp_detectors.graspgen_zmq import GraspGenZMQ
    except Exception:
        from graspgen_zmq import GraspGenZMQ
    return GraspGenZMQ(host=host, port=port)


def graspgenx(host='localhost', port=5557, gripper_name='magpie'):
    """GraspGenX (NVlabs cross-embodiment, ICRA'26) — ONE model, any gripper. Runs the
    MAGPIE gripper zero-shot via the Correll-lab swept-volume def. ZMQ server on 5557."""
    try:
        from grasp_detectors.graspgenx_zmq import GraspGenXZMQ
    except Exception:
        from graspgenx_zmq import GraspGenXZMQ
    return GraspGenXZMQ(host=host, port=port, gripper_name=gripper_name)


def gsnet(image='gsnet:latest', weights='~/GSNetModels/graspness_realsense.tar'):
    """GSNet/graspness (open core of AnyGrasp) runs COLD-START in docker — a one-shot
    `docker run --rm` per grasp, so it never shares VRAM with SAM3. See GSNetDocker."""
    try:
        from grasp_detectors.gsnet_docker import GSNetDocker
    except Exception:
        from gsnet_docker import GSNetDocker
    return GSNetDocker(image=image, weights=weights)
