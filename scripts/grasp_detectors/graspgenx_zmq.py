"""GraspGenX detector — a thin ZMQ client to a running GraspGenX server.

GraspGenX (NVlabs, https://github.com/NVlabs/GraspGenX) is a *cross-embodiment* 6-DOF
grasp model: a single model that generalises to ANY gripper by conditioning on the
gripper's swept-volume representation. Unlike GraspGen (one model per gripper), GraspGenX
handles the custom **MAGPIE** gripper zero-shot, using the Correll-lab gripper definition
at ``~/GraspGenX/assets/x_grippers/magpie/`` (config.json swept-volume + URDF + meshes).

The model runs in its own uv venv + ZMQ server on the GPU (see
``~/GraspGenX/client-server/``); this client only needs pyzmq + msgpack (no torch/CUDA),
so it lives in the magpie env. Protocol mirrors graspgenx/serving/zmq_client.py:

    request : msgpack {"action":"infer","point_cloud":(N,3) f32,"gripper_name":"magpie",
                       "num_grasps":..,"grasp_threshold":-1.0,"topk_num_grasps":..}
    reply   : msgpack {"grasps":(K,4,4) f32, "confidences":(K,) f32}

Start the server (one process, any gripper) with:
    cd ~/GraspGenX && uv run python client-server/graspgenx_server.py \
        --config ext/graspgenx_checkpoints/release --assets_dir assets \
        --default_gripper magpie --port 5557

Like GraspGen, the server is fed a CENTRED object cloud, so we subtract the centroid
before sending and add it back to the returned grasp translations (→ world frame).
"""

import numpy as np

try:
    import zmq
    import msgpack
    import msgpack_numpy
    msgpack_numpy.patch()   # one-time global patch; safe to call once at import
except ImportError:
    zmq = msgpack = msgpack_numpy = None

try:
    from grasp_detectors.base import Grasp, GraspDetector
except Exception:
    from base import Grasp, GraspDetector


class GraspGenXZMQ(GraspDetector):
    name = 'graspgenx'

    def __init__(self, host='localhost', port=5557, timeout_ms=120000,
                 gripper_name='magpie', num_grasps=64, topk=20, gripper_width=0.08,
                 top_down_deg=70.0):
        # gripper_name='magpie' → the server conditions on the MAGPIE swept volume.
        # num_grasps kept modest to fit alongside other GPU users on an 8GB card.
        # top_down_deg: the pickup executes a STRICTLY top-down grasp (it only uses the
        # returned grasp's XY + closing-axis yaw). GraspGenX is 6-DOF, so its best-scored
        # grasp can be a side grasp whose score won't hold top-down. We therefore keep only
        # grasps whose approach axis is within `top_down_deg` of vertical, so the highest
        # remaining score corresponds to a grasp the model actually likes from above.
        # 55→70: OBB augmentation adds realistic side faces, so the model now scores more
        # top-down grasps confidently; wider cone lets the best of those survive.
        # Set to None to disable. NOTE: verify the approach-axis convention on the robot —
        # if grasps come out rotated 90°, the closing/approach columns may be swapped.
        self.host, self.port = host, port
        self.timeout_ms = timeout_ms
        self.gripper_name = gripper_name
        self.num_grasps, self.topk = num_grasps, topk
        self.gripper_width = gripper_width
        self.top_down_deg = top_down_deg

    def _socket(self, timeout):
        ctx = zmq.Context.instance()
        s = ctx.socket(zmq.REQ)
        s.setsockopt(zmq.RCVTIMEO, timeout)
        s.setsockopt(zmq.SNDTIMEO, timeout)
        s.setsockopt(zmq.LINGER, 0)
        s.connect(f'tcp://{self.host}:{self.port}')
        return s

    def _request(self, payload, timeout):
        s = self._socket(timeout)
        try:
            s.send(msgpack.packb(payload, use_bin_type=True))
            resp = msgpack.unpackb(s.recv(), raw=False)
        finally:
            s.close()
        if isinstance(resp, dict) and 'error' in resp:
            raise RuntimeError(resp['error'])
        return resp

    def available(self) -> bool:
        try:
            return self._request({'action': 'health'}, 2000).get('status') == 'ok'
        except Exception:
            return False

    def _detect(self, points, colors=None, mask=None, tcp=None, intrinsics=None):
        pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        if len(pts) < 10:
            return []
        centroid = pts.mean(axis=0).astype(np.float32)
        pc = (pts - centroid).astype(np.float32)            # server wants a centred cloud
        resp = self._request({
            'action': 'infer', 'point_cloud': pc,
            'gripper_name': self.gripper_name,
            'num_grasps': int(self.num_grasps),
            'grasp_threshold': -1.0,
            'topk_num_grasps': int(self.topk),
        }, self.timeout_ms)
        grasps = np.asarray(resp['grasps'], dtype=float).reshape(-1, 4, 4)   # (K,4,4) centred
        confs = np.asarray(resp['confidences'], dtype=float).reshape(-1)
        out = []
        for g, c in zip(grasps, confs):
            pose = g.copy()
            pose[:3, 3] += centroid                           # → world frame
            out.append(Grasp(pose=pose, width=self.gripper_width, score=float(c)))
        # Keep only near-vertical-approach grasps so the strictly-top-down pickup executes
        # a grasp the model actually scored from above. The approach axis is the tool Z
        # (pose[:3,2]); |z-component| ~1 means vertical. Fall back to all if too few survive.
        if self.top_down_deg is not None and out:
            import math
            cos_thr = math.cos(math.radians(self.top_down_deg))
            td = [gr for gr in out if abs(gr.pose[2, 2]) >= cos_thr]
            out = td if len(td) >= 3 else out
        return out
