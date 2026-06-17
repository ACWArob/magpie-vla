"""
GraspGen detector — a thin ZMQ client to a running GraspGen server.

The GraspGen model runs in its own process/Docker on the GPU (see
~/GraspGen/client-server/README.md); this client only needs pyzmq + msgpack
(no CUDA), so it lives in the magpie env. Protocol reverse-engineered from
grasp_gen/serving/zmq_client.py so we don't have to import the grasp_gen package.

    request : msgpack {"action":"infer","point_cloud":(N,3) f32, "num_grasps":..,
                       "topk_num_grasps":.., "grasp_threshold":-1.0, ...}
    reply   : msgpack {"grasps":(M,4,4), "confidences":(M,)}   (centered frame)

The server expects a CENTERED object cloud, so we subtract the centroid before
sending and add it back to the returned grasp translations (→ world frame).
"""

import numpy as np

try:
    from grasp_detectors.base import Grasp, GraspDetector
except Exception:
    from base import Grasp, GraspDetector


class GraspGenZMQ(GraspDetector):
    name = 'graspgen'

    def __init__(self, host='localhost', port=5556, timeout_ms=60000,
                 num_grasps=64, topk=10, gripper_width=0.08):
        # num_grasps kept low (64 vs the 200 default) to fit GraspGen alongside SAM3
        # on an 8GB GPU — diffusion sampling is the VRAM/peak driver. Raise it on a
        # bigger GPU for denser proposals.
        self.host, self.port = host, port
        self.timeout_ms = timeout_ms
        self.num_grasps, self.topk = num_grasps, topk
        self.gripper_width = gripper_width

    def _socket(self, timeout):
        import zmq
        ctx = zmq.Context.instance()
        s = ctx.socket(zmq.REQ)
        s.setsockopt(zmq.RCVTIMEO, timeout)
        s.setsockopt(zmq.SNDTIMEO, timeout)
        s.setsockopt(zmq.LINGER, 0)
        s.connect(f'tcp://{self.host}:{self.port}')
        return s

    def _request(self, payload, timeout):
        import msgpack, msgpack_numpy
        msgpack_numpy.patch()
        s = self._socket(timeout)
        try:
            s.send(msgpack.packb(payload, use_bin_type=True))
            resp = msgpack.unpackb(s.recv(), raw=False)
        finally:
            s.close()
        if 'error' in resp:
            raise RuntimeError(resp['error'])
        return resp

    def available(self) -> bool:
        try:
            return self._request({'action': 'health'}, 1500).get('status') == 'ok'
        except Exception:
            return False

    def _detect(self, points, colors=None, mask=None, tcp=None, intrinsics=None):
        pts = np.asarray(points, dtype=np.float32)
        if len(pts) < 10:
            return []
        centroid = pts.mean(axis=0).astype(np.float32)
        pc = (pts - centroid).astype(np.float32)         # server wants a centred cloud
        resp = self._request({
            'action': 'infer', 'point_cloud': pc,
            'grasp_threshold': -1.0, 'num_grasps': self.num_grasps,
            'topk_num_grasps': self.topk, 'min_grasps': 40,
            'max_tries': 6, 'remove_outliers': True,
        }, self.timeout_ms)
        grasps = np.asarray(resp['grasps'], dtype=float)      # (M,4,4) centred frame
        confs = np.asarray(resp['confidences'], dtype=float)
        out = []
        for g, c in zip(grasps, confs):
            pose = np.array(g, dtype=float).reshape(4, 4).copy()
            pose[:3, 3] = pose[:3, 3] + centroid              # → world frame
            out.append(Grasp(pose=pose, width=self.gripper_width, score=float(c)))
        return out
