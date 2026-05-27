#!/usr/bin/env python3
"""
Benchmark SAM3 persistent server: cold load time vs warm inference time.
Starts the server, sends 3 requests, reports timing for each.

Usage:
    source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
    python3 scripts/test_sam3_server.py --object "measuring tape"
"""
import argparse
import json
import os
import subprocess
import sys
import tempfile
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


_SAM3_PYTHON = os.path.expanduser('~/sam3_env/bin/python3')
_SAM3_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sam3_infer.py')


class FrameGrabber(Node):
    def __init__(self):
        super().__init__('sam3_server_test')
        self.bridge = CvBridge()
        self.frame = None
        self.create_subscription(
            Image, '/camera/gripper_camera/color/image_raw', self._cb, 1)

    def _cb(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

    def grab(self, timeout=10.0):
        t = time.time()
        while time.time() - t < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.frame is not None:
                return self.frame
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--object', default='measuring tape', help='Object to detect')
    parser.add_argument('--runs', type=int, default=3, help='Number of warm inference runs')
    args = parser.parse_args()

    # ── 1. Grab a camera frame ─────────────────────────────────────────────────
    print('Waiting for camera frame...')
    rclpy.init()
    node = FrameGrabber()
    frame = node.grab(timeout=10.0)
    node.destroy_node()
    rclpy.shutdown()

    if frame is None:
        print('ERROR: no camera frame — is the camera node running?')
        sys.exit(1)

    tmp = tempfile.mktemp(suffix='.jpg')
    cv2.imwrite(tmp, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    print(f'  Frame captured: {frame.shape[1]}x{frame.shape[0]} px  saved → {tmp}')

    # ── 2. Start SAM3 server (cold) ────────────────────────────────────────────
    print('\nStarting SAM3 server (cold load)...')
    t_start = time.time()
    proc = subprocess.Popen(
        [_SAM3_PYTHON, _SAM3_SCRIPT, '--server'],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=open('/tmp/sam3_server_bench.log', 'w'),
        text=True, bufsize=1,
    )
    ready_line = proc.stdout.readline()
    cold_time = time.time() - t_start
    msg = json.loads(ready_line)
    if msg.get('status') != 'ready':
        print(f'ERROR: unexpected startup message: {ready_line}')
        proc.terminate()
        sys.exit(1)
    print(f'  Cold load: {cold_time:.1f}s')

    # ── 3. Warm inference runs ─────────────────────────────────────────────────
    print(f'\nRunning {args.runs} warm inference requests for "{args.object}"...')
    times = []
    for i in range(args.runs):
        req = json.dumps({'image': tmp, 'query': args.object})
        t0 = time.time()
        proc.stdin.write(req + '\n')
        proc.stdin.flush()
        resp_line = proc.stdout.readline()
        elapsed = time.time() - t0

        data = json.loads(resp_line)
        if 'error' in data:
            print(f'  Run {i+1}: ERROR — {data["error"]}')
        else:
            scores = data.get('scores', [])
            best_score = max(scores) if scores else 0.0
            px = sum(1 for b in np.frombuffer(
                __import__('base64').b64decode(data['mask_b64']), dtype=np.uint8
            ) if b) if data.get('mask_b64') else 0
            print(f'  Run {i+1}: {elapsed:.2f}s  score={best_score:.3f}  mask={px} px')
            times.append(elapsed)

    # ── 4. Summary ─────────────────────────────────────────────────────────────
    if times:
        print(f'\n=== SAM3 Server Benchmark ===')
        print(f'  Cold load:      {cold_time:.1f}s  (one-time, at node startup)')
        print(f'  Warm inference: {sum(times)/len(times):.2f}s avg  '
              f'(min {min(times):.2f}s, max {max(times):.2f}s)')
        print(f'  Speedup:        {cold_time/( sum(times)/len(times) ):.1f}x faster after warmup')
        print('=============================')

    proc.stdin.close()
    proc.terminate()
    os.unlink(tmp)


if __name__ == '__main__':
    main()
