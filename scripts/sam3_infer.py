#!/usr/bin/env python3
"""
SAM3 inference helper — must run in sam3_env (Python 3.12).
Called as a subprocess from the main ROS stack (Python 3.10).

Modes:
  Single-shot:
      ~/sam3_env/bin/python3 sam3_infer.py --image /tmp/frame.jpg --query "measuring tape"

  Stdin server (spawned as subprocess, one JSON request per stdin line):
      ~/sam3_env/bin/python3 sam3_infer.py --server
      Writes {"status": "ready"} then loops stdin→stdout.

  Background socket server (persistent, survives script restarts):
      ~/sam3_env/bin/python3 sam3_infer.py --socket [/tmp/sam3.sock]
      Writes {"status": "ready", "socket": "/tmp/sam3.sock"} then accepts
      one connection per request: recv JSON line → send JSON response → close.
      Any number of clients can connect while it is running.
"""
import argparse
import base64
import json
import os
import sys

# Must be set before any huggingface_hub import to allow model download
os.environ['HF_HUB_OFFLINE'] = '0'

import numpy as np


def _build_model():
    """Load SAM3 model and return processor. Called once."""
    from sam3.model_builder import build_sam3_image_model
    from sam3.model.sam3_image_processor import Sam3Processor
    model = build_sam3_image_model()
    return Sam3Processor(model)


def _infer(processor, image_path, query):
    """Run one SAM3 inference. Returns result dict."""
    import torch
    from PIL import Image

    def _cpu(x):
        if isinstance(x, torch.Tensor):
            return x.detach().cpu().numpy()
        if isinstance(x, (list, tuple)):
            return np.array([_cpu(xi) for xi in x])
        return np.array(x)

    image = Image.open(image_path).convert('RGB')
    state = processor.set_image(image)
    output = processor.set_text_prompt(state=state, prompt=query)

    masks  = output['masks']
    boxes  = _cpu(output['boxes']).astype(float)
    scores = _cpu(output['scores']).astype(float)

    if len(scores) == 0:
        return {'boxes': [], 'scores': [], 'mask_b64': None, 'mask_shape': []}

    best = int(np.argmax(scores))
    raw  = masks[best] if isinstance(masks, (list, tuple)) else masks[best]
    mask = _cpu(raw).astype(bool)
    # SAM3 may return [1, H, W] — squeeze to [H, W]
    while mask.ndim > 2:
        mask = mask[0]

    mask_b64 = base64.b64encode(mask.astype(np.uint8).tobytes()).decode()
    return {
        'boxes':      boxes.tolist(),
        'scores':     scores.tolist(),
        'mask_b64':   mask_b64,
        'mask_shape': list(mask.shape),   # always [H, W]
    }


_DEFAULT_SOCK = '/tmp/sam3.sock'


def _run_socket_server(processor, sock_path):
    import socket as _socket
    if os.path.exists(sock_path):
        os.unlink(sock_path)
    srv = _socket.socket(_socket.AF_UNIX, _socket.SOCK_STREAM)
    srv.bind(sock_path)
    srv.listen(4)
    print(json.dumps({'status': 'ready', 'socket': sock_path}), flush=True)
    try:
        while True:
            conn, _ = srv.accept()
            try:
                buf = b''
                while b'\n' not in buf:
                    chunk = conn.recv(65536)
                    if not chunk:
                        break
                    buf += chunk
                if not buf.strip():
                    continue
                req = json.loads(buf.decode().strip())
                result = _infer(processor, req['image'], req['query'])
                conn.sendall((json.dumps(result) + '\n').encode())
            except Exception as e:
                try:
                    conn.sendall((json.dumps({'error': str(e)}) + '\n').encode())
                except Exception:
                    pass
            finally:
                conn.close()
    finally:
        srv.close()
        if os.path.exists(sock_path):
            os.unlink(sock_path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--image',  help='Path to JPEG image (single-shot mode)')
    parser.add_argument('--query',  help='Text prompt (single-shot mode)')
    parser.add_argument('--server', action='store_true',
                        help='Stdin server: read JSON requests from stdin (subprocess use)')
    parser.add_argument('--socket', nargs='?', const=_DEFAULT_SOCK, metavar='PATH',
                        help=f'Background socket server on Unix socket (default: {_DEFAULT_SOCK})')
    args = parser.parse_args()

    try:
        processor = _build_model()

        if args.socket:
            _run_socket_server(processor, args.socket)
        elif args.server:
            print(json.dumps({'status': 'ready'}), flush=True)
            for line in sys.stdin:
                line = line.strip()
                if not line:
                    continue
                try:
                    req = json.loads(line)
                    result = _infer(processor, req['image'], req['query'])
                    print(json.dumps(result), flush=True)
                except Exception as e:
                    print(json.dumps({'error': str(e)}), flush=True)
        else:
            if not args.image or not args.query:
                print(json.dumps({'error': '--image and --query required in single-shot mode'}))
                sys.exit(1)
            print(json.dumps(_infer(processor, args.image, args.query)))

    except Exception as e:
        print(json.dumps({'error': str(e)}))
        sys.exit(1)


if __name__ == '__main__':
    main()
