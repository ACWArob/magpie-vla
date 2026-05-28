#!/usr/bin/env python3
"""
Dry-run analysis: detect block, get 3D position, compute grip params — NO arm/gripper movement.

Steps:
  1. Wait for camera frames + TCP pose
  2. Run Grounding DINO to detect the object
  3. Back-project depth to get 3D world position
  4. Call Gemini DeliGrasp descriptor for grip params
  5. Print everything — nothing moves

Usage:
    export GEMINI_API_KEY=your_key
    source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
    python3 scripts/test_dryrun_analyze.py --object "red block"
"""

import argparse
import os
import re
import sys
import time
from concurrent.futures import ThreadPoolExecutor

# ── Safety constants — MUST match deligrasp_node.py ──────────────────────────
GRIPPER_LENGTH = 0.231   # m  wrist flange → fingertip (open gripper)
HARD_FLOOR_Z   = 0.030   # m  physical floor limit (open-fingertip Z, measured 2026-05-27)
# ─────────────────────────────────────────────────────────────────────────────

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from google import genai
from google.genai import types
from magpie_control import poses
from magpie_control.homog_utils import homog_xform, R_krot
from magpie_control.gripper_arc import fingertip_drop as _gripper_drop

# Must match deligrasp_node._TCP_TO_CAM
_TCP_TO_CAM = homog_xform(
    rotnMatx=R_krot([0.0, 0.0, 1.0], -np.pi / 2.0),
    posnVctr=[0.0, 0.0, 0.120],
)

DG_DESCRIPTOR_PROMPT = """Control a robot gripper with force control and contact information. \
The gripper's parameters can be adjusted corresponding to the type of object that it is trying \
to grasp as well as the kind of grasp it is attempting to perform.
The gripper has a measurable max force of 16N and min force of 0.15N, a maximum aperture of \
105mm and a minimum aperture of 1mm.

Some grasps may be incomplete, intended for observing force information about a given object.
Describe the grasp strategy using the following form:

[start of description]
* This {CHOICE: [is, is not]} a new grasp.
* In accordance with the user instruction, this grasp should be [GRASP_DESCRIPTION: <str>].
* This is a {CHOICE: [complete, incomplete]} grasp.
* This grasp {CHOICE: [does, does not]} contain multiple grasps.
* This grasp is for an object with {CHOICE: [high, medium, low]} weight.
* The object has an approximate mass of [PNUM: 0.0] grams
* This grasp is for an object with {CHOICE: [high, medium, low]} compliance.
* The object has an approximate spring constant of [PNUM: 0.0] Newtons per meter.
* The gripper and object have an approximate friction coefficient of [PNUM: 0.0]
* This grasp should set the goal aperture to [PNUM: 0.0] mm.
* If the gripper slips, this grasp should close an additional [PNUM: 0.0] mm.
* If the gripper slips, this grasp should increase the output force by [PNUM: 0.0] Newtons.
* [optional] Because of [GRASP_DESCRIPTION: <str>], this grasp sets the force to be \
{CHOICE: [lower, higher]} than the default minimum grasp force.
[end of description]

Rules:
1. Replace {NUM} with a number, {PNUM} with a positive non-zero number.
2. Replace {CHOICE: [...]} with one of the listed choices.
3. Replace [GRASP_DESCRIPTION] with a description from the user instruction.
4. Set initial grasp force from object knowledge.
5. If deviating from default force, use the optional bullet to explain.
6. Estimate spring constant: 20 N/m (very soft) to 2000 N/m (very stiff).
7. If grasp slips, estimate aperture closure increase then output force increase.
8. Additional force = max(0.05, k * additional_closure * 0.0001).
9. Always start with [start of description] and end with [end of description].
10. Only use the bullet points from the template.
11. Give the full description. Do not skip non-optional points."""


def _quat_to_axisangle(w, x, y, z):
    angle = 2.0 * np.arccos(np.clip(w, -1.0, 1.0))
    s = np.sin(angle / 2.0)
    if s < 1e-10:
        return np.zeros(3)
    return angle * np.array([x, y, z]) / s


class AnalysisNode(Node):
    def __init__(self):
        super().__init__('dryrun_analyze')
        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.camera_info = None
        self.tcp_matrix  = None

        self.create_subscription(Image,      '/camera/gripper_camera/color/image_raw',        self._color_cb,   1)
        self.create_subscription(Image,      '/camera/gripper_camera/depth/image_rect_raw',   self._depth_cb,   1)
        self.create_subscription(CameraInfo, '/camera/gripper_camera/color/camera_info',      self._caminfo_cb, 1)
        self.create_subscription(PoseStamped,'/arm/tcp_pose',                                 self._tcp_cb,     1)

    def _color_cb(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

    def _depth_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def _caminfo_cb(self, msg):
        self.camera_info = msg

    def _tcp_cb(self, msg):
        p = msg.pose
        rv = _quat_to_axisangle(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z)
        self.tcp_matrix = poses.pose_vec_to_mtrx([
            p.position.x, p.position.y, p.position.z, rv[0], rv[1], rv[2]
        ])

    def wait_for_data(self, timeout=10.0):
        t = time.time()
        while time.time() - t < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(v is not None for v in [self.color_image, self.depth_image,
                                           self.camera_info, self.tcp_matrix]):
                return True
        return False


def detect_object(image_rgb, query, confidence=0.3):
    """Run Grounding DINO directly (bypasses magpie_perception version issues)."""
    import torch
    from PIL import Image as PILImage
    from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection

    model_id = 'IDEA-Research/grounding-dino-tiny'
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    processor = AutoProcessor.from_pretrained(model_id)
    model = AutoModelForZeroShotObjectDetection.from_pretrained(model_id).to(device)

    pil_img = PILImage.fromarray(image_rgb)
    text = query + '.'   # DINO requires period-terminated text
    inputs = processor(images=pil_img, text=text, return_tensors='pt').to(device)

    with torch.no_grad():
        outputs = model(**inputs)

    h, w = image_rgb.shape[:2]
    results = processor.post_process_grounded_object_detection(
        outputs,
        input_ids=inputs.input_ids,
        threshold=confidence,
        text_threshold=confidence,
        target_sizes=[(h, w)],
    )

    r = results[0]
    boxes  = r['boxes'].cpu().numpy()   # (N, 4) xyxy
    scores = r['scores'].cpu().numpy()  # (N,)
    labels = np.array(r['labels'])      # (N,) text labels

    return boxes, labels, scores


def detect_object_gemini(image_rgb, query, api_key):
    """Use Gemini Vision API to detect object. Returns (boxes, labels, scores)."""
    import cv2 as _cv2
    _, buf = _cv2.imencode('.jpg', _cv2.cvtColor(image_rgb, _cv2.COLOR_RGB2BGR))
    h, w = image_rgb.shape[:2]

    prompt = (
        f'Find the {query} in this image. '
        'Return ONLY the bounding box as [y_min, x_min, y_max, x_max] '
        'where each value is an integer 0-1000 scaled to image dimensions. '
        'If not visible, return: not found'
    )

    client = genai.Client(api_key=api_key)
    response = client.models.generate_content(
        model='gemini-2.5-flash',
        contents=[
            types.Part.from_bytes(data=buf.tobytes(), mime_type='image/jpeg'),
            prompt,
        ],
    )
    text = response.text.strip()
    print(f'  Gemini VLM response: {text}')

    m = re.search(r'\[(\d+),\s*(\d+),\s*(\d+),\s*(\d+)\]', text)
    if not m or 'not found' in text.lower():
        return np.zeros((0, 4)), np.array([]), np.array([])

    y1, x1, y2, x2 = [int(m.group(i)) for i in range(1, 5)]
    x1p = int(x1 * w / 1000)
    y1p = int(y1 * h / 1000)
    x2p = int(x2 * w / 1000)
    y2p = int(y2 * h / 1000)

    boxes  = np.array([[x1p, y1p, x2p, y2p]], dtype=float)
    scores = np.array([0.9])   # Gemini doesn't emit confidence scores
    labels = np.array([query])
    return boxes, labels, scores


_SAM3_PYTHON = os.path.expanduser('~/sam3_env/bin/python3')
_SAM3_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sam3_infer.py')


def _parse_sam3_response(data, query):
    """Parse SAM3 JSON response dict into (boxes, labels, scores, mask)."""
    import base64
    boxes  = np.array(data['boxes'],  dtype=float)
    scores = np.array(data['scores'], dtype=float)
    labels = np.array([query] * len(scores))
    mask = None
    if data.get('mask_b64') and len(boxes) > 0:
        raw  = base64.b64decode(data['mask_b64'])
        h, w = data['mask_shape']
        mask = np.frombuffer(raw, dtype=np.uint8).reshape(h, w).astype(bool)
        best = int(np.argmax(scores))
        print(f'  SAM3 mask: {mask.sum()} pixels  score={scores[best]:.3f}')
    return boxes, labels, scores, mask


def start_sam3_server():
    """Start SAM3 persistent server. Returns (proc, cold_load_seconds)."""
    import subprocess
    import json as _json
    print('  Starting SAM3 server (cold load)...')
    t0 = time.time()
    proc = subprocess.Popen(
        [_SAM3_PYTHON, _SAM3_SCRIPT, '--server'],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=open('/tmp/sam3_dryrun.log', 'w'),
        text=True, bufsize=1,
    )
    ready_line = proc.stdout.readline()
    cold_time = time.time() - t0
    msg = _json.loads(ready_line)
    if msg.get('status') != 'ready':
        proc.terminate()
        raise RuntimeError(f'SAM3 server unexpected startup message: {ready_line}')
    print(f'  SAM3 server ready  (cold load: {cold_time:.1f}s — warm calls will be ~1s)')
    return proc, cold_time


def detect_object_sam3(image_rgb, query, confidence=0.3):
    """SAM3 cold single-shot: spawns a fresh process each call (~12s load + ~1s inference).
    Falls back to DINO if sam3_env is unavailable."""
    import subprocess
    import tempfile
    import json as _json

    tmp_path = tempfile.mktemp(suffix='.jpg')
    cv2.imwrite(tmp_path, cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR))
    try:
        result = subprocess.run(
            [_SAM3_PYTHON, _SAM3_SCRIPT, '--image', tmp_path, '--query', query],
            capture_output=True, text=True, timeout=120,
        )
        if result.returncode != 0 or not result.stdout.strip():
            raise RuntimeError(
                f'rc={result.returncode} | stderr={result.stderr.strip()[:300] or "(empty)"} '
                f'| stdout={result.stdout.strip()[:100] or "(empty)"}'
            )
        data = _json.loads(result.stdout)
        if 'error' in data:
            raise RuntimeError(data['error'])
        return _parse_sam3_response(data, query)
    except Exception as e:
        print(f'  [WARN] SAM3 failed: {e} — falling back to DINO')
        boxes, labels, scores = detect_object(image_rgb, query, confidence)
        return boxes, labels, scores, None
    finally:
        os.unlink(tmp_path)


def detect_object_sam3_socket(image_rgb, query, confidence=0.3, sock_path='/tmp/sam3.sock'):
    """SAM3 via background socket server (~1s, server must already be running).
    Falls back to DINO if socket is unreachable."""
    import socket as _socket
    import tempfile
    import json as _json

    tmp_path = tempfile.mktemp(suffix='.jpg')
    cv2.imwrite(tmp_path, cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR))
    try:
        with _socket.socket(_socket.AF_UNIX, _socket.SOCK_STREAM) as s:
            s.connect(sock_path)
            s.sendall((_json.dumps({'image': tmp_path, 'query': query}) + '\n').encode())
            response = b''
            while True:
                chunk = s.recv(65536)
                if not chunk:
                    break
                response += chunk
        data = _json.loads(response.decode().strip())
        if 'error' in data:
            raise RuntimeError(data['error'])
        return _parse_sam3_response(data, query)
    except Exception as e:
        print(f'  [WARN] SAM3 socket failed: {e} — falling back to DINO')
        boxes, labels, scores = detect_object(image_rgb, query, confidence)
        return boxes, labels, scores, None
    finally:
        os.unlink(tmp_path)


def detect_object_sam3_warm(proc, image_rgb, query, confidence=0.3):
    """SAM3 warm single-shot: sends one request to an already-running server (~1s).
    Falls back to DINO if the server has died."""
    import tempfile
    import json as _json

    tmp_path = tempfile.mktemp(suffix='.jpg')
    cv2.imwrite(tmp_path, cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR))
    try:
        if proc is None or proc.poll() is not None:
            raise RuntimeError('SAM3 server is not running')
        req = _json.dumps({'image': tmp_path, 'query': query})
        proc.stdin.write(req + '\n')
        proc.stdin.flush()
        response_line = proc.stdout.readline()
        data = _json.loads(response_line)
        if 'error' in data:
            raise RuntimeError(data['error'])
        return _parse_sam3_response(data, query)
    except Exception as e:
        print(f'  [WARN] SAM3 warm call failed: {e} — falling back to DINO')
        boxes, labels, scores = detect_object(image_rgb, query, confidence)
        return boxes, labels, scores, None
    finally:
        os.unlink(tmp_path)


def call_descriptor(object_name, api_key):
    client = genai.Client(api_key=api_key)
    response = client.models.generate_content(
        model='gemini-2.5-flash',
        contents=f'Pick up the {object_name}.',
        config=types.GenerateContentConfig(system_instruction=DG_DESCRIPTOR_PROMPT),
    )
    text = response.text
    print('\n--- Gemini DeliGrasp Descriptor ---')
    print(text)
    print('------------------------------------\n')

    m = re.search(r'\[start of description\](.*?)\[end of description\]', text, re.DOTALL | re.IGNORECASE)
    if not m:
        raise RuntimeError('Descriptor tags not found in response')
    body = m.group(1)

    def g(pat):
        hit = re.search(pat, body, re.IGNORECASE)
        return float(hit.group(1)) if hit else None

    mass_g   = g(r'approximate mass of ([0-9.]+) grams')
    k        = g(r'spring constant of ([0-9.]+) Newtons per meter')
    mu       = g(r'friction coefficient of ([0-9.]+)')
    aperture = g(r'goal aperture to ([0-9.]+) mm')
    closure  = g(r'close an additional ([0-9.]+) mm')

    if any(v is None for v in [mass_g, k, mu, aperture, closure]):
        raise RuntimeError(f'Parse failed — mass={mass_g} k={k} mu={mu} aperture={aperture} closure={closure}')

    initial_force = min(16.0, max(0.15, (mass_g / 1000.0 * 9.81) / max(mu, 1e-6)))
    add_force     = max(0.05, closure * k * 0.0001)

    return dict(mass_g=mass_g, k=k, mu=mu, aperture_mm=aperture,
                closure_mm=closure, initial_force=initial_force, add_force=add_force)


def identify_object(image_rgb, api_key):
    """Ask Gemini what the main graspable object in the scene is. Returns a short name string."""
    import cv2 as _cv2
    _, buf = _cv2.imencode('.jpg', _cv2.cvtColor(image_rgb, _cv2.COLOR_RGB2BGR))
    client = genai.Client(api_key=api_key)
    response = client.models.generate_content(
        model='gemini-2.5-flash',
        contents=[
            types.Part.from_bytes(data=buf.tobytes(), mime_type='image/jpeg'),
            'What is the main graspable object in this image? '
            'Reply with only the object name, 2-4 words maximum, no punctuation.',
        ],
    )
    return response.text.strip().lower()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--object', default=None, help='Object name for detection and descriptor. '
                        'If omitted, Gemini identifies the object from the camera feed automatically.')
    parser.add_argument('--confidence', type=float, default=0.3, help='Detection confidence threshold (grounding_dino only)')
    parser.add_argument('--detector', default='grounding_dino',
                        choices=['grounding_dino', 'gemini', 'sam3'],
                        help='Detection backend: grounding_dino (local ViT), '
                             'gemini (cloud VLM), or sam3 (DINO + SAM3 mask, best depth)')
    parser.add_argument('--warm', action='store_true',
                        help='SAM3 only: start server inline (~12s cold load then ~1s inference)')
    parser.add_argument('--socket', nargs='?', const='/tmp/sam3.sock', metavar='PATH',
                        help='SAM3 only: connect to background socket server (~1s, no cold load). '
                             'Start server with: ~/sam3_env/bin/python3 scripts/sam3_infer.py --socket')
    parser.add_argument('--pcd', action='store_true',
                        help='SAM3 only: build segmented point cloud, run PCA, print centroid + grasp angle')
    parser.add_argument('--save-image', default='/tmp/dryrun_detection.jpg',
                        help='Path to save annotated detection image')
    parser.add_argument('--approach-height', type=float, default=0.10, metavar='M',
                        help='Approach height above object in metres (default 0.10). '
                             'Must match --ros-args -p approach_height:=X passed to the node.')
    parser.add_argument('--grasp-offset', type=float, default=0.02, metavar='M',
                        help='How far below object surface fingertips descend for mid-grip (default 0.02). '
                             'Must match --ros-args -p grasp_z_offset:=X passed to the node.')
    args = parser.parse_args()

    # ── Validate motion params before doing anything else ─────────────────────
    if args.approach_height < 0:
        print(f'ERROR: --approach-height {args.approach_height} m is negative — arm would go below object.')
        sys.exit(1)
    if args.grasp_offset < 0:
        print(f'ERROR: --grasp-offset {args.grasp_offset} m is negative — would lift instead of descend.')
        sys.exit(1)

    api_key = os.environ.get('GEMINI_API_KEY', '')
    if not api_key:
        print('ERROR: GEMINI_API_KEY not set.')
        sys.exit(1)

    rclpy.init()
    node = AnalysisNode()

    # ── 1. Wait for all sensor data ───────────────────────────────────────────
    print('Waiting for camera frames and arm pose...')
    if not node.wait_for_data(timeout=10.0):
        missing = [name for name, val in [
            ('color image', node.color_image), ('depth image', node.depth_image),
            ('camera info', node.camera_info), ('arm TCP pose', node.tcp_matrix)
        ] if val is None]
        print(f'ERROR: timed out waiting for: {", ".join(missing)}')
        sys.exit(1)
    print('  All sensor data received.')

    # Grab a fresh consistent frame
    rclpy.spin_once(node, timeout_sec=0.2)
    color  = node.color_image.copy()
    depth  = node.depth_image.copy()
    caminfo = node.camera_info
    tcp    = node.tcp_matrix.copy()

    print(f'\nArm TCP: x={tcp[0,3]:.3f}  y={tcp[1,3]:.3f}  z={tcp[2,3]:.3f} m')
    print(f'Image:   {color.shape[1]}x{color.shape[0]} px')
    print(f'Depth:   {depth.shape[1]}x{depth.shape[0]} px  '
          f'range {depth[depth>0].min() if (depth>0).any() else 0}–{depth.max()} mm\n')

    # ── 1b. Auto-identify object if not specified ─────────────────────────────
    if args.object is None:
        print('No --object specified. Asking Gemini to identify the object...')
        try:
            args.object = identify_object(color, api_key)
            print(f'  Detected object: "{args.object}"')
        except Exception as e:
            print(f'  ERROR: Could not auto-identify object: {e}')
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

    # ── 2. SAM3 warm server startup (before thread pool, so cold load is timed separately) ──
    sam3_proc = None
    if args.detector == 'sam3' and args.warm:
        sam3_proc, _ = start_sam3_server()

    # ── 3. Detection + Gemini descriptor in parallel ──────────────────────────
    if args.detector == 'gemini':
        print(f'Running Gemini VLM detector + descriptor for "{args.object}" (parallel)...')
    elif args.detector == 'sam3' and args.socket:
        print(f'Running SAM3 (socket ~1s) + descriptor for "{args.object}" in parallel...')
    elif args.detector == 'sam3' and args.warm:
        print(f'Running SAM3 (warm ~1s) + descriptor for "{args.object}" in parallel...')
    elif args.detector == 'sam3':
        print(f'Running SAM3 (cold ~12s) + descriptor for "{args.object}" in parallel...')
    else:
        print(f'Running Grounding DINO + descriptor for "{args.object}" in parallel...')

    _pool = ThreadPoolExecutor(max_workers=2)
    if args.detector == 'gemini':
        det_fut = _pool.submit(detect_object_gemini, color, args.object, api_key)
    elif args.detector == 'sam3' and args.socket:
        det_fut = _pool.submit(detect_object_sam3_socket, color, args.object, args.confidence, args.socket)
    elif args.detector == 'sam3' and args.warm:
        det_fut = _pool.submit(detect_object_sam3_warm, sam3_proc, color, args.object, args.confidence)
    elif args.detector == 'sam3':
        det_fut = _pool.submit(detect_object_sam3, color, args.object, args.confidence)
    else:
        det_fut = _pool.submit(detect_object, color, args.object, args.confidence)
    desc_fut = _pool.submit(call_descriptor, args.object, api_key)
    _pool.shutdown(wait=False)

    # Collect both futures — descriptor was running concurrently and is almost certainly
    # done by the time detection finishes (both are I/O-bound Gemini or CPU-bound DINO).
    det_result = det_fut.result()
    try:
        gp = desc_fut.result(timeout=30.0)
    except Exception as e:
        print(f'  [WARN] Gemini descriptor failed: {e} — safety check will use worst-case aperture')
        gp = None

    seg_mask = None
    if args.detector == 'sam3':
        boxes, labels, scores, seg_mask = det_result
    elif args.detector == 'gemini':
        boxes, labels, scores = det_result
    else:
        boxes, labels, scores = det_result

    annotated = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)

    if len(boxes) == 0:
        print('  No detections found.')
        best_u, best_v = color.shape[1] // 2, color.shape[0] // 2
        world_pos = None
    else:
        best = int(np.argmax(scores))
        x1, y1, x2, y2 = [int(v) for v in boxes[best]]

        # For SAM3: use mask centroid as the representative pixel; otherwise box centre
        if seg_mask is not None and seg_mask.any():
            ys, xs = np.where(seg_mask)
            best_u, best_v = int(xs.mean()), int(ys.mean())
            # Draw semi-transparent mask overlay
            overlay = annotated.copy()
            overlay[seg_mask] = (0, 200, 100)
            annotated = cv2.addWeighted(overlay, 0.35, annotated, 0.65, 0)
        else:
            best_u = (x1 + x2) // 2
            best_v = (y1 + y2) // 2

        print(f'  Best detection: "{labels[best]}"  score={scores[best]:.3f}')
        print(f'  Bounding box:   [{x1}, {y1}, {x2}, {y2}]')
        print(f'  Centroid pixel: ({best_u}, {best_v})'
              + (' (mask centroid)' if seg_mask is not None else ''))

        # Draw all detections
        for i, (box, label, score) in enumerate(zip(boxes, labels, scores)):
            bx1, by1, bx2, by2 = [int(v) for v in box]
            color_box = (0, 255, 0) if i == best else (128, 128, 128)
            cv2.rectangle(annotated, (bx1, by1), (bx2, by2), color_box, 2)
            cv2.putText(annotated, f'{label} {score:.2f}',
                        (bx1, by1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_box, 1)

        cv2.circle(annotated, (best_u, best_v), 6, (0, 0, 255), -1)

        # ── 3. Depth + 3D localisation ────────────────────────────────────
        if seg_mask is not None and seg_mask.any():
            valid = depth[seg_mask].astype(float)
            valid = valid[valid > 0]
            print(f'  Using SAM3 mask depth ({len(valid)} pixels)')
        else:
            pad = 5
            roi = depth[max(0, best_v-pad):best_v+pad, max(0, best_u-pad):best_u+pad].astype(float)
            valid = roi[roi > 0]
        if len(valid) == 0:
            print('  [WARN] No valid depth pixels at centroid — cannot compute 3D position')
            world_pos = None
        else:
            depth_m = float(np.median(valid)) / 1000.0
            k_mat = caminfo.k
            fx, fy = k_mat[0], k_mat[4]
            cx, cy = k_mat[2], k_mat[5]
            p_cam = np.array([
                (best_u - cx) * depth_m / fx,
                (best_v - cy) * depth_m / fy,
                depth_m,
            ])
            T = tcp @ _TCP_TO_CAM
            world_pos = (T @ np.array([*p_cam, 1.0]))[:3]

            print(f'\n  Depth at centroid:  {depth_m*100:.1f} cm')
            print(f'  Camera-frame pos:   x={p_cam[0]*100:.1f}  y={p_cam[1]*100:.1f}  z={p_cam[2]*100:.1f} cm')
            print(f'  World-frame pos:    x={world_pos[0]:.3f}  y={world_pos[1]:.3f}  z={world_pos[2]:.3f} m')

            # Fingertip target Z (what touches / hovers near the object)
            approach_fz = world_pos[2] + args.approach_height   # hover above object
            grasp_fz    = world_pos[2] - args.grasp_offset       # descend into object for mid-grip

            # TCP Z = fingertip_z + GRIPPER_LENGTH (wrist is above fingertips)
            approach_tcp_z = approach_fz + GRIPPER_LENGTH
            grasp_tcp_z    = grasp_fz    + GRIPPER_LENGTH

            print(f'\n  Motion params: approach_height={args.approach_height:.3f} m  '
                  f'grasp_offset={args.grasp_offset:.3f} m')
            print(f'  Approach: fingertip Z={approach_fz:.3f} m  TCP Z={approach_tcp_z:.3f} m')
            print(f'  Grasp:    fingertip Z={grasp_fz:.3f} m  TCP Z={grasp_tcp_z:.3f} m')

            # ── Pre-flight safety check — will it crash? ─────────────────────
            approach_safe = approach_fz >= HARD_FLOOR_Z

            goal_ap_mm   = float(gp['aperture_mm']) if gp else 0.0
            close_drop   = _gripper_drop(103.6, goal_ap_mm)
            grasp_eff_z  = grasp_fz - close_drop   # fingertip Z after closing
            grasp_safe   = grasp_eff_z >= HARD_FLOOR_Z

            print(f'\n=== SAFETY CHECK ===')
            print(f'  Floor (table): {HARD_FLOOR_Z:.3f} m')
            print(f'  Approach: fingertip {approach_fz:.3f} m  {"✓" if approach_safe else "✗ BELOW FLOOR"}')
            print(f'  Grasp:    open {grasp_fz:.3f} m  −  {close_drop*1000:.0f} mm arc drop'
                  f'  →  {grasp_eff_z:.3f} m  {"✓" if grasp_safe else "✗ WOULD HIT FLOOR"}')
            print(f'====================')
            if not approach_safe or not grasp_safe:
                print('\nABORTED — fingertip would crash into floor. Raise object or adjust --grasp-offset.')
                node.destroy_node()
                rclpy.shutdown()
                sys.exit(1)

    # Save annotated image
    cv2.imwrite(args.save_image, annotated)
    print(f'\n  Detection image saved → {args.save_image}')

    # ── 4. Point cloud analysis (SAM3 mask + depth → PCA) ─────────────────────
    if args.pcd and args.detector == 'sam3' and seg_mask is not None:
        try:
            import sys as _sys
            _sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
            from pointcloud_utils import build_segmented_pcd, denoise_pcd, analyse_pcd, print_pcd_results
            k = caminfo.k
            pts_raw, pcd_raw = build_segmented_pcd(seg_mask, depth, k, tcp, _TCP_TO_CAM)
            pcd_clean, pts_clean = denoise_pcd(pcd_raw)
            result = analyse_pcd(pcd_clean)
            print_pcd_results(result, len(pts_clean))
        except Exception as e:
            print(f'  [WARN] Point cloud analysis failed: {e}')
    elif args.pcd and args.detector != 'sam3':
        print('  [WARN] --pcd requires --detector sam3 (needs pixel mask)')

    # ── 5. Gemini DeliGrasp descriptor (already collected above) ─────────────
    print(f'\nGemini descriptor for "{args.object}":')
    if gp is not None:
        print('=== GRIP PARAMETERS (computed from descriptor) ===')
        print(f'  Object mass       : {gp["mass_g"]:.0f} g')
        print(f'  Spring constant   : {gp["k"]:.0f} N/m')
        print(f'  Friction coeff μ  : {gp["mu"]:.2f}')
        print(f'  Goal aperture     : {gp["aperture_mm"]:.1f} mm')
        print(f'  Additional closure: {gp["closure_mm"]:.1f} mm  (if slip)')
        print(f'  Initial force     : {gp["initial_force"]:.3f} N   ← set before close')
        print(f'  Additional force  : {gp["add_force"]:.3f} N   ← added per slip')
        print('==================================================')
    else:
        print('  (descriptor failed — see earlier warning)')

    # ── 5. Summary ────────────────────────────────────────────────────────────
    print('\n=== DRY RUN SUMMARY ===')
    if world_pos is not None:
        print(f'  Object world XYZ  : ({world_pos[0]:.3f}, {world_pos[1]:.3f}, {world_pos[2]:.3f}) m')
    else:
        print('  Object world XYZ  : N/A (detection or depth failed)')
    print('  Arm/gripper       : NOT moved (dry run)')
    print('=======================\n')

    if sam3_proc and sam3_proc.poll() is None:
        sam3_proc.stdin.close()
        sam3_proc.terminate()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
