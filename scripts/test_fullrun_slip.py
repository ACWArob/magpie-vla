#!/usr/bin/env python3
"""
Full grasp pipeline with adaptive slip detection.

Same as test_fullrun.py but replaces the one-shot gripper close with a
DeliGrasp-style adaptive loop:
  1. Close at initial_force (from Gemini descriptor)
  2. Wait for motor to settle
  3. Read /gripper/state — check measured force
  4. If force < contact threshold → slip detected → increase force + squeeze
     tighter by closure_mm → retry (up to --slip-retries times)

Steps:
  1. Wait for camera + TCP pose
  2. SAM3 detection + Gemini DeliGrasp descriptor (parallel)
  3. Safety check
  4. Open → approach → PCA rotate → descend → adaptive close → lift → return → release

Usage:
    export GEMINI_API_KEY=your_key
    source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
    python3 scripts/test_fullrun_slip.py --object "red block" --socket --grasp-offset 0.01
"""

import argparse
import os
import re
import sys
import time
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger

from google import genai
from google.genai import types
from magpie_control import poses
from magpie_control.homog_utils import homog_xform, R_krot
from magpie_control.gripper_arc import fingertip_drop as _gripper_drop
from magpie_msgs.msg import GripperState
from magpie_msgs.srv import MoveLinear, SetGripperForce, SetGripperPosition

GRIPPER_LENGTH = 0.231   # m  wrist flange → fingertip (open gripper)
HARD_FLOOR_Z   = 0.030   # m  physical table surface

# Slip threshold: if measured force is below this fraction of set force → slip
_SLIP_FORCE_RATIO = 0.4

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


def _straight_down_rotation(grasp_angle_deg=0.0):
    theta = np.radians(grasp_angle_deg)
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, s, 0], [s, -c, 0], [0, 0, -1]])


def _axisangle_to_quat(rv):
    angle = np.linalg.norm(rv)
    if angle < 1e-10:
        return (1.0, 0.0, 0.0, 0.0)
    axis = rv / angle
    s = np.sin(angle / 2.0)
    return (np.cos(angle / 2.0), axis[0] * s, axis[1] * s, axis[2] * s)


def _matrix_to_pose_msg(matrix):
    vec = poses.pose_mtrx_to_vec(np.array(matrix))
    w, x, y, z = _axisangle_to_quat(np.array(vec[3:]))
    msg = Pose()
    msg.position.x, msg.position.y, msg.position.z = vec[0], vec[1], vec[2]
    msg.orientation.w, msg.orientation.x = w, x
    msg.orientation.y, msg.orientation.z = y, z
    return msg


class FullRunSlipNode(Node):
    def __init__(self):
        super().__init__('fullrun_slip')
        self.bridge = CvBridge()
        self.color_image  = None
        self.depth_image  = None
        self.camera_info  = None
        self.tcp_matrix   = None
        self.gripper_state = None  # latest GripperState msg

        self.create_subscription(Image,        '/camera/gripper_camera/color/image_raw',      self._color_cb,   1)
        self.create_subscription(Image,        '/camera/gripper_camera/depth/image_rect_raw', self._depth_cb,   1)
        self.create_subscription(CameraInfo,   '/camera/gripper_camera/color/camera_info',    self._caminfo_cb, 1)
        self.create_subscription(PoseStamped,  '/arm/tcp_pose',                               self._tcp_cb,     1)
        self.create_subscription(GripperState, '/gripper/state',                              self._gripper_state_cb, 1)

        self.cli_move_l  = self.create_client(MoveLinear,         '/arm/move_l')
        self.cli_teach   = self.create_client(Trigger,             '/arm/teach_mode')
        self.cli_open    = self.create_client(Trigger,             '/gripper/open')
        self.cli_close   = self.create_client(Trigger,             '/gripper/close')
        self.cli_force   = self.create_client(SetGripperForce,     '/gripper/set_force')
        self.cli_set_pos = self.create_client(SetGripperPosition,  '/gripper/set_position')

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
    def _gripper_state_cb(self, msg):
        self.gripper_state = msg

    def wait_for_data(self, timeout=10.0):
        t = time.time()
        while time.time() - t < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(v is not None for v in [self.color_image, self.depth_image,
                                           self.camera_info, self.tcp_matrix]):
                return True
        return False

    def _call(self, client, request, timeout=30.0):
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f'Service not available: {client.srv_name}')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError(f'Service timed out: {client.srv_name}')
        return future.result()

    def move_l(self, matrix, speed=0.10, accel=0.2):
        req = MoveLinear.Request()
        req.target_pose = _matrix_to_pose_msg(matrix)
        req.speed = speed
        req.acceleration = accel
        req.async_mode = False
        resp = self._call(self.cli_move_l, req)
        if not resp.success:
            raise RuntimeError(f'MoveL failed: {resp.message}')

    def open_gripper(self):
        resp = self._call(self.cli_open, Trigger.Request())
        print(f'  Gripper: {resp.message}')

    def close_gripper(self):
        resp = self._call(self.cli_close, Trigger.Request())
        print(f'  Gripper: {resp.message}')

    def set_force(self, force_n):
        req = SetGripperForce.Request()
        req.max_force = float(force_n)
        resp = self._call(self.cli_force, req)
        print(f'  Force limit: {force_n:.3f} N  ({resp.message})')

    def set_position(self, pos_mm):
        req = SetGripperPosition.Request()
        req.position = float(max(0.0, pos_mm))
        resp = self._call(self.cli_set_pos, req)
        print(f'  Set position: {pos_mm:.1f} mm → actual {resp.actual_position:.1f} mm')

    def read_gripper_state(self, settle_sec=3.5):
        """Wait for motor to settle then return a fresh GripperState snapshot."""
        time.sleep(settle_sec)
        # Spin a few times to flush queued state messages and get a fresh one
        for _ in range(8):
            rclpy.spin_once(self, timeout_sec=0.15)
        return self.gripper_state

    def disable_teach_mode(self):
        if not self.cli_teach.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('teach_mode service not available — skipping')
            return
        resp = self._call(self.cli_teach, Trigger.Request())
        msg = resp.message.lower()
        if 'enabled' in msg:
            resp = self._call(self.cli_teach, Trigger.Request())
        print(f'  Teach mode: {resp.message}')


def adaptive_grasp(node, gp, min_force, max_retries=3):
    """
    DeliGrasp-style adaptive close loop.

    Mirrors Gripper.deligrasp() / check_slip() over ROS services + topic:

      - Move fingers to goal_aperture (Gemini descriptor) at CLOSE_FORCE_N
      - Read /gripper/state after settle
      - slip = state.force < initial_force * _SLIP_FORCE_RATIO
      - on slip: close tighter by closure_mm, increase force by add_force, retry

    Key distinction: initial_force is the *holding* force (physics minimum to
    not drop the object). CLOSE_FORCE_N is the *movement* force (minimum for
    the Dynamixel to overcome its own friction — may be much higher for light
    objects). These are decoupled so delicate objects get a gentle hold but
    the gripper can still physically close.

    Returns the final force used.
    """
    if gp is not None:
        initial_force = max(float(gp['initial_force']), min_force)
        add_force     = float(gp['add_force'])
        closure_mm    = float(gp['closure_mm'])
        goal_ap_mm    = float(gp['aperture_mm'])
    else:
        initial_force = max(0.5, min_force)
        add_force     = 0.5
        closure_mm    = 2.0
        goal_ap_mm    = 0.0

    # Motor needs at least ~5 N to overcome mechanism friction and actually move.
    # initial_force (holding force) may be much lower for lightweight objects.
    CLOSE_FORCE_N = max(initial_force, 5.0)

    # Slip threshold: measured force must exceed this after settling
    contact_threshold = max(0.15, initial_force * _SLIP_FORCE_RATIO)

    current_force = CLOSE_FORCE_N

    print(f'  Adaptive grasp: initial_force={initial_force:.3f} N  '
          f'close_force={CLOSE_FORCE_N:.1f} N  add_force={add_force:.3f} N  '
          f'closure={closure_mm:.1f} mm  max_retries={max_retries}')
    print(f'  Contact threshold: {contact_threshold:.3f} N')

    for attempt in range(max_retries + 1):
        print(f'  Attempt {attempt+1}: force={current_force:.3f} N')
        node.set_force(current_force)
        node.close_gripper()
        state = node.read_gripper_state(settle_sec=3.5)

        if state is None:
            print('  [WARN] No gripper state — assuming contact')
            break

        measured = state.force
        aperture = state.position
        print(f'    aperture={aperture:.1f} mm  measured_force={measured:.3f} N  '
              f'threshold={contact_threshold:.3f} N')

        if measured >= contact_threshold:
            print(f'  Contact confirmed — grip secure at {measured:.3f} N')
            break

        if attempt == max_retries:
            print(f'  [WARN] Max retries reached — proceeding with current grip')
            break

        # Slip detected — increase force and close again
        print(f'  Slip detected — increasing force and re-closing')
        current_force = min(16.0, current_force + add_force)

    return current_force


_SAM3_PYTHON = os.path.expanduser('~/sam3_env/bin/python3')
_SAM3_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sam3_infer.py')


def detect_object_sam3_socket(image_rgb, query, sock_path='/tmp/sam3.sock'):
    import base64, socket as _socket, tempfile, json as _json
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
        boxes  = np.array(data['boxes'],  dtype=float)
        scores = np.array(data['scores'], dtype=float)
        labels = np.array([query] * len(scores))
        mask = None
        if data.get('mask_b64') and len(boxes) > 0:
            import base64 as _b64
            raw  = _b64.b64decode(data['mask_b64'])
            h, w = data['mask_shape']
            mask = np.frombuffer(raw, dtype=np.uint8).reshape(h, w).astype(bool)
            best = int(np.argmax(scores))
            print(f'  SAM3 mask: {mask.sum()} pixels  score={scores[best]:.3f}')
        return boxes, labels, scores, mask
    except Exception as e:
        print(f'  [WARN] SAM3 socket failed: {e}')
        return np.zeros((0, 4)), np.array([]), np.array([]), None
    finally:
        os.unlink(tmp_path)


def _depth_at(depth, seg_mask, u, v_px):
    if seg_mask is not None and seg_mask.any():
        valid = depth[seg_mask].astype(float)
    else:
        pad = 5
        roi = depth[max(0, v_px-pad):v_px+pad, max(0, u-pad):u+pad].astype(float)
        valid = roi.ravel()
    valid = valid[valid > 0]
    return float(np.median(valid)) / 1000.0 if len(valid) > 0 else None


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

    NUM = r'([0-9]+(?:\.[0-9]+)?)'

    def g(pat):
        hit = re.search(pat, body, re.IGNORECASE)
        return float(hit.group(1)) if hit else None

    mass_g   = g(rf'approximate mass of {NUM} grams')
    k        = g(rf'spring constant of {NUM} Newtons per meter')
    mu       = g(rf'friction coefficient of {NUM}')
    aperture = g(rf'goal aperture to {NUM} mm')
    closure  = g(rf'close an additional {NUM} mm')

    if any(v is None for v in [mass_g, k, mu, aperture, closure]):
        raise RuntimeError(f'Parse failed — mass={mass_g} k={k} mu={mu} aperture={aperture} closure={closure}')

    initial_force = min(16.0, max(0.15, (mass_g / 1000.0 * 9.81) / max(mu, 1e-6)))
    add_force     = max(0.05, closure * k * 0.0001)

    return dict(mass_g=mass_g, k=k, mu=mu, aperture_mm=aperture,
                closure_mm=closure, initial_force=initial_force, add_force=add_force)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--object', default=None, help='Object to grasp')
    parser.add_argument('--approach-height', type=float, default=0.10, metavar='M')
    parser.add_argument('--grasp-offset',    type=float, default=None,  metavar='M',
                        help='Fingertip descent below object surface in m (default: auto from PCA)')
    parser.add_argument('--socket', nargs='?', const='/tmp/sam3.sock', metavar='PATH',
                        help='SAM3 socket path (default: /tmp/sam3.sock)')
    parser.add_argument('--min-force', type=float, default=0.0, metavar='N',
                        help='Override minimum gripper force in Newtons (0 = use descriptor)')
    parser.add_argument('--slip-retries', type=int, default=3, metavar='N',
                        help='Max adaptive tighten retries on slip detection (default: 3)')
    parser.add_argument('--dry-run', action='store_true',
                        help='Detect and compute poses only — do not move arm or gripper')
    args = parser.parse_args()

    api_key = os.environ.get('GEMINI_API_KEY', '')
    if not api_key:
        print('ERROR: GEMINI_API_KEY not set.')
        sys.exit(1)

    rclpy.init()
    node = FullRunSlipNode()

    print('Waiting for sensor data...')
    if not node.wait_for_data(timeout=10.0):
        print('ERROR: timed out waiting for camera/arm data.')
        sys.exit(1)
    print('  Sensor data received.')

    rclpy.spin_once(node, timeout_sec=0.2)
    color   = node.color_image.copy()
    depth   = node.depth_image.copy()
    caminfo = node.camera_info
    tcp     = node.tcp_matrix.copy()

    if args.object is None:
        print('No --object specified. Asking Gemini to identify the object...')
        _, buf = cv2.imencode('.jpg', cv2.cvtColor(color, cv2.COLOR_RGB2BGR))
        gclient = genai.Client(api_key=api_key)
        resp = gclient.models.generate_content(
            model='gemini-2.5-flash',
            contents=[
                types.Part.from_bytes(data=buf.tobytes(), mime_type='image/jpeg'),
                'What is the main graspable object in this image? '
                'Reply with only the object name, 2-4 words maximum, no punctuation.',
            ],
        )
        raw = resp.text.strip()
        # Gemini 2.5 leaks chain-of-thought — actual answer is last short phrase
        m = re.search(r'[.!?\n]\s*([^\n.!?]{2,40})\s*$', raw)
        args.object = (m.group(1).strip() if m else raw.split('\n')[-1].strip()).lower()
        args.object = ' '.join(args.object.split()[:4])
        print(f'  Detected object: "{args.object}"')

    sock = args.socket if args.socket else '/tmp/sam3.sock'
    print(f'Running SAM3 (socket ~1s) + Gemini descriptor for "{args.object}" in parallel...')

    _pool = ThreadPoolExecutor(max_workers=2)
    det_fut  = _pool.submit(detect_object_sam3_socket, color, args.object, sock)
    desc_fut = _pool.submit(call_descriptor, args.object, api_key)
    _pool.shutdown(wait=False)

    boxes, labels, scores, seg_mask = det_fut.result()
    try:
        gp = desc_fut.result(timeout=30.0)
    except Exception as e:
        print(f'  [WARN] Gemini descriptor failed: {e} — using worst-case aperture (0 mm)')
        gp = None

    if len(boxes) == 0:
        print(f'ERROR: "{args.object}" not found in image.')
        node.destroy_node(); rclpy.shutdown(); sys.exit(1)

    best = int(np.argmax(scores))
    x1, y1, x2, y2 = [int(v) for v in boxes[best]]

    if seg_mask is not None and seg_mask.any():
        ys, xs = np.where(seg_mask)
        u, v_px = int(xs.mean()), int(ys.mean())
    else:
        u, v_px = (x1 + x2) // 2, (y1 + y2) // 2

    print(f'  Best detection: "{labels[best]}"  score={scores[best]:.3f}')
    print(f'  Centroid pixel: ({u}, {v_px})' + (' (mask centroid)' if seg_mask is not None else ''))

    if seg_mask is not None and seg_mask.any():
        valid = depth[seg_mask].astype(float)
        valid = valid[valid > 0]
        print(f'  Using SAM3 mask depth ({len(valid)} pixels)')
    else:
        pad = 5
        roi = depth[max(0, v_px-pad):v_px+pad, max(0, u-pad):u+pad].astype(float)
        valid = roi[roi > 0]
    if len(valid) == 0:
        print('ERROR: no valid depth at detection centroid.')
        node.destroy_node(); rclpy.shutdown(); sys.exit(1)

    depth_m = float(np.median(valid)) / 1000.0
    fx, fy  = caminfo.k[0], caminfo.k[4]
    cx, cy  = caminfo.k[2], caminfo.k[5]
    p_cam   = np.array([(u - cx) * depth_m / fx, (v_px - cy) * depth_m / fy, depth_m])
    T       = tcp @ _TCP_TO_CAM
    p_world = (T @ np.array([*p_cam, 1.0]))[:3]

    print(f'  Object: x={p_world[0]:.3f}  y={p_world[1]:.3f}  z={p_world[2]:.3f} m')

    if gp is not None:
        print(f'\n=== GRIP PARAMETERS ===')
        print(f'  Mass: {gp["mass_g"]:.0f} g   μ={gp["mu"]:.2f}   k={gp["k"]:.0f} N/m')
        print(f'  Goal aperture: {gp["aperture_mm"]:.1f} mm')
        print(f'  Initial force: {gp["initial_force"]:.3f} N')
        print(f'  Add closure:   {gp["closure_mm"]:.1f} mm  Add force: {gp["add_force"]:.3f} N')
        print(f'=======================\n')

    # Auto grasp offset from PCA if not specified manually
    if args.grasp_offset is None:
        if seg_mask is not None and seg_mask.any():
            try:
                from pointcloud_utils import build_segmented_pcd, denoise_pcd, analyse_pcd
                pts_raw, pcd_raw = build_segmented_pcd(
                    seg_mask, depth, caminfo.k, tcp, _TCP_TO_CAM)
                _, pts_clean = denoise_pcd(pcd_raw)
                if len(pts_clean) >= 3:
                    pca_info = analyse_pcd(pts_clean)
                    obj_h = pca_info['extent_m'][2]
                    args.grasp_offset = float(np.clip(obj_h / 4.0, 0.005, 0.015))
                    print(f'  Auto grasp offset: {args.grasp_offset*1000:.0f} mm  '
                          f'(object height {obj_h*1000:.0f} mm from PCA)')
                else:
                    args.grasp_offset = 0.02
                    print(f'  [WARN] Too few PCD points — using default {args.grasp_offset} m')
            except Exception as e:
                args.grasp_offset = 0.02
                print(f'  [WARN] Auto offset failed: {e} — using {args.grasp_offset} m')
        else:
            args.grasp_offset = 0.02
            print(f'  [WARN] No mask for auto offset — using {args.grasp_offset} m')
    else:
        print(f'  Grasp offset: {args.grasp_offset} m (manual)')

    # Build poses
    R = _straight_down_rotation(0.0)
    approach = np.eye(4)
    approach[:3, :3] = R
    approach[:3, 3] = [p_world[0], p_world[1], p_world[2] + args.approach_height + GRIPPER_LENGTH]

    grasp = approach.copy()
    grasp[2, 3] = p_world[2] - args.grasp_offset + GRIPPER_LENGTH

    # Safety check
    approach_fz = p_world[2] + args.approach_height
    grasp_fz    = p_world[2] - args.grasp_offset
    goal_ap_mm  = float(gp['aperture_mm']) if gp else 0.0
    drop        = _gripper_drop(103.6, goal_ap_mm)
    grasp_eff_z = grasp_fz - drop

    print(f'=== SAFETY CHECK ===')
    print(f'  Floor (table): {HARD_FLOOR_Z:.3f} m')
    print(f'  Approach: fingertip {approach_fz:.3f} m  {"✓" if approach_fz >= HARD_FLOOR_Z else "✗ BELOW FLOOR"}')
    print(f'  Grasp:    open {grasp_fz:.3f} m  −  {drop*1000:.0f} mm arc drop'
          f'  →  {grasp_eff_z:.3f} m  {"✓" if grasp_eff_z >= HARD_FLOOR_Z else "✗ WOULD HIT FLOOR"}')
    print(f'====================')
    if approach_fz < HARD_FLOOR_Z or grasp_eff_z < HARD_FLOOR_Z:
        print('ABORTED — would crash. Adjust --grasp-offset or move object higher.')
        node.destroy_node(); rclpy.shutdown(); sys.exit(1)

    print(f'\nApproach TCP z={approach[2,3]:.3f} m')
    print(f'Grasp    TCP z={grasp[2,3]:.3f} m')

    if args.dry_run:
        print('\nDRY RUN — poses look good. Re-run without --dry-run to execute grasp.')
        node.destroy_node(); rclpy.shutdown(); return

    print('\nExecuting full grasp with slip detection. Ctrl-C at any point to abort.')

    print('\nDisabling teach mode...')
    node.disable_teach_mode()

    print('Opening gripper...')
    node.open_gripper()
    time.sleep(0.3)

    print('\nMoving to approach pose...')
    node.move_l(approach, speed=0.15, accel=0.3)
    print(f'  At approach. Fingertip ~{approach_fz:.3f} m above table.')
    time.sleep(0.5)

    # PCA rotation at approach height
    print('Getting PCA rotation from approach height...')
    rclpy.spin_once(node, timeout_sec=0.4)
    pca_color = node.color_image.copy()
    pca_depth = node.depth_image.copy()
    pca_tcp   = node.tcp_matrix.copy()
    _, _, _, pca_mask = detect_object_sam3_socket(pca_color, args.object, sock)

    if pca_mask is not None and pca_mask.any():
        try:
            import sys as _sys
            _sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
            from pointcloud_utils import (build_segmented_pcd, denoise_pcd,
                                          analyse_pcd, grasp_rotation_matrix)
            pts_raw, pcd_raw = build_segmented_pcd(
                pca_mask, pca_depth, caminfo.k, pca_tcp, _TCP_TO_CAM)
            _, pts_clean = denoise_pcd(pcd_raw)
            if len(pts_clean) >= 3:
                result     = analyse_pcd(pts_clean)
                extent     = result['extent_m']
                elongation = extent[0] / max(extent[1], 1e-6)

                angle = result['grasp_angle_deg'] % 180
                if angle > 90:
                    angle -= 180

                print(f'  PCA: extent {extent[0]*100:.1f} × {extent[1]*100:.1f} cm  '
                      f'elongation={elongation:.1f}x  angle={angle:.1f} deg')

                if elongation < 1.5:
                    print('  → nearly square — keeping default rotation')
                else:
                    R_aligned        = grasp_rotation_matrix(angle)
                    approach[:3, :3] = R_aligned
                    grasp[:3, :3]    = R_aligned
                    print(f'  Rotating wrist {angle:.1f} deg...')
                    node.move_l(approach, speed=0.05, accel=0.1)
                    time.sleep(0.3)
            else:
                print('  [WARN] Too few PCD points — keeping default rotation')
        except Exception as e:
            print(f'  [WARN] PCA failed: {e} — keeping default rotation')
    else:
        print('  [WARN] No mask for PCA — keeping default rotation')

    print('Descending to grasp pose...')
    node.move_l(grasp, speed=0.05, accel=0.1)
    print(f'  At grasp. Fingertip ~{grasp_fz:.3f} m.')
    time.sleep(0.5)

    # Adaptive grasp with slip detection
    print('\nAdaptive grasp (slip detection)...')
    final_force = adaptive_grasp(node, gp, args.min_force, max_retries=args.slip_retries)
    print(f'  Grasp complete. Final force: {final_force:.3f} N')

    print('\nLifting to approach height...')
    node.move_l(approach, speed=0.05, accel=0.1)
    time.sleep(1.0)

    print('Returning to starting position...')
    node.move_l(tcp, speed=0.15, accel=0.3)

    print('Releasing object...')
    node.open_gripper()

    print('\nFull grasp with slip detection complete.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
