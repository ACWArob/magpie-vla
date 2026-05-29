#!/usr/bin/env python3
"""
Half dry-run: detect object, compute poses, move arm to approach then grasp — NO gripper close.

Lets you physically verify the arm lands in the right place before committing to
a real grasp. The gripper stays open throughout. After reaching grasp pose the arm
retreats to the approach height and returns to safe position.

Usage:
    export GEMINI_API_KEY=your_key
    source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
    python3 scripts/test_halfrun.py --object "red block"
    python3 scripts/test_halfrun.py  # auto-identify object with Gemini
"""

import argparse
import os
import sys
import time

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
from magpie_msgs.srv import MoveLinear

GRIPPER_LENGTH = 0.231   # m
HARD_FLOOR_Z   = 0.030   # m  physical table surface

_TCP_TO_CAM = homog_xform(
    rotnMatx=R_krot([0.0, 0.0, 1.0], -np.pi / 2.0),
    posnVctr=[0.0, 0.0, 0.120],
)


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


class HalfRunNode(Node):
    def __init__(self):
        super().__init__('halfrun')
        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.camera_info = None
        self.tcp_matrix  = None

        self.create_subscription(Image,       '/camera/gripper_camera/color/image_raw',      self._color_cb,   1)
        self.create_subscription(Image,       '/camera/gripper_camera/depth/image_rect_raw', self._depth_cb,   1)
        self.create_subscription(CameraInfo,  '/camera/gripper_camera/color/camera_info',    self._caminfo_cb, 1)
        self.create_subscription(PoseStamped, '/arm/tcp_pose',                               self._tcp_cb,     1)

        self.cli_move_l    = self.create_client(MoveLinear, '/arm/move_l')
        self.cli_move_safe = self.create_client(Trigger, '/arm/move_safe')
        self.cli_open      = self.create_client(Trigger, '/gripper/open')
        self.cli_teach     = self.create_client(Trigger, '/arm/teach_mode')

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

    def move_safe(self):
        self._call(self.cli_move_safe, Trigger.Request())

    def open_gripper(self):
        self._call(self.cli_open, Trigger.Request())

    def disable_teach_mode(self):
        if not self.cli_teach.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('teach_mode service not available — skipping')
            return
        resp = self._call(self.cli_teach, Trigger.Request())
        msg = resp.message.lower()
        if 'enabled' in msg:
            # We just turned it ON — call again to turn it back OFF
            resp = self._call(self.cli_teach, Trigger.Request())
        print(f'  Teach mode: {resp.message}')


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
            raw  = base64.b64decode(data['mask_b64'])
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


def detect_object_gemini(image_rgb, query, api_key):
    import re
    _, buf = cv2.imencode('.jpg', cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR))
    h, w = image_rgb.shape[:2]
    client = genai.Client(api_key=api_key)
    response = client.models.generate_content(
        model='gemini-2.5-flash',
        contents=[
            types.Part.from_bytes(data=buf.tobytes(), mime_type='image/jpeg'),
            f'Find the {query} in this image. '
            'Return ONLY the bounding box as [y_min, x_min, y_max, x_max] '
            'where each value is an integer 0-1000 scaled to image dimensions. '
            'If not visible, return: not found',
        ],
    )
    text = response.text.strip()
    m = re.search(r'\[(\d+),\s*(\d+),\s*(\d+),\s*(\d+)\]', text)
    if not m or 'not found' in text.lower():
        return np.zeros((0, 4)), np.array([]), np.array([])
    y1, x1, y2, x2 = [int(m.group(i)) for i in range(1, 5)]
    boxes  = np.array([[int(x1*w/1000), int(y1*h/1000), int(x2*w/1000), int(y2*h/1000)]], dtype=float)
    scores = np.array([0.9])
    labels = np.array([query])
    return boxes, labels, scores


def identify_object(image_rgb, api_key):
    import re as _re
    _, buf = cv2.imencode('.jpg', cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR))
    client = genai.Client(api_key=api_key)
    response = client.models.generate_content(
        model='gemini-2.5-flash',
        contents=[
            types.Part.from_bytes(data=buf.tobytes(), mime_type='image/jpeg'),
            'What is the main graspable object in this image? '
            'Reply with only the object name, 2-4 words maximum, no punctuation.',
        ],
    )
    raw = response.text.strip()
    # Gemini 2.5 leaks chain-of-thought — actual answer is always the last short phrase
    m = _re.search(r'[.!?\n]\s*([^\n.!?]{2,40})\s*$', raw)
    name = (m.group(1).strip() if m else raw.split('\n')[-1].strip()).lower()
    return ' '.join(name.split()[:4])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--object', default=None, help='Object to grasp (Gemini identifies if omitted)')
    parser.add_argument('--approach-height', type=float, default=0.10, metavar='M')
    parser.add_argument('--grasp-offset',    type=float, default=0.02,  metavar='M')
    parser.add_argument('--dry-run', action='store_true', help='Detect and compute poses only — do not move arm')
    parser.add_argument('--detector', default='sam3', choices=['sam3', 'gemini'],
                        help='Detection backend (default: sam3)')
    parser.add_argument('--socket', nargs='?', const='/tmp/sam3.sock', metavar='PATH',
                        help='SAM3 socket path (default: /tmp/sam3.sock)')
    args = parser.parse_args()

    api_key = os.environ.get('GEMINI_API_KEY', '')
    if not api_key and args.detector == 'gemini':
        print('ERROR: GEMINI_API_KEY not set.')
        sys.exit(1)

    rclpy.init()
    node = HalfRunNode()

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


    # Auto-identify object
    if args.object is None:
        print('No --object specified. Asking Gemini to identify the object...')
        args.object = identify_object(color, api_key)
        print(f'  Detected object: "{args.object}"')

    # Detect with SAM3 socket (default) or Gemini
    sock = args.socket if args.socket else ('/tmp/sam3.sock' if args.detector == 'sam3' else None)
    if args.detector == 'sam3' and sock:
        print(f'Running SAM3 (socket ~1s) for "{args.object}"...')
        boxes, labels, scores, seg_mask = detect_object_sam3_socket(color, args.object, sock_path=sock)
    elif args.detector == 'sam3':
        print(f'Running SAM3 (cold ~12s) for "{args.object}"...')
        boxes, labels, scores, seg_mask = detect_object_sam3_socket(color, args.object)
    else:
        print(f'Detecting "{args.object}" with Gemini...')
        boxes, labels, scores = detect_object_gemini(color, args.object, api_key)
        seg_mask = None

    if len(boxes) == 0:
        print(f'ERROR: "{args.object}" not found in image.')
        node.destroy_node(); rclpy.shutdown(); sys.exit(1)

    best = int(np.argmax(scores))
    x1, y1, x2, y2 = [int(v) for v in boxes[best]]

    # Use mask centroid if available, otherwise box centre — exactly as test_dryrun_analyze.py
    if seg_mask is not None and seg_mask.any():
        ys, xs = np.where(seg_mask)
        u, v_px = int(xs.mean()), int(ys.mean())
    else:
        u, v_px = (x1 + x2) // 2, (y1 + y2) // 2

    print(f'  Best detection: "{labels[best]}"  score={scores[best]:.3f}')
    print(f'  Centroid pixel: ({u}, {v_px})' + (' (mask centroid)' if seg_mask is not None else ''))

    # Depth — use SAM3 mask pixels when available, else centroid patch
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
    drop        = _gripper_drop(103.6, 0.0)   # worst case (open to full close), no descriptor here
    grasp_eff_z = grasp_fz - drop

    print(f'\n=== SAFETY CHECK ===')
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
        print('\nDRY RUN — poses look good. Re-run without --dry-run to move the arm.')
        node.destroy_node(); rclpy.shutdown(); return

    print('\nGripper open → approach → grasp pose → back to start.')
    print('NO GRIPPER CLOSE. Ctrl-C at any point to abort.')

    # Execute
    print('\nDisabling teach mode...')
    node.disable_teach_mode()

    node.open_gripper()
    time.sleep(0.3)

    print('\nMoving to approach pose...')
    node.move_l(approach, speed=0.15, accel=0.3)
    print(f'  At approach. Fingertip ~{approach_fz:.3f} m above table.')
    time.sleep(1.0)

    print('Descending to grasp pose (gripper stays open)...')
    node.move_l(grasp, speed=0.05, accel=0.1)
    print(f'  At grasp. Fingertip ~{grasp_fz:.3f} m. Inspect position now.')
    time.sleep(2.0)

    print('Retreating to approach height...')
    node.move_l(approach, speed=0.10, accel=0.2)

    print('Returning to starting position...')
    node.move_l(tcp, speed=0.15, accel=0.3)

    print('\nHalf-run complete — positions verified. Run full pipeline when ready.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
