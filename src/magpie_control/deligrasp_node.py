"""
DeliGrasp ROS2 Node - Perception-driven grasping pipeline

Pipeline per grasp:
  1. Detect object in color image (Grounding DINO)
  2. Get 3D position from depth image + camera intrinsics
  3. Transform camera-frame point to world frame using live TCP pose
  4. Move arm to approach pose above the object
  5. Descend to grasp pose
  6. Execute DeliGrasp (force-controlled gripper action)
  7. Retreat to approach height
"""

import json as _json
import os
import subprocess
import tempfile
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseStamped
from std_srvs.srv import Trigger
from cv_bridge import CvBridge

from magpie_msgs.srv import MoveLinear
from magpie_msgs.action import DeliGrasp
from magpie_msgs.msg import DeliGraspParams

from magpie_control.homog_utils import homog_xform, R_krot
from magpie_control import poses

# TCP-to-camera transform — matches _CAMERA_XFORM in ur5.py
_TCP_TO_CAM = homog_xform(
    rotnMatx=R_krot([0.0, 0.0, 1.0], -np.pi / 2.0),
    posnVctr=[0.0, 0.0, 0.120],
)

# Distance from TCP (wrist flange) to fingertips along the tool Z axis.
# The UR5 TCP is NOT configured to the fingertip — it is at the flange.
# All grasp Z targets must add this offset so the fingertips land at the
# intended height, not the wrist.
GRIPPER_LENGTH = 0.231  # metres — wrist flange to fingertip (matches magpie_tooltip[2] in ur5.py)

# Lowest safe fingertip Z — measured 2026-05-27 in teach mode.
# Closed gripper at floor limit: TCP=0.282 m → fingertip=0.051 m. Adjusted to 0.047 m.
# Used for ALL poses (approach, grasp, retreat) so the arm never reaches a height
# where closing the gripper could hit the floor.
MIN_FINGERTIP_Z = 0.047  # metres


def fingertip_world_pos(tcp_matrix):
    """Return fingertip XYZ in world frame for any gripper orientation.
    Works for tilted grasps: fingertip = TCP_pos + tool_Z_axis * GRIPPER_LENGTH.
    """
    tool_z_world = tcp_matrix[:3, 2]  # tool Z column — points toward fingertips
    return tcp_matrix[:3, 3] + tool_z_world * GRIPPER_LENGTH


def _straight_down_rotation(grasp_angle_deg=0.0):
    """3×3 rotation for straight-down approach, wrist rotated by grasp_angle_deg."""
    theta = np.radians(grasp_angle_deg)
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, s, 0], [s, -c, 0], [0, 0, -1]])


def check_pose_safe(tcp_matrix, label='pose'):
    """Raise ValueError if fingertip would go below MIN_FINGERTIP_Z."""
    fingertip = fingertip_world_pos(tcp_matrix)
    if fingertip[2] < MIN_FINGERTIP_Z:
        raise ValueError(
            f'{label}: fingertip Z={fingertip[2]:.3f} m would go below '
            f'floor limit {MIN_FINGERTIP_Z:.3f} m'
        )

# DeliGrasp descriptor prompt — text-only, from deligrasp.github.io/assets/prompts/dg_descriptor.txt
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
1. If you see phrases like {NUM: default_value}, replace the entire phrase with a numerical \
value. If you see {PNUM: default_value}, replace it with a positive, non-zero numerical value.
2. If you see phrases like {CHOICE: [choice1, choice2, ...]}, it means you should replace the \
entire phrase with one of the choices listed. Be sure to replace all of them. If you are not \
sure about the value, just use your best judgement.
3. If you see phrases like [GRASP_DESCRIPTION: default_value], use information from the user \
instruction to provide a description of the grasp or the object to be grasped, including \
mentioned physical characteristics or features.
4. Using information from the user instruction about the object and the grasp description, set \
the initial grasp force either to this default value or an appropriate value.
5. If you deviate from the default force value, explain your reasoning using the optional bullet \
points. It is not common to deviate from the default value.
6. Using knowledge of the object and how compliant it is, estimate the spring constant of the \
object. This can range broadly from 20 N/m for a very soft object to 2000 N/m for a very stiff \
object.
7. Using knowledge of the object and the grasp description, if the grasp slips, first estimate \
an appropriate increase to the aperture closure, and then the gripper output force.
8. The increase in gripper output force the maximum value of (0.05 N, or the product of the \
estimated aperture closure, the spring constant of the object, and a damping constant 0.1: \
(k*additional_closure*0.0001)).
9. Provide the full description of the grasp plan, even if you may only need to change a few \
lines. Always start the description with [start of description] and end it with \
[end of description].
10. Do not add additional descriptions not shown above. Only use the bullet points given in \
the template.
11. Make sure to give the full description. Do not skip points if they are not optional."""


# ── Pose conversion helpers ────────────────────────────────────────────────────

def _axisangle_to_quat(rv):
    angle = np.linalg.norm(rv)
    if angle < 1e-10:
        return (1.0, 0.0, 0.0, 0.0)
    axis = rv / angle
    s = np.sin(angle / 2.0)
    return (np.cos(angle / 2.0), axis[0] * s, axis[1] * s, axis[2] * s)


def _quat_to_axisangle(w, x, y, z):
    angle = 2.0 * np.arccos(np.clip(w, -1.0, 1.0))
    s = np.sin(angle / 2.0)
    if s < 1e-10:
        return np.zeros(3)
    return angle * np.array([x, y, z]) / s


def _pose_msg_to_matrix(pose):
    rv = _quat_to_axisangle(
        pose.orientation.w, pose.orientation.x,
        pose.orientation.y, pose.orientation.z,
    )
    return poses.pose_vec_to_mtrx([
        pose.position.x, pose.position.y, pose.position.z,
        rv[0], rv[1], rv[2],
    ])


def _matrix_to_pose_msg(matrix):
    vec = poses.pose_mtrx_to_vec(np.array(matrix))
    w, x, y, z = _axisangle_to_quat(np.array(vec[3:]))
    msg = Pose()
    msg.position.x, msg.position.y, msg.position.z = vec[0], vec[1], vec[2]
    msg.orientation.w, msg.orientation.x = w, x
    msg.orientation.y, msg.orientation.z = y, z
    return msg


# ── Node ───────────────────────────────────────────────────────────────────────

class DeliGraspNode(Node):

    def __init__(self):
        super().__init__('deligrasp_node')

        # ReentrantCallbackGroup lets the grasp service callback call other
        # services without deadlocking the single ROS executor thread.
        self.cbg = ReentrantCallbackGroup()

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('object_query', 'object')
        self.declare_parameter('detection_confidence', 0.3)
        self.declare_parameter('approach_height', 0.10)   # m above object
        self.declare_parameter('grasp_z_offset', 0.02)    # m, fine descent past approach
        self.declare_parameter('initial_force', 1.5)      # N
        self.declare_parameter('additional_force', 0.2)   # N
        self.declare_parameter('additional_closure', 1.0) # mm
        # 'grounding_dino' or 'owlvit'
        self.declare_parameter('detector_type', 'grounding_dino')
        # LLM: set use_llm=true and llm_instruction to derive object_query via OpenAI
        self.declare_parameter('use_llm', False)
        self.declare_parameter('llm_instruction', '')
        # Gemini: set use_gemini=true to call Gemini with DeliGrasp descriptor prompt (text-only)
        self.declare_parameter('use_gemini', True)

        # ── State ───────────────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.color_image = None   # latest RGB frame (numpy HxWx3)
        self.depth_image = None   # latest depth frame (numpy HxW, mm uint16)
        self.camera_info = None   # sensor_msgs/CameraInfo
        self.tcp_matrix  = None   # latest TCP pose as 4x4 numpy array
        self._sam3_proc      = None   # persistent SAM3 server subprocess (Python 3.12)
        self._sam3_sock_path = None   # Unix socket path if background server is running

        # ── Subscriptions ───────────────────────────────────────────────────
        self.create_subscription(
            Image, '/camera/gripper_camera/color/image_raw', self._color_cb, 10,
            callback_group=self.cbg)
        self.create_subscription(
            Image, '/camera/gripper_camera/depth/image_rect_raw', self._depth_cb, 10,
            callback_group=self.cbg)
        self.create_subscription(
            CameraInfo, '/camera/gripper_camera/color/camera_info', self._caminfo_cb, 10,
            callback_group=self.cbg)
        self.create_subscription(
            PoseStamped, '/arm/tcp_pose', self._tcp_cb, 10,
            callback_group=self.cbg)

        # ── Service clients ──────────────────────────────────────────────────
        self.cli_move_l    = self.create_client(
            MoveLinear, '/arm/move_l', callback_group=self.cbg)
        self.cli_move_safe = self.create_client(
            Trigger, '/arm/move_safe', callback_group=self.cbg)
        self.cli_stop      = self.create_client(
            Trigger, '/arm/stop', callback_group=self.cbg)
        self.cli_open      = self.create_client(
            Trigger, '/gripper/open', callback_group=self.cbg)

        # ── Action client ────────────────────────────────────────────────────
        self.ac_deligrasp = ActionClient(
            self, DeliGrasp, '/gripper/deligrasp', callback_group=self.cbg)

        # ── Service server ───────────────────────────────────────────────────
        self.create_service(
            Trigger, 'grasp/execute', self._grasp_cb, callback_group=self.cbg)

        # ── Perception models ────────────────────────────────────────────────
        self._load_perception()
        self._check_realsense_usb_speed()

        self.get_logger().info('DeliGrasp Node ready')

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _color_cb(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

    def _depth_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def _caminfo_cb(self, msg):
        self.camera_info = msg

    def _tcp_cb(self, msg):
        self.tcp_matrix = _pose_msg_to_matrix(msg.pose)

    # ── Perception ─────────────────────────────────────────────────────────────

    def _check_realsense_usb_speed(self):
        """Warn if the RealSense D405 is connected to a USB <3.0 port (limits FPS to ~10)."""
        import subprocess, re
        try:
            out = subprocess.check_output(
                ['lsusb', '-v', '-d', '8086:0b5b'],
                stderr=subprocess.DEVNULL, text=True)
            match = re.search(r'bcdUSB\s+([\d.]+)', out)
            if match:
                version = float(match.group(1))
                if version < 3.0:
                    self.get_logger().warning(
                        f'RealSense D405 connected at USB {version:.2f} — '
                        'max ~10 FPS. Plug into a blue USB 3.0 port for 30 FPS.')
                else:
                    self.get_logger().info(f'RealSense D405 USB speed: {version:.2f} (OK)')
            else:
                self.get_logger().warning('RealSense D405 not detected via lsusb — is it plugged in?')
        except Exception:
            pass  # lsusb not available or camera not connected — not fatal

    def _load_perception(self):
        detector_type = self.get_parameter('detector_type').value
        self.detector = None
        if detector_type == 'grounding_dino':
            try:
                from magpie_perception.label_dino import LabelDINO
                self.detector = LabelDINO()
                self.detector.init()
                self.get_logger().info('Grounding DINO loaded')
            except ImportError:
                self.get_logger().warning('magpie_perception not installed — detector unavailable')
        elif detector_type == 'owlvit':
            try:
                from magpie_perception.label_owl import LabelOWL
                self.detector = LabelOWL()
                self.get_logger().info('OWL-ViT loaded')
            except ImportError:
                self.get_logger().warning('magpie_perception not installed — detector unavailable')
        elif detector_type == 'gemini':
            self.get_logger().info('Gemini VLM detector selected — will use cloud vision API')
        elif detector_type == 'sam3':
            _sock = '/tmp/sam3.sock'
            if os.path.exists(_sock):
                self._sam3_sock_path = _sock
                self.get_logger().info(f'SAM3 detector: found background server at {_sock} (warm ~1s)')
            else:
                self.get_logger().info('SAM3 detector: no background server found — starting inline (~12s cold load)')
                self._start_sam3_server()
        else:
            self.get_logger().warning(f'Unknown detector_type "{detector_type}" — use grounding_dino, owlvit, gemini, or sam3')

    def _start_sam3_server(self):
        """Launch sam3_infer.py --server and block until it prints {"status": "ready"}."""
        sam3_python = os.path.expanduser('~/sam3_env/bin/python3')
        sam3_script = os.path.expanduser('~/magpie_control/scripts/sam3_infer.py')
        self._sam3_proc = subprocess.Popen(
            [sam3_python, sam3_script, '--server'],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=open('/tmp/sam3_server.log', 'w'),  # avoid stderr pipe deadlock
            text=True, bufsize=1,
        )
        ready_line = self._sam3_proc.stdout.readline()
        try:
            msg = _json.loads(ready_line)
            if msg.get('status') == 'ready':
                self.get_logger().info('SAM3 server ready (model loaded)')
            else:
                self.get_logger().warning(f'Unexpected SAM3 startup message: {ready_line.strip()}')
        except Exception as e:
            self.get_logger().warning(f'SAM3 server startup parse error: {e}')

    def _resolve_query(self):
        """Return the object query string, optionally via LLM."""
        if self.get_parameter('use_llm').value:
            instruction = self.get_parameter('llm_instruction').value
            if not instruction:
                self.get_logger().warning('use_llm=true but llm_instruction is empty — falling back to object_query')
            else:
                try:
                    return self._llm_extract_object(instruction)
                except Exception as e:
                    self.get_logger().error(f'LLM query extraction failed: {e} — falling back to object_query')
        return self.get_parameter('object_query').value

    def _llm_extract_object(self, instruction: str) -> str:
        """Use OpenAI + magpie_prompts to extract object name from a natural language instruction."""
        import re, ast
        from openai import OpenAI
        from magpie_prompts.prompts.dg_command_enumerator import prompt_command_enumerator
        client = OpenAI()  # reads OPENAI_API_KEY from environment
        response = client.chat.completions.create(
            model='gpt-4o-mini',
            messages=[
                {'role': 'system', 'content': prompt_command_enumerator},
                {'role': 'user',   'content': instruction},
            ],
        )
        text = response.choices[0].message.content
        match = re.search(r'\[start of enumeration\](.*?)\[end of enumeration\]', text, re.DOTALL)
        if not match:
            raise RuntimeError(f'LLM did not return expected format: {text}')
        parsed = ast.literal_eval(match.group(1).strip())
        objects = parsed.get('objects', [])
        if not objects:
            raise RuntimeError(f'LLM returned no objects: {parsed}')
        query = objects[0]
        self.get_logger().info(f'LLM resolved "{instruction}" → query="{query}"')
        return query

    def _parse_descriptor(self, text: str):
        """Extract grasp fields from a DeliGrasp descriptor response.

        Returns a dict of raw values, or None if any required field is missing.
        """
        import re
        m = re.search(
            r'\[start of description\](.*?)\[end of description\]',
            text, re.DOTALL | re.IGNORECASE)
        if not m:
            return None
        body = m.group(1)

        def _get(pattern):
            hit = re.search(pattern, body, re.IGNORECASE)
            return float(hit.group(1)) if hit else None

        mass_grams       = _get(r'approximate mass of ([0-9.]+) grams')
        spring_constant  = _get(r'spring constant of ([0-9.]+) Newtons per meter')
        friction_coeff   = _get(r'friction coefficient of ([0-9.]+)')
        goal_aperture_mm = _get(r'goal aperture to ([0-9.]+) mm')
        add_closure      = _get(r'close an additional ([0-9.]+) mm')
        add_force_raw    = _get(r'increase the output force by ([0-9.]+) Newtons')

        if any(v is None for v in [
                mass_grams, spring_constant, friction_coeff,
                goal_aperture_mm, add_closure, add_force_raw]):
            return None

        return {
            'mass_grams':       mass_grams,
            'spring_constant':  spring_constant,
            'friction_coeff':   friction_coeff,
            'goal_aperture_mm': goal_aperture_mm,
            'additional_closure': add_closure,
        }

    def _estimate_params_from_descriptor(self, object_name: str):
        """Call Gemini text-only with the DeliGrasp descriptor prompt.

        Returns dict with keys: initial_force (N), additional_force (N),
        additional_closure (mm), goal_aperture_mm (mm). Returns None on failure.
        """
        import os
        from google import genai
        from google.genai import types

        api_key = os.environ.get('GEMINI_API_KEY', '')
        if not api_key:
            raise RuntimeError('GEMINI_API_KEY not set in environment')

        client = genai.Client(api_key=api_key)
        user_message = f'Pick up the {object_name}.'
        response = client.models.generate_content(
            model='gemini-2.5-flash',
            contents=user_message,
            config=types.GenerateContentConfig(system_instruction=DG_DESCRIPTOR_PROMPT),
        )
        text = response.text
        self.get_logger().info(f'Gemini descriptor for "{object_name}":\n{text}')

        parsed = self._parse_descriptor(text)
        if parsed is None:
            self.get_logger().error('Descriptor parse failed — response missing required fields')
            return None

        # Physics from the DeliGrasp paper
        mass_kg       = parsed['mass_grams'] / 1000.0
        friction      = max(parsed['friction_coeff'], 1e-6)
        initial_force = min(16.0, max(0.15, (mass_kg * 9.81) / friction))
        add_closure   = parsed['additional_closure']
        k             = parsed['spring_constant']
        add_force     = max(0.05, add_closure * k * 0.0001)

        self.get_logger().info(
            f'Descriptor — object="{object_name}"  '
            f'mass={parsed["mass_grams"]:.0f}g  '
            f'k={k:.0f}N/m  '
            f'μ={parsed["friction_coeff"]:.2f}  '
            f'aperture={parsed["goal_aperture_mm"]:.1f}mm  '
            f'add_closure={add_closure:.1f}mm  '
            f'initial_force={initial_force:.2f}N  '
            f'add_force={add_force:.3f}N')

        return {
            'initial_force':    initial_force,
            'additional_force': add_force,
            'additional_closure': add_closure,
            'goal_aperture_mm': parsed['goal_aperture_mm'],
        }

    def _detect(self, query, confidence):
        """Run detector. Returns (boxes, labels, scores, mask) — mask is H×W bool or None."""
        detector_type = self.get_parameter('detector_type').value
        if detector_type == 'gemini':
            return self._detect_gemini(query)
        if detector_type == 'sam3':
            return self._detect_sam3(query, confidence)
        # grounding_dino (default)
        import torch
        from PIL import Image as PILImage
        from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        processor = AutoProcessor.from_pretrained('IDEA-Research/grounding-dino-tiny')
        model = AutoModelForZeroShotObjectDetection.from_pretrained(
            'IDEA-Research/grounding-dino-tiny').to(device)
        pil_img = PILImage.fromarray(self.color_image)
        inputs = processor(images=pil_img, text=query + '.', return_tensors='pt').to(device)
        with torch.no_grad():
            outputs = model(**inputs)
        h, w = self.color_image.shape[:2]
        results = processor.post_process_grounded_object_detection(
            outputs, input_ids=inputs.input_ids,
            threshold=confidence, text_threshold=confidence,
            target_sizes=[(h, w)],
        )
        r = results[0]
        boxes  = r['boxes'].cpu().numpy()
        scores = r['scores'].cpu().numpy()
        labels = np.array(r['labels'])
        return boxes, labels, scores, None

    def _detect_sam3(self, query, confidence):
        """SAM3 detection — uses background socket server if running, else inline server.
        Falls back to DINO on any error."""
        import base64, cv2

        tmp_path = tempfile.mktemp(suffix='.jpg')
        cv2.imwrite(tmp_path, cv2.cvtColor(self.color_image, cv2.COLOR_RGB2BGR))

        try:
            if self._sam3_sock_path and os.path.exists(self._sam3_sock_path):
                # Background socket server path
                import socket as _socket
                with _socket.socket(_socket.AF_UNIX, _socket.SOCK_STREAM) as s:
                    s.connect(self._sam3_sock_path)
                    s.sendall((_json.dumps({'image': tmp_path, 'query': query}) + '\n').encode())
                    response = b''
                    while True:
                        chunk = s.recv(65536)
                        if not chunk:
                            break
                        response += chunk
                data = _json.loads(response.decode().strip())
            else:
                # Inline stdin server path
                if self._sam3_proc is None or self._sam3_proc.poll() is not None:
                    self.get_logger().warning('SAM3 server not running — restarting')
                    self._start_sam3_server()
                req = _json.dumps({'image': tmp_path, 'query': query})
                self._sam3_proc.stdin.write(req + '\n')
                self._sam3_proc.stdin.flush()
                data = _json.loads(self._sam3_proc.stdout.readline())

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
                self.get_logger().info(
                    f'SAM3 mask: {mask.sum()} pixels  score={scores[int(np.argmax(scores))]:.3f}')

            return boxes, labels, scores, mask

        except Exception as e:
            self.get_logger().warning(f'SAM3 failed: {e} — falling back to DINO')
            return self._detect_grounding_dino(query, confidence)
        finally:
            os.unlink(tmp_path)

    def _detect_grounding_dino(self, query, confidence):
        """Isolated DINO path, callable from _detect_sam3."""
        import torch
        from PIL import Image as PILImage
        from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        processor = AutoProcessor.from_pretrained('IDEA-Research/grounding-dino-tiny')
        model = AutoModelForZeroShotObjectDetection.from_pretrained(
            'IDEA-Research/grounding-dino-tiny').to(device)
        pil_img = PILImage.fromarray(self.color_image)
        inputs = processor(images=pil_img, text=query + '.', return_tensors='pt').to(device)
        with torch.no_grad():
            outputs = model(**inputs)
        h, w = self.color_image.shape[:2]
        results = processor.post_process_grounded_object_detection(
            outputs, input_ids=inputs.input_ids,
            threshold=confidence, text_threshold=confidence,
            target_sizes=[(h, w)],
        )
        r = results[0]
        boxes  = r['boxes'].cpu().numpy()
        scores = r['scores'].cpu().numpy()
        labels = np.array(r['labels'])
        return boxes, labels, scores, None

    def _detect_gemini(self, query):
        """Use Gemini Vision API to detect object. Returns (boxes, labels, scores)."""
        import os, re, cv2
        from google import genai
        from google.genai import types as gtypes

        api_key = os.environ.get('GEMINI_API_KEY', '')
        if not api_key:
            raise RuntimeError('GEMINI_API_KEY not set — required for gemini detector')

        h, w = self.color_image.shape[:2]
        _, buf = cv2.imencode('.jpg', cv2.cvtColor(self.color_image, cv2.COLOR_RGB2BGR))

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
                gtypes.Part.from_bytes(data=buf.tobytes(), mime_type='image/jpeg'),
                prompt,
            ],
        )
        text = response.text.strip()
        self.get_logger().info(f'Gemini VLM: {text}')

        m = re.search(r'\[(\d+),\s*(\d+),\s*(\d+),\s*(\d+)\]', text)
        if not m or 'not found' in text.lower():
            return np.zeros((0, 4)), np.array([]), np.array([]), None

        y1, x1, y2, x2 = [int(m.group(i)) for i in range(1, 5)]
        # Gemini returns 0-1000 normalized; convert to pixels
        x1p = int(x1 * w / 1000)
        y1p = int(y1 * h / 1000)
        x2p = int(x2 * w / 1000)
        y2p = int(y2 * h / 1000)

        boxes  = np.array([[x1p, y1p, x2p, y2p]], dtype=float)
        scores = np.array([0.9])   # Gemini doesn't emit confidence scores
        labels = np.array([query])
        return boxes, labels, scores, None

    # ── Geometry helpers ───────────────────────────────────────────────────────

    def _pixel_to_3d(self, u, v, depth_m):
        """Back-project pixel (u, v) at depth_m to camera-frame 3D point."""
        k = self.camera_info.k          # row-major 3x3 intrinsic matrix
        fx, fy = k[0], k[4]
        cx, cy = k[2], k[5]
        return np.array([
            (u - cx) * depth_m / fx,
            (v - cy) * depth_m / fy,
            depth_m,
        ])

    def _cam_to_world(self, p_cam):
        """Transform a 3D point from camera frame to robot world frame."""
        T = self.tcp_matrix @ _TCP_TO_CAM
        return (T @ np.array([*p_cam, 1.0]))[:3]

    def _analyse_grasp_angle(self, seg_mask):
        """Build point cloud from seg_mask + depth, run PCA, return grasp_angle_deg.
        Returns None on failure (caller falls back to default orientation)."""
        try:
            import sys as _sys
            import os as _os
            _scripts = _os.path.expanduser('~/magpie_control/scripts')
            if _scripts not in _sys.path:
                _sys.path.insert(0, _scripts)
            from pointcloud_utils import build_segmented_pcd, denoise_pcd, analyse_pcd
            k = self.camera_info.k
            pts_raw, pcd_raw = build_segmented_pcd(
                seg_mask, self.depth_image, k, self.tcp_matrix, _TCP_TO_CAM)
            _, pts_clean = denoise_pcd(pcd_raw)
            if len(pts_clean) < 10:
                return None
            result = analyse_pcd(pts_clean)
            angle = result['grasp_angle_deg']
            extent = result['extent_m']
            self.get_logger().info(
                f'PCA: grasp_angle={angle:.1f}° '
                f'extent=[{extent[0]*100:.1f}, {extent[1]*100:.1f}, {extent[2]*100:.1f}] cm')
            return angle
        except Exception as e:
            self.get_logger().warning(f'PCA grasp angle failed: {e} — using default orientation')
            return None

    # ── Service helpers ────────────────────────────────────────────────────────

    def _call(self, client, request, timeout=5.0):
        """Synchronous service call safe to use inside a ReentrantCallbackGroup."""
        if not client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(f'Service not available: {client.srv_name}')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError(f'Service timed out: {client.srv_name}')
        return future.result()

    def _move_l(self, matrix, speed=0.10, accel=0.2):
        req = MoveLinear.Request()
        req.target_pose = _matrix_to_pose_msg(matrix)
        req.speed = speed
        req.acceleration = accel
        req.async_mode = False
        resp = self._call(self.cli_move_l, req, timeout=30.0)
        if not resp.success:
            raise RuntimeError(f'MoveL failed: {resp.message}')

    # ── Main grasp pipeline ────────────────────────────────────────────────────

    def _grasp_cb(self, request, response):
        try:
            self._run_grasp_pipeline()
            response.success = True
            response.message = 'Grasp complete'
        except Exception as e:
            self.get_logger().error(f'Grasp failed: {e}')
            self._safe_abort()
            response.success = False
            response.message = str(e)
        return response

    def _run_grasp_pipeline(self):
        # Guard: all data must be available
        for name, val in [('color image', self.color_image),
                          ('depth image', self.depth_image),
                          ('camera info', self.camera_info),
                          ('arm pose',    self.tcp_matrix)]:
            if val is None:
                raise RuntimeError(f'No {name} received yet — is the pipeline running?')

        query      = self._resolve_query()
        confidence = self.get_parameter('detection_confidence').value
        approach_h = self.get_parameter('approach_height').value
        grasp_off  = self.get_parameter('grasp_z_offset').value

        # ── 1. Detect + Gemini descriptor in parallel ──────────────────────
        # Gemini descriptor only needs the object name (not the image or depth),
        # so it can run concurrently with detection.
        use_gemini = self.get_parameter('use_gemini').value
        self.get_logger().info(f'Detecting "{query}" (+ Gemini descriptor in parallel)')
        _pool = ThreadPoolExecutor(max_workers=2)
        det_fut  = _pool.submit(self._detect, query, confidence)
        desc_fut = _pool.submit(self._estimate_params_from_descriptor, query) if use_gemini else None
        _pool.shutdown(wait=False)  # futures run freely; we .result() below

        boxes, labels, scores, seg_mask = det_fut.result()
        if len(boxes) == 0:
            if desc_fut:
                desc_fut.cancel()
            raise RuntimeError(f'No "{query}" detected in current view')

        best = int(np.argmax(scores))
        x1, y1, x2, y2 = boxes[best]
        self.get_logger().info(
            f'Detected "{labels[best]}" (conf={scores[best]:.2f}) bbox=[{x1},{y1},{x2},{y2}]')

        # ── 2. 3D localisation ─────────────────────────────────────────────
        if seg_mask is not None and seg_mask.any():
            # SAM3: depth from all masked object pixels, centroid from mask
            valid = self.depth_image[seg_mask].astype(float)
            valid = valid[valid > 0]
            ys, xs = np.where(seg_mask)
            u, v = int(xs.mean()), int(ys.mean())
            self.get_logger().info(f'SAM3 mask depth: {len(valid)} pixels, centroid=({u},{v})')
        else:
            u = int((x1 + x2) / 2)
            v = int((y1 + y2) / 2)
            pad = 5
            roi = self.depth_image[
                max(0, v - pad):v + pad,
                max(0, u - pad):u + pad,
            ].astype(float)
            valid = roi[roi > 0]
        if len(valid) == 0:
            raise RuntimeError('Depth image has no valid pixels at detection centroid')
        depth_m = float(np.median(valid)) / 1000.0   # mm → m

        p_cam   = self._pixel_to_3d(u, v, depth_m)
        p_world = self._cam_to_world(p_cam)
        self.get_logger().info(
            f'Object world position: x={p_world[0]:.3f} y={p_world[1]:.3f} z={p_world[2]:.3f} m')

        # ── 3. PCA grasp angle (SAM3 mask → point cloud → wrist rotation) ───
        grasp_angle_deg = 0.0
        if seg_mask is not None and seg_mask.any():
            grasp_angle_deg = self._analyse_grasp_angle(seg_mask) or 0.0

        # ── 4. Open gripper ────────────────────────────────────────────────
        self.get_logger().info('Opening gripper')
        self._call(self.cli_open, Trigger.Request())
        time.sleep(0.3)

        # ── 5. Move to approach pose ───────────────────────────────────────
        # Orientation: straight down, wrist rotated by PCA grasp angle so
        # fingers grip perpendicular to the object's major axis.
        # TCP z = object_z + approach_h + GRIPPER_LENGTH
        approach = np.eye(4)
        approach[:3, :3] = _straight_down_rotation(grasp_angle_deg)
        approach[0, 3] = p_world[0]
        approach[1, 3] = p_world[1]
        approach[2, 3] = p_world[2] + approach_h + GRIPPER_LENGTH

        check_pose_safe(approach, 'approach')
        self.get_logger().info(
            f'Approach TCP z={approach[2,3]:.3f} m  '
            f'(fingertips at z={p_world[2]+approach_h:.3f} m)')
        self._move_l(approach, speed=0.15, accel=0.3)

        # ── 6. Descend to grasp pose ───────────────────────────────────────
        # Grasp: fingertips graze_off below block top surface for a mid-block grip.
        # TCP z = block_z - grasp_off + GRIPPER_LENGTH
        grasp = approach.copy()
        grasp[2, 3] = p_world[2] - grasp_off + GRIPPER_LENGTH

        check_pose_safe(grasp, 'grasp')
        self.get_logger().info('Descending to grasp pose')
        self._move_l(grasp, speed=0.05, accel=0.1)

        # ── 7. DeliGrasp ──────────────────────────────────────────────────
        self.get_logger().info('Executing DeliGrasp')
        params = DeliGraspParams()
        params.goal_aperture    = 30.0
        params.complete_grasp   = True

        if use_gemini and desc_fut is not None:
            try:
                gp = desc_fut.result(timeout=30.0)  # likely already done
                if gp is None:
                    raise RuntimeError('Descriptor parsing returned None')
                params.initial_force      = float(gp['initial_force'])
                params.additional_force   = float(gp['additional_force'])
                params.additional_closure = float(gp['additional_closure'])
                params.goal_aperture      = float(gp['goal_aperture_mm'])
            except Exception as e:
                self.get_logger().error(
                    f'Gemini descriptor failed: {e} — using config defaults')
                params.initial_force      = float(self.get_parameter('initial_force').value)
                params.additional_force   = float(self.get_parameter('additional_force').value)
                params.additional_closure = float(self.get_parameter('additional_closure').value)
        else:
            params.initial_force      = float(self.get_parameter('initial_force').value)
            params.additional_force   = float(self.get_parameter('additional_force').value)
            params.additional_closure = float(self.get_parameter('additional_closure').value)

        if not self.ac_deligrasp.wait_for_server(timeout_sec=3.0):
            raise RuntimeError('DeliGrasp action server not available')

        goal_future = self.ac_deligrasp.send_goal_async(
            DeliGrasp.Goal(params=params))
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=10.0)
        gh = goal_future.result()
        if not gh.accepted:
            raise RuntimeError('DeliGrasp goal rejected by action server')

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        result = result_future.result().result
        if not result.success:
            raise RuntimeError(f'DeliGrasp returned failure: {result.message}')

        self.get_logger().info(
            f'Grasped — aperture={result.final_aperture:.1f}mm '
            f'force={result.final_force:.2f}N')

        # ── 8. Retreat ────────────────────────────────────────────────────
        self.get_logger().info('Retreating')
        retreat = grasp.copy()
        retreat[2, 3] = p_world[2] + approach_h + GRIPPER_LENGTH
        check_pose_safe(retreat, 'retreat')
        self._move_l(retreat, speed=0.10, accel=0.2)

    def _safe_abort(self):
        """Best-effort safety recovery: open gripper and stop arm in place.
        Does NOT call move_safe — joint-space moves to Q_safe can cause
        unexpected large rotations if the arm is far from that configuration.
        """
        try:
            self._call(self.cli_open, Trigger.Request(), timeout=5.0)
        except Exception:
            pass
        try:
            self._call(self.cli_stop, Trigger.Request(), timeout=3.0)
        except Exception:
            pass

    def destroy_node(self):
        self.get_logger().info('Shutting down DeliGrasp Node')
        if self._sam3_proc and self._sam3_proc.poll() is None:
            self._sam3_proc.stdin.close()
            self._sam3_proc.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DeliGraspNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Executor error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
