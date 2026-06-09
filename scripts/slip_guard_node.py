"""
SlipGuardNode — monitors grip after contact is confirmed, using:

  PRIMARY  — Depth channel:
      Camera is palm-facing. When arm lifts with object held, camera-to-object
      distance stays constant. If object drops even 1mm, depth increases.
      This is more reliable than force (motor current proxy, no fingertip sensors).

  SECONDARY — Force channel:
      Catches fast catastrophic slips where depth hasn't changed yet.

Optical flow is NOT used — during arm motion the background shifts and would
cause constant false positives.

Activation:
  - Called ONLY after grasp contact is confirmed (never during approach/descend)
  - Reference depth captured from first camera frame after enable
  - Object pixel location (u, v) passed from notebook so depth is sampled at
    the correct spot, not just the image centre

Config topic: /slip_guard/config  [force_n, slip_thresh, obj_u, obj_v]
Services:     slip_guard/enable   (Trigger)
              slip_guard/disable  (Trigger)
Events:       /slip_guard/events  (String)
"""

import sys, os, time
import numpy as np

for _p in [
    '/opt/ros/humble/local/lib/python3.10/dist-packages',
    '/home/user/ws_ctrl/install/magpie_control/lib/python3.10/site-packages',
    '/home/user/ws_ctrl/install/magpie_msgs/local/lib/python3.10/dist-packages',
]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image

try:
    from magpie_msgs.msg import GripperState
    from magpie_msgs.srv import SetGripperForce
except ImportError as e:
    raise RuntimeError(f'magpie_msgs not found — source workspace first: {e}')

CAM_DEPTH_TOPIC = '/camera/gripper_camera/camera/depth/image_rect_raw'

SLIP_DEPTH_MM        = 1.0   # depth increase (mm) that triggers a re-clamp
FORCE_SLIP_RATIO     = 0.6   # force below thresh * this ratio → re-clamp (backup)
FORCE_STEP_N         = 1.0   # how much to increase force on each re-clamp
MIN_RECLAMP_INTERVAL = 0.3   # s — minimum gap between re-clamps (avoid thrashing)
DEPTH_SAMPLE_RADIUS  = 25    # px — radius around object pixel to sample depth


class SlipGuardNode(Node):

    def __init__(self):
        super().__init__('slip_guard_node')
        self.cbg = ReentrantCallbackGroup()

        # ── State ──────────────────────────────────────────────────────────────
        self._active       = False
        self._capture_ref  = False   # True right after enable, cleared after first depth frame
        self._target_force = 5.0
        self._slip_thresh  = 0.5
        self._force_step   = FORCE_STEP_N
        self._last_reclamp = 0.0

        # Reference depth (mm) at object pixel location — locked on first frame after enable
        self._ref_depth_mm = None
        self._obj_u        = -1      # object pixel col from notebook
        self._obj_v        = -1      # object pixel row from notebook

        # ── Subscribers ────────────────────────────────────────────────────────
        self.create_subscription(
            Image, CAM_DEPTH_TOPIC, self._depth_cb, 10,
            callback_group=self.cbg)
        self.create_subscription(
            GripperState, 'gripper/state', self._force_cb, 10,
            callback_group=self.cbg)
        self.create_subscription(
            Float32MultiArray, 'slip_guard/config', self._config_cb, 1,
            callback_group=self.cbg)

        # ── Gripper service clients ────────────────────────────────────────────
        self._clr = self.create_client(Trigger,         '/gripper/clear_error')
        self._frc = self.create_client(SetGripperForce, '/gripper/set_force')
        self._cls = self.create_client(Trigger,         '/gripper/close')

        # ── Service servers ────────────────────────────────────────────────────
        self.create_service(Trigger, 'slip_guard/enable',  self._enable_cb,
                            callback_group=self.cbg)
        self.create_service(Trigger, 'slip_guard/disable', self._disable_cb,
                            callback_group=self.cbg)

        # ── Publisher ──────────────────────────────────────────────────────────
        self._pub = self.create_publisher(String, '/slip_guard/events', 10)

        self.get_logger().info('SlipGuardNode ready — depth-primary slip detection')

    # ── Config callback ────────────────────────────────────────────────────────

    def _config_cb(self, msg: Float32MultiArray):
        """Receive [force, thresh, obj_u, obj_v, force_step] from notebook."""
        d = msg.data
        if len(d) >= 2:
            self._target_force = float(d[0])
            self._slip_thresh  = float(d[1])
        if len(d) >= 4:
            self._obj_u = int(d[2]) if d[2] >= 0 else -1
            self._obj_v = int(d[3]) if d[3] >= 0 else -1
        if len(d) >= 5 and d[4] > 0:
            self._force_step = float(d[4])   # per-object reclamp increment
        else:
            self._force_step = FORCE_STEP_N
        self.get_logger().debug(
            f'Config: force={self._target_force:.1f}N thresh={self._slip_thresh:.2f}N '
            f'obj_px=({self._obj_u},{self._obj_v}) step={self._force_step:.2f}N')

    # ── Enable / Disable ───────────────────────────────────────────────────────

    def _enable_cb(self, req, res):
        self._ref_depth_mm = None
        self._capture_ref  = True
        self._active       = True
        self._last_reclamp = 0.0
        msg = (f'ENABLED force={self._target_force:.1f}N thresh={self._slip_thresh:.2f}N '
               f'obj_px=({self._obj_u},{self._obj_v})')
        self._log(msg)
        res.success = True; res.message = msg
        return res

    def _disable_cb(self, req, res):
        self._active      = False
        self._capture_ref = False
        self._ref_depth_mm = None
        self._log('DISABLED')
        res.success = True; res.message = 'slip guard disabled'
        return res

    # ── Depth callback — PRIMARY slip detector ─────────────────────────────────

    def _depth_cb(self, msg: Image):
        if not self._active:
            return

        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        h, w  = depth.shape

        # Determine sample region: use object pixel if provided, else image centre
        if self._obj_u > 0 and self._obj_v > 0:
            cy = min(max(self._obj_v, DEPTH_SAMPLE_RADIUS), h - DEPTH_SAMPLE_RADIUS)
            cx = min(max(self._obj_u, DEPTH_SAMPLE_RADIUS), w - DEPTH_SAMPLE_RADIUS)
        else:
            cy, cx = h // 2, w // 2

        r  = DEPTH_SAMPLE_RADIUS
        roi = depth[cy-r:cy+r, cx-r:cx+r].astype(float)
        valid = roi[roi > 0]

        if valid.size < 15:
            return   # not enough depth pixels (occluded or at edge)

        current_mm = float(np.median(valid))

        # Capture reference on first frame after enable
        if self._capture_ref:
            self._ref_depth_mm = current_mm
            self._capture_ref  = False
            self._log(f'REFERENCE depth={current_mm:.1f}mm at px=({cx},{cy})')
            return

        if self._ref_depth_mm is None:
            return

        delta_mm = current_mm - self._ref_depth_mm
        if delta_mm > SLIP_DEPTH_MM:
            self._reclamp(f'depth_slip +{delta_mm:.1f}mm (ref={self._ref_depth_mm:.1f} now={current_mm:.1f})')
            # Shift reference forward so we don't keep triggering on accumulated slip
            self._ref_depth_mm = current_mm

    # ── Force callback — SECONDARY slip detector ───────────────────────────────

    def _force_cb(self, msg: GripperState):
        if not self._active or self._capture_ref:
            return
        threshold = self._slip_thresh * FORCE_SLIP_RATIO
        if msg.force < threshold:
            self._reclamp(f'force_drop F={msg.force:.3f}N<{threshold:.3f}N')

    # ── Re-clamp ───────────────────────────────────────────────────────────────

    def _reclamp(self, reason: str):
        now = time.monotonic()
        if now - self._last_reclamp < MIN_RECLAMP_INTERVAL:
            return
        self._last_reclamp = now
        self._target_force = min(16.0, self._target_force + self._force_step)
        self._log(f'RECLAMP {reason} → force now {self._target_force:.1f}N')

        # clear_error → set_force → close (async, non-blocking)
        self._clr.call_async(Trigger.Request())
        time.sleep(0.04)
        req = SetGripperForce.Request()
        req.max_force = float(self._target_force)
        self._frc.call_async(req)
        time.sleep(0.04)
        self._cls.call_async(Trigger.Request())

    def _log(self, msg: str):
        self.get_logger().info(f'[SlipGuard] {msg}')
        m = String(); m.data = f'{time.time():.3f} {msg}'
        self._pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    from rclpy.executors import MultiThreadedExecutor
    node = SlipGuardNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
