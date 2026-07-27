"""Observation-state assembly for V2 recording (docs/V2_AUTOGRASP_PLAN.md D3/A4).

Fixes the verified V1 bug: the 9th state channel `wrist_fz` was recorded as a
literal 0.0 in every V1 episode, even though /ft_sensor/wrench (WrenchStamped)
publishes a real vertical force at ~the wrist. This module builds the state
vector from live readings and wires `wrench.force.z` into that slot.

State layout is kept IDENTICAL to V1 so V1 and V2 datasets share a schema and
the same training code:

    [x, y, z, rx, ry, rz, grip_mm, grip_force, wrist_fz]   (9,)  float32

`wrist_fz` is signed: negative = downward load (object weight during lift,
table contact during the guarded place-down). That sign is what makes the
place-down guard and lift-load checks work, so we keep it raw (no abs()).

Pure functions + a tiny subscriber mixin — no ROS import at module load, so
this is unit-testable off-robot.
"""
from __future__ import annotations

import numpy as np

STATE_NAMES = ['x', 'y', 'z', 'rx', 'ry', 'rz', 'grip_mm', 'grip_force', 'wrist_fz']
STATE_DIM = len(STATE_NAMES)
_WRIST_FZ_IDX = STATE_NAMES.index('wrist_fz')


def build_state(tcp_vec, grip_mm, grip_force, wrist_fz):
    """Assemble the 9-dim observation state as float32.

    tcp_vec    : (6,) [x,y,z,rx,ry,rz] TCP pose (rotvec), robot frame
    grip_mm    : gripper aperture in mm (node.gs.position)
    grip_force : gripper force in N (node.gs.force)
    wrist_fz   : vertical wrist force in N (wrench.force.z) — REAL, not 0.0
    """
    tcp = np.asarray(tcp_vec, dtype=np.float32).reshape(-1)
    if tcp.shape[0] != 6:
        raise ValueError(f'tcp_vec must be length 6, got {tcp.shape[0]}')
    return np.array([*tcp,
                     float(grip_mm), float(grip_force), float(wrist_fz)],
                    dtype=np.float32)


def wrist_fz_from_wrench(wrench_msg, default=0.0):
    """Extract vertical force from a geometry_msgs/WrenchStamped (or None).

    Returns `default` when no reading is available yet, so a late-arriving FT
    node degrades to V1 behavior instead of crashing — but a warning-worthy
    all-zero episode is now detectable (see fz_is_live)."""
    if wrench_msg is None:
        return float(default)
    return float(wrench_msg.wrench.force.z)


def fz_is_live(state_frames, eps=1e-3):
    """Guard against silently re-shipping the V1 all-zero bug.

    Given the stacked (T, 9) states of an episode, return True only if wrist_fz
    actually VARIES — a dead/unsubscribed FT sensor reads a constant (usually
    0), which is exactly the V1 failure. Call this at end_episode and refuse to
    keep episodes whose fz channel is flat if the sensor is meant to be on.
    """
    arr = np.asarray(state_frames, dtype=np.float32)
    if arr.ndim != 2 or arr.shape[1] != STATE_DIM or arr.shape[0] < 3:
        return False
    return float(arr[:, _WRIST_FZ_IDX].std()) > eps


class WrenchCapture:
    """Mixin/helper: subscribe to /ft_sensor/wrench and cache the latest Fz.

    Attach to the collection node. Kept import-light: the caller passes the
    already-imported message type so this file has no hard ROS dependency.

        from geometry_msgs.msg import WrenchStamped
        node.ft = WrenchCapture(node, WrenchStamped)
        ...
        fz = node.ft.fz            # latest vertical force, or 0.0 before first msg
    """

    def __init__(self, node, wrench_msg_type, topic='/ft_sensor/wrench'):
        self._msg = None
        self.node = node
        node.create_subscription(wrench_msg_type, topic, self._cb, 10)

    def _cb(self, msg):
        self._msg = msg

    @property
    def fz(self):
        return wrist_fz_from_wrench(self._msg)

    @property
    def has_reading(self):
        return self._msg is not None
