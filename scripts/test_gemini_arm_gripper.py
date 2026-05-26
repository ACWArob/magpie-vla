#!/usr/bin/env python3
"""
DeliGrasp pipeline test: arm + gripper with Gemini descriptor (text-only).

1. Call Gemini DeliGrasp descriptor with object name → get physics-based grasp params
2. Open gripper
3. Optionally move forward up to MAX_MOVE (default: no movement — position arm first)
4. Close gripper at computed initial_force
5. Retreat back to start pose

No camera needed for Gemini params. Camera is NOT required for this test.

Hardware required: UR5 arm (ur5_node), MAGPIE gripper (gripper_node).

Usage:
    export GEMINI_API_KEY=your_key
    source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
    python3 scripts/test_gemini_arm_gripper.py --object "wooden block"
    python3 scripts/test_gemini_arm_gripper.py --object "water bottle" --forward 0.03
"""

import argparse
import os
import re
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from std_srvs.srv import Trigger
from magpie_msgs.srv import MoveLinear, SetGripperForce
from google import genai
from google.genai import types

MAX_MOVE = 0.05  # hard cap: 5 cm in any direction

# DeliGrasp descriptor prompt — kept in sync with deligrasp_node.py
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


def call_descriptor(object_name: str, api_key: str) -> dict:
    client = genai.Client(api_key=api_key)
    response = client.models.generate_content(
        model='gemini-2.5-flash',
        contents=f'Pick up the {object_name}.',
        config=types.GenerateContentConfig(system_instruction=DG_DESCRIPTOR_PROMPT),
    )
    text = response.text
    print('\n--- Gemini descriptor ---')
    print(text)
    print('-------------------------\n')

    m = re.search(r'\[start of description\](.*?)\[end of description\]', text, re.DOTALL | re.IGNORECASE)
    if not m:
        raise RuntimeError('Descriptor tags not found in response')
    body = m.group(1)

    def g(pattern):
        hit = re.search(pattern, body, re.IGNORECASE)
        return float(hit.group(1)) if hit else None

    mass_g   = g(r'approximate mass of ([0-9.]+) grams')
    k        = g(r'spring constant of ([0-9.]+) Newtons per meter')
    mu       = g(r'friction coefficient of ([0-9.]+)')
    aperture = g(r'goal aperture to ([0-9.]+) mm')
    closure  = g(r'close an additional ([0-9.]+) mm')

    if any(v is None for v in [mass_g, k, mu, aperture, closure]):
        raise RuntimeError(f'Parse failed: mass={mass_g} k={k} mu={mu} aperture={aperture} closure={closure}')

    initial_force = min(16.0, max(0.15, (mass_g / 1000.0 * 9.81) / max(mu, 1e-6)))
    add_force     = max(0.05, closure * k * 0.0001)

    print(f'  mass={mass_g:.0f}g  k={k:.0f}N/m  μ={mu:.2f}  aperture={aperture:.1f}mm')
    print(f'  initial_force={initial_force:.2f}N  add_force={add_force:.3f}N  add_closure={closure:.1f}mm')

    return {
        'initial_force':    initial_force,
        'additional_force': add_force,
        'additional_closure': closure,
        'goal_aperture_mm': aperture,
    }


class ArmGripperNode(Node):
    def __init__(self):
        super().__init__('gemini_arm_gripper_test')
        self.tcp = None
        self.create_subscription(PoseStamped, '/arm/tcp_pose', self._tcp_cb, 1)
        self.cli_move_l = self.create_client(MoveLinear,      '/arm/move_l')
        self.cli_open   = self.create_client(Trigger,         '/gripper/open')
        self.cli_close  = self.create_client(Trigger,         '/gripper/close')
        self.cli_force  = self.create_client(SetGripperForce, '/gripper/set_force')
        for cli in [self.cli_move_l, self.cli_open, self.cli_close, self.cli_force]:
            cli.wait_for_service(timeout_sec=10.0)

    def _tcp_cb(self, msg):
        self.tcp = msg.pose

    def _call(self, cli, req, timeout=10.0):
        f = cli.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=timeout)
        return f.result()

    def get_tcp(self):
        t = time.time()
        while self.tcp is None and time.time() - t < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.tcp

    def open(self):
        r = self._call(self.cli_open, Trigger.Request())
        self.get_logger().info(f'open: {r.message}')

    def close(self):
        r = self._call(self.cli_close, Trigger.Request())
        self.get_logger().info(f'close: {r.message}')

    def set_force(self, force_n):
        req = SetGripperForce.Request()
        req.max_force = force_n
        r = self._call(self.cli_force, req)
        self.get_logger().info(f'set_force({force_n:.2f}N): {r.message}')

    def move_l(self, pose):
        req = MoveLinear.Request()
        req.target_pose = pose
        r = self._call(self.cli_move_l, req, timeout=20.0)
        self.get_logger().info(f'move_l: {r.message}')
        return r.success


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--object',  default='object',
                        help='Object name for Gemini descriptor (e.g. "wooden block")')
    parser.add_argument('--forward', type=float, default=0.0,
                        help='Move forward this many metres before gripping (max 0.05, default 0)')
    args = parser.parse_args()

    forward = min(abs(args.forward), MAX_MOVE)

    api_key = os.environ.get('GEMINI_API_KEY', '')
    if not api_key:
        print('ERROR: GEMINI_API_KEY not set.')
        sys.exit(1)

    # 1. Get grasp params from Gemini (text-only, no camera)
    print(f'Calling Gemini descriptor for "{args.object}"...')
    params = call_descriptor(args.object, api_key)

    # 2. Start ROS, get current arm pose
    rclpy.init()
    node = ArmGripperNode()

    print('Reading arm pose...')
    tcp = node.get_tcp()
    if tcp is None:
        print('ERROR: no TCP pose received — is ur5_node running?')
        sys.exit(1)

    start = Pose()
    start.position.x  = tcp.position.x
    start.position.y  = tcp.position.y
    start.position.z  = tcp.position.z
    start.orientation = tcp.orientation
    print(f'Start TCP: x={start.position.x:.3f} y={start.position.y:.3f} z={start.position.z:.3f}')

    # 3. Open gripper + set force
    node.open()
    node.set_force(params['initial_force'])
    time.sleep(0.3)

    # 4. Move forward if requested (capped at MAX_MOVE)
    if forward > 0:
        fwd = Pose()
        fwd.position.x  = start.position.x
        fwd.position.y  = start.position.y - forward
        fwd.position.z  = start.position.z
        fwd.orientation = start.orientation
        print(f'Moving forward {forward*100:.1f} cm...')
        node.move_l(fwd)
    else:
        print('No forward movement — gripping at current position.')

    # 5. Close gripper
    print('Closing gripper...')
    node.close()
    time.sleep(1.0)

    # 6. Retreat to start
    print('Retreating to start...')
    node.move_l(start)

    node.destroy_node()
    rclpy.shutdown()
    print('Done.')


if __name__ == '__main__':
    main()
