#!/usr/bin/env python3
"""
Test: call Gemini DeliGrasp descriptor (text-only) for grasp params,
then set gripper force and close.

No camera needed — Gemini estimates physics from the object name alone.

Hardware required: MAGPIE gripper (12V + USB).

Usage:
    export GEMINI_API_KEY=your_key
    source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
    # Terminal 1:
    sg dialout -c "bash -c 'source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash && ros2 run magpie_control gripper_node'"
    # Terminal 2:
    python3 scripts/test_gemini_gripper.py --object "wooden block"
"""

import argparse
import math
import os
import re
import sys
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from magpie_msgs.srv import SetGripperForce
from google import genai
from google.genai import types

# Paste of DG_DESCRIPTOR_PROMPT — kept in sync with deligrasp_node.py
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
entire phrase with one of the choices listed. Be sure to replace all of them.
3. If you see phrases like [GRASP_DESCRIPTION: default_value], use information from the user \
instruction to provide a description of the grasp or the object to be grasped.
4. Using information from the user instruction, set the initial grasp force to an appropriate value.
5. If you deviate from the default force value, explain your reasoning using the optional bullet.
6. Estimate the spring constant of the object (20 N/m for very soft to 2000 N/m for very stiff).
7. If the grasp slips, estimate appropriate aperture closure increase, then output force increase.
8. The increase in output force = max(0.05, k * additional_closure * 0.0001).
9. Always start with [start of description] and end with [end of description].
10. Do not add additional descriptions. Only use the bullet points given in the template.
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
        raise RuntimeError('Descriptor tags not found in Gemini response')
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
        raise RuntimeError(f'Could not parse all fields: mass={mass_g} k={k} mu={mu} aperture={aperture} closure={closure}')

    initial_force = min(16.0, max(0.15, (mass_g / 1000.0 * 9.81) / max(mu, 1e-6)))
    add_force     = max(0.05, closure * k * 0.0001)

    print(f'  mass         : {mass_g:.0f} g')
    print(f'  spring const : {k:.0f} N/m')
    print(f'  friction μ   : {mu:.2f}')
    print(f'  goal aperture: {aperture:.1f} mm')
    print(f'  add_closure  : {closure:.1f} mm')
    print(f'  initial_force: {initial_force:.2f} N  (computed from mg/μ)')
    print(f'  add_force    : {add_force:.3f} N')

    return {
        'initial_force':    initial_force,
        'additional_force': add_force,
        'additional_closure': closure,
        'goal_aperture_mm': aperture,
    }


class GripperNode(Node):
    def __init__(self):
        super().__init__('gemini_gripper_tester')
        self.cli_force = self.create_client(SetGripperForce, '/gripper/set_force')
        self.cli_open  = self.create_client(Trigger, '/gripper/open')
        self.cli_close = self.create_client(Trigger, '/gripper/close')
        for cli in [self.cli_force, self.cli_open, self.cli_close]:
            cli.wait_for_service(timeout_sec=10.0)

    def open(self):
        f = self.cli_open.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
        self.get_logger().info(f'open: {f.result().message}')

    def close(self):
        f = self.cli_close.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
        self.get_logger().info(f'close: {f.result().message}')

    def set_force(self, force_n: float):
        req = SetGripperForce.Request()
        req.max_force = force_n
        f = self.cli_force.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
        self.get_logger().info(f'set_force({force_n:.2f}N): {f.result().message}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--object', default='object',
                        help='Object name sent to Gemini descriptor (e.g. "wooden block")')
    args = parser.parse_args()

    api_key = os.environ.get('GEMINI_API_KEY', '')
    if not api_key:
        print('ERROR: GEMINI_API_KEY not set.')
        sys.exit(1)

    # 1. Get params from Gemini (no camera needed)
    print(f'Calling Gemini descriptor for "{args.object}"...')
    params = call_descriptor(args.object, api_key)

    # 2. Control gripper
    rclpy.init()
    node = GripperNode()

    node.open()
    node.set_force(params['initial_force'])

    print('Closing in:')
    for i in [3, 2, 1]:
        print(f'  {i}...')
        time.sleep(1.0)
    print('Closing gripper...')
    node.close()

    print('Holding 3 seconds...')
    time.sleep(3.0)

    print('Opening gripper...')
    node.open()

    node.destroy_node()
    rclpy.shutdown()
    print('Done.')


if __name__ == '__main__':
    main()
