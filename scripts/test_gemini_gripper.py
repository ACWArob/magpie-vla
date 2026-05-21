#!/usr/bin/env python3
"""
Test: capture one camera frame, call Gemini for DeliGrasp params,
then set gripper force and close.

Hardware required: RealSense camera, MAGPIE gripper (USB + 12V).
No arm needed.

Usage:
    export GEMINI_API_KEY=your_key
    source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
    # Terminal 1:
    sg dialout -c "bash -c 'source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash && ros2 run magpie_control gripper_node'"
    # Terminal 2:
    python3 scripts/test_gemini_gripper.py --task "pick up the block"
"""

import argparse
import os
import re
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from magpie_msgs.srv import SetGripperForce
import PIL.Image
from google import genai
from google.genai import types
from magpie_prompts.prompts.mp_prompt_tc_vision_phys import prompt_thinker


class GeminiGripperTester(Node):
    def __init__(self, task: str, api_key: str):
        super().__init__('gemini_gripper_tester')
        self.task = task
        self.api_key = api_key
        self.bridge = CvBridge()
        self.image = None

        self.sub = self.create_subscription(
            Image, '/camera/gripper_camera/color/image_raw',
            self._img_cb, 1)

        self.cli_set_force = self.create_client(SetGripperForce, '/gripper/set_force')
        self.cli_open      = self.create_client(Trigger, '/gripper/open')
        self.cli_close     = self.create_client(Trigger, '/gripper/close')

        self.get_logger().info('Waiting for camera frame and gripper services...')
        self.cli_set_force.wait_for_service(timeout_sec=10.0)
        self.cli_open.wait_for_service(timeout_sec=10.0)
        self.cli_close.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('Gripper services ready.')

    def _img_cb(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

    def call_gemini(self) -> dict:
        client = genai.Client(api_key=self.api_key)
        pil_img = PIL.Image.fromarray(self.image)
        response = client.models.generate_content(
            model='gemini-2.5-flash',
            contents=[self.task, pil_img],
            config=types.GenerateContentConfig(system_instruction=prompt_thinker),
        )
        text = response.text

        print('\n--- Gemini raw response ---')
        print(text)
        print('---------------------------\n')

        def _extract(pattern, default):
            m = re.search(pattern, text, re.IGNORECASE)
            return float(m.group(1)) if m else default

        return {
            'initial_force':    _extract(r'contact force to\s+([\d.]+)\s*Newtons', 1.5),
            'additional_force': _extract(r'increase the output force by\s+([\d.]+)\s*Newtons', 0.2),
            'spring_constant':  _extract(r'spring constant of\s+([\d.]+)\s*Newtons per meter', 0.0),
        }

    def open_gripper(self):
        req = Trigger.Request()
        future = self.cli_open.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        res = future.result()
        self.get_logger().info(f'open: success={res.success}  {res.message}')
        return res.success

    def set_force(self, force_n: float):
        req = SetGripperForce.Request()
        req.max_force = force_n
        future = self.cli_set_force.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        res = future.result()
        self.get_logger().info(f'set_force({force_n:.2f}N): success={res.success}  {res.message}')
        return res.success

    def close_gripper(self):
        req = Trigger.Request()
        future = self.cli_close.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        res = future.result()
        self.get_logger().info(f'close: success={res.success}  {res.message}')
        return res.success


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--task', default='grasp the object',
                        help='Task description sent to Gemini with the image')
    args = parser.parse_args()

    api_key = os.environ.get('GEMINI_API_KEY', '')
    if not api_key:
        print('ERROR: GEMINI_API_KEY not set.')
        sys.exit(1)

    rclpy.init()
    node = GeminiGripperTester(args.task, api_key)

    # 1. Capture frame
    print('Capturing camera frame...')
    while node.image is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    print('Got frame.')

    # 2. Call Gemini
    print(f'Calling Gemini (task: "{args.task}")...')
    params = node.call_gemini()
    print('=== DeliGrasp Parameters from Gemini ===')
    print(f'  initial_force    : {params["initial_force"]:.2f} N')
    print(f'  additional_force : {params["additional_force"]:.2f} N')
    print(f'  spring_constant  : {params["spring_constant"]:.1f} N/m')
    print('=========================================\n')

    # 3. Open gripper and set force limit
    print('Opening gripper...')
    node.open_gripper()
    node.set_force(params['initial_force'])

    # 4. Countdown then close — gripper auto-stops at force limit on contact
    print('Closing in:')
    for i in [3, 2, 1]:
        print(f'  {i}...')
        time.sleep(1.0)
    print('Closing gripper...')
    node.close_gripper()

    # 5. Hold for 3 seconds, then open
    print('Holding for 3 seconds...')
    time.sleep(3.0)
    print('Opening gripper...')
    node.open_gripper()

    node.destroy_node()
    rclpy.shutdown()
    print('Done.')


if __name__ == '__main__':
    main()
