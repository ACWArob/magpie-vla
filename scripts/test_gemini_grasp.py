#!/usr/bin/env python3
"""
Standalone test: capture one camera frame and call Gemini for DeliGrasp params.
No arm or gripper needed — just the RealSense camera.

Usage:
    export GEMINI_API_KEY=your_key
    source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash
    python3 scripts/test_gemini_grasp.py --task "pick up the water bottle"
"""

import argparse
import os
import re
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import PIL.Image
from google import genai
from google.genai import types
from magpie_prompts.prompts.mp_prompt_tc_vision_phys import prompt_thinker


class GeminiTester(Node):
    def __init__(self, task: str):
        super().__init__('gemini_tester')
        self.task = task
        self.bridge = CvBridge()
        self.image = None
        self.sub = self.create_subscription(
            Image, '/camera/gripper_camera/color/image_raw',
            self._cb, 1)
        self.get_logger().info('Waiting for camera frame...')

    def _cb(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')


def call_gemini(image_rgb, task: str, api_key: str) -> dict:
    client = genai.Client(api_key=api_key)
    pil_img = PIL.Image.fromarray(image_rgb)
    response = client.models.generate_content(
        model='gemini-2.5-flash',
        contents=[task, pil_img],
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--task', default='grasp the object',
                        help='Task description sent to Gemini with the image')
    args = parser.parse_args()

    api_key = os.environ.get('GEMINI_API_KEY', '')
    if not api_key:
        print('ERROR: GEMINI_API_KEY not set. Run: export GEMINI_API_KEY=your_key')
        sys.exit(1)

    rclpy.init()
    node = GeminiTester(args.task)

    print('Capturing camera frame...')
    while node.image is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    print('Got frame.')

    node.destroy_node()
    rclpy.shutdown()

    print(f'Calling Gemini (task: "{args.task}")...')
    params = call_gemini(node.image, args.task, api_key)

    print('=== DeliGrasp Parameters from Gemini ===')
    print(f'  initial_force    : {params["initial_force"]:.2f} N')
    print(f'  additional_force : {params["additional_force"]:.2f} N')
    print(f'  spring_constant  : {params["spring_constant"]:.1f} N/m')
    print('=========================================')


if __name__ == '__main__':
    main()
