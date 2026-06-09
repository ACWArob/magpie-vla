"""
VLM Consistency Test — measures how stable Gemini responses are across N calls
for the three query types used in the MAGPIE pipeline.

Usage:
    python3 scripts/vlm_consistency.py <image_path> [--object "red cube"] [--n 20]
    python3 scripts/vlm_consistency.py --capture [--object "red cube"] [--n 20]

image_path : path to a saved RGB image (jpg/png)
--capture  : grab a fresh frame from the ROS camera instead of loading a file
--object   : object name for strategy + DeliGrasp tests (if omitted, auto-detect
             runs first and uses its most common answer)
--n        : number of repetitions per test (default 20)

Outputs a table to stdout and saves results to tests/vlm_consistency_<date>.md
"""

import argparse
import os
import re
import sys
import time
import statistics
from collections import Counter
from datetime import datetime

# ── CLI ────────────────────────────────────────────────────────────────────────

parser = argparse.ArgumentParser()
parser.add_argument('image', nargs='?', default=None, help='Path to test image (jpg/png)')
parser.add_argument('--capture', action='store_true',
                    help='Capture a fresh frame from the ROS camera')
parser.add_argument('--object', default=None, dest='obj',
                    help='Object name for strategy + DeliGrasp tests')
parser.add_argument('--n', type=int, default=20, help='Repetitions per test (default 20)')
parser.add_argument('--major', type=float, default=None,
                    help='PCA major axis in mm from a real scan (for strategy test)')
parser.add_argument('--minor', type=float, default=None,
                    help='PCA minor axis in mm from a real scan (for strategy test)')
args = parser.parse_args()

if args.capture:
    print('Capturing frame from ROS camera ...')
    import rclpy
    from sensor_msgs.msg import Image as RosImage
    import numpy as np, cv2
    _CAM_TOPIC = '/camera/gripper_camera/camera/color/image_raw'
    _frame = None
    def _cb(msg):
        global _frame
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        _frame = arr.copy()
    if not rclpy.ok():
        rclpy.init()
    _node = rclpy.create_node('vlm_capture')
    _node.create_subscription(RosImage, _CAM_TOPIC, _cb, 1)
    for _ in range(60):
        rclpy.spin_once(_node, timeout_sec=0.1)
        if _frame is not None:
            break
    _node.destroy_node()
    if _frame is None:
        sys.exit(f'No frame received from {_CAM_TOPIC} — are the nodes running?')
    _save_path = '/tmp/vlm_capture.jpg'
    cv2.imwrite(_save_path, cv2.cvtColor(_frame, cv2.COLOR_RGB2BGR))
    print(f'Saved capture to {_save_path}  ({_frame.shape[1]}x{_frame.shape[0]})')
    args.image = _save_path
elif args.image is None:
    sys.exit('Provide an image path or use --capture')

if not os.path.exists(args.image):
    sys.exit(f'Image not found: {args.image}')

# ── Gemini setup ───────────────────────────────────────────────────────────────

_env_path = os.path.join(os.path.dirname(__file__), '..', '.env')
with open(_env_path) as _f:
    for _line in _f:
        _line = _line.strip()
        if _line and not _line.startswith('#') and '=' in _line:
            _k, _v = _line.split('=', 1)
            os.environ.setdefault(_k.strip(), _v.strip().strip('"').strip("'"))

api_key = os.environ.get('GEMINI_API_KEY', '')
if not api_key:
    sys.exit('GEMINI_API_KEY not set in .env')

from google import genai
from google.genai import types as gtypes

client = genai.Client(api_key=api_key)
MODEL  = 'gemini-2.5-flash'

with open(args.image, 'rb') as f:
    img_bytes = f.read()
ext = os.path.splitext(args.image)[1].lower()
mime = 'image/png' if ext == '.png' else 'image/jpeg'

def call(contents, system=None, delay=0.5):
    """Single Gemini call; returns text. Retries once on rate-limit."""
    cfg = gtypes.GenerateContentConfig(system_instruction=system) if system else None
    for attempt in range(2):
        try:
            r = client.models.generate_content(model=MODEL, contents=contents, config=cfg)
            time.sleep(delay)
            return r.text.strip()
        except Exception as e:
            if attempt == 0:
                time.sleep(5)
            else:
                return f'ERROR: {e}'

def img_part():
    return gtypes.Part.from_bytes(data=img_bytes, mime_type=mime)

# ── DeliGrasp descriptor prompt (from deligrasp_node.py) ──────────────────────

DG_PROMPT = """Control a robot gripper with force control and contact information. \
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
1. If you see phrases like {NUM: default_value}, replace the entire phrase with a numerical value.
2. If you see phrases like {PNUM: default_value}, replace it with a positive, non-zero numerical value.
3. If you see phrases like {CHOICE: [choice1, choice2, ...]}, replace with one of the choices.
4. If you see phrases like [GRASP_DESCRIPTION: default_value], describe the grasp or object.
5. Set the initial grasp force to an appropriate value.
6. Estimate the spring constant: 20 N/m (very soft) to 2000 N/m (very stiff).
7. If the grasp slips, estimate aperture closure increase then force increase.
8. Force increase = max(0.05 N, k * additional_closure * 0.0001).
9. Provide the full description. Always start with [start of description] and end with [end of description].
10. Do not add additional descriptions. Only use the bullet points given.
11. Give the full description. Do not skip points if they are not optional."""


def parse_dg(text):
    m = re.search(r'\[start of description\](.*?)\[end of description\]',
                  text, re.DOTALL | re.IGNORECASE)
    if not m:
        return None
    body = m.group(1)
    def _get(pattern):
        hit = re.search(pattern, body, re.IGNORECASE)
        return float(hit.group(1)) if hit else None
    return {
        'mass_g':      _get(r'approximate mass of ([0-9.]+) grams'),
        'k':           _get(r'spring constant of ([0-9.]+) Newtons per meter'),
        'mu':          _get(r'friction coefficient of ([0-9.]+)'),
        'aperture_mm': _get(r'goal aperture to ([0-9.]+) mm'),
    }


# ── Test 1: Auto-detect ────────────────────────────────────────────────────────

def test_autodetect(n):
    print(f'\n[1/3] Auto-detect ({n} calls) ...')
    # Exact prompt used in magpie_demo cell c08b / redetect
    prompt = 'What is the main graspable object in this image? Reply 2-4 words only, no punctuation.'
    results = []
    for i in range(n):
        text = call([img_part(), prompt])
        result = ' '.join(text.strip().lower().split()[:4])
        results.append(result)
        print(f'  {i+1:>2}: {results[-1]}')
    return results


# ── Test 2: Grasp strategy ─────────────────────────────────────────────────────

def test_strategy(obj_name, n, major_mm=None, minor_mm=None):
    synthetic = major_mm is None or minor_mm is None
    if synthetic:
        major_mm, minor_mm = 80, 50
        print(f'\n[2/3] Grasp strategy for "{obj_name}" ({n} calls) ...')
        print(f'  [!] No real PCA values — using synthetic major={major_mm}mm minor={minor_mm}mm')
        print(f'      Pass --major and --minor from a real scan for meaningful results.')
    else:
        print(f'\n[2/3] Grasp strategy for "{obj_name}" ({n} calls) ...')
    ratio = major_mm / max(minor_mm, 1)
    prompt = (
        f'Object: "{obj_name}"\n'
        f'Point cloud extents: major={major_mm:.0f}mm, minor={minor_mm:.0f}mm '
        f'(ratio={ratio:.2f})\n\n'
        f'Choose the best gripper strategy — reply with EXACTLY one word:\n'
        f'  symmetric  — shape is round/square, any angle works (cube, ball, cylinder)\n'
        f'  short_side — grip perpendicular to longest dimension (box, book, phone)\n'
        f'  long_side  — grip parallel to longest dimension (pen, banana, screwdriver)\n'
    )
    results = []
    for i in range(n):
        text = call([img_part(), prompt])
        word = text.lower().split()[0] if text else 'parse_error'
        if word not in ('symmetric', 'short_side', 'long_side'):
            word = f'other:{word}'
        results.append(word)
        print(f'  {i+1:>2}: {results[-1]}')
    return results


# ── Test 3: DeliGrasp params ───────────────────────────────────────────────────

def test_deligrasp(obj_name, n):
    print(f'\n[3/3] DeliGrasp params for "{obj_name}" ({n} calls) ...')
    user_msg = f'Pick up the {obj_name}.'
    results = []
    for i in range(n):
        text = call(user_msg, system=DG_PROMPT, delay=1.0)
        parsed = parse_dg(text)
        if parsed:
            results.append(parsed)
            print(f'  {i+1:>2}: mass={parsed["mass_g"]}g  k={parsed["k"]}N/m  '
                  f'mu={parsed["mu"]}  aperture={parsed["aperture_mm"]}mm')
        else:
            print(f'  {i+1:>2}: PARSE FAILED')
    return results


# ── Stats helpers ──────────────────────────────────────────────────────────────

def str_stats(results):
    c = Counter(results)
    mode, mode_count = c.most_common(1)[0]
    pct = mode_count / len(results) * 100
    return mode, pct, dict(c.most_common())

def num_stats(values):
    if not values:
        return None, None, None
    return statistics.mean(values), statistics.stdev(values) if len(values) > 1 else 0.0, values


# ── Main ───────────────────────────────────────────────────────────────────────

N = args.n

detect_results   = test_autodetect(N)
detect_mode, detect_pct, detect_counts = str_stats(detect_results)

obj_name = args.obj or detect_mode
print(f'\nUsing object name: "{obj_name}"')

strategy_results = test_strategy(obj_name, N, major_mm=args.major, minor_mm=args.minor)
strat_mode, strat_pct, strat_counts = str_stats(strategy_results)

dg_results = test_deligrasp(obj_name, N)
dg_ok = [r for r in dg_results if all(v is not None for v in r.values())]

# ── Print summary ──────────────────────────────────────────────────────────────

print('\n' + '='*60)
print('SUMMARY')
print('='*60)

print(f'\nTest 1 — Auto-detect ({N} calls)')
print(f'  Most common: "{detect_mode}" ({detect_pct:.0f}%)')
for name, cnt in detect_counts.items():
    print(f'    {cnt:>2}x  {name}')

print(f'\nTest 2 — Grasp strategy for "{obj_name}" ({N} calls)')
print(f'  Most common: {strat_mode} ({strat_pct:.0f}%)')
for s, cnt in strat_counts.items():
    print(f'    {cnt:>2}x  {s}')

print(f'\nTest 3 — DeliGrasp params for "{obj_name}" ({N} calls, {len(dg_ok)} parsed OK)')
if dg_ok:
    for field in ('mass_g', 'k', 'mu', 'aperture_mm'):
        vals = [r[field] for r in dg_ok]
        mu_v, sd_v, _ = num_stats(vals)
        cv = (sd_v / mu_v * 100) if mu_v else 0
        print(f'  {field:<12}  mean={mu_v:>8.2f}  std={sd_v:>7.2f}  CV={cv:>5.1f}%')

# ── Save to markdown ───────────────────────────────────────────────────────────

out_dir  = os.path.join(os.path.dirname(__file__), '..', 'tests')
out_path = os.path.join(out_dir, f'vlm_consistency_{datetime.now().strftime("%Y-%m-%d_%H%M")}.md')

with open(out_path, 'w') as f:
    f.write(f'# VLM Consistency Test\n\n')
    f.write(f'**Date:** {datetime.now().strftime("%Y-%m-%d %H:%M")}  \n')
    f.write(f'**Image:** `{args.image}`  \n')
    f.write(f'**Model:** {MODEL}  \n')
    f.write(f'**N per test:** {N}  \n\n')

    f.write('## Test 1 — Auto-detect (object name)\n\n')
    f.write(f'Prompt: *"What is this object? Reply with ONLY a 2-4 word description."*\n\n')
    f.write(f'| Response | Count | % |\n|---|---|---|\n')
    for name, cnt in detect_counts.items():
        f.write(f'| {name} | {cnt} | {cnt/N*100:.0f}% |\n')
    f.write(f'\n**Consistency: {detect_pct:.0f}% on most common answer "{detect_mode}"**\n\n')

    f.write('## Test 2 — Grasp strategy\n\n')
    f.write(f'Object: `{obj_name}`  \n')
    f.write(f'| Strategy | Count | % |\n|---|---|---|\n')
    for s, cnt in strat_counts.items():
        f.write(f'| {s} | {cnt} | {cnt/N*100:.0f}% |\n')
    f.write(f'\n**Consistency: {strat_pct:.0f}% on `{strat_mode}`**\n\n')

    f.write('## Test 3 — DeliGrasp params\n\n')
    f.write(f'Object: `{obj_name}`  \nParsed OK: {len(dg_ok)}/{N}  \n\n')
    if dg_ok:
        f.write('| Parameter | Mean | Std Dev | CV (%) |\n|---|---|---|---|\n')
        units = {'mass_g': 'g', 'k': 'N/m', 'mu': '', 'aperture_mm': 'mm'}
        for field in ('mass_g', 'k', 'mu', 'aperture_mm'):
            vals = [r[field] for r in dg_ok]
            mu_v, sd_v, _ = num_stats(vals)
            cv = (sd_v / mu_v * 100) if mu_v else 0
            f.write(f'| {field} ({units[field]}) | {mu_v:.2f} | {sd_v:.2f} | {cv:.1f}% |\n')
        f.write('\n*CV = coefficient of variation; lower = more consistent*\n')

print(f'\nResults saved to {out_path}')
