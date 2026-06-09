"""
Measure actual publish rates for all MAGPIE sensors over a fixed window.

Run with:
    python3 scripts/measure_poll_rates.py

Requires all nodes running (gripper, ur5, ft_sensor, camera).
"""

import sys
import time
import rclpy
from rclpy.node import Node
from collections import defaultdict

for _p in [
    '/opt/ros/humble/local/lib/python3.10/dist-packages',
    '/home/user/ws_ctrl/install/magpie_control/lib/python3.10/site-packages',
    '/home/user/ws_ctrl/install/magpie_msgs/local/lib/python3.10/dist-packages',
]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import WrenchStamped

try:
    from magpie_msgs.msg import GripperState
    _has_gripper_msg = True
except ImportError:
    _has_gripper_msg = False
    print('[warn] magpie_msgs not found — gripper/state will not be measured')

MEASURE_SECS = 10  # measurement window

SENSORS = [
    # (display_name, topic, msg_type, configured_hz)
    ('Arm joint states', 'arm/joint_states',                           JointState,    500),
    ('F/T sensor',       'ft_sensor/wrench',                           WrenchStamped,  50),
    ('Camera color',     '/camera/gripper_camera/camera/color/image_raw', Image,        10),
]
if _has_gripper_msg:
    SENSORS.insert(2, ('Gripper state', 'gripper/state', GripperState, 10))


class RateMeasurer(Node):
    def __init__(self):
        super().__init__('rate_measurer')
        self._counts   = defaultdict(int)
        self._first_ts = {}
        self._last_ts  = {}
        self._gaps     = defaultdict(list)

        for name, topic, msg_type, _ in SENSORS:
            # Use a closure to capture name per subscription
            def make_cb(n):
                def cb(msg):
                    now = time.monotonic()
                    if n in self._last_ts:
                        self._gaps[n].append(now - self._last_ts[n])
                    else:
                        self._first_ts[n] = now
                    self._last_ts[n] = now
                    self._counts[n] += 1
                return cb
            self.create_subscription(msg_type, topic, make_cb(name), 10)

    def report(self):
        print()
        print(f'{"Sensor":<22} {"Topic":<48} {"Config Hz":>9} {"Actual Hz":>9} {"Jitter ms":>10}')
        print('-' * 104)
        for name, topic, _, cfg_hz in SENSORS:
            count = self._counts[name]
            if count < 2:
                print(f'{name:<22} {topic:<48} {cfg_hz:>9} {"NO DATA":>9} {"—":>10}')
                continue
            elapsed = self._last_ts[name] - self._first_ts[name]
            actual_hz = (count - 1) / elapsed if elapsed > 0 else 0.0
            gaps = self._gaps[name]
            if len(gaps) > 1:
                import statistics
                jitter_ms = statistics.stdev(gaps) * 1000
            else:
                jitter_ms = 0.0
            match = '✓' if abs(actual_hz - cfg_hz) / max(cfg_hz, 1) < 0.10 else '!'
            print(f'{name:<22} {topic:<48} {cfg_hz:>9} {actual_hz:>9.1f} {jitter_ms:>9.1f}ms  {match}')

        print()
        print('Notes:')
        print('  ! = actual rate differs from configured by >10%')
        print('  F/T sensor: node default is 50 Hz — to increase, relaunch with:')
        print('    --ros-args -p poll_rate:=250')
        print('  Gripper: node timer is 10 Hz — actual may be lower if USB/serial is the bottleneck')


def main():
    rclpy.init()
    node = RateMeasurer()

    print(f'Measuring sensor poll rates for {MEASURE_SECS} s ...')
    print('(make sure all nodes are running and arm is active)')

    start = time.monotonic()
    while time.monotonic() - start < MEASURE_SECS:
        rclpy.spin_once(node, timeout_sec=0.005)

    node.report()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
