#!/usr/bin/env python3
"""Minimal test: move arm up/down N cm from current position at 2 cm/s.
Usage: python3 test_move_up.py [delta_cm]   (positive=up, negative=down, default=4)
"""
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger
from magpie_msgs.srv import GetPose, MoveLinear


class MoveUpNode(Node):
    def __init__(self):
        super().__init__('move_up_test')
        self.cli_get  = self.create_client(GetPose,    '/arm/get_pose')
        self.cli_move = self.create_client(MoveLinear, '/arm/move_l')
        self.cli_teach = self.create_client(Trigger,   '/arm/teach_mode')

    def _call(self, client, req, timeout=15.0):
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f'Service unavailable: {client.srv_name}')
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            raise RuntimeError(f'Timed out: {client.srv_name}')
        return fut.result()

    def ensure_teach_off(self):
        resp = self._call(self.cli_teach, Trigger.Request())
        if 'enabled' in resp.message.lower():
            resp = self._call(self.cli_teach, Trigger.Request())
        print(f'  {resp.message}')


def main():
    delta_m = float(sys.argv[1]) / 100.0 if len(sys.argv) > 1 else 0.04
    rclpy.init()
    node = MoveUpNode()

    print('Disabling teach mode...')
    node.ensure_teach_off()

    print('Getting current pose...')
    resp = node._call(node.cli_get, GetPose.Request())
    if not resp.success:
        print(f'ERROR getting pose: {resp.message}')
        return
    p = resp.current_pose
    print(f'  Current TCP: x={p.position.x:.3f}  y={p.position.y:.3f}  z={p.position.z:.3f}')

    target = Pose()
    target.position.x = p.position.x
    target.position.y = p.position.y
    target.position.z = p.position.z + delta_m
    target.orientation = p.orientation

    print(f'  Target  TCP: x={target.position.x:.3f}  y={target.position.y:.3f}  z={target.position.z:.3f}')
    direction = 'up' if delta_m >= 0 else 'down'
    print(f'Moving {direction} {abs(delta_m)*100:.0f} cm at 2 cm/s...')

    req = MoveLinear.Request()
    req.target_pose = target
    req.speed       = 0.02
    req.acceleration = 0.05
    req.async_mode  = False

    resp = node._call(node.cli_move, req)
    print(f'  Result: success={resp.success}  message="{resp.message}"')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
