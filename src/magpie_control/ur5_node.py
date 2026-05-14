"""
UR5 ROS2 Node - Wraps existing UR5_Interface for ROS control
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

from magpie_msgs.srv import MoveJoint, MoveLinear, GetPose, SetSpeed

from magpie_control.ur5 import UR5_Interface
from magpie_control import poses

# Standard UR5 joint names expected by ROS tooling
_JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]


def _axisangle_to_quat(rv):
    """Convert axis-angle rotation vector to (w, x, y, z) quaternion."""
    angle = np.linalg.norm(rv)
    if angle < 1e-10:
        return (1.0, 0.0, 0.0, 0.0)
    axis = rv / angle
    s = np.sin(angle / 2.0)
    return (np.cos(angle / 2.0), axis[0] * s, axis[1] * s, axis[2] * s)


def _quat_to_axisangle(w, x, y, z):
    """Convert (w, x, y, z) quaternion to axis-angle rotation vector."""
    angle = 2.0 * np.arccos(np.clip(w, -1.0, 1.0))
    s = np.sin(angle / 2.0)
    if s < 1e-10:
        return np.zeros(3)
    return angle * np.array([x, y, z]) / s


def _pose_msg_to_matrix(pose):
    """Convert geometry_msgs/Pose to 4x4 homogeneous matrix."""
    rv = _quat_to_axisangle(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
    )
    vec = [pose.position.x, pose.position.y, pose.position.z,
           rv[0], rv[1], rv[2]]
    return poses.pose_vec_to_mtrx(vec)


def _matrix_to_pose_msg(matrix):
    """Convert 4x4 homogeneous matrix to geometry_msgs/Pose."""
    from geometry_msgs.msg import Pose
    vec = poses.pose_mtrx_to_vec(np.array(matrix))
    w, x, y, z = _axisangle_to_quat(np.array(vec[3:]))
    msg = Pose()
    msg.position.x = vec[0]
    msg.position.y = vec[1]
    msg.position.z = vec[2]
    msg.orientation.w = w
    msg.orientation.x = x
    msg.orientation.y = y
    msg.orientation.z = z
    return msg


class UR5Node(Node):
    """ROS2 node wrapping UR5_Interface for arm control."""

    def __init__(self):
        super().__init__('ur5_node')

        self.declare_parameter('robot_ip', '192.168.0.4')
        self.declare_parameter('publish_rate', 10)
        self.declare_parameter('default_linear_speed', 0.25)
        self.declare_parameter('default_linear_accel', 0.5)
        self.declare_parameter('default_joint_speed', 1.05)
        self.declare_parameter('default_joint_accel', 1.4)

        robot_ip = self.get_parameter('robot_ip').value
        self.lin_speed = self.get_parameter('default_linear_speed').value
        self.lin_accel = self.get_parameter('default_linear_accel').value
        self.rot_speed = self.get_parameter('default_joint_speed').value
        self.rot_accel = self.get_parameter('default_joint_accel').value

        try:
            self.get_logger().info(f'Connecting to UR5 at {robot_ip}...')
            self.ur5 = UR5_Interface(robotIP=robot_ip)
            self.ur5.start()
            self.get_logger().info('UR5 connected successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to UR5: {e}')
            raise

        # Publishers
        self.pub_joints = self.create_publisher(JointState, 'arm/joint_states', 10)
        self.pub_tcp = self.create_publisher(PoseStamped, 'arm/tcp_pose', 10)

        # Services
        self.create_service(MoveJoint,  'arm/move_j',    self.move_j_callback)
        self.create_service(MoveLinear, 'arm/move_l',    self.move_l_callback)
        self.create_service(GetPose,    'arm/get_pose',  self.get_pose_callback)
        self.create_service(SetSpeed,   'arm/set_speed', self.set_speed_callback)
        self.create_service(Trigger,    'arm/move_safe', self.move_safe_callback)
        self.create_service(Trigger,    'arm/stop',      self.stop_callback)

        pub_rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / pub_rate, self.publish_state)

        self.get_logger().info('UR5 Node initialized')

    def publish_state(self):
        """Publish joint states and TCP pose at fixed rate."""
        try:
            now = self.get_clock().now().to_msg()
            q = self.ur5.get_joint_angles()

            js = JointState()
            js.header.stamp = now
            js.name = _JOINT_NAMES
            js.position = q.tolist()
            self.pub_joints.publish(js)

            tcp = PoseStamped()
            tcp.header.stamp = now
            tcp.header.frame_id = 'base'
            tcp.pose = _matrix_to_pose_msg(self.ur5.get_tcp_pose())
            self.pub_tcp.publish(tcp)
        except Exception as e:
            self.get_logger().warning(f'Error publishing arm state: {e}')

    def move_j_callback(self, request, response):
        """Move to joint configuration."""
        try:
            q = list(request.joint_positions)
            speed = request.speed if request.speed > 0.0 else self.rot_speed
            accel = request.acceleration if request.acceleration > 0.0 else self.rot_accel
            self.get_logger().info(f'MoveJ to {[f"{v:.3f}" for v in q]}')
            self.ur5.moveJ(q, rotSpeed=speed, rotAccel=accel,
                           asynch=request.async_mode)
            response.success = True
            response.message = 'MoveJ complete'
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'MoveJ failed: {e}')
        return response

    def move_l_callback(self, request, response):
        """Move end-effector linearly to target pose."""
        try:
            matrix = _pose_msg_to_matrix(request.target_pose)
            speed = request.speed if request.speed > 0.0 else self.lin_speed
            accel = request.acceleration if request.acceleration > 0.0 else self.lin_accel
            self.get_logger().info(
                f'MoveL to xyz=[{request.target_pose.position.x:.3f}, '
                f'{request.target_pose.position.y:.3f}, '
                f'{request.target_pose.position.z:.3f}]'
            )
            self.ur5.moveL(matrix, linSpeed=speed, linAccel=accel,
                           asynch=request.async_mode)
            response.success = True
            response.message = 'MoveL complete'
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'MoveL failed: {e}')
        return response

    def get_pose_callback(self, request, response):
        """Return current TCP pose and joint angles."""
        try:
            response.current_pose = _matrix_to_pose_msg(self.ur5.get_tcp_pose())
            response.joint_positions = self.ur5.get_joint_angles().tolist()
            response.success = True
            response.message = 'OK'
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'GetPose failed: {e}')
        return response

    def set_speed_callback(self, request, response):
        """Update default linear and joint speeds."""
        try:
            if request.speed > 0.0:
                self.lin_speed = request.speed
                self.rot_speed = request.speed
            if request.acceleration > 0.0:
                self.lin_accel = request.acceleration
                self.rot_accel = request.acceleration
            response.success = True
            response.message = (f'Speed set to {self.lin_speed:.2f}, '
                                f'accel to {self.lin_accel:.2f}')
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def move_safe_callback(self, request, response):
        """Move to pre-defined safe joint configuration."""
        try:
            self.get_logger().info('Moving to safe position...')
            self.ur5.move_safe(rotSpeed=self.rot_speed,
                               rotAccel=self.rot_accel, asynch=False)
            response.success = True
            response.message = 'At safe position'
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'MoveSafe failed: {e}')
        return response

    def stop_callback(self, request, response):
        """Stop all arm motion immediately."""
        try:
            self.get_logger().warning('ARM STOP called')
            self.ur5.ctrl.stopL()
            response.success = True
            response.message = 'Arm stopped'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def destroy_node(self):
        self.get_logger().info('Shutting down UR5 Node...')
        try:
            self.ur5.stop()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UR5Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
