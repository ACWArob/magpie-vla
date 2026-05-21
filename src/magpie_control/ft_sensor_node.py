"""
F/T Sensor ROS2 Node - Wraps OptoForce sensor for ROS
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Trigger

# Import existing FT sensor class
from magpie_control.ft_sensor import OptoForce


class FTSensorNode(Node):
    """ROS2 node wrapping OptoForce F/T sensor"""
    
    def __init__(self):
        super().__init__('ft_sensor_node')
        
        # Declare parameters
        self.declare_parameter('ip_address', '192.168.0.5')
        self.declare_parameter('port', 49152)
        self.declare_parameter('poll_rate', 50)
        
        # Get parameters
        ip = self.get_parameter('ip_address').value
        port = self.get_parameter('port').value
        poll_rate = self.get_parameter('poll_rate').value
        
        # Initialize sensor
        try:
            self.get_logger().info(f'Connecting to OptoForce at {ip}:{port}...')
            self.sensor = OptoForce(ip_address=ip, port=port, poll_rate=poll_rate)
            self.sensor.connect()
            self.get_logger().info('OptoForce sensor connected successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to F/T sensor: {e}')
            raise
        
        # Create publisher
        self.pub_wrench = self.create_publisher(WrenchStamped, 'ft_sensor/wrench', 10)
        
        # Create service for zeroing
        self.srv_zero = self.create_service(Trigger, 'ft_sensor/zero', self.zero_callback)
        
        # Create timer for publishing readings
        pub_rate = 1.0 / poll_rate  # Convert Hz to period in seconds
        self.timer = self.create_timer(pub_rate, self.publish_wrench)
        
        self.get_logger().info(f'F/T Sensor Node initialized at {poll_rate} Hz')
    
    def publish_wrench(self):
        """Publish force-torque reading"""
        try:
            reading = self.sensor.recv_datum()
            if reading and len(reading) == 6:
                msg = WrenchStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'ft_sensor'
                msg.wrench.force.x = reading[0]
                msg.wrench.force.y = reading[1]
                msg.wrench.force.z = reading[2]
                msg.wrench.torque.x = reading[3]
                msg.wrench.torque.y = reading[4]
                msg.wrench.torque.z = reading[5]

                self.pub_wrench.publish(msg)
        except Exception as e:
            self.get_logger().warning(f'Error reading F/T sensor: {e}')
    
    def zero_callback(self, request, response):
        """Service callback to zero/bias the sensor"""
        try:
            self.get_logger().info('Zeroing F/T sensor...')
            self.sensor.send_datagram(self.sensor.cmd.COMMANDS['set_bias_1'])
            response.success = True
            response.message = 'F/T sensor zeroed successfully'
        except Exception as e:
            response.success = False
            response.message = f'Failed to zero sensor: {str(e)}'
            self.get_logger().error(response.message)
        
        return response
    
    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down F/T Sensor Node...')
        try:
            self.sensor.send_datagram(self.sensor.cmd.COMMANDS['stop_data'])
            self.sensor.sock_r.close()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FTSensorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
