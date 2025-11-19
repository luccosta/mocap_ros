import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import requests
import json

class CmdVelHttpBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_http_bridge')

        # Declare and get IP parameter
        self.declare_parameter('robot_ip', '192.168.0.136')
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value

        # Subscribe to /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Last received time
        self.last_cmd_time = self.get_clock().now()

        # Watchdog timer: checks every 50ms
        self.watchdog_timer = self.create_timer(0.05, self.watchdog_callback)

        self.get_logger().info(f'Listening to /cmd_vel and forwarding to {self.robot_ip}')

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        self.send_command(linear_x, angular_z)

    def watchdog_callback(self):
        now = self.get_clock().now()
        timeout = rclpy.duration.Duration(seconds=1.0)

        if now - self.last_cmd_time > timeout:
            self.send_command(0.0, 0.0)

    def send_command(self, x: float, z: float):
        command = {"T": 13, "X": x, "Z": z}
        url = f"http://{self.robot_ip}/js?json=" + json.dumps(command)

        try:
            response = requests.get(url, timeout=1.0)
            self.get_logger().debug(f"Sent: {command} | Response: {response.text}")
        except Exception as e:
            self.get_logger().warn(f"Failed to send command: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelHttpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

