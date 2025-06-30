import rclpy
from rclpy.node import Node
import socket
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct

class MocapRosDriver(Node):
    def __init__(self):
        super().__init__('mocap_ros_driver')

        self.udp_ip = '127.0.0.1'
        self.udp_port = 6666

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.setblocking(False)

        self.publisher = self.create_publisher(PointCloud2, 'pointcloud', 10)

        self.timer = self.create_timer(0.05, self.receive_udp)

    def receive_udp(self):
        try:
            data, _ = self.sock.recvfrom(65535) 
            points = np.frombuffer(data, dtype=np.float32).reshape(3, -1).T 
            msg = self.create_pointcloud2(points)
            self.publisher.publish(msg)
        except BlockingIOError:
            pass
        except Exception as e:
            self.get_logger().error(f'Erro ao processar dados UDP: {str(e)}')

    def create_pointcloud2(self, points):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        data = b''.join([struct.pack('fff', *point) for point in points])
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_dense = True
        cloud_msg.data = data

        return cloud_msg

def main(args=None):
    rclpy.init(args=args)
    node = MocapRosDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
