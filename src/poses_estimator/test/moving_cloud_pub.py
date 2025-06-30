#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct
import math
import time

class MovingCloudPublisher(Node):
    def __init__(self):
        super().__init__('moving_cloud_pub')
        self.pub = self.create_publisher(PointCloud2, '/input_cloud', 10)
        self.t = 0.0

        self.base_points = [
            np.array([
                [0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 2.0, 0.0],
                [1.0, 0.0, 0.0]
            ], dtype=np.float32),
            np.array([
                [0.0 + 3.0, 0.0, 0.0],
                [1.0 + 3.0, 1.0, 0.0],
                [2.0 + 3.0, 0.0, 0.0],
            ], dtype=np.float32),
            np.array([
                [0.0 - 3.0, 0.0, 0.0],
                [1.0 - 3.0, 0.0, 0.0],
                [1.0 - 3.0, 1.0, 0.0],
                [0.0 - 3.0, 1.0, 0.0],
            ], dtype=np.float32),
        ]

        self.publish_cloud()
        time.sleep(30)
        self.timer = self.create_timer(0.1, self.publish_cloud)

    def publish_cloud(self):
        # Calcula transform: rotação Z oscillante e translação circular
        angle = 0.5 * math.sin(self.t)        # rotação ±0.5 rad
        dx = 0.5 * math.cos(self.t)           # translação X
        dy = 0.5 * math.sin(self.t)           # translação Y

        cosA = math.cos(angle)
        sinA = math.sin(angle)
        R = np.array([
            [cosA, -sinA, 0],
            [sinA, cosA, 0],
            [0, 0, 1]
        ], dtype=np.float32)
        
        points_data = b''
        length = 0
        for base_points in self.base_points:
            pts = (base_points @ R.T) + np.array([dx, dy, 0], dtype=np.float32)
            points_data += b''.join([struct.pack('fff', *p) for p in pts])
            length += len(pts)

        # Monta a mensagem PointCloud2
        msg = PointCloud2()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1
        msg.width = length
        msg.is_bigendian = False
        msg.is_dense = True
        msg.point_step = 3 * 4
        msg.row_step = msg.point_step * msg.width
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        msg.data = points_data

        self.pub.publish(msg)
        self.get_logger().info(f'Published moving cloud at t={self.t:.2f}s, angle={angle:.2f}')

        self.t += 0.1

def main():
    rclpy.init()
    node = MovingCloudPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
