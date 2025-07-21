import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
import numpy as np

class PID:
    def __init__(self, K_p=0.0, K_i=0.0, K_d=0.0, dt=0.05):
        self.dt = dt
        self.K_p = K_p
        self.K_i = K_i
        self.K_d = K_d
        self.reset()

    def get_output(self, e):
        if self.prev_e is None:
            self.prev_e = e

        self.curr_e = e
        self.accu_e += e * self.dt
        self.diff_e = (self.curr_e - self.prev_e) / self.dt

        output = self.K_p * e + self.K_i * self.accu_e + self.K_d * self.diff_e
        self.prev_e = self.curr_e

        return output

    def reset(self):
        self.prev_e = None
        self.curr_e = 0.0
        self.accu_e = 0.0
        self.diff_e = 0.0

class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')

        # Parameters
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('wheel_base', 0.115)
        self.r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.s = self.get_parameter('wheel_base').get_parameter_value().double_value

        self.VelocityController = PID(K_p=1.0, K_i=0.0, K_d=0.0, dt=0.05)
        self.OrientationController = PID(K_p=3.0, K_i=0.0, K_d=0.0, dt=0.05)

        # Subscribers
        self.robot_pose = None
        self.target_pose = None

        self.create_subscription(Pose, '/robot_pose', self.robot_pose_callback, 10)
        self.create_subscription(Pose, '/target_pose', self.target_pose_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer
        self.create_timer(0.05, self.control_loop)

    def robot_pose_callback(self, msg):
        self.robot_pose = msg

    def target_pose_callback(self, msg):
        self.target_pose = msg

    def control_loop(self):
        if self.robot_pose is None or self.target_pose is None:
            return

        # Robot position and heading
        pos = np.array([self.robot_pose.position.x, self.robot_pose.position.y])
        yaw = self.get_yaw_from_quaternion(self.robot_pose.orientation)
        front_vector = np.array([np.cos(yaw), np.sin(yaw)])

        # Target position
        target_pos = np.array([self.target_pose.position.x, self.target_pose.position.y])
        path_vector = target_pos - pos
        target_distance = np.linalg.norm(path_vector)

        if target_distance < 0.05:
            self.publish_velocity(0.0, 0.0)
            return

        target_vector = path_vector / target_distance

        # Control errors
        orientation_error = np.cross(front_vector, target_vector)
        position_error = np.dot(front_vector, target_vector) * target_distance

        # PID outputs
        v = self.VelocityController.get_output(position_error)
        omega = self.OrientationController.get_output(orientation_error)

        self.publish_velocity(v, omega)

    def publish_velocity(self, linear, angular):
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.cmd_pub.publish(twist)

    @staticmethod
    def get_yaw_from_quaternion(q):
        import math
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
