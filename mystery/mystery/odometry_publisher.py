import rclpy
from rclpy.node import Node
import random
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry_publisher')
        self.publisher_ = self.create_publisher(Odometry, 'flatfish/odometry', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Odometry()
        msg.twist.twist.linear.x = random.uniform(-1, 1)
        msg.twist.twist.linear.y = random.uniform(-1, 1)
        msg.twist.twist.linear.z = random.uniform(-1, 1)
        msg.twist.twist.angular.x = random.uniform(-1, 1)
        msg.twist.twist.angular.y = random.uniform(-1, 1)
        msg.twist.twist.angular.z = random.uniform(-1, 1)

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {self.i}')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)
    odometry_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
