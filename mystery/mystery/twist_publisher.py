import rclpy
from rclpy.node import Node
import random
from geometry_msgs.msg import TwistStamped

class TwistPublisher(Node):

    def __init__(self):
        super().__init__('twist_publisher')
        self.publisher_ = self.create_publisher(TwistStamped, 'twist', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = TwistStamped()
        msg.twist.linear.x = random.uniform(-1, 1)
        msg.twist.linear.y = random.uniform(-1, 1)
        msg.twist.linear.z = random.uniform(-1, 1)
        msg.twist.angular.x = random.uniform(-1, 1)
        msg.twist.angular.y = random.uniform(-1, 1)
        msg.twist.angular.z = random.uniform(-1, 1)

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {self.i}')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    twist_publisher = TwistPublisher()
    rclpy.spin(twist_publisher)
    twist_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
