import rclpy
from rclpy.node import Node
import random
from flatfish_msgs.msg import ThrusterStatus

class ThrusterStatusPublisher(Node):

    def __init__(self):
        super().__init__('thruster_status_publisher')
        self.publisher_surge_left = self.create_publisher(ThrusterStatus, '/flatfish/thruster_surge_left/thruster_status', 10)
        self.publisher_surge_right = self.create_publisher(ThrusterStatus, '/flatfish/thruster_surge_right/thruster_status', 10)
        self.publisher_sway_front = self.create_publisher(ThrusterStatus, '/flatfish/thruster_sway_front/thruster_status', 10)
        self.publisher_sway_rear = self.create_publisher(ThrusterStatus, '/flatfish/thruster_sway_rear/thruster_status', 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = ThrusterStatus()
        msg.speed = random.uniform(-1, 1)
        self.publisher_surge_left.publish(msg)

        msg = ThrusterStatus()
        msg.speed = random.uniform(-1, 1)
        self.publisher_surge_right.publish(msg)

        msg = ThrusterStatus()
        msg.speed = random.uniform(-1, 1)
        self.publisher_sway_front.publish(msg)

        msg = ThrusterStatus()
        msg.speed = random.uniform(-1, 1)
        self.publisher_sway_rear.publish(msg)

        self.get_logger().info(f'Publishing: {self.i}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    thruster_status_publisher = ThrusterStatusPublisher()
    rclpy.spin(thruster_status_publisher)
    thruster_status_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
