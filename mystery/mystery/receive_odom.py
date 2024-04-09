import rclpy
from rclpy.node import Node
from rclpy.time import Time

from flatfish_msgs.msg import TrainingData
# from flatfish_msgs.msg import ThrusterStatus
from flatfish_msgs.msg import ThrusterSpeeds
from thruster_enitech.msg import ThrusterStatus
from nav_msgs.msg import Odometry

import message_filters


class Receiver(Node):
    def __init__(self):
        super().__init__('receiver')
        self._have_new_data = False
        self._current_twist = None           # last one captured, waiting to be matched
        self._current_thruster_surge_left = None  # last one captured, waiting to be matched

        self._thruster_surge_left_subscription = message_filters.Subscriber(self, ThrusterStatus, '/flatfish/thruster_surge_left/thruster_status')
        # @TODO: Add subscription for other thrusters

        self._odometry_subscription = message_filters.Subscriber(self, Odometry, '/flatfish/odom')

        self._synchronizer = message_filters.ApproximateTimeSynchronizer([self._thruster_surge_left_subscription, self._odometry_subscription], 10, 0.2, allow_headerless=True)

        self._synchronizer.registerCallback(self.synced_callback)

        self._publisher_ = self.create_publisher(
            TrainingData, 'gathered_data', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publisher_callback)

    def publisher_callback(self):
        if not self._have_new_data:
            return
        msg = TrainingData()
        msg.twist = self._current_twist
        msg.thrusters.speed_surge_left = self._current_thruster_surge_left.speed
        msg.header.stamp = self.get_clock().now().to_msg()
        self._publisher_.publish(msg)
        self.get_logger().info('I published: "%s"' % msg.twist)
        self._have_new_data = False

    def synced_callback(self, thruster_surge_left, odometry):
        self.get_logger().info('I heard stuff!')
        self._current_thruster_surge_left = thruster_surge_left
        self._current_twist = odometry.twist.twist
        self._have_new_data = True



def main(args=None):
    rclpy.init(args=args)
    receiver = Receiver()
    rclpy.spin(receiver)
    receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
