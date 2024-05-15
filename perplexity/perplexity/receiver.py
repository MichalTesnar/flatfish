import rclpy
from rclpy.node import Node
import message_filters
from math import isnan

from flatfish_msgs.msg import TrainingData, KerasReadyTrainingData
from nav_msgs.msg import Odometry

# from flatfish_msgs.msg import ThrusterStatus # use if working locally
# use if working running on flatfish
from thruster_enitech.msg import ThrusterStatus


ALLOWED_TIME_DIFFERENCE = 0.1
PUBLISHER_PERIOD = 0.01
PUBLISHER_QUEUE_SIZE = 100
SUBSCRIBER_QUEUE_SIZE = 100


class Receiver(Node):
    def __init__(self):
        super().__init__('receiver')
        self._have_new_data = False
        self._sample = None
        self._target = None
        self._data_validity = False

        self._thruster_surge_left_subscription = message_filters.Subscriber(
            self, ThrusterStatus, '/flatfish/thruster_surge_left/thruster_status')
        self._thruster_surge_right_subscription = message_filters.Subscriber(
            self, ThrusterStatus, '/flatfish/thruster_surge_right/thruster_status')
        self._thruster_sway_front_subscription = message_filters.Subscriber(
            self, ThrusterStatus, '/flatfish/thruster_sway_front/thruster_status')
        self._thruster_sway_rear_subscription = message_filters.Subscriber(
            self, ThrusterStatus, '/flatfish/thruster_sway_rear/thruster_status')
        self._odometry_subscription = message_filters.Subscriber(
            self, KerasReadyTrainingData, '/differentiated_data')

        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self._thruster_surge_left_subscription,
             self._thruster_surge_right_subscription,
             self._thruster_sway_front_subscription,
             self._thruster_sway_rear_subscription,
             self._odometry_subscription],
            SUBSCRIBER_QUEUE_SIZE, ALLOWED_TIME_DIFFERENCE, allow_headerless=False)

        self._synchronizer.registerCallback(self.synced_callback)

        self._publisher_ = self.create_publisher(
            KerasReadyTrainingData, 'gathered_data', PUBLISHER_QUEUE_SIZE)
        self.timer = self.create_timer(PUBLISHER_PERIOD, self.publisher_callback)

    def publisher_callback(self):
        if not self._have_new_data:
            return
        msg = KerasReadyTrainingData()
        msg.sample = self._sample
        msg.target = self._target
        msg.header.stamp = self.get_clock().now().to_msg()
        self._publisher_.publish(msg)
        self._have_new_data = False
        self.get_logger().info('I published!')

    def synced_callback(self, thruster_surge_left, thruster_surge_right, thruster_sway_front, thruster_sway_rear,  odometry):
        self._sample = odometry.sample
        self._target = odometry.target
        self._sample[3] = thruster_surge_left.speed
        self._sample[4] = thruster_surge_right.speed
        self._sample[5] = thruster_sway_front.speed
        self._sample[6] = thruster_sway_rear.speed
        self._have_new_data = True

def main(args=None):
    rclpy.init(args=args)
    receiver = Receiver()
    rclpy.spin(receiver)
    receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
