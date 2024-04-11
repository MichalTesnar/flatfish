import rclpy
from rclpy.node import Node
import message_filters
from math import isnan

from flatfish_msgs.msg import TrainingData
from nav_msgs.msg import Odometry

# from flatfish_msgs.msg import ThrusterStatus # use if working locally
# use if working running on flatfish
from thruster_enitech.msg import ThrusterStatus


ALLOWED_TIME_DIFFERENCE = 0.2
PUBLISHER_PERIOD = 0.1


class Receiver(Node):
    def __init__(self):
        super().__init__('receiver')
        self._have_new_data = False
        self._current_twist = None
        self._current_pose = None
        self._current_thruster_surge_left = None
        self._current_thruster_surge_right = None
        self._current_thruster_sway_front = None
        self._current_thruster_sway_rear = None

        self._thruster_surge_left_subscription = message_filters.Subscriber(
            self, ThrusterStatus, '/flatfish/thruster_surge_left/thruster_status')
        self._thruster_surge_right_subscription = message_filters.Subscriber(
            self, ThrusterStatus, '/flatfish/thruster_surge_right/thruster_status')
        self._thruster_sway_front_subscription = message_filters.Subscriber(
            self, ThrusterStatus, '/flatfish/thruster_sway_front/thruster_status')
        self._thruster_sway_rear_subscription = message_filters.Subscriber(
            self, ThrusterStatus, '/flatfish/thruster_sway_rear/thruster_status')

        self._odometry_subscription = message_filters.Subscriber(
            self, Odometry, '/flatfish/odom_simple/odom')

        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self._thruster_surge_left_subscription,
             self._thruster_surge_right_subscription,
             self._thruster_sway_front_subscription,
             self._thruster_sway_rear_subscription,
             self._odometry_subscription],
            10, ALLOWED_TIME_DIFFERENCE, allow_headerless=True)

        self._synchronizer.registerCallback(self.synced_callback)

        self._publisher_ = self.create_publisher(
            TrainingData, 'gathered_data', 10)
        timer_period = PUBLISHER_PERIOD  # seconds
        self.timer = self.create_timer(timer_period, self.publisher_callback)

    def publisher_callback(self):
        if not self._have_new_data:
            return
        msg = TrainingData()
        msg.twist = self._current_twist
        msg.pose = self._current_pose
        msg.thrusters.speed_surge_left = self._current_thruster_surge_left.speed
        msg.thrusters.speed_surge_right = self._current_thruster_surge_right.speed
        msg.thrusters.speed_sway_front = self._current_thruster_sway_front.speed
        msg.thrusters.speed_sway_rear = self._current_thruster_sway_rear.speed
        msg.header.stamp = self.get_clock().now().to_msg()
        self._publisher_.publish(msg)
        self._have_new_data = False
        self.get_logger().info('I published!')

    def check_data_validity(self, data):
        """ Checks that the incoming data does not contain NaN values
            in the twist part of the message. This can happen due to 
            proximity of DVL sensor to the wall. """
        self.get_logger().info('I am getting invalid data!')
        self.get_logger().info(data.linear.x)
        self.get_logger().info(data.linear.y)
        self.get_logger().info(data.linear.z)
        if isnan(data.linear.x) or isnan(data.linear.y) or isnan(data.linear.z):
            return False
        return True

    def synced_callback(self, thruster_surge_left, thruster_surge_right, thruster_sway_front, thruster_sway_rear,  odometry):
        if self.check_data_validity(odometry.twist.twist):
            return

        self._current_twist = odometry.twist.twist
        self._current_pose = odometry.pose.pose
        self._current_thruster_surge_left = thruster_surge_left
        self._current_thruster_surge_right = thruster_surge_right
        self._current_thruster_sway_front = thruster_sway_front
        self._current_thruster_sway_rear = thruster_sway_rear
        self._have_new_data = True


def main(args=None):
    rclpy.init(args=args)
    receiver = Receiver()
    rclpy.spin(receiver)
    receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
