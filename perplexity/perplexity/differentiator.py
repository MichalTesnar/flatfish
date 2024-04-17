import rclpy
from rclpy.node import Node
from rclpy.time import Time
import numpy as np
from math import isnan
from flatfish_msgs.msg import KerasReadyTrainingData
from nav_msgs.msg import Odometry

ALLOWED_TIME_DIFFERENCE = 0.05
PUBLISHER_PERIOD = 0.01
PUBLISHER_QUEUE_SIZE = 10
SUBSCRIBER_QUEUE_SIZE = 10
CONVERSION_CONSTANT = 1e9
NORMALIZE_THRUSTERS = 65
DERIVATIVE_QUEUE_SIZE = 5


class Differentiator(Node):
    def __init__(self):
        super().__init__('receiver')
        self._have_new_data = False
        self._current_twist = None
        self._current_pose = None
        self._time_stamp = None
        self._prev_data = []

        self.odometry_subscription = self.create_subscription(
            Odometry,
            '/flatfish/odom_simple/odom',
            self.incoming_data_callback,
            SUBSCRIBER_QUEUE_SIZE)

        self._publisher_ = self.create_publisher(
            KerasReadyTrainingData, 'differentiated_data', PUBLISHER_QUEUE_SIZE)
        self.timer = self.create_timer(
            PUBLISHER_PERIOD, self.publisher_callback)

    def publisher_callback(self):
        if not self._have_new_data:
            return
        msg = KerasReadyTrainingData()
        msg.sample = self._current_sample
        msg.target = self._current_target
        msg.header.stamp = self._time_stamp
        self._publisher_.publish(msg)
        self._have_new_data = False
        self.get_logger().info('I published!')

    def check_data_validity(self, data):
        return not (isnan(data.linear.x) or isnan(data.linear.y) or isnan(data.linear.z))

    def _get_accelerations(self, data, time_stamp, validity):
        if not validity:
            self._prev_data = []
            return False, []
        self._prev_data.append((data, time_stamp))
        if len(self._prev_data) <= DERIVATIVE_QUEUE_SIZE:
            return False, []
        self._prev_data = self._prev_data[-(DERIVATIVE_QUEUE_SIZE+1):]
        derivatives = np.empty((0, 6), float)
        for i in range(1, DERIVATIVE_QUEUE_SIZE + 1):
            current_data, current_time = self._prev_data[i]
            previous_data, previous_time = self._prev_data[i-1]
            derivative = (current_data - previous_data) / \
                (current_time - previous_time)
            derivatives = np.vstack((derivatives, derivative))
        return True, np.mean(derivatives, axis=0)

    def incoming_data_callback(self, msg):
        # get data
        time_stamp_secs = Time.from_msg(
            msg.header.stamp).nanoseconds/CONVERSION_CONSTANT
        validity = self.check_data_validity(msg.twist.twist)
        self._time_stamp = msg.header.stamp
        # get transformed velocities
        linear_x = msg.twist.twist.linear.x
        linear_y = msg.twist.twist.linear.y
        linear_z = msg.twist.twist.linear.z
        angular_x = msg.twist.twist.angular.x
        angular_y = msg.twist.twist.angular.y
        angular_z = msg.twist.twist.angular.z
        # get thruster data
        sample = np.array([linear_x, linear_y, angular_z, 0, 0, 0, 0])
        # get accelerations
        valid_data_flag, target = self._get_accelerations(
            np.array([linear_x, linear_y, linear_z, angular_x, angular_y, angular_z]), time_stamp_secs, validity)
        # return if not enough data
        if not valid_data_flag:
            return
        # prepare for publishing
        target = np.array([target[0], target[1], target[5]])
        self._current_sample = sample
        self._current_target = target
        self._have_new_data = True


def main(args=None):
    rclpy.init(args=args)
    differentiator = Differentiator()
    rclpy.spin(differentiator)
    differentiator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
