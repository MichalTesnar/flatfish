import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import TwistStamped
from flatfish_msgs.msg import JointCommandStamped
from flatfish_msgs.msg import TrainingData


class Receiver(Node):
    def __init__(self):
        super().__init__('receiver')
        self.have_new_data = False
        self.current_twist = None           # last one captured, waiting to be matched
        self.current_thruster_status = None  # last one captured, waiting to be matched

        self.to_send_twist = None           # paired up data, ready to be sent
        self.to_send_thruster_status = None

        self.thruster_subscription = self.create_subscription(
            JointCommandStamped,
            'thurster_status',
            self.thruster_callback,
            10)
        self.twist_subscription = self.create_subscription(
            TwistStamped,
            'twist',
            self.twist_callback,
            10)

        self.publisher_ = self.create_publisher(
            TrainingData, 'gathered_data', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publisher_callback)

    def publisher_callback(self):
        if not self.have_new_data:
            return
        msg = TrainingData()
        msg.twist = self.to_send_twist

        msg.thruster = self.to_send_thruster_status
        self.publisher_.publish(msg)
        self.to_send_twist = None
        self.to_send_status = None
        self.have_new_data = False

    def thruster_callback(self, msg):
        # update the current variable
        self.current_thruster_status = msg
        # if timestamp matches
        if self.current_twist is not None and abs(Time.from_msg(self.current_twist.header.stamp).nanoseconds - Time.from_msg(msg.header.stamp).nanoseconds) < 1:
            # put data into output variables
            self.to_send_twist = self.current_twist
            self.to_send_thruster_status = self.current_thruster_status
            # clean the temporary variables
            self.current_twist = None
            self.current_thruster_status = None
            # put up a flag for publishing data
            self.have_new_data = True

    def twist_callback(self, msg):
        # update the current variable
        self.current_twist = msg
        # if timestamps match
        if self.current_thruster_status is not None and abs(Time.from_msg(self.current_thruster_status.header.stamp).nanoseconds - Time.from_msg(msg.header.stamp).nanoseconds) < 1:
            # put data into output variables
            self.to_send_twist = self.current_twist
            self.to_send_thruster_status = self.current_thruster_status
            # clean the temporary variables
            self.current_twist = None
            self.current_thruster_status = None
            # put up a flag for publishing data
            self.have_new_data = True


def main(args=None):
    rclpy.init(args=args)
    receiver = Receiver()
    rclpy.spin(receiver)
    receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
