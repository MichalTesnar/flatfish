from uq_model import AIOModel
from tensorflow.keras import models
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from flatfish_msgs.msg import TrainingData, ModelWeights, KerasReadyTrainingData
from shutil import rmtree
import numpy as np
from scipy.spatial.transform import Rotation as R


CONVERSION_CONSTANT = 1e9
NORMALIZE_THRUSTERS = 65
DERIVATIVE_QUEUE_SIZE = 5
PUBLISHER_PERIOD = 0.01
SUBSCRIBER_QUEUE_SIZE = 10
PUBLISHER_QUEUE_SIZE = 10


class InferenceNode(Node):
    def __init__(self):
        super().__init__('inferencer')
        self.have_new_data = False
        self.current_training_data = None
        self.current_sample = None
        self.current_target = None
        self.uncertainty = None
        self.current_path_to_weights = None
        self.have_new_weights = False
        self.model = AIOModel()
        self.prev_data = []  # for numerical derivations

        self.model_weights_subscription = self.create_subscription(
            ModelWeights,
            'model_weights_path',
            self.model_weights_callback,
            SUBSCRIBER_QUEUE_SIZE)

        self.thruster_subscription = self.create_subscription(
            TrainingData,
            'gathered_data',
            self.incoming_data_callback,
            SUBSCRIBER_QUEUE_SIZE)

        self.publisher_ = self.create_publisher(
            KerasReadyTrainingData, 'infered_data', PUBLISHER_QUEUE_SIZE)
        self.timer = self.create_timer(
            PUBLISHER_PERIOD, self.publisher_callback)

        self.loaded_weights_publisher_ = self.create_publisher(
            ModelWeights, 'loaded_weights_path', PUBLISHER_QUEUE_SIZE)
        self.timer = self.create_timer(
            PUBLISHER_PERIOD, self.loaded_weights_publisher_callback)

    def publisher_callback(self):
        if not self.have_new_data:
            return
        msg = KerasReadyTrainingData()
        msg.sample = self.current_sample
        msg.target = self.current_target
        msg.uncertainty = self.uncertainty
        self.publisher_.publish(msg)

        self.current_training_data = None
        self.have_new_data = False
        self.get_logger().info(f'P: New Training Data')

    def loaded_weights_publisher_callback(self):
        if not self.have_new_weights:
            return
        msg = ModelWeights()
        msg.path = self.current_path_to_weights
        self.get_logger().info(f' P: New Evaluation Weights')
        self.loaded_weights_publisher_.publish(msg)
        self.current_path_to_weights = None
        self.have_new_weights = False

    def _get_accelerations(self, data, time_stamp, validity):
        if not validity:
            self.prev_data = []
            return False, []
        self.prev_data.append((data, time_stamp))
        if len(self.prev_data) <= DERIVATIVE_QUEUE_SIZE:
            return False, []
        self.prev_data = self.prev_data[-(DERIVATIVE_QUEUE_SIZE+1):]
        derivatives = np.empty((0, 6), float)
        for i in range(1, DERIVATIVE_QUEUE_SIZE + 1):
            current_data, current_time = self.prev_data[i]
            previous_data, previous_time = self.prev_data[i-1]
            derivative = (current_data - previous_data) / \
                (current_time - previous_time)
            derivatives = np.vstack((derivatives, derivative))
        return True, np.mean(derivatives, axis=0)

    def incoming_data_callback(self, msg):
        # get data
        time_stamp_secs = Time.from_msg(
            msg.header.stamp).nanoseconds/CONVERSION_CONSTANT
        validity = msg.validity
        # get transformed velocities
        linear_x = msg.twist.linear.x
        linear_y = msg.twist.linear.y
        linear_z = msg.twist.linear.z
        angular_x = msg.twist.angular.x
        angular_y = msg.twist.angular.y
        angular_z = msg.twist.angular.z
        # get thruster data
        thruster_surge_left = msg.thrusters.speed_surge_left/NORMALIZE_THRUSTERS
        thruster_surge_right = msg.thrusters.speed_surge_right/NORMALIZE_THRUSTERS
        thruster_sway_front = msg.thrusters.speed_sway_front/NORMALIZE_THRUSTERS
        thruster_sway_rear = msg.thrusters.speed_sway_rear/NORMALIZE_THRUSTERS
        # assemble sample
        sample = np.array([linear_x, linear_y, angular_z, thruster_surge_left,
                          thruster_surge_right, thruster_sway_front, thruster_sway_rear])
        # get accelerations
        valid_data_flag, target = self._get_accelerations(
            np.array([linear_x, linear_y, linear_z, angular_x, angular_y, angular_z]), time_stamp_secs, validity)
        # return if not enough data
        if not valid_data_flag:
            return
        target = np.array([target[0], target[1], target[5]])
        # assess uncertainty
        pred_mean, pred_std = self.model.predict(sample.reshape(1, -1))
        #
        self.uncertainty = float(np.mean(pred_std))
        self.current_sample = sample
        self.current_target = target
        self.have_new_data = True

    def model_weights_callback(self, msg):
        model_path = msg.path
        self.model.model.load_weights(model_path)
        # rmtree(model_path) # this is now done by the evaluator
        self.current_path_to_weights = model_path
        self.have_new_weights = True
        self.get_logger().info(f'S: Weights updated')


def main(args=None):
    rclpy.init(args=args)
    inference_node = InferenceNode()
    rclpy.spin(inference_node)
    inference_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
