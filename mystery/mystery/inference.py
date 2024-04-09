from uq_model import AIOModel
from tensorflow.keras import models
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from flatfish_msgs.msg import TrainingData
from flatfish_msgs.msg import ModelWeights
from shutil import rmtree
import numpy as np


CONVERSION_CONSTANT = 1e-9

UNCERTAINTY_THRESHOLD = 0.001

class InferenceNode(Node):
    def __init__(self):
        super().__init__('receiver')
        self.have_new_data = False
        self.current_training_data = None
        self.model = AIOModel(training_set=([], []))

        # for numerical derivations
        self.prev_data = []

        self.model_weights_subscription = self.create_subscription(
            ModelWeights,
            'model_weights_path',
            self.model_weights_callback,
            10)

        self.thruster_subscription = self.create_subscription(
            TrainingData,
            'gathered_data',
            self.incoming_data_callback,
            10)

        self.publisher_ = self.create_publisher(
            TrainingData, 'infered_data', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.publisher_callback)

    def publisher_callback(self):
        if not self.have_new_data:
            return
        msg = self.current_training_data
        self.publisher_.publish(msg)

        self.current_training_data = None
        self.have_new_data = False

    def _get_accelerations(self, data, time_stamp):
        self.prev_data.append((data, time_stamp))
        if len(self.prev_data) <= 5:
            return False, []
        derivatives = np.empty((0, 6), float)
        for i in range(1, 6):
            current_data, current_time = self.prev_data[i]
            previous_data, previous_time = self.prev_data[i-1]
            derivative = (current_data - previous_data)/((current_time - previous_time)/CONVERSION_CONSTANT)
            derivatives = np.vstack((derivatives, derivative))
        return True, np.median(derivatives, axis=0)
        

    def incoming_data_callback(self, msg):
        # A command to move multiple joints with a timestamp
        time_stamp = Time.from_msg(msg.header.stamp).nanoseconds

        linear_x = msg.twist.linear.x
        linear_y = msg.twist.linear.y
        linear_z = msg.twist.linear.z
        angular_x = msg.twist.angular.x
        angular_y = msg.twist.angular.y
        angular_z = msg.twist.angular.z
        thruster_surge_left = msg.thrusters.speed_surge_left
        thruster_surge_right = msg.thrusters.speed_surge_right
        thruster_sway_front = msg.thrusters.speed_sway_front
        thruster_sway_rear = msg.thrusters.speed_sway_rear

        sample = np.array([linear_x, linear_y, linear_z, angular_x, angular_y, angular_z,
                  thruster_surge_left, thruster_surge_right, thruster_sway_front, thruster_sway_rear])
        flag, target = self._get_accelerations(
            np.array([linear_x, linear_y, linear_z, angular_x, angular_y, angular_z]), time_stamp)
        
        if not flag:
            return
        
        pred_mean, pred_std = self.model.predict(sample.reshape(1, -1))
        uncertainty = float(np.mean(pred_std))

        if uncertainty > UNCERTAINTY_THRESHOLD:
            # set flag to publish new data
            self.current_training_data = msg
            self.current_training_data.uncertainty = uncertainty
            self.have_new_data = True

    def model_weights_callback(self, msg):
        model_path = msg.path
        self.model.model.load_weights(model_path)
        rmtree(model_path)
        print("Model weights updated")


def main(args=None):
    rclpy.init(args=args)
    inference_node = InferenceNode()
    rclpy.spin(inference_node)
    inference_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
