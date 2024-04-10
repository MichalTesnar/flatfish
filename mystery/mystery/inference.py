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

class InferenceNode(Node):
    def __init__(self):
        super().__init__('receiver')
        self.have_new_data = False
        self.current_training_data = None
        self.current_sample = None
        self.current_target = None
        self.uncertainty = None
        self.model = AIOModel(training_set=(np.empty((0, 10), float), np.empty((0, 6), float)))

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
            KerasReadyTrainingData, 'infered_data', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publisher_callback)

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
        self.get_logger().info(f'Published new data')

    def _get_accelerations(self, data, time_stamp):
        self.prev_data.append((data, time_stamp))
        if len(self.prev_data) <= 5:
            return False, []
        self.prev_data = self.prev_data[-6:]
        derivatives = np.empty((0, 6), float)
        for i in range(1, 6):
            current_data, current_time = self.prev_data[i]
            previous_data, previous_time = self.prev_data[i-1]
            derivative = (current_data - previous_data)/((current_time - previous_time)/CONVERSION_CONSTANT)
            # this is nanoseconds, should it be converted?
            derivatives = np.vstack((derivatives, derivative))
        return True, np.mean(derivatives, axis=0)
    
    def _transform_to_body_frame(self, msg):
        quaternion = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        rotation = R.from_quat(quaternion)
        inverse_rotation = rotation.inv()
        linear_velocity = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        angular_velocity = np.array([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])
        linear_velocity_body = inverse_rotation.apply(linear_velocity)
        angular_velocity_body = inverse_rotation.apply(angular_velocity)
        return linear_velocity_body, angular_velocity_body


    def incoming_data_callback(self, msg):
        # get data
        time_stamp = Time.from_msg(msg.header.stamp).nanoseconds
        linear_velocity_body, angular_velocity_body = self._transform_to_body_frame(msg)
        # get transformed velocities
        linear_x = linear_velocity_body[0]
        linear_y = linear_velocity_body[1]
        linear_z = linear_velocity_body[2]
        angular_x = angular_velocity_body[0]
        angular_y = angular_velocity_body[1]
        angular_z = angular_velocity_body[2]
        # get thruster data
        thruster_surge_left = msg.thrusters.speed_surge_left
        thruster_surge_right = msg.thrusters.speed_surge_right
        thruster_sway_front = msg.thrusters.speed_sway_front
        thruster_sway_rear = msg.thrusters.speed_sway_rear
        # assemble sample
        sample = np.array([linear_x, linear_y, linear_z, angular_x, angular_y, angular_z,
                  thruster_surge_left, thruster_surge_right, thruster_sway_front, thruster_sway_rear])
        # self.get_logger().info(str(sample))
        # get accelerations
        valid_data_flag, target = self._get_accelerations(
            np.array([linear_x, linear_y, linear_z, angular_x, angular_y, angular_z]), time_stamp)
        # self.get_logger().info(str(target))
        # return if not enough data
        if not valid_data_flag:
            return
        # assess uncertainty
        pred_mean, pred_std = self.model.predict(sample.reshape(1, -1))
        self.uncertainty = float(np.mean(pred_std))
        self.current_sample = sample
        self.current_target = target
        self.have_new_data = True

    def model_weights_callback(self, msg):
        model_path = msg.path
        self.model.model.load_weights(model_path)
        rmtree(model_path)
        self.get_logger().info(f'Model weights updated')


def main(args=None):
    rclpy.init(args=args)
    inference_node = InferenceNode()
    rclpy.spin(inference_node)
    inference_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
