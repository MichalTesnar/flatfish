from uq_model import AIOModel
from buffer import ReplayBuffer
from tensorflow.keras import models
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from flatfish_msgs.msg import ModelWeights, Dataset, KerasReadyTrainingData
from shutil import rmtree
import numpy as np


CONVERSION_CONSTANT = 1e9
NORMALIZE_THRUSTERS = 65
DERIVATIVE_QUEUE_SIZE = 5
PUBLISHER_PERIOD = 0.01
SUBSCRIBER_QUEUE_SIZE = 100
PUBLISHER_QUEUE_SIZE = 100

TRAINING_THRESHOLD = 0
TRAINING_COUNTER = 100
QUEUE_SIZE = 1000

TRAINING_SET_SIZE = TRAINING_COUNTER

class InferenceNode(Node):
    def __init__(self):
        super().__init__('inferencer')
        self.model = AIOModel()
        self.buffer = ReplayBuffer(self.model, QUEUE_SIZE, mode='score') # "score" or "uniform"
        self.counter = 0
        self.episode_counter = 0

        self.current_path_to_weights = None
        self.have_new_weights = False

        self.model_weights_subscription = self.create_subscription(
            ModelWeights,
            'model_weights_path',
            self.model_weights_callback,
            SUBSCRIBER_QUEUE_SIZE)

        self.thruster_subscription = self.create_subscription(
            KerasReadyTrainingData,
            'gathered_data',
            self.incoming_data_callback,
            SUBSCRIBER_QUEUE_SIZE)

        self.publisher_ = self.create_publisher(
            Dataset, 'infered_data', PUBLISHER_QUEUE_SIZE)
        
        self.timer = self.create_timer(
            PUBLISHER_PERIOD, self.publisher_callback)

        self.loaded_weights_publisher_ = self.create_publisher(
            ModelWeights, 'loaded_weights_path', PUBLISHER_QUEUE_SIZE)
        self.timer = self.create_timer(
            PUBLISHER_PERIOD, self.loaded_weights_publisher_callback)

    def publisher_callback(self):
        if self.counter < TRAINING_COUNTER:
            return
        
        dataset, weights, maximum_score = self.buffer.sample(TRAINING_SET_SIZE, self.episode_counter)
        weighted_dataset = []
        for ((sample, target), weight) in zip(dataset, weights):
            msg = KerasReadyTrainingData()
            msg.sample = sample
            msg.target = target
            msg.training_weight = float(weight)
            weighted_dataset.append(msg)

        if maximum_score < TRAINING_THRESHOLD:
            self.get_logger().info(f'P: Data not good enough :)')
            return
        msg = Dataset()
        msg.dataset = weighted_dataset

        self.publisher_.publish(msg)
        self.counter = 0
        self.episode_counter += 1
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

    def incoming_data_callback(self, msg):
        sample = np.array(msg.sample)
        target = np.array(msg.target)

        self.buffer.push((sample, target))
        self.get_logger().info(f'S: Data received')
        self.counter += 1

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
