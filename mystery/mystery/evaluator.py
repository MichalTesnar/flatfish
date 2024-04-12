from uq_model import AIOModel
from tensorflow.keras import models
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from flatfish_msgs.msg import EvaluationMetrics, ModelWeights, KerasReadyTrainingData
from shutil import rmtree
import numpy as np
import pandas as pd

CONVERSION_CONSTANT = 1e9

class EvaluatorNode(Node):
    def __init__(self):
        super().__init__('evaluator')
        self.have_new_data = False
        self.model = AIOModel()
        # open csv file called data.csv
        self.data = pd.read_csv('src/rosbags/data.csv', nrows=1000)
        # read first 10 columns as sample, then 6 as target
        self.test_sample = self.data.iloc[:, :7].values
        self.test_target = self.data.iloc[:, 7:].values
        self.test_set_size = len(self.test_sample)

        self.model_weights_subscription = self.create_subscription(
            ModelWeights,
            'loaded_weights_path',
            self.model_weights_callback,
            10)

        self.publisher_ = self.create_publisher(
            EvaluationMetrics, 'evaluation_metrics', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publisher_callback)

    def publisher_callback(self):
        if not self.have_new_data:
            return
        
        pred_mean, _ = self.model.predict(self.test_sample)
        MSE = np.sum(np.square(self.test_target - pred_mean))/self.test_set_size
        print(MSE)
        msg = EvaluationMetrics()
        msg.mse = MSE
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)

        self.have_new_data = False
        #self.get_logger().info(f'Published new data')

    def model_weights_callback(self, msg):
        model_path = msg.path
        self.model.model.load_weights(model_path)
        rmtree(model_path)
        #self.get_logger().info(f'Model weights updated')
        self.have_new_data = True


def main(args=None):
    rclpy.init(args=args)
    evaluator_node = EvaluatorNode()
    rclpy.spin(evaluator_node)
    evaluator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
