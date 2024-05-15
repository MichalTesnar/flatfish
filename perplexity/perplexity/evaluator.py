from uq_model import AIOModel
import rclpy
from rclpy.node import Node
from flatfish_msgs.msg import EvaluationMetrics, ModelWeights
from shutil import rmtree
import numpy as np
import pandas as pd
from sklearn.metrics import r2_score
import csv
import os

CONVERSION_CONSTANT = 1e9
PUBLISHER_PERIOD = 0.1
PUBLISHER_QUEUE_SIZE = 100
SUBSCRIBER_QUEUE_SIZE = 100
FEATURES = 7


class EvaluatorNode(Node):
    def __init__(self):
        super().__init__('evaluator')
        self.have_new_data = False
        self.model = AIOModel()
        self.data = pd.read_csv("test_set.csv")
        # normalize the data between 0 and 1
        self.data = (self.data - self.data.min()) / (self.data.max() - self.data.min())

        self.test_sample = self.data.iloc[:, :FEATURES].values
        self.test_target = self.data.iloc[:, FEATURES:].values
        self.test_set_size = len(self.test_sample)

        # create new file to write mse and r2 into
        # Open a CSV file in write mode
        csv_file = open('evaluation_metrics.csv', 'w', newline='')
        self.csv_writer = csv.writer(csv_file)

        self.model_weights_subscription = self.create_subscription(
            ModelWeights,
            'loaded_weights_path',
            self.model_weights_callback,
            SUBSCRIBER_QUEUE_SIZE)

        self.publisher_ = self.create_publisher(
            EvaluationMetrics, 'evaluation_metrics', PUBLISHER_QUEUE_SIZE)
        self.timer = self.create_timer(PUBLISHER_PERIOD, self.publisher_callback)

    def publisher_callback(self):
        if not self.have_new_data:
            return

        pred_mean, _ = self.model.predict(self.test_sample)
        MSE = np.sum(np.square(self.test_target - pred_mean)) / \
            self.test_set_size
        R2 = r2_score(self.test_target, pred_mean) # using mean of R2 of all variables
        self.get_logger().info('MSE: ' + str(MSE) + ' R2: ' + str(R2))
        msg = EvaluationMetrics()
        msg.mse = MSE
        msg.r2 = R2

        # save mse and r2 to file as row in csv
        self.csv_writer.writerow([MSE, R2])

        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)

        self.have_new_data = False

    def model_weights_callback(self, msg):
        model_path = msg.path
        self.model.model.load_weights(model_path)
        rmtree(model_path) # remove the model weights file
        self.have_new_data = True


def main(args=None):
    rclpy.init(args=args)
    evaluator_node = EvaluatorNode()
    rclpy.spin(evaluator_node)
    evaluator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
