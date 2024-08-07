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
import matplotlib.pyplot as plt
import tensorflow as tf

CONVERSION_CONSTANT = 1e9
PUBLISHER_PERIOD = 0.1
PUBLISHER_QUEUE_SIZE = 100
SUBSCRIBER_QUEUE_SIZE = 100
FEATURES = 6


class EvaluatorNode(Node):
    def __init__(self):
        super().__init__('evaluator')
        self.have_new_data = False
        self.model = AIOModel()
        self.data = pd.read_csv("gathered_long_mission.csv")
        self.counter = 0
        # normalize the data between 0 and 1
        # self.data = (self.data - self.data.min()) / (self.data.max() - self.data.min())

        self.test_sample = self.data.iloc[:, :FEATURES].values
        self.test_target = self.data.iloc[:, FEATURES:].values

        # X_cols = [0, 1, 2, 7, 8, 9]
        # self.test_sample = self.data.iloc[:, X_cols].values
        # y_cols = [3, 4, 5, 6]
        # self.test_target = self.data.iloc[:, y_cols].values
        self.test_set_size = len(self.test_sample)

        # create new file to write mse and r2 into
        # Open a CSV file in write mode


        file_number = 0
        while os.path.exists(f'evaluation_metrics_{file_number}.csv'):
            file_number += 1
        csv_file = open(f'evaluation_metrics_{file_number}.csv', 'w', newline='')
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

        pred_mean, pred_std = self.model.predict(self.test_sample)
        # make figure of prediction vs target

        targets_fig, targets_axs = plt.subplots(4, 1, figsize=(20, 10))
        for i in range(4):
            x = range(len(self.test_target[:, i]))  # Define the x-values
            targets_axs[i].plot(x, self.test_target[:, i], label='True')
            targets_axs[i].plot(x, pred_mean[:, i], label='Pred')
            targets_axs[i].fill_between(x, pred_mean[:, i] - pred_std[:, i], pred_mean[:, i] + pred_std[:, i], alpha=0.5)
            # draw vertical line from i*100 and at i*100+100
            targets_axs[i].axvline(x=self.counter*100, color='gray', linestyle='--', linewidth=0.5)
            targets_axs[i].axvline(x=self.counter*100 + 100, color='gray', linestyle='--', linewidth=0.5)
            # label y axis
            targets_axs[i].set_ylabel(f'Thruster {i+1}', fontsize=15)
        targets_axs[-1].legend(fontsize=15, loc='upper left')  # Add legend to each subplot if needed
        targets_fig.tight_layout()

        # plt.show()  # Ensure to show the plot

        # save
        plt.savefig(f'train_evaluator_{self.counter}.png')
        # plt.show()

        MSE = np.sum(np.square(self.test_target - pred_mean)) / \
            self.test_set_size
        R2 = r2_score(self.test_target, pred_mean) # using mean of R2 of all variables
        self.get_logger().info(str(self.counter) + 'MSE: ' + str(MSE) + ' R2: ' + str(R2))
        msg = EvaluationMetrics()
        msg.mse = MSE
        msg.r2 = R2

        # save mse and r2 to file as row in csv
        self.csv_writer.writerow([MSE, R2])

        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)

        self.have_new_data = False
        self.counter += 1

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
