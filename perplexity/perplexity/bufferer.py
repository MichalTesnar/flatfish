from uq_model import AIOModel
from buffer import ReplayBuffer
from tensorflow.keras import models
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from flatfish_msgs.msg import ModelWeights, Dataset, KerasReadyTrainingData, Buffer
from shutil import rmtree
import numpy as np
import queue.Queue as Queue


PUBLISHER_PERIOD = 0.01
SUBSCRIBER_QUEUE_SIZE = 100
PUBLISHER_QUEUE_SIZE = 100

TRAINING_THRESHOLD = 0
TRAINING_COUNTER = 50
QUEUE_SIZE = 300

TRAINING_SET_SIZE = TRAINING_COUNTER

class BuffererNode(Node):
    def __init__(self):
        super().__init__('bufferer')
        self.queue = Queue(maxsize=QUEUE_SIZE)
        self.counter = 0

        self.thruster_subscription = self.create_subscription(
            KerasReadyTrainingData,
            'gathered_data',
            self.incoming_data_callback,
            SUBSCRIBER_QUEUE_SIZE)

        self.publisher_ = self.create_publisher(
            Buffer, 'buffered_data', PUBLISHER_QUEUE_SIZE)
        
        self.timer = self.create_timer(
            PUBLISHER_PERIOD, self.publisher_callback)

    def publisher_callback(self):
        if self.counter < TRAINING_COUNTER:
            return
        
        msg = Buffer()
        msg.dataset = Queue.queue

        self.publisher_.publish(msg)
        self.counter = 0
        self.episode_counter += 1
        self.get_logger().info(f'P: New Buffered Data')

    def incoming_data_callback(self, msg):
        # remove last if full
        if self.queue.full():
            self.queue.get()
        # add new data
        self.queue.put(msg)
        
        self.get_logger().info(f'S: Data received')
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    bufferer_node = BuffererNode()
    rclpy.spin(bufferer_node)
    bufferer_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
