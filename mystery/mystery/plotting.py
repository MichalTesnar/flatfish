#!/usr/bin/env python 
import numpy as np
from matplotlib import pyplot as plt
import rclpy
from flatfish_msgs.msg import KerasReadyTrainingData
import rclpy
from rclpy.node import Node


counter = 0

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            KerasReadyTrainingData,
            'infered_data',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        global counter
        plt.plot(counter, msg.sample[0], "*")
        plt.axis("equal")
        plt.draw()
        plt.pause(0.00000000001)

        counter += 1


def main(args=None):
    
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()