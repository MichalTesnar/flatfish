import rclpy
from rclpy.node import Node
import numpy as np
import uuid

from uq_model import AIOModel

from flatfish_msgs.msg import ModelWeights, KerasReadyTrainingData

class TrainingNode(Node):
    def __init__(self):
        super().__init__('receiver')
        self.have_new_data = False
        self.aio_model = AIOModel(training_set=(np.empty((0, 10), float), np.empty((0, 6), float)))
        self.path_to_weights = None

        self.training_data_subscription = self.create_subscription(
            KerasReadyTrainingData,
            'infered_data',
            self.incoming_data_callback,
            10)

        self.publisher_ = self.create_publisher(ModelWeights, 'model_weights_path', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.publisher_callback)

    def publisher_callback(self):
        if not self.have_new_data:
            return
        id_of_weights = str(uuid.uuid4())
        directory_of_weights = "model_weights/" + id_of_weights
        self.aio_model.model.save_weights(directory_of_weights, verbose=False)
        msg = ModelWeights()
        msg.path = directory_of_weights
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published new weights')

    def incoming_data_callback(self, msg):
        sample, target = np.array(msg.sample), np.array(msg.target)
        training_flag = self.aio_model.update_own_training_set((sample, target), msg.uncertainty)
        if training_flag:
            history = self.aio_model.retrain()
            self.get_logger().info(f"Model has been retrained.")
            self.have_new_data = True


def main(args=None):
    rclpy.init(args=args)

    training_node = TrainingNode()

    rclpy.spin(training_node)

    training_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
