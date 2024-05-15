import rclpy
from rclpy.node import Node
import numpy as np
import uuid
from uq_model import AIOModel
from flatfish_msgs.msg import ModelWeights, KerasReadyTrainingData, Dataset

PUBLISHER_PERIOD = 0.01
SUBSCRIBER_QUEUE_SIZE = 100
PUBLISHER_QUEUE_SIZE = 100


class TrainingNode(Node):
    def __init__(self):
        super().__init__('trainer')
        self.have_new_data = False
        self.aio_model = AIOModel()
        self.path_to_weights = None

        self.training_data_subscription = self.create_subscription(
            Dataset,
            'infered_data',
            self.incoming_data_callback,
            SUBSCRIBER_QUEUE_SIZE)

        self.publisher_ = self.create_publisher(
            ModelWeights, 'model_weights_path', PUBLISHER_QUEUE_SIZE)
        self.timer = self.create_timer(PUBLISHER_PERIOD, self.publisher_callback)

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
        self.have_new_data = False

    def incoming_data_callback(self, msg):
        dataset = msg.dataset
        samples = np.array([data.sample for data in dataset])
        targets = np.array([data.target for data in dataset])
        weights = np.array([data.training_weight for data in dataset])
        self.aio_model.update_own_training_set(samples, targets, weights)
        self.aio_model.retrain()
        self.have_new_data = True


def main(args=None):
    rclpy.init(args=args)
    training_node = TrainingNode()
    rclpy.spin(training_node)
    training_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
