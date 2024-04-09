import rclpy
from rclpy.node import Node
import json
import numpy as np

from model import SimpleRegressionModel
from uq_model import AIOModel

from flatfish_msgs.msg import ModelWeights
from flatfish_msgs.msg import TrainingData



class Receiver(Node):
    def __init__(self):
        super().__init__('receiver')
        self.have_new_data = False
        self.current_twist = None
        self.current_thruster_status = None
        self.uncertainty = None
        self.model = SimpleRegressionModel(6)

        self.thruster_subscription = self.create_subscription(
            TrainingData,
            'infered_data',
            self.incoming_data_callback,
            10)

        self.publisher_ = self.create_publisher(ModelWeights, 'model_weights', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.publisher_callback)

    def publisher_callback(self):
        if not self.have_new_data:
            return
        model_json = self.model.model.to_json()
        model_json_str = json.dumps(model_json)
        msg = ModelWeights()
        msg.data = model_json_str
        self.publisher_.publish(msg)
        print("Now I have published.")

    def incoming_data_callback(self, msg):
        X_train = np.array([[msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z, msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]])
        y_train = np.array([msg.thruster.velocities[0]])
        history = self.model.train(X_train, y_train, epochs=10, batch_size=32)

        self.have_new_data = True


def main(args=None):
    rclpy.init(args=args)

    receiver = Receiver()

    rclpy.spin(receiver)

    receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
