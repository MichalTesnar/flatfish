from model import SimpleRegressionModel 
from tensorflow.keras import models
import rclpy
from rclpy.node import Node
from flatfish_msgs.msg import TrainingData
from flatfish_msgs.msg import ModelWeights

class InferenceNode(Node):
    def __init__(self):
        super().__init__('receiver')
        self.have_new_data = False
        self.uncertainty = None
        self.current_training_data = None
        self.model = SimpleRegressionModel(6)

        self.model_weights_subscription = self.create_subscription(
            ModelWeights,
            'model_weights',
            self.model_weights_callback,
            10)
        
        self.thruster_subscription = self.create_subscription(
            TrainingData,
            'gathered_data',
            self.incoming_data_callback,
            10)

        self.publisher_ = self.create_publisher(TrainingData, 'infered_data', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.publisher_callback)

    def publisher_callback(self):
        if not self.have_new_data:
            return
        msg = self.current_training_data
        
        self.publisher_.publish(msg)
        
        self.current_training_data = None
        self.have_new_data = False

    def incoming_data_callback(self, msg):
        twist = msg.twist
        thruster = msg.thruster
        # run inference
        # @TODO
        uncertainty = 0.5
        # if uncertainty is high
        if uncertainty > 0.3:
            # set flag to publish new data
            self.uncertainty = uncertainty
            self.current_twist = twist
            self.current_thruster_status = thruster
            self.have_new_data = True
            print("Received and published new data")
    
    def model_weights_callback(self, msg):
        model_json_str = msg.data
        model_json = json.loads(model_json_str)
        self.model.model = models.model_from_json(model_json)
        print("Model weights updated")


def main(args=None):
    rclpy.init(args=args)
    inference_node = InferenceNode()
    rclpy.spin(inference_node)
    inference_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
