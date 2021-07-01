import os
import sys
import rclpy
import numpy as np
import PIL

from rclpy.node import Node
from sensor_msgs.msg import Image

from .dnn.navigation_model import NavigationModel


class NavModelNode(Node):
    """
    Navigation model ROS node that uses PyTorch on camera images
    """
    def __init__(self):
        super().__init__('nav_model', namespace='jetbot/nav_model')
        
        # get node parameters
        self.declare_parameter('model')
        self.declare_parameter('type', 'regression')
        
        model_path = self.get_parameter('model').value
        model_type = self.get_parameter('type').value
        
        self.get_logger().info(f"model = {model_path}")
        self.get_logger().info(f"type = {model_type}")
        
        if model_path is None:
            raise ValueError('must specify PyTorch model path parameter (e.g. model:=/path/to/your/model.pth)')
        
        # create image subscriber
        self.image_subscriber = self.create_subscription(Image, 'image_in', self.image_listener, 10)
      
    def image_listener(self, msg):
        self.get_logger().debug(f"recieved image:  {msg.width}x{msg.height}, {msg.encoding}")
        self.get_logger().debug(str(msg.header))

        if msg.encoding != 'rgb8':
            raise ValueError(f"image encoding is '{msg.encoding}' (expected rgb8)")
            
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

        
def main(args=None):
    rclpy.init(args=args)
    node = NavModelNode()
    node.get_logger().info("starting processing loop...")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
