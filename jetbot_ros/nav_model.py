import os
import sys
import math
import rclpy
import numpy as np
import PIL
import cv2

from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from jetbot_ros.dnn.navigation_model import NavigationModel


class NavModelNode(Node):
    """
    Navigation model ROS node that uses PyTorch on camera images
    """
    def __init__(self):
        super().__init__('nav_model', namespace='jetbot')
        
        # create topics
        self.image_subscriber = self.create_subscription(Image, 'camera/image_raw', self.image_listener, 10)
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # get node parameters
        self.declare_parameter('model')
        self.declare_parameter('type', 'regression')
        self.declare_parameter('speed_gain', 0.15)
        self.declare_parameter('steering_gain', 0.4)
        self.declare_parameter('visualize', False)
        
        model_path = self.get_parameter('model').value
        model_type = self.get_parameter('type').value
        
        self.get_logger().info(f"model = {model_path}")
        self.get_logger().info(f"type = {model_type}")

        if model_path is None:
            raise ValueError('must specify PyTorch model path parameter (e.g. model:=/path/to/your/model.pth)')
        
        # load model
        self.model = NavigationModel(model_path, type=model_type)

    def image_listener(self, msg):
        self.get_logger().debug(f"recieved image:  {msg.width}x{msg.height}, {msg.encoding}")
        #self.get_logger().debug(str(msg.header))

        if msg.encoding != 'rgb8' and msg.encoding != 'bgr8':
            raise ValueError(f"image encoding is '{msg.encoding}' (expected rgb8 or bgr8)")
            
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

        if msg.encoding == 'bgr8':
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            
        # predict the center point
        xy = self.model.infer(PIL.Image.fromarray(img))
        
        x = xy[0]
        y = (0.5 - xy[1]) / 2.0
        
        # convert to steering angle
        steering_angle = np.arctan2(x, y)
        self.get_logger().info(f'x={x:.2f}  y={y:.2f}  angle={math.degrees(steering_angle):.1f}')
                
        # publish velocity message
        twist = Twist()
        
        twist.linear.x = self.get_parameter('speed_gain').value
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = -steering_angle * self.get_parameter('steering_gain').value
        
        self.velocity_publisher.publish(twist)

        # visualization
        if self.get_parameter('visualize').value:
            px = min(max(((xy[0] + 1) / 2) * img.shape[1], 0), img.shape[1])
            py = min(max(((xy[1] + 1) / 2) * img.shape[0], 0), img.shape[0])
            
            cv_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            cv_img = cv2.circle(cv_img, (int(px),int(py)), radius=5, color=(50,155,255), thickness=2)
            cv2.imshow(f"{self.get_namespace()}/{self.get_name()} Inference", cv_img)
            cv2.waitKey(1)            
                
    def destroy_node(self):
        twist = Twist()
        
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        
        self.get_logger().info('shutting down, stopping robot...')
        self.velocity_publisher.publish(twist)
        
        super().destroy_node()
        
def main(args=None):
    rclpy.init(args=args)
    node = NavModelNode()
    node.get_logger().info("starting processing loop...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
