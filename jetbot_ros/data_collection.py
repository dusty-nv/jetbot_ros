import os
import sys
import math
import rclpy
import numpy as np
import cv2

from rclpy.node import Node
from datetime import datetime
from PIL import Image as PIL_Image

from std_msgs.msg import String
from sensor_msgs.msg import Image


class DataCollectionNode(Node):
    """
    ROS note for collecting / annotating images.
    """
    def __init__(self):
        super().__init__('data_collection', namespace='jetbot')
        
        # create topics
        self.image_subscriber = self.create_subscription(Image, 'camera/image_raw', self.image_listener, 10)
        self.keys_subscriber = self.create_subscription(String, 'keys', self.key_listener, 10)
        
        # get node parameters
        self.declare_parameter('data_path')
        self.data_path = self.get_parameter('data_path').value
        self.get_logger().info(f"data_path = {self.data_path}")

        if self.data_path is None:
            raise ValueError('must specify data_path parameter (e.g. data_path:=/path/to/save/your/dataset)')

        os.makedirs(self.data_path, exist_ok=True)
        
        self.collect = False
        self.xy_label = None
        
    def key_listener(self, msg):
        if msg.data == 'c':
            self.collect = True
        
    def image_listener(self, msg):
        self.get_logger().debug(f"recieved image:  {msg.width}x{msg.height}, {msg.encoding}")
        #self.get_logger().debug(str(msg.header))

        if msg.encoding != 'rgb8':
            raise ValueError(f"image encoding is '{msg.encoding}' (expected rgb8)")
        
        if not self.collect:
            return
            
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

        title = 'Click on path center point'
        cv2.imshow(title, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        cv2.setMouseCallback(title, self.click_event)
        cv2.waitKey(0)
        
        if self.xy_label is not None:
            img_path = os.path.join(self.data_path, f"xy_{self.xy_label[0]:03d}_{self.xy_label[1]:03d}_{datetime.now().strftime('%Y%m%d-%H%M%S-%f')}.jpg")
            PIL_Image.fromarray(img).save(img_path)
            self.get_logger().info(f"saved {msg.width}x{msg.height} image to '{img_path}'")
            
        self.collect = False
        self.xy_label = None
    
    def click_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.xy_label = (x,y)
            cv2.destroyAllWindows()    
        
def main(args=None):
    rclpy.init(args=args)
    node = DataCollectionNode()
    node.get_logger().info("Press 'c' in the teleop window to collect data")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
