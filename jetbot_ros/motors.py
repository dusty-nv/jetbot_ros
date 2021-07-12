#!/usr/bin/env python
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist


class MotorController(Node):
    """
    Abstract motor controller base node for supporting different JetBots.
    Can be extended to support any diff drive by overriding set_speed(),
    or any node that subscribes to the /jetbot/cmd_vel Twist message.
    """
    def __init__(self):
        super().__init__('motors', namespace='jetbot')
        
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.twist_listener, 10)
        
    def destroy_node(self):
        self.stop()
        
    def set_speed(self, left, right):
        """
        Sets the motor speeds between [-1.0, 1.0]
        Override this function for other motor controller setups.
        """
        raise NotImplementedError('MotorController subclasses should implement set_speed()')

    def stop(self):
        self.set_speed(0,0)

    def twist_listener(self, msg):
        print(msg)

        #set_speed(motor_left_ID,  -1.0)
        #set_speed(motor_right_ID,  1.0) 

    
if __name__ == '__main__':
    raise NotImplementedError("motors.py shouldn't be instantiated directly - instead use motors_nvidia.py, motors_waveshare.py, ect")
    
    
	

