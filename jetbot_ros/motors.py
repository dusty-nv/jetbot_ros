import rclpy
import math

from rclpy.node import Node
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult


class MotorController(Node):
    """
    Abstract motor controller base node for supporting different JetBots.
    Can be extended to support any diff drive by overriding set_speed(),
    or any node that subscribes to the /jetbot/cmd_vel Twist message.
    """
    def __init__(self):
        super().__init__('motors', namespace='jetbot')
        
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.twist_listener, 10)
        
        self.declare_parameter('left_trim', 0.0)
        self.declare_parameter('right_trim', 0.0)
        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('max_rpm', 200)              # https://www.adafruit.com/product/3777
        self.declare_parameter('wheel_separation', 0.1016)  # 4 inches
        self.declare_parameter('wheel_diameter', 0.060325)  # 2 3/8 inches
        
        self.left_trim = self.get_parameter('left_trim').value
        self.right_trim = self.get_parameter('right_trim').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.max_rpm = self.get_parameter('max_rpm').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        
        self.add_on_set_parameters_callback(self.parameters_callback)
         
        self.last_x = -999
        self.last_rot = -999
        
    def destroy_node(self):
        self.get_logger().info(f"shutting down, stopping robot...")
        self.stop()
        
    def parameters_callback(self, params):
        for param in params:
            if param.name == 'left_trim':
                self.left_trim = param.value
            elif param.name == 'right_trim':
                self.right_trim = param.value
            elif param.name == 'max_pwm':
                self.max_pwm = param.value
            elif param.name == 'wheel_separation':
                self.wheel_separation = param.value
            else:
                raise ValueError(f'unknown parameter {param.name}')
                
        return SetParametersResult(successful=True)
        
    def set_speed(self, left, right):
        """
        Sets the motor speeds between [-1.0, 1.0]
        Override this function for other motor controller setups.
        Should take into account left_trim, right_trim, and max_pwm.
        """
        raise NotImplementedError('MotorController subclasses should implement set_speed()')

    def stop(self):
        self.set_speed(0,0)

    def twist_listener(self, msg):
        x = msg.linear.x
        rot = msg.angular.z
        
        if x == self.last_x and rot == self.last_rot:
            return
            
        self.last_x = x
        self.last_rot = rot
        
        # https://github.com/ros-simulation/gazebo_ros_pkgs/blob/231a7219b36b8a6cdd100b59f66a3df2955df787/gazebo_plugins/src/gazebo_ros_diff_drive.cpp#L331
        left = x - rot * self.wheel_separation / 2.0
        right = x + rot * self.wheel_separation / 2.0
        
        # convert velocities to [-1,1]
        max_speed = (self.max_rpm / 60.0) * 2.0 * math.pi * (self.wheel_diameter * 0.5)

        left = max(min(left, max_speed), -max_speed) / max_speed
        right = max(min(right, max_speed), -max_speed) / max_speed
        
        self.get_logger().info(f"x={x:.03f} rotation={rot:.03f} -> left={left:.03f} right={right:.03f}  (max_speed={max_speed:.03f} m/s)")
        self.set_speed(left, right)

    
if __name__ == '__main__':
    raise NotImplementedError("motors.py shouldn't be instantiated directly - instead use motors_nvidia.py, motors_waveshare.py, ect")
    
    
	

