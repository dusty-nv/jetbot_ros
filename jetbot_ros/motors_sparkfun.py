import rclpy

from jetbot_ros.motors import MotorController
import qwiic


MAX_SPEED = 255

class MotorControllerSparkfun(MotorController):
    """
    Motor controller node that supports the Sparkfun JetBot.
    @see motors.py for the base class to implement different controllers.
    """
    
    def __init__(self):
        super().__init__()
        self.driver = qwiic.QwiicScmd()
        
    def set_speed(self, left, right):
        self.driver.set_drive(0, 0, int(left * MAX_SPEED))
        self.driver.set_drive(1, 0, int(right * MAX_SPEED))
        self.driver.enable()
 

def main(args=None):
    rclpy.init(args=args)
    
    node = MotorControllerSparkfun()
    node.get_logger().info("listening for velocity messages...")
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()
