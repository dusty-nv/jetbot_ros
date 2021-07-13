import rclpy

from rclpy.node import Node
from jetbot_ros.motors import MotorController

from Adafruit_MotorHAT import Adafruit_MotorHAT



class MotorControllerWaveshare(MotorController):
    """
    Motor controller node that supports the Waveshare JetBot.
    @see motors.py for the base class to implement different controllers.
    """
    MOTOR_LEFT = 1      # left motor ID
    MOTOR_RIGHT = 2     # right motor ID
    
    def __init__(self):
        super().__init__()
        
        # open Adafruit MotorHAT driver
        self.driver = Adafruit_MotorHAT(i2c_bus=1)
        
        # get motor objects from driver
        self.motors = {
            self.MOTOR_LEFT : self.driver.getMotor(self.MOTOR_LEFT),
            self.MOTOR_RIGHT : self.driver.getMotor(self.MOTOR_RIGHT)
        }
        
        self.pwm_channels = {
            self.MOTOR_LEFT : (1, 0),
            self.MOTOR_RIGHT : (2, 3)
        }
        
    def set_speed(self, left, right):
        """
        Sets the motor speeds between [-1.0, 1.0]
        """
        self._set_pwm(self.MOTOR_LEFT, left, self.left_trim)
        self._set_pwm(self.MOTOR_RIGHT, right, self.right_trim)
      
    def _set_pwm(self, motor, value, trim):
        # apply trim and convert [-1,1] to PWM value
        pwm = int(min(max((abs(value) + trim) * self.max_pwm, 0), self.max_pwm))
        self.motors[motor].setSpeed(pwm)

        # set the motor direction
        ina, inb = self.pwm_channels[motor]
        
        if value > 0:
            self.motors[motor].run(Adafruit_MotorHAT.FORWARD)
            self.driver._pwm.setPWM(ina, 0, pwm * 16)
            self.driver._pwm.setPWM(inb, 0, 0)
        elif value < 0:
            self.motors[motor].run(Adafruit_MotorHAT.BACKWARD)
            self.driver._pwm.setPWM(ina, 0, 0)
            self.driver._pwm.setPWM(inb, 0, pwm * 16)
        else:
            self.motors[motor].run(Adafruit_MotorHAT.RELEASE)
            self.driver._pwm.setPWM(ina, 0, 0)
            self.driver._pwm.setPWM(inb, 0, 0)
 

def main(args=None):
    rclpy.init(args=args)
    
    node = MotorControllerWaveshare()
    node.get_logger().info("listening for velocity messages...")
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()
    
	

