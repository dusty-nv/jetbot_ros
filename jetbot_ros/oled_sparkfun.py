import rclpy
import qwiic
from jetbot_ros.oled import OLEDController


TEXT_SECTION_HEIGHT = 16

class OLEDControllerSparkfun(OLEDController):
    """
    OLED controller node for Sparkfun Micro OLED (used on the Sparkfun JetBot)
    @see oled.py for the base class to impelement different OLED controllers.
    """
    def __init__(self):
        super().__init__()
        self.display = qwiic.QwiicMicroOled()
        self.display.begin()
        self.display.display()
        self.display.set_font_type(0)
        
    def render(self, text):
        self.display.clear(self.display.PAGE)
        for idx, txt in enumerate(text):
            self.display.set_cursor(0, idx * TEXT_SECTION_HEIGHT)
            self.display.print(txt)
        self.display.display()


def main(args=None):
    rclpy.init(args=args)
    
    node = OLEDControllerSparkfun()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()
    