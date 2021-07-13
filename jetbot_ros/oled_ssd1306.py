import rclpy
import Adafruit_SSD1306

from jetbot_ros.oled import OLEDController

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont



class OLEDControllerSSD1306(OLEDController):
    """
    OLED controller node for Adafruit SSD1306 (used on the original NVIDIA JetBot)
    @see oled.py for the base class to impelement different OLED controllers.
    """
    def __init__(self):
        super().__init__()
        
        # 128x32 display with hardware I2C:
        self.display = Adafruit_SSD1306.SSD1306_128_32(rst=None, i2c_bus=1, gpio=1) # setting gpio to 1 is hack to avoid platform detection
        
        self.display.begin()
        self.display.clear()
        self.display.display()
        
        # canvas image
        self.image = Image.new('1', (self.display.width, self.display.height))
        self.canvas = ImageDraw.Draw(self.image)
        
        # default font
        self.font = ImageFont.load_default()
        
    def render(self, text):
        # Draw a black filled box to clear the image.
        self.canvas.rectangle((0, 0, self.display.width, self.display.height), outline=0, fill=0)
        
        # Draw text
        for idx, txt in enumerate(text):
            self.canvas.text((0, -2 + idx * 8), txt, font=self.font, fill=255)
            
        # Present the image
        self.display.image(self.image)
        self.display.display()


def main(args=None):
    rclpy.init(args=args)
    
    node = OLEDControllerSSD1306()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()
    