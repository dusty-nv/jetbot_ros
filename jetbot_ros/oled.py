import rclpy
import subprocess

from rclpy.node import Node
from std_msgs.msg import String


class OLEDController(Node):
    """
    Abstract OLED controller base node for supporting different JetBots.
    Can be extended to support any display by overriding Render().
    """
    def __init__(self):
        super().__init__('oled', namespace='jetbot')
        
        self.sub = self.create_subscription(String, 'oled/text', self.text_listener, 10)
        self.user_text = None
        self.create_timer(1.0, self.timer_callback)
        
    def render(self, text):
        """
        Abstract method for rendering a list of strings.
        Override this function for other OLED display drivers.
        """
        raise NotImplementedError('OLEDController subclasses should implement render()')
        
    def text_listener(self, msg):
        self.user_text = msg.data
        self.get_logger().info(f"user text = {self.user_text}")
        
    def timer_callback(self):
        text = [
            f"eth0: {self.get_ip_address('eth0')}" if self.user_text is None else self.user_text,
            f"wlan0: {self.get_ip_address('wlan0')}",
            self.get_memory_usage(),
            self.get_disk_usage()
        ]
  
        self.render(text)
        self.get_logger().info(f"{self.get_cpu_usage()}  {text[2]}  {text[3]}")
        
    @staticmethod
    def get_ip_address(interface):
        if OLEDController.get_network_interface_state(interface) == 'down':
            return None
        cmd = "ifconfig %s | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'" % interface
        return str(subprocess.check_output(cmd, shell=True).decode('ascii')[:-1])

    @staticmethod
    def get_network_interface_state(interface):
        return str(subprocess.check_output('cat /sys/class/net/%s/operstate' % interface, shell=True).decode('ascii')[:-1])

    @staticmethod
    def get_cpu_usage():
        cmd = "top -bn1 | grep load | awk '{printf \"CPU Load: %.2f\", $(NF-2)}'"
        return str(subprocess.check_output(cmd, shell=True).decode('utf-8'))
        
    @staticmethod
    def get_memory_usage():
        cmd = "free -m | awk 'NR==2{printf \"Mem: %s/%sMB %.2f%%\", $3,$2,$3*100/$2 }'"
        return str(subprocess.check_output(cmd, shell=True).decode('utf-8'))
        
    @staticmethod
    def get_disk_usage():
        cmd = "df -h | awk '$NF==\"/\"{printf \"Disk: %d/%dGB %s\", $3,$2,$5}'"
        return str(subprocess.check_output(cmd, shell=True).decode('utf-8'))
   
   
if __name__ == '__main__':
    raise NotImplementedError("oled.py shouldn't be instantiated directly - instead use oled_ssd1306.py, ect")
    
    