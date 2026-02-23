import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class SpektrumBridge(Node):
    def __init__(self):
        super().__init__('spektrum_bridge')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Jetson Nano UART port (/dev/ttyTHS1 is J41 pins 8/10)
        # Using 115200 as standard for Jetson UART compatibility
        self.ser = serial.Serial('/dev/ttyTHS1', 115200, timeout=0.01)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info("Spektrum Bridge Started - Listening on /dev/ttyTHS1")

    def timer_callback(self):
        if self.ser.in_waiting >= 16:
            data = self.ser.read(16)
            
            # DSMX 2048 Parsing Logic:
            # Bytes 0-1: System/Fade count
            # Bytes 2-15: 7 channels (2 bytes each)
            channels = {}
            for i in range(1, 8):
                part = data[i*2 : i*2+2]
                val = int.from_state(part, byteorder='big')
                # 2048 mode: bits 0-10 are value, 11-14 are channel ID
                channel_id = (val >> 11) & 0x0F
                channel_val = val & 0x07FF
                channels[channel_id] = channel_val

            # Safety Mux: Check Channel 5 (Aux 1 / Gear Switch)
            # Typically 342 (Low) to 1705 (High) in DSMX
            # Only publish if switch is 'UP' (Enable AI/Manual)
            is_enabled = channels.get(5, 0) > 1000 
            
            msg = Twist()
            if is_enabled:
                # Map Throttle (Ch0) to linear.x and Aileron (Ch1) to angular.z
                # Normalized to approx -1.0 to 1.0
                msg.linear.x = (channels.get(0, 1024) - 1024) / 1024.0
                msg.angular.z = (channels.get(1, 1024) - 1024) / 1024.0
                self.publisher_.publish(msg)
            else:
                # Force 0 if switch is down (Safety Kill)
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SpektrumBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
