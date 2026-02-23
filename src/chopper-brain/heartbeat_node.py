import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import subprocess

class HeartbeatNode(Node):
    def __init__(self):
        super().__init__('heartbeat_node')
        self.publisher_ = self.create_publisher(Float32, 'diagnostics/gpu_temp', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.get_logger().info("Chopper Heartbeat (GPU Monitor) Started")

    def get_gpu_temp(self):
        # On Jetson, we read from the thermal zone
        try:
            temp_str = subprocess.check_output(['cat', '/sys/class/thermal/thermal_zone1/temp']).decode('utf-8')
            return float(temp_str) / 1000.0
        except:
            return 0.0

    def timer_callback(self):
        msg = Float32()
        msg.data = self.get_gpu_temp()
        self.publisher_.publish(msg)
        # Log a heartbeat to terminal
        self.get_logger().info(f"❤️ Heartbeat | GPU Temp: {msg.data:.1f}°C")

def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
