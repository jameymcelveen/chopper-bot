import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.get_logger().info("Motor Controller Node Initialized")

    def listener_callback(self, msg):
        # Differential Drive Kinematics
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Calculate Left and Right wheel speeds (-1.0 to 1.0)
        left_speed = linear - angular
        right_speed = linear + angular
        
        # TODO: Send these values to your specific ESCs/Motor Drivers
        # self.get_logger().info(f"Speeds -> L: {left_speed:.2f}, R: {right_speed:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
