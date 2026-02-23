#!/bin/bash

# --- Configuration ---
BRAIN_PYTHON_DIR="src/chopper-brain/chopper_brain"

echo "⚙️ Adding Motor Logic & PWM Bridge..."

# 1. Create the Motor Controller Node
cat <<EOF > "$BRAIN_PYTHON_DIR/motor_node.py"
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
EOF

# 2. Update setup.py to include the new entry point
if [ -f "src/chopper-brain/setup.py" ]; then
    if ! grep -q "motor_node =" src/chopper-brain/setup.py; then
        echo "📝 Updating setup.py entry points..."
        sed -i '' "/'spektrum_node =/a \\
            'motor_node = chopper_brain.motor_node:main'," src/chopper-brain/setup.py
    fi
fi

# 3. Add Motor Node to the Launch File
if [ -f "src/chopper-description/launch/robot-launch.py" ]; then
    if ! grep -q "executable='motor_node'" src/chopper-description/launch/robot-launch.py; then
        echo "🚀 Adding Motor Node to robot-launch.py..."
        sed -i '' "/Node(/i \\
        Node(package='chopper_brain', executable='motor_node', name='motor_controller')," src/chopper-description/launch/robot-launch.py
    fi
fi

chmod +x "$BRAIN_PYTHON_DIR/motor_node.py"
echo "✅ Motor interface logic added. Run 'make ros-build'."