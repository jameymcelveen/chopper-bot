#!/bin/bash

# --- Configuration ---
PROJECT_NAME="chopper-bot"
BRAIN_PKG_DIR="src/chopper-brain"
BRAIN_PYTHON_DIR="$BRAIN_PKG_DIR/chopper_brain" # Correct underscore for Python

echo "🚀 Applying Gemini's Idempotent Patches..."

# 1. Ensure Python Package Structure (Underscore Fix)
if [ ! -d "$BRAIN_PYTHON_DIR" ]; then
    echo "🔧 Fixing Python package naming (dash to underscore)..."
    mkdir -p "$BRAIN_PYTHON_DIR"
    touch "$BRAIN_PYTHON_DIR/__init__.py"
    # Move node if it was in the wrong place
    [ -f "src/chopper-brain/chopper-brain/spektrum_node.py" ] && mv "src/chopper-brain/chopper-brain/spektrum_node.py" "$BRAIN_PYTHON_DIR/"
fi

# 2. Update docker-compose.yml to include Serial Device Mapping
if ! grep -q "devices:" docker-compose.yml; then
    echo "🔌 Mapping /dev/ttyTHS1 into Docker..."
    # Uses sed to insert the device mapping after the privileged line
    sed -i '' '/privileged: true/a \
    devices:\
      - "/dev/ttyTHS1:/dev/ttyTHS1"' docker-compose.yml
fi

# 3. Enhance spektrum_node.py with Failsafe & Buffer Flush
# This checks if the reset_input_buffer code is already there
if [ -f "$BRAIN_PYTHON_DIR/spektrum_node.py" ] && ! grep -q "reset_input_buffer" "$BRAIN_PYTHON_DIR/spektrum_node.py"; then
    echo "🛡 Adding Serial Failsafe to spektrum_node.py..."
    # Inject buffer reset into __init__
    sed -i '' '/self.ser = serial.Serial/a \
        self.ser.reset_input_buffer()' "$BRAIN_PYTHON_DIR/spektrum_node.py"
fi

# 4. Create display.launch.py (if it doesn't exist)
LAUNCH_DIR="src/chopper-description/launch"
mkdir -p "$LAUNCH_DIR"
if [ ! -f "$LAUNCH_DIR/display-launch.py" ]; then
    echo "📺 Creating Rviz display launch file..."
    cat <<EOF > "$LAUNCH_DIR/display-launch.py"
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': 'placeholder'}] # Will be xacro later
        ),
        Node(
            package='chopper_brain',
            executable='spektrum_node',
            name='spektrum_bridge'
        )
    ])
EOF
fi

# 5. Finalize setup.py Entry Points
if [ -f "$BRAIN_PKG_DIR/setup.py" ]; then
    sed -i '' "s/chopper-brain.spektrum_node/chopper_brain.spektrum_node/g" "$BRAIN_PKG_DIR/setup.py"
fi

echo "✅ Patches applied. You can now run 'make ros-build'."