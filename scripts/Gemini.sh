#!/bin/bash

# --- Configuration ---
BRAIN_PKG_DIR="src/chopper-brain"
DESC_PKG_DIR="src/chopper-description"

echo "🛠 Syncing Dependencies & URDF Launchers..."

# 1. Sync package.xml dependencies for chopper-brain
# Using a simple grep check before appending to avoid duplicates
sync_dependency() {
    local pkg_file=$1
    local dep=$2
    if ! grep -q "<exec_depend>$dep</exec_depend>" "$pkg_file"; then
        echo "➕ Adding $dep to $pkg_file"
        # Insert before the closing </package> tag
        sed -i '' "/<\/package>/i \\
  <exec_depend>$dep</exec_depend>" "$pkg_file"
    fi
}

[ -f "$BRAIN_PKG_DIR/package.xml" ] && sync_dependency "$BRAIN_PKG_DIR/package.xml" "python3-serial"
[ -f "$BRAIN_PKG_DIR/package.xml" ] && sync_dependency "$BRAIN_PKG_DIR/package.xml" "geometry_msgs"

# 2. Add Xacro support to chopper-description
[ -f "$DESC_PKG_DIR/package.xml" ] && sync_dependency "$DESC_PKG_DIR/package.xml" "xacro"
[ -f "$DESC_PKG_DIR/package.xml" ] && sync_dependency "$DESC_PKG_DIR/package.xml" "robot_state_publisher"

# 3. Create a professional Robot Launch file
# This version actually handles Xacro processing for the Orin Nano
mkdir -p "$DESC_PKG_DIR/launch"
cat <<EOF > "$DESC_PKG_DIR/launch/robot-launch.py"
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('chopper_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'chopper.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config.toxml()}]
        ),
        Node(
            package='chopper_brain',
            executable='spektrum_node',
            name='spektrum_bridge',
            output='screen'
        )
    ])
EOF

echo "✅ XML & Launch configs synced."