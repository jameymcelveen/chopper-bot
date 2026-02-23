# --- 1. Fix Package Structure (Missing boilerplate) ---
BRAIN_PKG_DIR="$REPO_ROOT/src/chopper-brain"
BRAIN_PYTHON_DIR="$BRAIN_PKG_DIR/chopper_brain"

echo "📦 Reconstructing ROS 2 package structure..."

# Create the internal python package directory
mkdir -p "$BRAIN_PYTHON_DIR"

# Move the loose nodes into the internal package directory (idempotent)
for node in "$BRAIN_PKG_DIR"/*_node.py; do
    if [ -f "$node" ]; then
        mv "$node" "$BRAIN_PYTHON_DIR/"
        echo "🚚 Moved $(basename "$node") to internal package folder."
    fi
done

# Ensure __init__.py exists for Python imports
touch "$BRAIN_PYTHON_DIR/__init__.py"

# 2. Re-generate setup.py if missing
if [ ! -f "$BRAIN_PKG_DIR/setup.py" ]; then
    echo "📄 Creating missing setup.py..."
    cat <<EOF > "$BRAIN_PKG_DIR/setup.py"
from setuptools import setup

package_name = 'chopper_brain'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    description='Chopper Brain Nodes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'spektrum_node = chopper_brain.spektrum_node:main',
            'motor_node = chopper_brain.motor_node:main',
            'heartbeat_node = chopper_brain.heartbeat_node:main',
        ],
    },
)
EOF
    mkdir -p "$BRAIN_PKG_DIR/resource"
    touch "$BRAIN_PKG_DIR/resource/chopper_brain"
fi

# 3. Re-generate package.xml if missing
if [ ! -f "$BRAIN_PKG_DIR/package.xml" ]; then
    echo "📄 Creating missing package.xml..."
    cat <<EOF > "$BRAIN_PKG_DIR/package.xml"
<?xml version="1.0"?>
<package format="3">
  <name>chopper_brain</name>
  <version>0.0.1</version>
  <description>Nodes for Chopper</description>
  <maintainer email="dev@todo.todo">Developer</maintainer>
  <license>Apache-2.0</license>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>python3-serial</exec_depend>
  <export><build_type>ament_python</build_type></export>
</package>
EOF
fi