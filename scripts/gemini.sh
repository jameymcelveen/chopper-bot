#!/bin/bash

# --- Robust Path Discovery ---
# Get the directory where THIS script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# The root is one level up
REPO_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"

# Force source using the absolute path we just found
if [ -f "$SCRIPT_DIR/utils.sh" ]; then
    source "$SCRIPT_DIR/utils.sh"
else
    echo "❌ Error: Cannot find $SCRIPT_DIR/utils.sh"
    exit 1
fi

cd "$REPO_ROOT"

# --- 2020 Skeleton URDF Update ---

URDF_FILE="src/chopper-description/urdf/chopper.urdf.xacro"

if [ -f "$URDF_FILE" ]; then
    echo "🏗️  Updating URDF with 2020 Extrusion dimensions..."
    # We will define a 300mm x 300mm base for Chopper
    cat <<EOF > "$URDF_FILE"
<?xml version="1.0"?>
<robot name="chopper" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <material name="aluminum"><color rgba="0.5 0.5 0.5 1.0"/></material>
  <material name="orange"><color rgba="1.0 0.5 0.0 1.0"/></material>

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.05"/> </geometry>
      <material name="aluminum"/>
    </visual>
  </link>

  <link name="head_link">
    <visual>
      <geometry>
        <cylinder radius="0.15" length="0.2"/>
      </geometry>
      <material name="orange"/>
    </visual>
  </link>

  <joint name="head_joint" type="fixed">
    <parent link="base_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 0.125" rpy="0 0 0"/>
  </joint>

</robot>
EOF
fi

COMMIT_MSG="feat: define 2020 aluminum frame dimensions in URDF"
copy_to_clipboard "$COMMIT_MSG"