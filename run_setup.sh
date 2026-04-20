#!/bin/bash
# Run this script INSIDE the Docker container from ~/meridian_ws.
# The container should have ROS 2 Jazzy and MuJoCo installed.
set -eo pipefail

# Inside Docker, ~/meridian_ws is the workspace root.
# This script assumes it's run from that directory.
WS="$(cd "$(dirname "$0")" && pwd)"
ASSETS=$WS/src/meridian_description/assets
SCRIPTS=$WS/src/meridian_sim/scripts

source /opt/ros/jazzy/setup.bash

echo "================================================================"
echo "Step 1: colcon build"
echo "================================================================"
cd $WS
colcon build --symlink-install
echo "colcon build: OK"

echo ""
echo "================================================================"
echo "Step 2: Install UR description + generate URDF"
echo "================================================================"
if ! dpkg -l ros-jazzy-ur-description &>/dev/null; then
  apt-get install -y ros-jazzy-ur-description
fi
mkdir -p $ASSETS
ros2 run xacro xacro \
  /opt/ros/jazzy/share/ur_description/urdf/ur.urdf.xacro \
  ur_type:=ur5e \
  name:=ur5e \
  > $ASSETS/ur5e.urdf
echo "URDF written: $ASSETS/ur5e.urdf ($(wc -l < $ASSETS/ur5e.urdf) lines)"

echo ""
echo "================================================================"
echo "Step 3: Convert URDF -> MJCF"
echo "================================================================"
python3 $SCRIPTS/convert_urdf_to_mjcf.py

echo ""
echo "================================================================"
echo "Step 4: Augment MJCF"
echo "================================================================"
python3 $SCRIPTS/augment_mjcf.py

echo ""
echo "================================================================"
echo "Step 5: Test MJCF"
echo "================================================================"
DISPLAY= python3 $SCRIPTS/test_mjcf.py

echo ""
echo "================================================================"
echo "All steps complete"
echo "================================================================"
