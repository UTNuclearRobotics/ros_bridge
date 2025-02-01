#!/bin/bash

set -e
echo "Starting Noetic <-> Humble ROS bridge..."

# Source ROS2 and bridge environments
source /opt/ros/$ROS2_DISTRO/setup.bash
cd /bridge_ws
source install/setup.bash

# Launch the bridge
ros2 run ros1_bridge dynamic_bridge
exec "$@"