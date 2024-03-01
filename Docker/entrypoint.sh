#!/bin/bash

set -e
echo "ROS1 bridge entrypoint"

# source ROS2 and bridge environments
source "/opt/ros/$ROS2_DISTRO/setup.bash"
cd /bridge_ws
source install/setup.bash

# display bridged messages and launch the bridge
ros2 run ros1_bridge dynamic_bridge --print-pairs
ros2 run ros1_bridge dynamic_bridge
exec "$@"