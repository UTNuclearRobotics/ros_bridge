#!/bin/bash

set -e
echo "ROS1 bridge entrypoint"

# Source ROS2 and bridge environments
source /opt/ros/$ROS2_DISTRO/setup.bash
cd /bridge_ws
source install/setup.bash

# HUMBLE ADDS
# source "/opt/ros/$ROS2_DISTRO/setup.bash"
# source /ros2_ws/install/local_setup.bash
# cd /bridge_ws
# source install/setup.bash

# source /opt/ros/$ROS1_DISTRO/setup.bash
# source /opt/ros/$ROS2_DISTRO/setup.bash
# cd /bridge_ws
# source install/setup.bash

# Launch the bridge
ros2 run ros1_bridge dynamic_bridge
exec "$@"