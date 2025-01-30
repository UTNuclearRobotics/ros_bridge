Running this Docker container will spawn a `ros1_bridge` node to generate mappings for messages between ROS1 and ROS2. You do not need to do anything inside the shell of the Docker. Anytime the prerequisites are met for a topic, the messages will be seen between ROS1 and ROS2.

**Prerequisites:**
- A topic is being published on side A. The message type of this topic should be known inside the Docker container (list below).
- At least 1 subscriber listening for the topic on side B. You will most likely have to make a dumby script for testing.

**Message types:**
- `std_msgs` (default with ROS)
- `nav_msgs`
- `sensor_msgs`
- `tf2_msgs`

To add more message types, make sure the message type is installed in the Docker image. For custom messages, see the README.md in https://github.com/ros2/ros1_bridge.
