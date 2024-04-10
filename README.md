Running this Docker container will spawn a `ros1_bridge` node to generate mappings for messages between ROS1 and ROS2. Make sure you have at least a ROS2 subscriber listening for the ROS1 topic for it to show up in your `ros2 topic list`.

**Message types:**
- `nav_msgs`
- `sensor_msgs`
- `tf2_msgs`
