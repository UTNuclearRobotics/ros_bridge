## ROS Bridge Docker (Noetic <-> Galactic)

Running this Docker container will spawn a `ros_bridge` node to generate mappings for messages between ROS1 and ROS2.

*** This branch bridges ROS1 noetic with ROS2 galactic. If you need to bridge ROS1 noetic to ROS2 humble, see the `noetic-humble` branch.

### Prerequisites

- A topic is being published on side A. The message type of this topic should be known inside the Docker container (list below).
- At least one subscriber listening for the topic on side B. You will most likely have to make a dummy script for testing.

### Steps to Start the ROS Bridge

1. Source `bash_utils`:
    ```bash
    source bash_utils
    ```
2. Build the ROS bridge:
    ```bash
    ros_bridge_build
    ```
3. Start the ROS bridge:
    ```bash
    ros_bridge_start
    ```
4. Verify the node is running:
    ```bash
    ros2 node list
    ```
    You should see the `ros_bridge` node listed.
5. When done, stop the ROS bridge:
    ```bash
    ros_bridge_stop
    ```

### Notes

- There is no need to perform any actions inside the Docker container's shell.
- The `ros_bridge` can be started before or after the topics are being published. As long as the prerequisites are met, messages will be correctly bridged between ROS1 and ROS2.

### Message Types

- `std_msgs` (default with ROS)
- `nav_msgs`
- `sensor_msgs`
- `tf2_msgs`

### Branch

Feel free to create a branch for custom message types for your project. There are comments for you to follow in the source code.
