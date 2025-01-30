# ROS Bridge (Noetic <-> Humble)
Running this Docker container will spawn a `ros1_bridge` node to generate mappings for messages between ROS1 and ROS2. You do not need to do anything inside the shell of the Docker. Anytime the prerequisites are met for a topic, the messages will be seen between ROS1 and ROS2.

**Msg & Srv Types Available out of the Box**
- `std_msgs` 
- `nav_msgs`
- `sensor_msgs`
- `tf2_msgs`

**Prerequisites for Running**
- A topic is being published on side A. The message type of this topic should be known inside the Docker container (list below).
- At least 1 subscriber listening for the topic on side B. You will most likely have to make a dumby script for testing.

## 1. Installation and Configuration
1. Clone repository
   
   ```shell
   git clone -b noetic-humble git@github.com:UTNuclearRobotics/ros_bridge.git
   ```
3. Add custom msgs & srvs (Optional)
   1. Copy/Paste your ROS1 and ROS2 packages into `/ros1_ws/src/` and `/ros2_ws/src`, respectively.
      
      > _**Note:** The names of your custom packages MUST match and you MUST post append the package names with `_msgs`._

## 2. Build and Run
1. Build Docker image
   ```shell
   make build
   ```
2. Start Docker container
   ```shell
   make start
   ```

   > **Note:** On container start the `ros_bridge` will run automatically. In your ROS1 and ROS2 terminals, you should now see the `/ros_bridge` node if you list all the running nodes. You do not need to do anything else.
   
## 3. Monitoring
1. Open Docker shell
   
   ```shell
   make shell
   ```
3. 🐳 View the services and messages that are being bridged.
   1. Source the ROS2 workspace in the container
      
       > **Note:** You can run the following command using the alias `sws`
       ```shell
       source /bridge_ws/install/setup.bash
       ```
       
   2. Print all bridged msgs & srvs

        > **Note:** You can run the following command using the alias `bridge_pairs`
        ```shell
        ros2 run ros1_bridge dynamic_bridge --print-pairs
        ```
