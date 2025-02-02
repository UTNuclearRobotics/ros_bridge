# ROS Bridge (Noetic <-> Humble)
Running this Docker container will spawn a `ros1_bridge` node to generate mappings for messages between ROS1 and ROS2. You do not need to do anything inside the shell of the Docker. Anytime the prerequisites are met for a topic, the messages will be seen between ROS1 and ROS2.

**Msg & Srv Types Available out of the Box**
- `std_msgs` 
- `nav_msgs`
- `sensor_msgs`
- `tf2_msgs`

**Runtime Nuances and Best Practices**
- **Custom Msgs & Srvs**
    - The packages _MUST_ have the exact same package name and be post fixed with "`_msgs`".
    - For Example:
      - ROS1 Pkg Name: `my_bridged_pkg_msgs`
      - ROS2 Pkg Name: `my_bridged_pkg_msgs`
- **Publishing Msgs from ROS2 ➡ ROS1**
    - You need to start a ROS1 subscriber node that subscribes to the exact topic name and type you plan to publish from ROS2 _BEFORE_ you start the ROS2 publisher
 

## 1. Installation and Configuration
1. Clone repository
   
   ```shell
   git clone -b noetic-humble git@github.com:UTNuclearRobotics/ros_bridge.git
   ```
2. Add custom msgs & srvs (Optional)
   1. Place your ROS1 and ROS2 packages into `/ros1_ws/src/` and `/ros2_ws/src`, respectively.
      
      > _**Note:** The names of your custom packages MUST match and you MUST post append the package names with "`_msgs`"._
      
3. Set ROS Environment Variables
   1. Open `ros_bridge.env` environment variables file with your favorite editor.
   2. Set the following ROS1 and ROS2 variables to match your ROS1 and ROS2 environment settings you are bridging to.
      - `ROS_MASTER_URI` - ROS1 master ip address
      - `ROS_DOMAIN_ID`  - ROS2 domain ID
      - `ROS_LOCALHOST_ONLY` - 0 for false and 1 for true
        
4. Adjust CYCLONE DDS settings in `cyclonedds.yml` if neccessary.
5. Copy / Paste the `cyclonedds.yml` configuration file into your ROS2 workspace.

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
