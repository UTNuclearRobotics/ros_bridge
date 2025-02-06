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
 

## 1. Installation and Setup
1. Clone repository
   
   ```shell
   git clone -b noetic-humble git@github.com:UTNuclearRobotics/ros_bridge.git
   ```
   
### ROS BRIDGE Configuration
1. Adding Custom Messages & Services (Optional)
   
   > **Note:** The names of your packages with the custom messages and services MUST match everywhere. You MUST post append the package names with "`_msgs`".
   
   1. Place your ROS1 packages into `/ros1_ws/src/`
   2. Place your ROS2 packages into `/ros2_ws/src/`
      
      
      
3. Open `ros_bridge.env` and set your ROS1 and ROS2 environment variables to match your ROS1 and ROS2 environments.
      - `ROS_MASTER_URI`      - ROS1 master ip address
      - `ROS_DOMAIN_ID`       - ROS2 domain ID
      - `ROS_LOCALHOST_ONLY`  - 0 (False) or 1 (True)
  
4. Adjust CYCLONE DDS settings in `cyclonedds.yml`.
   > **Note:** This file should not need to be modified unless you have specific configurations you would like to use in with your current ROS2 environment.
   
### ROS 2 Environment Configuration
1. Ensure a replica of the `cyclonedds.yml` file exists in the ROS2 environment. Ensure the following environment variables in your ROS2 environment.
   
   - `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
   - `CYCLONEDDS_URI=<local_path>/cyclonedds.xml`
   - `ROS_DOMAIN_ID`       - Matching domain to the ID set in `ros_bridge.env`
   - `ROS_LOCALHOST_ONLY`  - Matching value to the variable set in `ros_bridge.env`


## 2. Build and Run
1. Build Docker image
   ```shell
   make build
   ```
2. Start your ROS1 master.
3. Run the ROS BRIDGE

   > **Note:** On container start the `ros_bridge` will run automatically. In your ROS1 and ROS2 terminals, you should now see the `/ros_bridge` node if you list all the running nodes. You do not need to do anything else.
   ```shell
   make start
   ```
4. See **Runtime Nuances and Best Practices** section at the top to understand the order in which to run publishers and subscribers.
   
## 3. Monitoring
1. Open Docker shell. Note this default shell will have ROS1 sourced already.
   
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

        > **Note:** You can run the following command using the alias `pairs`
        ```shell
        ros2 run ros1_bridge dynamic_bridge --print-pairs
        ```
