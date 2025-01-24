FROM ros:galactic-ros-base-focal

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Setup ROS1 sources.list and keys 
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list 
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 

ENV ROS1_DISTRO noetic 
ENV ROS2_DISTRO galactic 

# Install ROS1 debian packages 
RUN apt-get update && apt-get install -y --no-install-recommends \ 
    ros-${ROS1_DISTRO}-ros-comm \
    ros-${ROS1_DISTRO}-sensor-msgs \
    ros-${ROS1_DISTRO}-tf2-msgs \
    ros-${ROS1_DISTRO}-nav-msgs \
    && rm -rf /var/lib/apt/lists/* 

# Install ROS2 debian packages 
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS2_DISTRO}-sensor-msgs \
    ros-${ROS2_DISTRO}-tf2-msgs \
    ros-${ROS2_DISTRO}-nav-msgs \
    ros-${ROS2_DISTRO}-rmw-fastrtps-cpp \
    ros-${ROS2_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS2_DISTRO}-rmw \
    qtbase5-dev \
    && rm -rf /var/lib/apt/lists/*

# Add additional ROS1/ROS2 message packages here
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ros-${ROS1_DISTRO}-something-msgs \
    # ros-${ROS2_DISTRO}-something-msgs \
    && rm -rf /var/lib/apt/lists/*

# ROS1 custom message workspace
COPY ros1_ws /ros1_ws
WORKDIR /ros1_ws
RUN source /opt/ros/${ROS1_DISTRO}/setup.bash && \
    catkin_make

# ROS2 custom message workspace
COPY ros2_ws /ros2_ws
WORKDIR /ros2_ws
RUN source /opt/ros/${ROS2_DISTRO}/setup.bash && \
    colcon build

# Clone bridge source code and build from source 
RUN mkdir -p /bridge_ws/src 
WORKDIR /bridge_ws/src 
RUN git clone -b ${ROS2_DISTRO} https://github.com/ros2/ros1_bridge.git 
WORKDIR /bridge_ws 
RUN source /opt/ros/${ROS1_DISTRO}/setup.bash && \ 
    source /opt/ros/${ROS2_DISTRO}/setup.bash && \
    colcon build --symlink-install --packages-skip ros1_bridge 

# Build workspace and ros1_bridge from source
RUN source /opt/ros/${ROS1_DISTRO}/setup.bash && \ 
    source /opt/ros/${ROS2_DISTRO}/setup.bash && \ 
    source install/setup.bash && \ 
    colcon build --packages-select ros1_bridge --cmake-force-configure

# Copy the entrypoint into the image
COPY ./entrypoint.sh /entrypoint.sh

# Run this script on startup
ENTRYPOINT /entrypoint.sh