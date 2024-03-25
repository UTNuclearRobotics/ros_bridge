FROM ros:galactic-ros-base-focal

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# setup ros1 sources.list and keys 
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list 
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 

ENV ROS1_DISTRO noetic 
ENV ROS2_DISTRO galactic 

# install ROS1 debian packages 
RUN apt-get update && apt-get install -y --no-install-recommends \ 
    ros-noetic-ros-comm \
    ros-noetic-sensor-msgs \
    ros-noetic-tf2-msgs \
    ros-noetic-nav-msgs \
    && rm -rf /var/lib/apt/lists/* 

# install ROS2 debian packages 
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-galactic-sensor-msgs \
    ros-galactic-tf2-msgs \
    ros-galactic-nav-msgs \
    qtbase5-dev \
    ros-${ROS2_DISTRO}-rmw-fastrtps-cpp \
    ros-${ROS2_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS2_DISTRO}-rmw \
    && rm -rf /var/lib/apt/lists/* 

# clone bridge source code and build from source 
RUN mkdir -p /bridge_ws/src 
WORKDIR /bridge_ws/src 
RUN git clone -b ${ROS2_DISTRO} https://github.com/ros2/ros1_bridge.git 
WORKDIR /bridge_ws 
RUN source /opt/ros/${ROS1_DISTRO}/setup.bash && \ 
    source /opt/ros/${ROS2_DISTRO}/setup.bash && \
    colcon build --symlink-install --packages-skip ros1_bridge 

# build workspace and ros1_bridge from source
RUN source /opt/ros/${ROS1_DISTRO}/setup.bash && \ 
    source /opt/ros/${ROS2_DISTRO}/setup.bash && \ 
    source install/setup.bash && \ 
    colcon build --packages-select ros1_bridge --cmake-force-configure

# copy the entrypoint into the image
COPY ./entrypoint.sh /entrypoint.sh

# run this script on startup
ENTRYPOINT /entrypoint.sh