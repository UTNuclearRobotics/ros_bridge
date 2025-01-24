FROM ros:noetic-ros-core-focal

# Install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    && rm -rf /var/lib/apt/lists/*

# Bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# Install ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# ROS1 custom message workspace
COPY /ros1_ws /ros1_ws
WORKDIR /ros1_ws 
RUN /bin/bash -c ". /opt/ros/noetic/setup.bash && catkin_make"

CMD ["roscore"]
