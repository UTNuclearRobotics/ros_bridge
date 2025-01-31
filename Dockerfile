# Adopted from the following:
#   https://github.com/smith-doug/ros1_bridge
#   https://github.com/TommyChangUMD/ros-humble-ros1-bridge-builder/


FROM ros:humble-ros-core-jammy

# SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# # Setup ROS1 sources.list and keys
# RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
# RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

ENV ROS1_DISTRO=noetic
ENV ROS2_DISTRO=humble

# This sets the shell to bash with error handling flags:
# - pipefail: Makes a pipeline fail if any command in it fails
# - errexit: Makes the script exit immediately if a command fails
# This allows using ; instead of && to chain commands since errors will be caught automatically
SHELL ["/bin/bash", "-o", "pipefail", "-o", "errexit", "-c"]

###########################
# 1.) Bring system up to the latest ROS desktop configuration
###########################
RUN apt-get -y update
RUN apt-get -y upgrade
RUN apt-get -y install ros-humble-desktop python3-colcon-common-extensions

###########################
# 2.) Temporarily remove ROS2 apt repository
###########################
RUN mv /etc/apt/sources.list.d/ros2-latest.list /root/
RUN apt-get update

###########################
# 3.) comment out the catkin conflict
###########################
RUN sed  -i -e 's|^Conflicts: catkin|#Conflicts: catkin|' /var/lib/dpkg/status
RUN apt-get install -f

###########################
# 4.) force install these packages
###########################
RUN apt-get download python3-catkin-pkg
RUN apt-get download python3-rospkg
RUN apt-get download python3-rosdistro
RUN dpkg --force-overwrite -i python3-catkin-pkg*.deb
RUN dpkg --force-overwrite -i python3-rospkg*.deb
RUN dpkg --force-overwrite -i python3-rosdistro*.deb
RUN apt-get install -f

###########################
# 5.) Install the latest ROS1 desktop configuration
# see https://packages.ubuntu.com/jammy/ros-desktop-dev
# note: ros-desktop-dev automatically includes tf tf2
###########################
RUN apt-get -y install ros-desktop-dev

# fix ARM64 pkgconfig path issue -- Fix provided by ambrosekwok
RUN if [[ $(uname -m) = "arm64" || $(uname -m) = "aarch64" ]]; then                     \
      cp /usr/lib/x86_64-linux-gnu/pkgconfig/* /usr/lib/aarch64-linux-gnu/pkgconfig/;   \
    fi

###########################
# 6.) Restore the ROS2 apt repos and set compilation options.
#   For example, to include ROS tutorial message types, pass
#   "--build-arg ADD_ros_tutorials=1" to the docker build command.
###########################
RUN mv /root/ros2-latest.list /etc/apt/sources.list.d/
RUN apt-get -y update

# (ROS 1) First, build ROS 1 messages:
COPY ros1_ws /ros1_ws
WORKDIR /ros1_ws
RUN bash -c "unset ROS_DISTRO && \
    time colcon build"
# RUN unset ROS_DISTRO && \
#     time colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# (ROS 2) Then build the ROS 2 messages:
COPY ros2_ws /ros2_ws
WORKDIR /ros2_ws
RUN source /opt/ros/humble/setup.bash && \
    colcon build
# RUN source /opt/ros/humble/setup.bash && \
#     time colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# # Install ROS1 debian packages
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     ros-${ROS1_DISTRO}-ros-comm \
#     ros-${ROS1_DISTRO}-sensor-msgs \
#     ros-${ROS1_DISTRO}-geometry-msgs \
#     ros-${ROS1_DISTRO}-std-msgs \
#     ros-${ROS1_DISTRO}-tf2-msgs \
#     ros-${ROS1_DISTRO}-tf2 \
#     ros-${ROS1_DISTRO}-tf2-ros \
#     ros-${ROS1_DISTRO}-nav-msgs \
#     && rm -rf /var/lib/apt/lists/*

# # Install ROS2 debian packages
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     ros-${ROS2_DISTRO}-sensor-msgs \
#     ros-${ROS2_DISTRO}-geometry-msgs \
#     ros-${ROS2_DISTRO}-std-msgs \
#     ros-${ROS2_DISTRO}-tf2-msgs \
#     ros-${ROS2_DISTRO}-nav-msgs \
#     ros-${ROS2_DISTRO}-rmw-fastrtps-cpp \
#     ros-${ROS2_DISTRO}-rmw-cyclonedds-cpp \
#     ros-${ROS2_DISTRO}-rmw \
#     qtbase5-dev \
#     && rm -rf /var/lib/apt/lists/*

# # Add additional ROS1/ROS2 message packages here
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     # ros-${ROS1_DISTRO}-something-msgs \
#     # ros-${ROS2_DISTRO}-something-msgs \
#     && rm -rf /var/lib/apt/lists/*

# # (ROS 1) First, build ROS 1 messages:
# COPY ros1_ws /ros1_ws
# WORKDIR /ros1_ws
# RUN source /opt/ros/${ROS1_DISTRO}/setup.bash && \
#     catkin_make_isolated --install

# # (ROS 2) Then build the ROS 2 messages:
# COPY ros2_ws /ros2_ws
# WORKDIR /ros2_ws
# RUN source /opt/ros/${ROS2_DISTRO}/setup.bash && \
#     colcon build --packages-select tdm_msgs

# (ROS 1 and ROS 2) Finally, build the bridge:
RUN mkdir -p /bridge_ws/src
WORKDIR /bridge_ws/src
RUN git clone -b action_bridge_humble https://github.com/smith-doug/ros1_bridge.git;
WORKDIR /bridge_ws
RUN source /ros1_ws/install/setup.bash && \
    source /ros2_ws/install/setup.bash && \
    MEMG=$(printf "%.0f" $(free -g | awk '/^Mem:/{print $2}')) && \
    NPROC=$(nproc) && \
    MIN=$((MEMG<NPROC ? MEMG : NPROC)) && \
    echo "Please wait...  running $MIN concurrent jobs to build ros1_bridge" && \
    time MAKEFLAGS="-j $MIN" colcon build --event-handlers console_direct+

RUN apt-get -y clean all; apt-get -y update

RUN ROS1_LIBS="libxmlrpcpp.so";                                                 \
     ROS1_LIBS="$ROS1_LIBS librostime.so";                                      \
     ROS1_LIBS="$ROS1_LIBS libroscpp.so";                                       \
     ROS1_LIBS="$ROS1_LIBS libroscpp_serialization.so";                         \
     ROS1_LIBS="$ROS1_LIBS librosconsole.so";                                   \
     ROS1_LIBS="$ROS1_LIBS librosconsole_log4cxx.so";                           \
     ROS1_LIBS="$ROS1_LIBS librosconsole_backend_interface.so";                 \
     ROS1_LIBS="$ROS1_LIBS liblog4cxx.so";                                      \
     ROS1_LIBS="$ROS1_LIBS libcpp_common.so";                                   \
     ROS1_LIBS="$ROS1_LIBS libb64.so";                                          \
     ROS1_LIBS="$ROS1_LIBS libaprutil-1.so";                                    \
     ROS1_LIBS="$ROS1_LIBS libapr-1.so";                                        \
     ROS1_LIBS="$ROS1_LIBS libactionlib.so.1d";                                 \
     cd /bridge_ws/install/ros1_bridge/lib;                        \
     for soFile in $ROS1_LIBS; do                                               \
       soFilePath=$(ldd libros1_bridge.so | grep $soFile | awk '{print $3;}');  \
       cp $soFilePath ./;                                                       \
     done

# RUN echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
RUN echo "alias sws='source /bridge_ws/install/setup.bash'" >> ~/.bashrc
RUN echo "alias bridge_pairs='ros2 run ros1_bridge dynamic_bridge --print-pairs'" >> ~/.bashrc

COPY .zbashrc_cyclonedds.xml /ros2_ws/.zbashrc_cyclonedds.xml

# Update apt cache, install vim, git, and ssh, and then, remove the apt cache
RUN apt-get update && \
    apt-get install -y vim ros-$ROS2_DISTRO-cyclonedds ros-$ROS2_DISTRO-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# # Build workspace and ros1_bridge from source
# RUN source /opt/ros/${ROS1_DISTRO}/setup.bash && \
#     source /opt/ros/${ROS2_DISTRO}/setup.bash && \
#     source /ros1_ws/devel/setup.bash && \
#     source /ros2_ws/install/setup.bash && \
#     # source /bridge_ws/install/setup.bash && \


RUN echo "Hola, amigo!"
# Copy the entrypoint into the image
COPY ./entrypoint.sh /entrypoint.sh

# Run this script on startup
ENTRYPOINT /entrypoint.sh
