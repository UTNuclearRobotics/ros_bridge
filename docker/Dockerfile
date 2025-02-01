# This Dockerfile is derived and adapted from the following repositories:
# - https://github.com/smith-doug/ros1_bridge
# - https://github.com/TommyChangUMD/ros-humble-ros1-bridge-builder/

FROM ros:humble-ros-core-jammy

ENV DEBIAN_FRONTEND=noninteractive

ENV ROS1_DISTRO=noetic
ENV ROS2_DISTRO=humble

# Set bash as shell with pipefail and errexit flags for better error handling
SHELL ["/bin/bash", "-o", "pipefail", "-o", "errexit", "-c"]
# SHELL ["/bin/bash", "-c"]

# bring system up to the latest ROS desktop configuration
RUN apt-get -y update
RUN apt-get -y upgrade
RUN apt-get -y install \
    python3-colcon-common-extensions \
    vim \
    ros-$ROS2_DISTRO-desktop \
    ros-$ROS2_DISTRO-cyclonedds \
    ros-$ROS2_DISTRO-rmw-cyclonedds-cpp

# temporarily remove ROS2 apt repository
RUN mv /etc/apt/sources.list.d/ros2-latest.list /root/
RUN apt-get update

# comment out the catkin conflict
RUN sed  -i -e 's|^Conflicts: catkin|#Conflicts: catkin|' /var/lib/dpkg/status
RUN apt-get install -f

# force install these packages
RUN apt-get download python3-catkin-pkg
RUN apt-get download python3-rospkg
RUN apt-get download python3-rosdistro
RUN dpkg --force-overwrite -i python3-catkin-pkg*.deb
RUN dpkg --force-overwrite -i python3-rospkg*.deb
RUN dpkg --force-overwrite -i python3-rosdistro*.deb
RUN apt-get install -f

# install the latest ROS1 desktop configuration
#   see https://packages.ubuntu.com/jammy/ros-desktop-dev
#   note: ros-desktop-dev automatically includes tf tf2
RUN apt-get -y install ros-desktop-dev

# fix ARM64 pkgconfig path issue -- Fix provided by ambrosekwok
RUN if [[ $(uname -m) = "arm64" || $(uname -m) = "aarch64" ]]; then                     \
      cp /usr/lib/x86_64-linux-gnu/pkgconfig/* /usr/lib/aarch64-linux-gnu/pkgconfig/;   \
    fi

# restore the ROS2 apt repos and set compilation options.
RUN mv /root/ros2-latest.list /etc/apt/sources.list.d/
RUN apt-get -y update

# (ROS 1) First, build ROS 1 msgs & srvs:
COPY ros1_ws /ros1_ws
WORKDIR /ros1_ws
RUN bash -c "unset ROS_DISTRO && \
    time colcon build"
# RUN bash -c "unset ROS_DISTRO && \
#     time colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# (ROS 2) Second, build ROS 2 msgs & srvs:
COPY ros2_ws /ros2_ws
WORKDIR /ros2_ws
RUN source /opt/ros/humble/setup.bash && \
    colcon build
# RUN source /opt/ros/humble/setup.bash && \
#     time colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# (ROS 1 and ROS 2) Last, clone and build the ros1 to ros2 bridge:
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

# clean up the apt cache
RUN apt-get -y clean all; apt-get -y update

# copy the ros1 libraries to the bridge
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

# add aliases to the bashrc
RUN echo "alias sws='source /bridge_ws/install/setup.bash'" >> ~/.bashrc
RUN echo "alias pairs='ros2 run ros1_bridge dynamic_bridge --print-pairs'" >> ~/.bashrc

# copy the cyclonedds config file
COPY .zbashrc_cyclonedds.xml /root/.zbashrc_cyclonedds.xml

# Copy the entrypoint into the image
COPY ./docker/entrypoint.sh /entrypoint.sh

# Run this script on startup
ENTRYPOINT /entrypoint.sh
