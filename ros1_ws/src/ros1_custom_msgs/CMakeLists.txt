cmake_minimum_required(VERSION 3.0.2)
project(ros1_custom_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  rospy
)

add_message_files(
  FILES
  MyMessage.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

catkin_install_python(PROGRAMS
  scripts/publisher.py
  scripts/subscriber.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)