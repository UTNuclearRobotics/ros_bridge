cmake_minimum_required(VERSION 3.0.2)
project(ros1_custom_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  # Add your message dependencies here
)

add_message_files(
  FILES
  MyMessage.msg
  # Add your message files here
)

generate_messages(
  DEPENDENCIES
  std_msgs
  # Add your message dependencies here
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
