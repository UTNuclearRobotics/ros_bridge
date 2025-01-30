cmake_minimum_required(VERSION 3.0.2)
project(tdm_msgs)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  tf2_msgs
  message_generation
)

# Add the service files
add_service_files(
  FILES
  ImageToSeg3dPrompt.srv
)

# Generate services
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  tf2_msgs
  geometry_msgs
)

# catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES speech_recognition
#  CATKIN_DEPENDS audio_common_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


