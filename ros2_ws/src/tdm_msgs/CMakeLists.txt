cmake_minimum_required(VERSION 3.8)
project(tdm_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

install(
  FILES my_mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME})

# generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ImageToSeg3dPrompt.srv"
  DEPENDENCIES 
  sensor_msgs 
  geometry_msgs 
  tf2_msgs
)

ament_package()
