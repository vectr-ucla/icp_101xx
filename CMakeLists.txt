cmake_minimum_required(VERSION 2.8.3)
project(icp_101xx)

find_package(catkin 
  REQUIRED COMPONENTS 
  roscpp
  std_msgs
  message_generation
)

## Declare ROS messages, services and actions
add_message_files(
  FILES
    Barometer.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS src 
  LIBRARIES icp_101xx
  CATKIN_DEPENDS 
    roscpp
    message_runtime
    std_msgs
)

include_directories(
  src
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-std=c++11)

# Declare a C++ executable
add_executable(icp_101xx_node ros/node/main.cc ros/node/RosManager.cc ros/node/serial.cc)
add_dependencies(icp_101xx_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(icp_101xx_node ${catkin_LIBRARIES})
