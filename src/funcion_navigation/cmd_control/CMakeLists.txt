cmake_minimum_required(VERSION 2.8.3)
project(cmd_control)

#set(CMAKE_CXX_FLAGS "-std=c++11 -Ofast")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  message_generation
  roscpp
  std_msgs
  message_filters
)
# Generate messages in the 'msg' folder
add_message_files(
   FILES
   keyDataOrParam.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

find_package(cmake_modules)


catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs message_runtime 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(cmd_control_node src/cmd_control.cpp)
target_link_libraries(cmd_control_node
    ${catkin_LIBRARIES}
    )

  add_executable(cmd_control_pad src/cmd_control_pad.cpp)
  target_link_libraries(cmd_control_pad
    ${catkin_LIBRARIES})
