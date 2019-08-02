cmake_minimum_required(VERSION 2.8.3)
project(cmd_control)

#set(CMAKE_CXX_FLAGS "-std=c++11 -Ofast")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  message_generation
  move_base_msgs
  actionlib
  roscpp
  std_msgs
  message_filters
)
# Generate messages in the 'msg' folder
add_message_files(
   FILES
   keyDataOrParam.msg
 )

add_service_files(
  FILES
  cmd_control.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

find_package(cmake_modules)


catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs message_runtime 
  move_base_msgs  actionlib
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

    add_executable(auto_move_node src/auto_move_node.cpp)
    target_link_libraries(auto_move_node
    ${catkin_LIBRARIES})


    add_executable(data_record_node src/data_record_node.cpp)
    target_link_libraries(data_record_node
    ${catkin_LIBRARIES})


