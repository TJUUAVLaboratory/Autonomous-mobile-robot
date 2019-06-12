cmake_minimum_required(VERSION 2.8.3)
project(map_srv)


#set(CMAKE_CXX_FLAGS "-std=c++11 -Ofast")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  message_filters
)
# Generate messages in the 'msg' folder


add_service_files(
  FILES
  mapSave.srv
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

add_executable(map_srv_node src/map_srv.cpp)
target_link_libraries(map_srv_node
    ${catkin_LIBRARIES}
    )



