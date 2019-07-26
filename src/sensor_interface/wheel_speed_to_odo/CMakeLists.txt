cmake_minimum_required(VERSION 2.8.3)
project(wheel_speed_odo)

set(CMAKE_CXX_STANDARD 11)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs sensor_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  ## wheelSpeed.msg
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  message_filters
  eigen_conversions
  robot_msgs
  tf
)
catkin_package(
    INCLUDE_DIRS 
    CATKIN_DEPENDS message_runtime std_msgs sensor_msgs geometry_msgs tf2 tf2_eigen
)
include_directories(
  src/
  ${catkin_INCLUDE_DIRS}
)
add_executable(wheel_speed_odo src/wheel_odo.cpp)
target_link_libraries(wheel_speed_odo
  ${catkin_LIBRARIES}
)
