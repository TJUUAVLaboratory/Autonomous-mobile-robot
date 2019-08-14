cmake_minimum_required(VERSION 2.8.3)
project(sonars_to_pointcloud)

## 编译选项
set(CMAKE_CXX_FLAGS "-std=c++11 -g -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wno-reorder" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -DNDEBUG -Wno-reorder -O2" CACHE STRING "" FORCE)

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  message_filters
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)

catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS laser_geometry message_filters  roscpp sensor_msgs tf2 tf2_ros tf2_sensor_msgs
)

include_directories(
  
  ${catkin_INCLUDE_DIRS}
)

add_executable(sonars_to_pointcloud src/sonars_to_pointcloud.cpp)
target_link_libraries(sonars_to_pointcloud ${catkin_LIBRARIES})


add_executable(rplidar_to_pointcloud src/rplidar_to_pointcloud.cpp)
target_link_libraries(rplidar_to_pointcloud ${catkin_LIBRARIES})
