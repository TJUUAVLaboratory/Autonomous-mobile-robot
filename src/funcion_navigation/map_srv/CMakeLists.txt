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
  image_transport
  cv_bridge
)
# Generate messages in the 'msg' folder
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# file(GLOB jsoncpp_source  lib/cjson/cJSON.c)
# add_library(cjson  ${jsoncpp_source})


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
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs message_runtime   image_transport
  cv_bridge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/lib/cjson
)

add_executable(fake_odom_node src/fake_odom_test.cpp)
target_link_libraries(fake_odom_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}  cjson
    )

## 使用pad 预约点位
add_executable(check_odom_node src/check_odom_position.cpp)
target_link_libraries(check_odom_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}  cjson
    )


## load costmap and click move to goal 订阅 /move_base_simple/goal
add_executable(click_move_to_goal src/click_move_to_goal.cpp)
target_link_libraries(click_move_to_goal
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}  cjson
    )