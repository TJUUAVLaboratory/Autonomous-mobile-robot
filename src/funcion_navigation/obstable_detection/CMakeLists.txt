cmake_minimum_required(VERSION 2.8.3)
project(obstable_detection)

set(CMAKE_CXX_FLAGS "-std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation  ## custom message
  geometry_msgs
  std_msgs
  move_base_msgs
  sensor_msgs
  laser_geometry
  roscpp
  tf
  pcl_conversions
  pcl_ros 
)



find_package(PCL REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIR})




add_message_files(
  FILES
  obstable_detection_msg.msg
)



# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mrobot_navigation
 CATKIN_DEPENDS geometry_msgs  std_msgs move_base_msgs roscpp rospy tf visualization_msgs
 laser_geometry
 message_runtime  ## custom message
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS},
  INCLUDE_DIRS include
  ${PROJECT_SOURCE_DIR}/lib/cjson

)


## Declare a C++ library
# add_library(mrobot_navigation
#   src/${PROJECT_NAME}/mrobot_navigation.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(mrobot_navigation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

file(GLOB jsoncpp_source  lib/cjson/cJSON.c)

add_library(cjson  ${jsoncpp_source})


## Declare a C++ executable
add_executable(obstable_detection_node src/obstable_detection.cpp)

target_link_libraries(obstable_detection_node
  ${catkin_LIBRARIES}  cjson
)

