cmake_minimum_required(VERSION 2.8.3)
project(aibee_navi)
set(CMAKE_CXX_FLAGS "-std=c++11")
find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        rospy
        tf
        geometry_msgs
        visualization_msgs
        message_generation
        pluginlib  ## load ros plugin
        actionlib  ## move base action
        move_base_msgs
        nav_msgs

        ## global navi
        nav_core
        navfn


)
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})



catkin_package(
    CATKIN_DEPENDS
        geometry_msgs
        sensor_msgs
        nav_msgs
        roscpp
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# move_base
add_library(aibee_navi
  src/aibee_navi.cpp
)
target_link_libraries(aibee_navi
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )
add_dependencies(aibee_navi ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(aibee_navi_node
  src/aibee_navi_node.cpp
)
add_dependencies(aibee_navi_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(aibee_navi_node aibee_navi)
set_target_properties(aibee_navi_node PROPERTIES OUTPUT_NAME aibee_navi) #生成同名称的动态库和静态库

