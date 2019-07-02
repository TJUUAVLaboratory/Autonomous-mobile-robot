cmake_minimum_required(VERSION 2.8.3)
project(global_navi)
set(CMAKE_CXX_FLAGS "-std=c++11")
find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        rospy
        tf
        geometry_msgs
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
add_library(global_navi
  src/global_navi.cpp
)
target_link_libraries(global_navi
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )
add_dependencies(global_navi ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(global_navi_node
  src/global_navi_node.cpp
)
add_dependencies(global_navi_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(global_navi_node global_navi)
set_target_properties(global_navi_node PROPERTIES OUTPUT_NAME global_navi) #生成同名称的动态库和静态库

