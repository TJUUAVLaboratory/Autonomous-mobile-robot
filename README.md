<!-- vscode-markdown-toc -->
* 1. [仿真环境与交换(mrobot_gazebo/mrobot_description)](#mrobot_gazebomrobot_description)
* 2. [perception](#perception)
* 3. [mapping](#mapping)
	* 3.1. [map_server](#map_server)
* 4. [navigation](#navigation)
* 5. [mrobot_navigation](#mrobot_navigation)

<!-- vscode-markdown-toc-config
	numbering=true
	autoSave=true
	/vscode-markdown-toc-config -->
<!-- /vscode-markdown-toc -->



# Autonomous-mobile-robot

> 使用`ROS+gazebo`搭建一个 室内/室外的轮式机器人的仿真场景，然后集成`ROS Navigation`机器人感知导航算法，然后不断地完善和丰富移动机器人的功能，比如：
- 可以使用多种视觉/雷达/轮盘传感器 来做机器人的定位和状态估计
- 可以完善mave_base中的global_costmap global_planner, local_costmap local_planner的功能
- 完善机器人建图的功能以及 map server
- 完善机器人的 exploration自主探索陌生环境的能力


室内移动机器人自主感知与导航



##  1. <a name='mrobot_gazebomrobot_description'></a>仿真环境与交换(mrobot_gazebo/mrobot_description)

**需要的库与插件**
gazebo_plugins/  gazebo_ros/  gazebo_ros_control  

**需要的机器人描述部件  URDF**

- 机器人body mrobot_body.urdf.cacro  mrobot.urdf.cacro   
- 相机的模型 camera.xacro  usb_camera
- kinect.xacro  RGBD相机模型 (mesher渲染的)
- rplidar.xacro 二维激光雷达模型
- mrobot_with_camera.urdf.xacro  机器人+urb_camera   1280*720 50Hz
- mrobot_with_kinect.urdf.xacro  机器人+kinect		 640*480 RGB+depth 45Hz
- mrobot_with_rplidar.urdf.xacro 机器人+rplidar      5Hz

**需要的仿真环境 world**

- cloister / playground / playpen / room




##  2. <a name='perception'></a>perception 


##  3. <a name='mapping'></a>mapping

###  3.1. <a name='map_server'></a>map_server
 
 > - 加载现有的地图  
   - 更新地图  
   - 发布/map topic



image_loader.cpp   map_server_image_loader .pgm地图的加载
map_saver.cpp	   map_saver  
main.cpp    	   map_server exe  


>
nav_msgs/OccupancyGrid 
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
nav_msgs/MapMetaData info
  time map_load_time
  float32 resolution
  uint32 width
  uint32 height
  geometry_msgs/Pose origin
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
int8[] data

地图的表现形式 Occupancy Grid Map 

##  4. <a name='navigation'></a>navigation


### move_base


##  5. <a name='mrobot_navigation'></a>mrobot_navigation




## obstable recognition V0

0. **gazebo robot and world** (robot with lidar)
  - lidar 发布 /scan话题

1. **map server** : load map  & rviz visualization
  - YAMLCPP read gmapping config yaml
  - 发布地图话题

2. **Odom**  AMCL 粒子滤波


3. **设置航线**

