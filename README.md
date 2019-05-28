<!-- vscode-markdown-toc -->
* 1. [TODO list:](#TODOlist:)
* 2. [aibee_robot 定义机器人仿真与实测](#aibee_robot)
* 3. [function_navigation 根据需求定义机器人自主功能](#function_navigation)
* 4. [ROS_Navigation 机器人的定位/导航功能包](#ROS_Navigation)
	* 4.1. [map_server](#map_server)
	* 4.2. [move_base](#move_base)
* 5. [ros_gazebo_sim 移动机器人仿真系统](#ros_gazebo_sim)
	* 5.1. [仿真环境与交换(mrobot_gazebo/mrobot_description)](#mrobot_gazebomrobot_description)
	* 5.2. [mrobot_teleop  键盘操控机器人](#mrobot_teleop)

<!-- vscode-markdown-toc-config
	numbering=true
	autoSave=true
	/vscode-markdown-toc-config -->
<!-- /vscode-markdown-toc -->



# Autonomous-mobile-robot 室内移动机器人自主感知与导航

##  1. <a name='TODOlist:'></a>TODO list:
> 使用`ROS+gazebo`搭建一个 室内/室外的轮式机器人的仿真场景，然后集成`ROS Navigation`机器人感知导航算法，然后不断地完善和丰富移动机器人的功能，比如：
- 可以使用多种视觉/雷达/轮盘传感器 来做机器人的定位和状态估计
- 可以完善mave_base中的global_costmap global_planner, local_costmap local_planner的功能
- 完善机器人建图的功能以及 map server
- 完善机器人的 exploration自主探索陌生环境的能力


============================================================================================
##  2. <a name='aibee_robot'></a>aibee_robot 定义机器人仿真与实测

**aibee_robot package**

>  roslaunch aibee_robot obstable_recognition_gazebo.launch
ROS-gazebo仿真: 
机器人挂载LaserSensor, 预加载仿真环境的 map
使用 AMCL定位/ map_server加载地图/ move_base全局与局部的路径规划
使用RVIZ 进行可视化,并可以在RVIZ中选定 goal



============================================================================================
##  3. <a name='function_navigation'></a>function_navigation 根据需求定义机器人自主功能




============================================================================================
##  4. <a name='ROS_Navigation'></a>ROS_Navigation 机器人的定位/导航功能包

###  4.1. <a name='map_server'></a>map_serverr 
 > - 加载现有的地图  
   - 更新地图  
   - 发布/map topic



image_loader.cpp   map_server_image_loader .pgm地图的加载
map_saver.cpp	   map_saver  
main.cpp    	   map_server exe  

nav_msgs/OccupancyGrid 
地图的表现形式 Occupancy Grid Map 



###  4.2. <a name='move_base'></a>move_base





##  5. <a name='ros_gazebo_sim'></a>ros_gazebo_sim 移动机器人仿真系统

============================================================================================

###  5.1. <a name='mrobot_gazebomrobot_description'></a>仿真环境与交换(mrobot_gazebo/mrobot_description)

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

###  5.2. <a name='mrobot_teleop'></a>mrobot_teleop  键盘操控机器人

> mrobot_teleop.launch  使用键盘控制机器人运动


