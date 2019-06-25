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
> 使用`ROS+gazebo`搭建一个 室内/室外的轮式机器人的仿真场景，
然后集成`ROS Navigation`机器人感知导航算法，
然后不断地完善和丰富移动机器人的功能，比如：

- 可以使用多种视觉/雷达/轮盘传感器 来做机器人的定位和状态估计
- 可以完善mave_base中的global_costmap global_planner, local_costmap local_planner的功能
- 完善机器人建图的功能以及 map server
- 完善机器人的 exploration自主探索陌生环境的能力


============================================================================================
##  2. <a name='aibee_robot'></a>aibee_robot 定义机器人仿真与实测

**aibee_robot package**

> ROS-gazebo仿真: 
>  roslaunch aibee_robot obstable_recognition_gazebo.launch
- 机器人挂载 rplidar,  {/scan  sensor_msgs/LaserScan}
- map_serve预加载仿真环境的map  {cloister_gmapping.yaml -> /map  nav_msgs/OccupancyGrid}
- 使用 AMCL定位  {订阅/tf /scan 发布/tf}
- move_base全局与局部的路径规划 {订阅/tf /map /odom /move_base/action_topic} {发布 global/plan local/plan (nav_msgs/Path)  /cmd_vel (geometry_msgs/Twist 三轴线速度和三轴角速度)}
- 使用RVIZ 进行可视化,并可以在RVIZ中选定 goal


[!gazebo 仿真](doc/obstable_recognition_gazebo.png)



> 真实机器人测试:
- 机器人挂载 



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




============================================================================================
##  5. <a name='ros_gazebo_sim'></a>ros_gazebo_sim 移动机器人仿真系统



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



============================================================================================

## improved_SLAM 
> - ros navigation  slam-gmapping  amcl定位等方法都是针对 2D激光雷达数据接口的算法, laserscan
- 而robot 使用的是 velodyne_VLP_16的3D激光雷达
- 所以使用一些高级的 laser-SLAM / V-Slam算法替换原本的ros navigation中的功能包

###  [LOAM](Lidar Odometry and Mapping in Real-time)

- [loam_velodyne 原版本](https://github.com/laboshinl/loam_velodyne)
- [A-LOAM 港科大优化版本 uses Eigen and Ceres Solver to simplify code structure](https://github.com/HKUST-Aerial-Robotics/A-LOAM)


```
test:

roslaunch aloam_velodyne aloam_velodyne_VLP_16.launch
rosbag play  data/nsh_indoor_outdoor.bag
```



## costmap 2D
 ----------------------------------------------------------

> -  costmap 使用了传感器数据和来自固态地图的信息，
通过costmap_2d::Costmap2DROS对象(Object)，来保存和更新关于障碍物的信息。

> - costmap实例化了就成了Layer, 通过Costmap2DROS类来实例化，放在了以LayeredCostmap类下的对象里，由LayeredCostmap管理。

**地图类型**
生成costmap_2d::Costmap2DROS有2中方式：
- `static_map` seed it with a user-generated static map
- `rolling_window` give it a width and height and to set the rolling_window parameter to be true
> rolling_window parameter 让机器人处在costmap正中间，当机器人走太远时，放下障碍物。这种一般用在里程计调整架构（odometric coordinate frame）中，这种架构里，机器人只关心在一个local area里的障碍物。这里涉及到的rolling_window参数是用来设置在机器人移动过程中是否需要滚动窗口，以保持机器人处于中心位置。


**注意 frame_id and tf_tree**
- global_frame          /map
- robot_base_frame      /base_link

- footprint 机器人的垂直投影




## costmap的配置文件

> 导航功能包使用两种代价地图存储周围环境中的障碍信息，一种用于全局路径规划，一种用于本地路径规划和实时避障。两种代价地图需要使用一些共同和独立的配置文件：
    - 通用配置文件      costmap_common_params.yaml  global & local每人一个   
    - 全局规划配置文件   global_costmap_params.yaml
    - 本地规划配置文件   local_costmap_params.yaml

1. **通用配置文件**
> costmap用来存储周围环境的障碍信息，其中需要注明地图关注的机器人传感器消息，以便于地图信息进行更行
所以 costmap_common_param 传感器消息的相关配置


```
obstacle_range: 2.5
raytrace_range: 3.0
max_obstacle_height: 0.6
min_obstacle_height: 0.0
// 用来设置costmap中障碍物的相关阈值，
// obstacle_range参数用来设置机器人检测障碍物的最大范围，设置为2.5意为在2.5米范围内检测到的障碍信息
// raytrace_range参数用来设置机器人检测自由空间的最大范围，设置为3.0意为在3米范围内，机器人将根据传感器的信息，清除范围内的自由空间

# footprint: [[x0, y0], [x1, y1], ... [xn, yn]]  
robot_radius: 0.165
# robot_radius: 0.5
# inflation_radius: 0.1
inflation_radius: 0.5
//这些参数用来设置机器人在二维地图上的占用面积，如果机器人外形是圆形，则需要设置机器人的外形半径。所有参数以机器人的中心作为坐标（0，0）点
// inflation_radius参数是设置障碍物的膨胀参数，也就是机器人应该与障碍物保持的最小安全距离，这里设置为0.55意为为机器人规划的路径应该与障碍物保持0.5米以上的安全距离。


observation_sources: scan
scan: {data_type: LaserScan, topic: /scan, marking: true, clearing: true, expected_update_rate: 0}

// observation_sources参数列出了代价地图需要关注的所有传感器信息
// marking和clearing参数用来表示是否需要使用传感器的实时信息来添加或清楚代价地图中的障碍物信息
```

2. **全局costmap配置文件**


```
global_costmap:
   global_frame: /map
   robot_base_frame: /base_link
// global costmap在/map参考系下运行，
// robot_base_frame指定参考系，用于获取运动中机器人的位姿

   update_frequency: 1.0
   publish_frequency: 0

// golbal costmap 更新频率和用于可视化时地图发布频率

   static_map: true
   rolling_window: false
// 一般global costmap使用static_map, local_costmap使用rolling_windows 

   resolution: 0.05
   transform_tolerance: 1.0
   map_type: costmap
//地图的分辨率， tf获取时，数据的时间的tolerance   

```

3. **局部costmap配置文件**

```
local_costmap:
   global_frame: /odom
   robot_base_frame: /base_link

   update_frequency: 3.0
   publish_frequency: 1.0

   static_map: false
   rolling_window: true
// 一般global costmap使用static_map, local_costmap使用rolling_windows    

   width: 6.0
   height: 6.0
   resolution: 0.05
   transform_tolerance: 1.0
   map_type: costmap

```



## costmap_2d::Costmap2D  CLASS
> 实现了基础的数据结构，用来存储和读取2D costmap的数据结构。


## layeredCostmap 
> 实例化不同的layer  plugins, 并且计算一个总计的 score
用来keep track of each of the layers

**每个层是在Costmap2DROS中，用pluginlib来实例化，并加入到LayeredCostmap类的对象中**
- `Static Map Layer` - The static map layer represents a largely unchanging portion of the costmap, like those generated by SLAM.
- `Obstacle Map Layer` - The obstacle layer tracks the obstacles as read by the sensor data. The ObstacleCostmapPlugin marks and raytraces obstacles in two dimensions, while theVoxelCostmapPlugin does so in three dimensions.
- `Inflation Layer` - The inflation layer is an optimization that adds new values around lethal obstacles (i.e. inflates the obstacles) in order to make the costmap represent the configuration space of the robot.
- `Other Layers` - Other layers can be implemented and used in the costmap via pluginlib. Any additional plugins are welcomed to be listed and linked to below.




## Costmap2DPublisher
> A tool to periodically publish visualization data from a Costmap2D

发布话题：
/costmap
/costmap_updates

ros costmap 发布的时候，需要将值映射到-1～100
    - -1 unknown
    - 0 no obstable
    - 99 inscribed obstacle 内切障碍物
    - 100 lethal obstacle 致命的障碍物，置信度高
    - 1-252 映射到 1～98

## costmap_2d

- size_x width
- size_y heigh

**流程**
1. check frame_id  global_frame && robot_base_frame
2. check if we want a `rolling window` version of the costmap
3. layered_costmap_ = new `LayeredCostmap`(global_frame_, rolling_window, track_unknown_spluginspace);
4. load paugin layer, don't have `plugins`, resetOldParameters()
    - static_layer
    - obstacle_layer
    - inflation_layer
5. topic sub & pub
    - sub footprint_topic ==> Costmap2DROS::setUnpaddedRobotFootprintPolygon
    - pub footprint  ==> setUnpaddedRobotFootprint(makeFootprintFromParams(private_nh));

6. costmap2d publisher
    publisher_ = new `Costmap2DPublisher`(&private_nh, layered_costmap_->getCostmap(), global_frame_, "costmap", always_send_full_costmap);

7.  启动定时器createTimer ---> Costmap2DROS::`movementCB` check if the robot is moving 
    从tf中读取机器人的位置，判断机器人是否运动 （设置robot_stopped_标志位）

8.  启动 dynamic reconfigure server ---> Costmap2DROS::`reconfigureCB`

9. map_update_thread
    - map_update_thread_ = new boost::thread(boost::bind(&Costmap2DROS::`mapUpdateLoop`, this, map_update_frequency));
    - **updateMap()**
    - **publishCostmap()**




global costmap 包含 
- static_layer and  inflation_layer and obstacle_layer

local_cost map 包含
- inflation_layer and obstacle_layer

1. static_layer 订阅/map
2. obstacle_layer 订阅传感器信息，然后更新cost
3. inflation_layer 
 ----------------------------------------------------------



 ## local planner and global planner
 ----------------------------------------------------------
 MoveBaseActionServer ==> executeCb()  监听是否收到goalpoint
 
 



 ----------------------------------------------------------


 ## 高级功能
 - actionlib