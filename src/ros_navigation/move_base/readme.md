
# move_base





## costmap 

创建costmap
planner_costmap_ros_    = new costmap_2d::Costmap2DROS("global_costmap", tf_);
controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);  //注意tf tree


planner_costmap_ros_->pause();
controller_costmap_ros_->pause();

planner_costmap_ros_->start();
controller_costmap_ros_->start();

**在make plan时，要将costmap 数据锁住**
boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex())); 


planner_costmap_ros_->getRobotPose(global_pose);
planner_costmap_ros_->getGlobalFrameID();
planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);

planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);


planner_costmap_ros_->resetLayers();

planner_costmap_ros_->stop();
controller_costmap_ros_->stop();


## costmap 与路径规划的联系
planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);