/*********************************************************************
*
planner是全局规划
controller是局部规划
*********************************************************************/
#include "global_navi/global_navi.h"
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <geometry_msgs/Twist.h> //下发指令
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_cloud.h>
#include <laser_geometry/laser_geometry.h> // laserScan ==>Pointcloud2
#include <pcl_conversions/pcl_conversions.h>

namespace global_navi {

/* ******************************************************* 
  global_planner 构造函数

  - create global planner costmap ==> static+inflation+obstacle
  - create global planner thread ==> navfn/NavfnROS
*******************************************************
 */ 
  GlobalNavi::GlobalNavi(tf::TransformListener& tf) :
    tf_(tf),
    as_(NULL),  //action server
    planner_costmap_ros_(NULL), 
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"), 
    //pose list
    planner_plan_(NULL), latest_plan_(NULL), 
    runPlanner_(false),  new_global_plan_(false) 
  {

    //move_base action server  监听 move_base_msgs::MoveBaseGoal消息
    as_ = new MoveBaseActionServer(ros::NodeHandle(), "global_planner", boost::bind(&GlobalNavi::executeCb, this, _1), false);
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    //get some parameters that will be global to the move base node
    std::string global_planner;
    private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));  // global_planner plugin  
    private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
    private_nh.param("global_costmap/global_frame", global_frame_, std::string("/map"));
    
    private_nh.param("planner_frequency", planner_frequency_, 0.0);
    private_nh.param("planner_patience", planner_patience_, 5.0);
    private_nh.param("max_planning_retries", max_planning_retries_, -1);  // disabled by default

    //set up plan triple buffer
    planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    latest_plan_  = new std::vector<geometry_msgs::PoseStamped>();

    //set up the planner's thread  配置全局规划的线程
    planner_thread_ = new boost::thread(boost::bind(&GlobalNavi::planThread, this)); //开启 planThread线程

    //for comanding the base  发布命令
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0 );

    // /move_base/goal target_pose send to move_bashe action server
    ros::NodeHandle action_nh("global_planner"); //发布action goal
    action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);

    //we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
    //they won't get any useful information back about its status, but this is useful for tools
    //like nav_view and rviz
    ros::NodeHandle simple_nh("move_base_simple"); //从RVIZ订阅目标点
    goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&GlobalNavi::goalCB, this, _1));

    path_waypoints_pub = nh.advertise<visualization_msgs::Marker>("waypoints_marker", 1);
    laserScan_sub = nh.subscribe("/scan", 1, &GlobalNavi::laserDataCallBack, this);

    //发布障碍物信息
    obstableMsg_pub = nh.advertise<std_msgs::String>("obstableMsg", 10, true);
    
    private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
    private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);
  

    // costmap part ********************
    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    planner_costmap_ros_->pause();

    //initialize the global planner
    try
    {
      planner_ = bgp_loader_.createInstance(global_planner);
      planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_); //初始化一个global_planner navfn_ros
    } 
    catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
      exit(1);
    }

    
    // Start actively updating costmaps based on sensor data
    planner_costmap_ros_->start();
    
    ROS_WARN("start update global cospmap");

    //advertise a service for getting a plan  [发布 start and goal 请求 global plan]
    make_plan_srv_ = private_nh.advertiseService("make_plan", &GlobalNavi::planService, this);

    //advertise a service for clearing the costmaps
    clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &GlobalNavi::clearCostmapsService, this);

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps initially");
      planner_costmap_ros_->stop();
    }

    //initially, we'll need to make a plan
    state_ = PLANNING;

    //we're all set up now so we can start the action server
    as_->start();
    ROS_WARN("MoveBaseAction service start");
  }



/* ******************************************************* 
  回调函数
    - goalCB  from RVIZ or other sub move_base_simple topic ==> actiongoal
    - 
*******************************************************
 */ 
  

  //订阅目标点的回调函数 发给action server
  void GlobalNavi::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal){
    ROS_WARN("GlobalNavi::goalCB");
    ROS_DEBUG_NAMED("move_base","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    move_base_msgs::MoveBaseActionGoal action_goal;
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose = *goal;
    action_goal_pub_.publish(action_goal);
  }


/* ******************************************************* 
  server function
  - clearCostmapsService 把没一层costmap都reset 一下
   

*******************************************************
 */ 
  // 把每一个层都 reset 一下
  bool GlobalNavi::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
  {
    ROS_WARN("GlobalNavi::clearCostmapService");
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_planner(*(planner_costmap_ros_->getCostmap()->getMutex()));
    planner_costmap_ros_->resetLayers();
    return true;
  }

/*
  >> planService (请求一次全局规划的服务)
  1. 确保有一个 global costmap
  2. 确保能够的到机器人的位置
  3. clear机器人附近的costmap
  4. 尝试直接对于goal进行 make global plan,成功了,直接得到 global_plan vector pose list
  5. 如果在goal规划不出路径,就在goal附近偏移进行规划路径
  6. 将规划的路径填充到 nav_msgs::GetPlan::Response
  */
  bool GlobalNavi::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp)
  {
    ROS_WARN("GlobalNavi::planService");

    if(as_->isActive()){  //
      ROS_ERROR("move_base must be in an inactive state to make a plan for an external user");
      return false;
    }
    //make sure we have a costmap for our planner
    if(planner_costmap_ros_ == NULL){
      ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
      return false;
    }

    // 确定起点 
    geometry_msgs::PoseStamped start;
    //if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
    if(req.start.header.frame_id.empty())
    {
        tf::Stamped<tf::Pose> global_pose;
        if(!planner_costmap_ros_->getRobotPose(global_pose)){
          ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
          return false;
        }
        tf::poseStampedTFToMsg(global_pose, start);
    }
    else
    {
        start = req.start;
    }

    //update the copy of the costmap the planner uses
    clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);

    //first try to make a plan to the exact desired goal
    std::vector<geometry_msgs::PoseStamped> global_plan;  //global plan waypoint list
    //尝试make global plan, 
    if(!planner_->makePlan(start, req.goal, global_plan) || global_plan.empty()){
      ROS_DEBUG_NAMED("move_base","Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance", 
          req.goal.pose.position.x, req.goal.pose.position.y);

      // 如果没有规划成功
      //search outwards for a feasible goal within the specified tolerance
      geometry_msgs::PoseStamped p;
      p = req.goal;
      bool found_legal = false;
      float resolution = planner_costmap_ros_->getCostmap()->getResolution();
      float search_increment = resolution*3.0; //偏移的步长
      if(req.tolerance > 0.0 && req.tolerance < search_increment) search_increment = req.tolerance; //限幅
      for(float max_offset = search_increment; max_offset <= req.tolerance && !found_legal; max_offset += search_increment) {
        for(float y_offset = 0; y_offset <= max_offset && !found_legal; y_offset += search_increment) {
          for(float x_offset = 0; x_offset <= max_offset && !found_legal; x_offset += search_increment) {
            // 如果在goal规划不出路径,就在goal附近偏移进行规划路径
            //don't search again inside the current outer layer
            if(x_offset < max_offset-1e-9 && y_offset < max_offset-1e-9) continue;

            //search to both sides of the desired goal
            for(float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0) {

              //if one of the offsets is 0, -1*0 is still 0 (so get rid of one of the two)
              if(y_offset < 1e-9 && y_mult < -1.0 + 1e-9) continue;

              for(float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0) {
                if(x_offset < 1e-9 && x_mult < -1.0 + 1e-9) continue;

                p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
                p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;

                if(planner_->makePlan(start, p, global_plan)){
                  if(!global_plan.empty()){

                    //adding the (unreachable) original goal to the end of the global plan, in case the local planner can get you there
                    //(the reachable goal should have been added by the global planner)
                    global_plan.push_back(req.goal); //在规划出来的路径上添加最后的点

                    found_legal = true;
                    ROS_DEBUG_NAMED("move_base", "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                    break;
                  }
                }
                else{
                  ROS_DEBUG_NAMED("move_base","Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                }
              }
            }
          }
        }
      }
    }

    //copy the plan into a message to send out 填充 srv msgs 
    resp.plan.poses.resize(global_plan.size());
    for(unsigned int i = 0; i < global_plan.size(); ++i){
      resp.plan.poses[i] = global_plan[i];
    }

    return true;
  }


/* ******************************************************* 
  功能函数
  - clearCostmapWindows   清除在costmap上机器人周围的的obstable

*******************************************************
 */ 

  /*
    >> 清除在costmap上机器人周围的的obstable
    对 planner_costmap_ros_操作
    1. 先得到机器人在costmap上的global pose (getRobotPose)
    2. 然后以机器人位置中心圈一个矩形, 并清除 getCostmap()->setConvexPolygonCost
    对 controller_costmap_ros_ 同样操作一遍
  */
  void GlobalNavi::clearCostmapWindows(double size_x, double size_y)
  {

    ROS_WARN("GlobalNavi::clearCostmapWindows");
    tf::Stamped<tf::Pose> global_pose;

    //clear the planner's costmap
    planner_costmap_ros_->getRobotPose(global_pose);

    std::vector<geometry_msgs::Point> clear_poly;
    double x = global_pose.getOrigin().x();
    double y = global_pose.getOrigin().y();
    geometry_msgs::Point pt;

    //上下左右的四个点
    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    //清空这个矩形, 设定为free space
    planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
  }


  


  // 析构函数 释放costmap and  planThread
  GlobalNavi::~GlobalNavi()
  {
    
    if(as_ != NULL)
      delete as_;

    if(planner_costmap_ros_ != NULL)
      delete planner_costmap_ros_;

    planner_thread_->interrupt();
    planner_thread_->join();

    delete planner_thread_;

    delete planner_plan_;
    delete latest_plan_;

    planner_.reset();
  }

 
 /*
 >> 全局路径规划的方法
  1. lock costMap
  2. make sure planner costmap
  3. make sure  start pose 并转换到global pose
  4. make plan
 */
  bool GlobalNavi::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

    ROS_WARN("GlobalNavi::makePlan");

    // lock costMap
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex())); 

    //make sure to set the plan to be empty initially
    plan.clear();

    //since this gets called on handle activate
    if(planner_costmap_ros_ == NULL) {
      ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
      return false;
    }

    //get the starting pose of the robot
    tf::Stamped<tf::Pose> global_pose;
    if(!planner_costmap_ros_->getRobotPose(global_pose)) { //得到current global pose
      ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
      return false;
    }

    geometry_msgs::PoseStamped start;
    tf::poseStampedTFToMsg(global_pose, start); //pose 数据类型转换

    //if the planner fails or returns a zero length plan, planning failed
    // 已知cost map & start and goal , planner的离散路径点放在plan中
    if(!planner_->makePlan(start, goal, plan) || plan.empty()){ //调用global_plan  make_plan
      ROS_WARN("Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
      return false;
    }
    return true;
  }


  //发布机器人暂停的命令
  void GlobalNavi::publishZeroVelocity(){

    ROS_WARN("GlobalNavi::publishZeroVelocity");

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
  }


  // 验证四元素
  bool GlobalNavi::isQuaternionValid(const geometry_msgs::Quaternion& q){

    ROS_WARN("GlobalNavi::isQuaternionValid");
    //first we need to check if the quaternion has nan's or infs
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
      ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
      return false;
    }

    tf::Quaternion tf_q(q.x, q.y, q.z, q.w);

    //next, we need to check if the length of the quaternion is close to zero
    if(tf_q.length2() < 1e-6){
      ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
      return false;
    }

    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    tf_q.normalize();

    tf::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }

    return true;
  }


  /*
  >> goal_pose 坐标转换
  1. msg-->tf_msg  (poseStampedMsgToTF)
  2. 利用tf 转换成 global_pose  (transformPose)
  3. tf_msg-->msg  (poseStampedTFToMsg)

  */
  geometry_msgs::PoseStamped GlobalNavi::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg){

    ROS_WARN("GlobalNavi::goalToGlobalFrame");
    std::string global_frame = planner_costmap_ros_->getGlobalFrameID();//map
    tf::Stamped<tf::Pose> goal_pose, global_pose;
    poseStampedMsgToTF(goal_pose_msg, goal_pose); //msg-->tf_msg

    //just get the latest available transform... for accuracy they should send
    //goals in the frame of the planner
    goal_pose.stamp_ = ros::Time();

    try{
      //利用tf 获取机器人坐标系和全局坐标系的关系
      tf_.transformPose(global_frame, goal_pose, global_pose); // tf_msg -- tf_msg_global
    }
    catch(tf::TransformException& ex){
      ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
          goal_pose.frame_id_.c_str(), global_frame.c_str(), ex.what());
      return goal_pose_msg;
    }

    geometry_msgs::PoseStamped global_pose_msg;
    tf::poseStampedTFToMsg(global_pose, global_pose_msg);
    return global_pose_msg;
  }



// 通过boost thread的条件变量planner_cond_ 唤醒线程 planThread
  void GlobalNavi::wakePlanner(const ros::TimerEvent& event)
  {
    ROS_WARN("GlobalNavi::wakePlanner");
    // we have slept long enough for rate
    planner_cond_.notify_one();  //唤醒线程
  }



/*
>>  全局规划的线程
  1. lock planner_mutex_
  2. check running the planner
  3. planner plan vector清空
  4. makePlan
  5. if gotPlan 
  6. 如果成功还会接着尝试
  7. 最后如果不成功, 转换状态为CLEARING, 并PLANNING_R

*/
  void GlobalNavi::planThread(){

    ROS_WARN("GlobalNavi::planThread");
    ros::NodeHandle n;
    ros::Timer timer;
    bool wait_for_wake = false;
    //boost 独占锁  lock planner_mutex
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    while(n.ok()){
      //check if we should run the planner (the mutex is locked)
      while(wait_for_wake || !runPlanner_){
        //if we should not be running the planner then suspend this thread
        ROS_DEBUG_NAMED("global_plan_thread","Planner thread is suspending");
        planner_cond_.wait(lock);
        wait_for_wake = false;
      }
      ros::Time start_time = ros::Time::now();

      //time to plan! get a copy of the goal and unlock the mutex
      geometry_msgs::PoseStamped temp_goal = planner_goal_; //目标点
      lock.unlock();
      ROS_DEBUG_NAMED("move_base_plan_thread","Planning...");

      //run planner 全局路径规划
      planner_plan_->clear();  //planner plan vector清空
      bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);

      if(gotPlan)
      {
        ROS_WARN("Got Plan with %zu points!", planner_plan_->size());
        //pointer swap the plans under mutex (the controller will pull from latest_plan_)
        std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;
        lock.lock();
        planner_plan_ = latest_plan_;
        latest_plan_ = temp_plan;
        last_valid_plan_ = ros::Time::now();
        planning_retries_ = 0;
        new_global_plan_ = true;  //makePlan got global plan path


      // visiualization path waypoints
      visiualization_waypoints(*planner_plan_);


        //make sure we only start the controller if we still haven't reached the goal
        if(runPlanner_)
          state_ = CONTROLLING; //状态转为局部路径规划
        if(planner_frequency_ <= 0)
          runPlanner_ = false;
        lock.unlock();
      }
      //if we didn't get a plan and we are in the planning state (the robot isn't moving)
      else if(state_==PLANNING){
        ROS_DEBUG_NAMED("move_base_plan_thread","No Plan...");
        ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);

        //check if we've tried to make a plan for over our time limit or our maximum number of retries
        //issue #496: we stop planning when one of the conditions is true, but if max_planning_retries_
        //is negative (the default), it is just ignored and we have the same behavior as ever
        lock.lock();
        planning_retries_++;
        if(runPlanner_ &&  //如果一直规划不出来
           (ros::Time::now() > attempt_end || planning_retries_ > uint32_t(max_planning_retries_))){
          //we'll move into our obstacle clearing mode
          state_ = CLEARING;
          runPlanner_ = false;  // proper solution for issue #523
          publishZeroVelocity();
        }

        lock.unlock();
      }

      //take the mutex for the next iteration
      lock.lock();

      //setup sleep interface if needed
      if(planner_frequency_ > 0){
        ros::Duration sleep_time = (start_time + ros::Duration(1.0/planner_frequency_)) - ros::Time::now();
        if (sleep_time > ros::Duration(0.0)){
          wait_for_wake = true;
          timer = n.createTimer(sleep_time, &GlobalNavi::wakePlanner, this);
        }
      }
    }
  }

  void GlobalNavi::visiualization_waypoints(const std::vector<geometry_msgs::PoseStamped>& waypoints)
  {
     visualization_msgs::Marker waypoints_marker;
     waypoints_marker.header.frame_id = waypoints.front().header.frame_id;
     waypoints_marker.header.stamp = ros::Time::now();
     waypoints_marker.ns = global_frame_;
     waypoints_marker.action = visualization_msgs::Marker::ADD;

     waypoints_marker.id = 0;
     waypoints_marker.type = visualization_msgs::Marker::LINE_LIST;
     waypoints_marker.scale.x = 0.2;
     waypoints_marker.scale.y = 0.2;
     
     waypoints_marker.color.b = 1.0f;
     waypoints_marker.color.g = 1.0f;
     for(int i=0; i< waypoints.size(); i++)
     {
       waypoints_marker.points.push_back(waypoints[i].pose.position);
     }
     path_waypoints_pub.publish(waypoints_marker);



  }

  
  // 收到一个 GlobalNavi goal 点之后
  /*
  >> GlobalNaviGoal  execute
  1. move_base_msgs::GlobalNaviGoal 话题
  2. 将goal转换到全局坐标系
  3. 开启planThread
  4. 执行executeCycle(goal, global_plan)
  */
  void GlobalNavi::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
  {
    //先判断四元素
    ROS_WARN("GlobalNavi::executeCb");
    if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
      // 
      as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
      return;
    }

    // 坐标转换 转换到 /map系下 goal
    geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);

    //we have a goal so start the planner
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    planner_goal_ = goal;
    runPlanner_ = true;
    planner_cond_.notify_one(); //等待线程解除阻塞
    lock.unlock();

    current_goal_pub_.publish(goal); //publish goal
    std::vector<geometry_msgs::PoseStamped> global_plan;

    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Starting up costmaps that were shut down previously");
      planner_costmap_ros_->start();
    }

    //we want to make sure that we reset the last time we had a valid plan and control
    last_valid_control_ = ros::Time::now();
    last_valid_plan_ = ros::Time::now();
    last_oscillation_reset_ = ros::Time::now();
    planning_retries_ = 0;

    ros::NodeHandle n;
    while(n.ok())
    {
      //判断 active server是否可抢占
      if(as_->isPreemptRequested()) 
      {
        ROS_WARN("action server is PreemptRequested ");
        if(as_->isNewGoalAvailable())
        {
          //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
          move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();

          if(!isQuaternionValid(new_goal.target_pose.pose.orientation)){
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
          }

          goal = goalToGlobalFrame(new_goal.target_pose);

          //we'll make sure that we reset our state for the next execution cycle

          state_ = PLANNING;

          //we have a new goal so make sure the planner is awake
          lock.lock();
          planner_goal_ = goal;
          runPlanner_ = true;
          planner_cond_.notify_one();
          lock.unlock();

          //publish the goal point to the visualizer
          ROS_DEBUG_NAMED("move_base","move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
          current_goal_pub_.publish(goal);

          //make sure to reset our timeouts and counters
          last_valid_control_ = ros::Time::now();
          last_valid_plan_ = ros::Time::now();
          last_oscillation_reset_ = ros::Time::now();
          planning_retries_ = 0;
        }
        else { // as_->isNewGoalAvailable() false
          //if we've been preempted explicitly we need to shut things down
          resetState();

          //notify the ActionServer that we've successfully preempted
          ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");
          as_->setPreempted();

          //we'll actually return from execute after preempting
          return;
        }
      }
    }

    //if the node is killed then we'll abort and return
    as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
    return;
  }


  //直线距离
  double GlobalNavi::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }


  /*
  >> executeCycle
  1. 得到机器人当前位置
  2. publishFeedback
  3. check oscillation
  4. check controller_costmap_ros_->isCurrent()
  5. input goal, get global plan
  */
  bool GlobalNavi::executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan){

    ROS_WARN("GlobalNavi::executeCycle");
    geometry_msgs::Twist cmd_vel;

    //update feedback to correspond to our curent position
    tf::Stamped<tf::Pose> global_pose;
    planner_costmap_ros_->getRobotPose(global_pose);// tf中得到位置
    geometry_msgs::PoseStamped current_position;
    tf::poseStampedTFToMsg(global_pose, current_position); //tf-->msg  /map系下的位置

    //push the feedback out
    move_base_msgs::MoveBaseFeedback feedback;
    feedback.base_position = current_position;
    as_->publishFeedback(feedback); // moveAction server反馈当前机器人的位姿

    //check to see if we've moved far enough to reset our oscillation timeout
    if(distance(current_position, oscillation_pose_) >= oscillation_distance_)
    {
      last_oscillation_reset_ = ros::Time::now();
      oscillation_pose_ = current_position;

    }

    //the move_base state machine, handles the control logic for navigation
      //if we are in a planning state, then we'll attempt to make a plan
      if(state_ == PLANNING)
        {
          boost::recursive_mutex::scoped_lock lock(planner_mutex_);
          runPlanner_ = true;
          planner_cond_.notify_one();
        }
        ROS_DEBUG_NAMED("move_base","Waiting for plan, in the planning state.");
        


    //we aren't done yet
    return false;
  }



  void GlobalNavi::resetState(){
    ROS_WARN("GlobalNavi::resetState");
    // Disable the planner thread
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    runPlanner_ = false;
    lock.unlock();

    // Reset statemachine
    state_ = PLANNING;
    publishZeroVelocity();

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps");
      planner_costmap_ros_->stop();
    }
  }

// obstacle_detction
void GlobalNavi::laserDataCallBack(const sensor_msgs::LaserScanConstPtr scan_msg)
{
     // ROS_INFO("received laserScan message");
        sensor_msgs::LaserScan scan;
        scan.header.stamp = scan_msg->header.stamp;
        scan.header.frame_id = scan_msg->header.frame_id;
        scan.angle_min = scan_msg->angle_min;
        scan.angle_max = scan_msg->angle_max;
        scan.angle_increment = scan_msg->angle_increment;
        scan.scan_time = scan_msg->scan_time;
        scan.range_min = scan_msg->range_min;
        scan.range_max = scan_msg->range_max;

        scan.ranges = scan_msg->ranges;
        scan.intensities = scan_msg->intensities;
        laser_geometry::LaserProjection projector;
        
        sensor_msgs::PointCloud cloud_out;
        projector.projectLaser(scan, cloud_out, -1.0, laser_geometry::channel_option::Index);
        // pointCloud_pub.publish(cloud_out);   
      int count = 0;
       for(int i=0; i<cloud_out.points.size(); i++)
        {
            if(hypot(cloud_out.points[i].x, cloud_out.points[i].y) < safety_distance)
            {
                count++;
            }
        }
        if(count > safety_tolerance)
        {
                std_msgs::String obstable_msg;
                obstable_msg.data = "obstable beyond the threshold of safety distance";
                obstableMsg_pub.publish(obstable_msg);
                hasObstablePoints = true;
        }
        else 
          hasObstablePoints = false;               

  }


  // obstacle_detction
  void GlobalNavi::velodyneDataCallBack(const sensor_msgs::PointCloud2ConstPtr&  velodyneData)
  {
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    pcl::fromROSMsg(*velodyneData.get(), *cloud.get());

    // for (size_t i = 0; i < cloud->points.size(); i++) {

    //   pcl::PointXYZI p;      
    //   p.x = cloud->points[i].x;
    //   p.y = cloud->points[i].y;
    //   p.z = cloud->points[i].z;
    //   p.intensity = cloud->points[i].intensity;
      
    // }

      int count = 0;
       for(int i=0; i<cloud->points.size(); i++)
        {
            if(hypot(cloud->points[i].x, cloud->points[i].y) < safety_distance)
            {
                count++;
            }
        }
        if(count > safety_tolerance)
        {
                std_msgs::String obstable_msg;
                obstable_msg.data = "obstable beyond the threshold of safety distance";
                obstableMsg_pub.publish(obstable_msg);
                hasObstablePoints = true;
        }
        else 
          hasObstablePoints = false;    
  }  

};


