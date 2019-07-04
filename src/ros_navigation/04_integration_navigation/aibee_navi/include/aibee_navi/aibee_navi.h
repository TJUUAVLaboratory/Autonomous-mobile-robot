/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef AIBEE_NAVI_H
#define AIBEE_NAVI_H

#include <vector>
#include <string>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>  //全局规划给局部规划的plan

#include <pluginlib/class_loader.h>
#include <std_srvs/Empty.h>


#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h> // velodyne sensor data
#include "aibee_navi/point_types.h"




namespace global_navi 
{
  // MoveBaseActionServer
  typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;  
  

/*
 >> MoveBaseState RecoveryTrigger 状态机
 RecoveryTrigger 对应的是在相应状态下出现失败

  PLANING状态      唤醒 planThread
  planThread 找到plan进入CONTROLLING状态, 如果找不到进入CLEARING,recovery_trigger_ = PLANNING_R

  CONTROLLING状态 
  首先检测 是否到达终点, 如果达到终点,resetState() 关闭planThread runPlanner = false
  如果震荡条件满足, state_ = CLEARING; recovery_trigger_ = OSCILLATION_R

  tc_->computeVelocityCommands(cmd_vel)
  如果局部路径规划失败, 检测是否超时, 超时则state_ = CLEARING; recovery_trigger_ = CONTROLLING_R;
*/
  enum MoveBaseState {
    PLANNING,  //全局
    CONTROLLING, //局部
    CLEARING
  };



  /**
   * @class MoveBase
   * @brief A class that uses the actionlib::ActionServer interface that moves the robot base to a goal location.
   */
  class GlobalNavi {
    public:
      /**
       * @brief  Constructor for the actions
       * @param tf A reference to a TransformListener
       */
      GlobalNavi(tf::TransformListener& tf); 
      virtual ~GlobalNavi();


       // action server的 feedback 
      bool executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan);

    private:
      /**
       * @brief  A service call that clears the costmaps of obstacles
       * @param req The service request 
       * @param resp The service response
       * @return True if the service call succeeds, false otherwise
       * 清除Costmap上的 obstables的服务
       */
      bool clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

      /**
       * @brief  A service call that can be made when the action is inactive that will return a plan
       * @param  req The goal request
       * @param  resp The plan request
       * @return True if planning succeeded, false otherwise
       */
      bool planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);

      /**
       * @brief  Make a new global plan
       * @param  goal The goal to plan to
       * @param  plan Will be filled in with the plan made by the planner
       * @return  True if planning succeeds, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);


      /**
       * @brief  Clears obstacles within a window around the robot
       * @param size_x The x size of the window
       * @param size_y The y size of the window
       */
      void clearCostmapWindows(double size_x, double size_y);

      /**
       * @brief  Publishes a velocity command of zero to the base
       */
      void publishZeroVelocity();

      /**
       * @brief  Reset the state of the move_base action and send a zero velocity command to the base
       */
      void resetState();
      // get a goal 
      void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

      void planThread();

      // extcute the goal and make plan
      void executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);

      bool isQuaternionValid(const geometry_msgs::Quaternion& q);

      double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

      geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);

      /**
       * @brief This is used to wake the planner at periodic intervals.
       */
      void wakePlanner(const ros::TimerEvent& event);

       /* ******************************************************************************
          add obstable navigation 
          if have Obstable Points, need to stop
        ******************************************************************************* */
        void velodyneDataCallBack(const sensor_msgs::PointCloud2ConstPtr&  velodyneData);
        void laserDataCallBack(const sensor_msgs::LaserScanConstPtr scan_msg);

        void visiualization_waypoints(std::vector<geometry_msgs::PoseStamped>* waypoints);
  
      tf::TransformListener& tf_;  //tf 坐标变换关系

      MoveBaseActionServer* as_; // Action server

      boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_; //global planner实例化对象
      costmap_2d::Costmap2DROS* planner_costmap_ros_; //维护costMap的类, 有global costMap和 local costMap      
      std::string robot_base_frame_, global_frame_;


      tf::Stamped<tf::Pose> global_pose_;  //定义一个全局坐标系
      double planner_frequency_; //全局规划,局部规划的频率, 内切,外切半径的控制
      double planner_patience_;  //全局规划  patience ｔｉｍｅ
      int32_t max_planning_retries_;  //规划失败 重试的次数
      uint32_t planning_retries_;
      double  clearing_radius_; 
      ros::Publisher current_goal_pub_, vel_pub_, action_goal_pub_;  //发布话题
      ros::Subscriber goal_sub_;        //订阅话题
      ros::ServiceServer make_plan_srv_, clear_costmaps_srv_; //定义两个服务
      bool shutdown_costmaps_, clearing_rotation_allowed_;
      double oscillation_timeout_, oscillation_distance_; //震荡


      MoveBaseState state_; //movebase state enum


      ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_; //时间记录
      geometry_msgs::PoseStamped oscillation_pose_;

      // 加载类 pluginlib::ClassLoader 
      pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;


      //set up plan triple buffer
      std::vector<geometry_msgs::PoseStamped>* planner_plan_;
      std::vector<geometry_msgs::PoseStamped>* latest_plan_;


      // set up the planner's thread
      // runPlanner_  planner_cond_ 通过这两个变量来控制planThread的运行
      bool runPlanner_;
      boost::recursive_mutex        planner_mutex_;
      boost::condition_variable_any planner_cond_; //boost_thread的条件变量
      geometry_msgs::PoseStamped    planner_goal_;
      boost::thread*                planner_thread_;


    //   boost::recursive_mutex configuration_mutex_;
    //   dynamic_reconfigure::Server<move_base::MoveBaseConfig> *dsrv_;      
    //   void reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level);
    //   move_base::MoveBaseConfig last_config_;
    //   move_base::MoveBaseConfig default_config_;

      bool setup_, p_freq_change_, c_freq_change_;
      bool new_global_plan_;

      // add flag of obstable detect
      bool hasObstablePoints;  //(true need stop)
      float safety_distance = 0.5; //安全阈值
      int  safety_tolerance = 1; //point数量 

      // add subscriber of laser ******************
      ros::Subscriber laserScan_sub;
      ros::Subscriber velodyneData_sub;
      ros::Publisher  obstableMsg_pub;   
      ros::Publisher  path_waypoints_pub;        
  }; // end of class global_navi

}; //end of namespace global_navi
#endif

