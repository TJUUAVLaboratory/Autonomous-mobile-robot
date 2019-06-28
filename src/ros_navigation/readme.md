
## ros navigation 重构


### planning 结构

#### nav_core  planning_plugin 基类

**base_global_planner [`global_planner基类`]**
```
基类名称： nav_core::BaseGlobalPlanner
方法：
virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) 

virtual bool makePlan(const geometry_msgs::PoseStamped& start, 
                            const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan,
                            double& cost)                            
```

**base_local_planner [`local_planner基类`]**
```
基类名称： nav_core::BaseLocalPlanner
virtual void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)

virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)

virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel)

virtual bool isGoalReached()

```

**recovery_behavior [`recovery_behavior基类`]**
```
基类名称： nav_core::RecoveryBehavior
virtual void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap)


virtual void runBehavior() 

```

#### global_planning

