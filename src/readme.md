
## ros navigation 重构


### 2. mapping 

**2.0.1 map_server: load map**
```
loadMapFromFile(nav_msgs::GetMap::Response* resp,
                const char* fname, double res, bool negate,
                double occ_th, double free_th, double* origin,
                MapMode mode)
```

**costmap_2d: use occupied map generate layered costmap**

```bash
## plugin 基类名称 Layer
pluginlib::ClassLoader<Layer> plugin_loader_;
## 根据type生成相应的 plugin实例，每一个实例都是给予Layer的 comtmap Layer层
boost::shared_ptr<Layer> plugin = plugin_loader_.createInstance(type);

costmap_plugins.xml
基类：costmap_2d::Layer

plugin类： 
costmap_2d::InflationLayer
costmap_2d::ObstacleLayer
costmap_2d::StaticLayer
costmap_2d::VoxelLayer


initialize(LayeredCostmap* parent, std::string name, tf::TransformListener *tf)


void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y)

void updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)

void deactivate()
void activate() 
void activate() 



```


### 3. planning 结构

#### 3.0. nav_core  planning_plugin 基类

```
基类名称： nav_core::BaseGlobalPlanner
方法：
virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) 

virtual bool makePlan(const geometry_msgs::PoseStamped& start, 
                            const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan,
                            double& cost)                            
```
**3.0.1. base_global_planner [`global_planner类`]**


```
navfn           ==> navfn::NavfnROS                 global_planner plugin
global_planner  ==> global_planner::GlobalPlanner   global_planner plugin


```


**3.0.2. base_local_planner [`local_planner类`]**

```

```

```
基类名称： nav_core::BaseLocalPlanner
virtual void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)

virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)

virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel)

virtual bool isGoalReached()

```

**3.0.3. recovery_behavior [`recovery_behavior基类`]**
```
基类名称： nav_core::RecoveryBehavior
virtual void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap)


virtual void runBehavior() 

```

#### 3.1. global_planning  plugin

**navfn Dijkstra/A*star全局路径规划**
`navfn::NavfnROS`
```
bgp_plugin.xml  lib/libnavfn
plugin_name: navfn::NavfnROS


void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame)

bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan);

bool computePotential(const geometry_msgs::Point& world_point)
double getPointPotential(const geometry_msgs::Point& world_point)
bool validPointPotential(const geometry_msgs::Point& world_point)


bool getPlanFromPotential(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp)
void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, double r, double g, double b, double a);

```

#### 3.2. local_planning  plugin

**base_local_planner DWA动态路径规划**
`base_local_planner::TrajectoryPlannerROS`
```
blp_plugin.xml  lib/libtrajectory_planner_ros

void initialize(std::string name, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* costmap_ros)

bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)

bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel)

bool isGoalReached()        

bool checkTrajectory(double vx_samp, double vy_samp, double vtheta_samp, bool update_map = true)

double scoreTrajectory(double vx_samp, double vy_samp, double vtheta_samp, bool update_map = true)

TrajectoryPlanner* getPlanner() const { return tc_; }

```

## MoveBaseActionServer


## planService

nav_msgs::GetPlan::Request

start
goal

tolerance