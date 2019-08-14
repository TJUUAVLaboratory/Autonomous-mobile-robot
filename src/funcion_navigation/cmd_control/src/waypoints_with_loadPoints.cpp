#include <iostream>
#include <algorithm>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib/client/simple_action_client.h> //move_base action client
#include <visualization_msgs/Marker.h>
#include <boost/thread.hpp>
#include <vector>
#include <map>
#include <fstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h> //amcl initialpose
#include <geometry_msgs/Twist.h> //imit cmd
#include <geometry_msgs/PoseArray.h> // read particalCloud

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int waypoints_num;
std::vector<geometry_msgs::PoseStamped> waypoints_in;
std::vector<geometry_msgs::PoseStamped> waypoints_out;
bool move_flag = false;
ros::Publisher marker_pub;
ros::Publisher initPose_pub;
ros::Publisher initOrientationPub;
ros::Subscriber particalCloudSub;
ros::Subscriber amclPoseSub;
int window_size = 1;
double every_distence;

std::string waypoints_file;
std::string load_waypoints_file;
std::ofstream save_points;
std::ifstream load_points;
std::vector<geometry_msgs::Pose> partical_Poses;

geometry_msgs::PoseWithCovarianceStamped currentPose;
geometry_msgs::PoseWithCovarianceStamped  amclInitPose;

//
inline double distencePairPose(geometry_msgs::Pose poseA, geometry_msgs::Pose poseB)
{
   return pow((poseA.position.x - poseB.position.x), 2.0) + pow((poseA.position.y - poseB.position.y), 2.0);
}

void pub_amcl_initialPose()
{
    amclInitPose.header.frame_id = std::string("map");
    amclInitPose.header.stamp = ros::Time::now();

    amclInitPose.pose.pose.position.x = -0.131986;
    amclInitPose.pose.pose.position.y = -2.27238;
    amclInitPose.pose.pose.position.z = 0;

    amclInitPose.pose.pose.orientation.x = 0;
    amclInitPose.pose.pose.orientation.x = 0;
    amclInitPose.pose.pose.orientation.x = -0.730745;
    amclInitPose.pose.pose.orientation.x = 0.682651;

    initPose_pub.publish(amclInitPose);
    ROS_INFO_STREAM("pub init pose:"
                    << " x: " << amclInitPose.pose.pose.position.x
                    << " y:" << amclInitPose.pose.pose.position.y);

}

// pub  init cmd
void pub_init_orientation_cmd(float thata)
{
    
    geometry_msgs::Twist cmd;
    cmd.angular.z = thata;
    initOrientationPub.publish(cmd);
    ros::Duration(3.0);
}



// yaw angle ==> pose orientation
void set_angle(geometry_msgs::PoseStamped *pose, double angle)
{
    pose->pose.orientation = tf::createQuaternionMsgFromYaw(angle);
}

// 设置航点角度
void setAngleBasedOnPositionDerivative(std::vector<geometry_msgs::PoseStamped> &path, int index)
{
    int index0 = std::max(0, index - window_size);
    int index1 = std::min((int)path.size() - 1, index + window_size);

    double x0 = path[index0].pose.position.x,
           y0 = path[index0].pose.position.y,
           x1 = path[index1].pose.position.x,
           y1 = path[index1].pose.position.y;

    double angle = atan2(y1 - y0, x1 - x0);
    set_angle(&path[index], angle);
}

void visualization_waypoints(const std::vector<geometry_msgs::PoseStamped> &waypoints)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = waypoints.front().header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.ns = "waypoints";

    marker.type = visualization_msgs::Marker::POINTS;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.color.g = 1.0;
    marker.color.a = 1.0;
    marker.id = 10000;
    // marker.lifetime = ros::Duration(2.0);
    for (std::size_t i = 0; i < waypoints.size(); i++)
    {
        geometry_msgs::Point point;
        point.x = waypoints[i].pose.position.x;
        point.y = waypoints[i].pose.position.y;
        point.z = 0;
        marker.points.push_back(point);
    }
    marker_pub.publish(marker);
}

// loda the waypoints
bool wayPointLoad(std::vector<geometry_msgs::PoseStamped> &wayPoints)
{
    load_points.open(load_waypoints_file.c_str());
    if (load_points.fail())
    {
        ROS_ERROR_STREAM("can not open the file of: " << load_waypoints_file);
        return false;
    }
    // read every points
    while (!load_points.eof())
    {
        std::string line;
        std::getline(load_points, line);
        if(! line.empty())
        {
            std::stringstream one_point;
            one_point << line;
            
            geometry_msgs::PoseStamped waypoint_pose;
            one_point >> waypoint_pose.pose.position.x;
            one_point >> waypoint_pose.pose.position.y;
            one_point >> waypoint_pose.pose.position.z;

            one_point >> waypoint_pose.pose.orientation.x;
            one_point >> waypoint_pose.pose.orientation.y;
            one_point >> waypoint_pose.pose.orientation.z;
            one_point >> waypoint_pose.pose.orientation.w;

            waypoint_pose.header.frame_id = "map";
            waypoint_pose.header.stamp = ros::Time::now();

            wayPoints.push_back(waypoint_pose);
        }
    }
    if(!wayPoints.empty())
    {
        ROS_WARN_STREAM("load the waypoints size: "<< wayPoints.size());
        visualization_waypoints(wayPoints);
    }
    else
    {
        ROS_ERROR("load the waypoints is empty");
    }
    
    return true;

}

void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &amclPose_msg)
{
    currentPose.pose.pose = amclPose_msg->pose.pose;
}


void particalCloudCallback(const geometry_msgs::PoseArrayConstPtr &partical_msg)
{
    int partical_size = partical_msg->poses.size();
    //if(partical_size>0)
    //    partical_Poses.clear();

    geometry_msgs::Pose current_pose = currentPose.pose.pose;
    double distence_sum = 0;
    for (int i=0; i<partical_size; i++)
    {
       // partical_Poses.push_back(partical_msg->poses[i]);
        distence_sum += distencePairPose(current_pose, partical_msg->poses[i]);
    }
    every_distence = distence_sum/partical_size;
    //ROS_WARN_STREAM("the every distence: " << every_distence);       
}

// received ervey waypoints
void wayPointsCallback(const geometry_msgs::PointStampedPtr &point_msg)
{
    static int waypoint_id = 0;
    geometry_msgs::PoseStamped temp_waypoint;
    temp_waypoint.header.frame_id = point_msg->header.frame_id;

    temp_waypoint.pose.position.x = point_msg->point.x;
    temp_waypoint.pose.position.y = point_msg->point.y;
    temp_waypoint.pose.position.z = 0;

    waypoints_in.push_back(temp_waypoint);
    waypoint_id++;

    ROS_INFO_STREAM("waypoint_size: " << waypoints_in.size());
    visualization_waypoints(waypoints_in);
    if (waypoint_id >= waypoints_num)
    {
        save_points.open(waypoints_file);
        if (save_points.fail())
            ROS_ERROR_STREAM("can not open the file of waypoints" << waypoints_file);
        //waypoints_in = waypoints_in;
        for (std::size_t i = 0; i < waypoints_in.size(); i++)
        {
            setAngleBasedOnPositionDerivative(waypoints_in, i); //修改航向
            save_points << waypoints_in[i].pose.position.x << " "
                        << waypoints_in[i].pose.position.y << " "
                        << waypoints_in[i].pose.position.z << " "
                        << waypoints_in[i].pose.orientation.x << " "
                        << waypoints_in[i].pose.orientation.y << " "
                        << waypoints_in[i].pose.orientation.z << " "
                        << waypoints_in[i].pose.orientation.w << " "
                        << std::endl;
        }
        save_points.close();
        ROS_INFO("save the waypoints");
        move_flag = true;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "data_record_node");
    ros::NodeHandle nh("~");
    nh.param("waypoints_num", waypoints_num, 5); // the number of waypoints
    nh.param("waypoints_file", waypoints_file, std::string("waypoints.txt"));
    nh.param("load_waypoints_file", load_waypoints_file, std::string("waypoints_load.txt"));
    
    marker_pub = nh.advertise<visualization_msgs::Marker>("goal_waypoints", 10);
    initPose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 5);
    initOrientationPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
    particalCloudSub = nh.subscribe("/particlecloud", 1, &particalCloudCallback);
    amclPoseSub = nh.subscribe("/amcl_pose", 1, &amclPoseCallback);
    //sub waypoint
    ros::Subscriber wayPointsSub = nh.subscribe("/clicked_point", 1, &wayPointsCallback);
    ros::Rate rate_loop(30);
    MoveBaseClient movebase_client("move_base", true);
    while (!movebase_client.waitForServer(ros::Duration(2.0)))
    {
        ROS_INFO("waiting for the move_base server to connect");
    }
    ros::AsyncSpinner spinner(4);
    spinner.start();

    //load the waypoints files;
    if(wayPointLoad(waypoints_in))
    {
        ROS_INFO("the waypoints are ready");
        //move_flag = true;
    }

    // set and pub init pose
    pub_amcl_initialPose();

    // cmd to orientation
    do
    {
      pub_init_orientation_cmd(3.1415);
      move_flag = false;
      ros::Duration(3.0).sleep();
      ROS_WARN("waitting for relocalization");
    }while(every_distence > 0.005);

    if(every_distence < 0.005)
        move_flag = true;

    std::vector<geometry_msgs::PoseStamped>::iterator forward_goal = waypoints_in.begin();
    int point_id = 0;
    while (ros::ok())
    {

        if ((!waypoints_in.empty()) && move_flag == true)
        {
            ROS_INFO("the auto move thread");
            move_base_msgs::MoveBaseGoal goal;

            goal.target_pose.header.frame_id = waypoints_in[point_id].header.frame_id;
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose.position.x = waypoints_in[point_id].pose.position.x;
            goal.target_pose.pose.position.y = waypoints_in[point_id].pose.position.y;

            goal.target_pose.pose.orientation.x = waypoints_in[point_id].pose.orientation.x;
            goal.target_pose.pose.orientation.y = waypoints_in[point_id].pose.orientation.y;
            goal.target_pose.pose.orientation.z = waypoints_in[point_id].pose.orientation.z;
            goal.target_pose.pose.orientation.w = waypoints_in[point_id].pose.orientation.w;
            movebase_client.sendGoal(goal);
            //movebase_client.sendGoalAndWait(goal, ros::Duration(20), ros::Duration(20));
            bool finished_within_time = movebase_client.waitForResult(ros::Duration(30));
            if (!finished_within_time)
            {
                movebase_client.cancelGoal();
                continue;
            }

            forward_goal++;
            point_id++;
            if (forward_goal == waypoints_in.end())
            {
                forward_goal == waypoints_in.begin();
                move_flag = true;
            }
        }
        rate_loop.sleep();
    }
    return 0;
}