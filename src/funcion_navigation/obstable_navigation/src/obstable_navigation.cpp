

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/String.h>
#include <math.h>
#include <cmath>
#include <stack>
#include <utility>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib/client/simple_action_client.h> //move_base action client
#include <laser_geometry/laser_geometry.h> // laserScan ==>Pointcloud2

#include <boost/thread.hpp>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
// #include <iostream>
// #include <vector>
using namespace std;
/*
    - 订阅 next_waypoint (2D Nav Goal)
    - 订阅 next_waypose (publish point)

    - publish  next_waypoint
    - publish  next_waypose
    - move_base 规划路径 
    - 移动障碍物 发布obstable distance

*/
class ObstableNavagation
{
public:
    ObstableNavagation(void)
    {
        // publisher
        next_waypoint_pub = nh.advertise<geometry_msgs::PointStamped>("next_waypoint", 10, true);
        next_waypose_pub = nh.advertise<geometry_msgs::PoseStamped>("next_waypose", 10, true);
        //发布障碍物信息
        obstableMsg_pub = nh.advertise<std_msgs::String>("obstableMsg", 10, true);

        //pub robot cmd
        pointCloud_pub = nh.advertise<sensor_msgs::PointCloud>("pointCloud", 5, true);

        //subscriber
        //from RVIZ
        // NavGoal_sub = nh.subscribe("/move_base_simple/goal", 1, &ObstableNavagation::sub_2D_NavGoal_Callback, this);
        NavGoal_sub = nh.subscribe("/clicked_pose", 1, &ObstableNavagation::sub_2D_NavGoal_Callback, this);
        point_sub = nh.subscribe("/clicked_point", 1, &ObstableNavagation::sub_2D_point_Callback, this);
        initialPose_sub = nh.subscribe("/initialpose", 1, &ObstableNavagation::sub_initialPose_Callback, this);
        laserScan_sub = nh.subscribe("/scan", 1, &ObstableNavagation::sub_laserScan_Callback, this);

           

        // obstable_thread = new boost::thread(boost::bind(&ObstableNavagation::moveToGoal, this));
    
     
    }

// according getting poses move to goal
void moveToPose(stack<geometry_msgs::PoseStamped>& poses)
 {
        //启动服务 navigation_control
        MoveBaseClient movebase_client("move_base", true);
        while (!movebase_client.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("waiting for the move_base server to connect");
        }
        while(!poses.empty())
        {
            move_base_msgs::MoveBaseGoal goal;

            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            
            goal.target_pose.pose.position.x = poses.top().pose.position.x;
            goal.target_pose.pose.position.y = poses.top().pose.position.y;
            goal.target_pose.pose.position.z = 0;

            goal.target_pose.pose.orientation.x = poses.top().pose.orientation.x;
            goal.target_pose.pose.orientation.y = poses.top().pose.orientation.y;
            goal.target_pose.pose.orientation.z = poses.top().pose.orientation.z;
            goal.target_pose.pose.orientation.w = poses.top().pose.orientation.w;

            poses.pop();

            movebase_client.sendGoal(goal);
            bool finished_within_time = movebase_client.waitForResult(ros::Duration(60));

            if(!finished_within_time)
            {
                movebase_client.cancelGoal();
                ROS_WARN("time out achieving goal");
            }
            else 
            {        
                actionlib::SimpleClientGoalState status = movebase_client.getState();                    
            }
        }        
    }

// according getting points move to goal
    void moveToGoal(stack<geometry_msgs::PointStamped>& points)
    {
        MoveBaseClient movebase_client("move_base", true);
        while (!movebase_client.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("waiting for the move_base server to connect");
        }

        while(!points.empty())
        {
            move_base_msgs::MoveBaseGoal goal;

            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            
            goal.target_pose.pose.position.x = points.top().point.x;
            goal.target_pose.pose.position.y = points.top().point.y;
            goal.target_pose.pose.position.z = 0;

            goal.target_pose.pose.orientation.x = 0;
            goal.target_pose.pose.orientation.y = 0;
            goal.target_pose.pose.orientation.z = 0;
            goal.target_pose.pose.orientation.w = 1;

            points.pop();

            movebase_client.sendGoal(goal);
            bool finished_within_time = movebase_client.waitForResult(ros::Duration(60));

            if(!finished_within_time)
            {
                movebase_client.cancelGoal();
                ROS_WARN("time out achieving goal");
            }
            else 
            {        
                actionlib::SimpleClientGoalState status = movebase_client.getState();                    
            }
        }        
    }

public:
    stack<geometry_msgs::PointStamped> selectPoints;
    stack<geometry_msgs::PoseStamped> selectPoses;
    

private:   
    ros::NodeHandle nh;
    ros::Publisher next_waypoint_pub;
    ros::Publisher next_waypose_pub;
    ros::Publisher pointCloud_pub;
    ros::Publisher obstableMsg_pub;   
    
    

    ros::Subscriber NavGoal_sub;
    ros::Subscriber point_sub;
    ros::Subscriber initialPose_sub;
    ros::Subscriber laserScan_sub;    

    geometry_msgs::PoseStamped nextPose;
    geometry_msgs::PointStamped nextPoint;

    geometry_msgs::PoseStamped lastPose;
    geometry_msgs::PointStamped lastPoint;
    
    boost::thread* obstable_thread;

    

    float safety_distance = 0.8; //安全阈值
    int  safety_tolerance = 1; //point数量
    float distance_traveled ;      
//     enum goal_states {
//             "PENDING",
//             "ACTIVE",
//             "RECALLED",
//             "REJECTED",
//             "PREEMPTED",
//             "ABORTED",
//             "SUCCEEDED",
//             "LOST"
//   };

    
 public:
    void sub_2D_NavGoal_Callback(const geometry_msgs::PoseStampedPtr& pose_msg)
    {
        lastPose = nextPose; 
        nextPose.header.frame_id = "/odom";
        nextPose.pose.position.x = pose_msg->pose.position.x;
        nextPose.pose.position.y = pose_msg->pose.position.y;
        nextPose.pose.position.z = 0;

        nextPose.pose.orientation.x = 0;
        nextPose.pose.orientation.y = 0;
        nextPose.pose.orientation.z = 0;        
        nextPose.pose.orientation.w = 1;  

        float distance = sqrt(
                                pow((nextPose.pose.position.x-lastPose.pose.position.x),2) +
                                pow((nextPose.pose.position.y-lastPose.pose.position.y),2)

        );
        distance_traveled += distance;

        ROS_INFO_STREAM("current distance: " << distance);
        ROS_INFO_STREAM("traveled distance: " << distance_traveled);
        ROS_INFO_STREAM("next  navigation goal pose: " << nextPose.pose.position.x <<", "<<nextPose.pose.position.y);
        next_waypose_pub.publish(nextPose);
        selectPoses.push(nextPose);
        ROS_INFO_STREAM("select_pose_size: "<< selectPoses.size());

        if(selectPoses.size() >=5)
            moveToPose(selectPoses);
    }


    void sub_2D_point_Callback(const geometry_msgs::PointStampedPtr& point_msg)
    {
        nextPoint.header.frame_id = "/odom";
        nextPoint.point.x = point_msg->point.x;
        nextPoint.point.y = point_msg->point.y;
        nextPoint.point.z = point_msg->point.z;

        ROS_INFO_STREAM("next navigation point: "<< nextPoint.point.x<<", "<<nextPoint.point.y);
        next_waypoint_pub.publish(nextPoint);
        selectPoints.push(nextPoint);
        ROS_INFO_STREAM("select_point_size: "<< selectPoints.size());

        if(selectPoints.size() >=10)
            moveToGoal(selectPoints);
    }

    void sub_initialPose_Callback(const geometry_msgs::PoseWithCovarianceStampedPtr initPose_msg)
    {
        nextPose.header.frame_id = "/odom";
        nextPose.pose.position.x = initPose_msg->pose.pose.position.x;
        nextPose.pose.position.y = initPose_msg->pose.pose.position.y;
        nextPose.pose.position.z = initPose_msg->pose.pose.position.z;

        nextPose.pose.orientation.w = initPose_msg->pose.pose.orientation.w;
        nextPose.pose.orientation.x = initPose_msg->pose.pose.orientation.x;
        nextPose.pose.orientation.y = initPose_msg->pose.pose.orientation.y;
        nextPose.pose.orientation.z = initPose_msg->pose.pose.orientation.z;   
    }

    void sub_laserScan_Callback(const sensor_msgs::LaserScanConstPtr scan_msg)
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
        // ROS_INFO_STREAM("laserScan size: " << scan.ranges.size());

        laser_geometry::LaserProjection projector;
        // sensor_msgs::PointCloud2 cloud_out;
        sensor_msgs::PointCloud cloud_out;
        projector.projectLaser(scan, cloud_out, -1.0, laser_geometry::channel_option::Index);
        // ROS_INFO_STREAM("pointCloud size: " << cloud_out.points.size());
        pointCloud_pub.publish(cloud_out);

        
        /*
        for(int i=0; i<cloud_out.points.size(); i++)
        {
            static int count = 0;
            if(hypot(cloud_out.points[i].x, cloud_out.points[i].y) < safety_distance)
            {
                // ROS_WARN("beyond the threshold of safety distance");
                count++;
            }

            if(count > safety_tolerance)
            {
                ROS_WARN("send  safety topic");
                std_msgs::String obstable_msg;
                obstable_msg.data = "obstable beyond the threshold of safety distance";
                obstableMsg_pub.publish(obstable_msg);
                count = 0;
            }
            else 
                ROS_DEBUG("that is OK");             

        }
        */



    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstable_navigation");
    ROS_INFO("obstable_navigation is running ...");
    ObstableNavagation obstableNavigation;
    // obstableNavigation.update_publish();
    // if(obstableNavigation.selectPoints.size() >=10)
    //     obstableNavigation.moveToGoal(obstableNavigation.selectPoints);
    ros::spin();

    return 0;

}