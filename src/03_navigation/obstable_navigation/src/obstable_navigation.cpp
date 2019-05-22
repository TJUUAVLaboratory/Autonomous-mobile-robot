

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <stack>
#include <utility>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib/client/simple_action_client.h> //move_base action client

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
        next_waypose_pub = nh.advertise<geometry_msgs::PoseStamped>("next_waypose_pub", 10, true);

        //pub robot cmd
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);

        //subscriber
        //from RVIZ
        NavGoal_sub = nh.subscribe("/move_base_simple/goal", 1, &ObstableNavagation::sub_2D_NavGoal_Callback, this);
        point_sub = nh.subscribe("/clicked_point", 1, &ObstableNavagation::sub_2D_point_Callback, this);
        initialPose_sub = nh.subscribe("/initialpose", 1, &ObstableNavagation::sub_initialPose_Callback, this);

     
    }


    void moveToGoal(stack<geometry_msgs::PointStamped>& points)
    {
        MoveBaseClient movebase_client("move_base", true);
        while (!movebase_client.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("waiting for the move_base server to connect");
        }

        //循环to goal
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
    

private:   
    ros::NodeHandle nh;
    ros::Publisher next_waypoint_pub;
    ros::Publisher next_waypose_pub;
    ros::Publisher cmd_vel_pub;
    

    ros::Subscriber NavGoal_sub;
    ros::Subscriber point_sub;
    ros::Subscriber initialPose_sub;
    

    geometry_msgs::PoseStamped nextPose;
    geometry_msgs::PointStamped nextPoint;

    geometry_msgs::PoseStamped lastPose;
    geometry_msgs::PointStamped lastPoint;
    

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
        nextPose.pose.position.z = pose_msg->pose.position.z;

        nextPose.pose.orientation.x = pose_msg->pose.orientation.x;
        nextPose.pose.orientation.y = pose_msg->pose.orientation.y;
        nextPose.pose.orientation.z = pose_msg->pose.orientation.z;        
        nextPose.pose.orientation.w = pose_msg->pose.orientation.w;  

        float distance = sqrt(
                                pow((nextPose.pose.position.x-lastPose.pose.position.x),2) +
                                pow((nextPose.pose.position.y-lastPose.pose.position.y),2)

        );
        distance_traveled += distance;

        ROS_INFO_STREAM("current distance: " << distance);
        ROS_INFO_STREAM("traveled distance: " << distance_traveled);
        ROS_INFO_STREAM("next  navigation goal pose: " << nextPose.pose.position.x <<", "<<nextPose.pose.position.y);
        next_waypose_pub.publish(nextPose);
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