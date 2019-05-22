

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>

using namespace std;


#include <ros/console.h>


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
        next_waypose_pub = nh.advertise<geometry_msgs::PointStamped>("next_waypose_pub", 10, true);

        //subscriber
        NavGoal_sub = nh.subscribe("/move_base_simple/goal", 1, &ObstableNavagation::sub_2D_NavGoal_Callback, this);
        point_sub = nh.subscribe("/clicked_point", 1, &ObstableNavagation::sub_2D_point_Callback, this);
        initialPose_sub = nh.subscribe("/initialpose", 1, &ObstableNavagation::sub_initialPose_Callback, this);
        
    }


    update_publish()
    {
        
    }

    

private:   
    ros::NodeHandle nh;
    ros::Publisher next_waypoint_pub;
    ros::Publisher next_waypose_pub;

    ros::Subscriber NavGoal_sub;
    ros::Subscriber point_sub;
    ros::Subscriber initialPose_sub;
    

    geometry_msgs::PoseStamped nextPose;
    geometry_msgs::PointStamped nextPoint;

    geometry_msgs::PoseStamped lastPose;
    geometry_msgs::PointStamped lastPoint;

    float distance_traveled ;


    
 public:
    void sub_2D_NavGoal_Callback(const geometry_msgs::PoseStampedPtr& pose_msg)
    {
        lastPose = nextPose; 
        nextPose.header.frame_id = pose_msg->header.frame_id;
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

    }

    void sub_2D_point_Callback(const geometry_msgs::PointStampedPtr& point_msg)
    {
        nextPoint.header.frame_id = point_msg->header.frame_id;
        nextPoint.point.x = point_msg->point.x;
        nextPoint.point.y = point_msg->point.y;
        nextPoint.point.z = point_msg->point.z;

        ROS_INFO_STREAM("next navigation point: "<< nextPoint.point.x<<", "<<nextPoint.point.y);

    }

    void sub_initialPose_Callback(const geometry_msgs::PoseWithCovarianceStampedPtr initPose_msg)
    {
        nextPose.header.stamp = initPose_msg->header.stamp;
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
    ObstableNavagation obstableNavigation;
    ros::spin();

    return 0;

}