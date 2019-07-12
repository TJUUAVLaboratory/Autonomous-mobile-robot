#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib/client/simple_action_client.h> //move_base action client
#include <deque>
#include <map>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int waypoints_num;
std::deque<geometry_msgs::PointStamped> waypoints_out;
std::deque<geometry_msgs::PointStamped> waypoints_in;
bool  move_flag;


// according getting points move to goal
void moveToGoal(std::deque<geometry_msgs::PointStamped> &points)
{
    waypoints_out.clear();
    MoveBaseClient movebase_client("move_base", true);
    while (!movebase_client.waitForServer(ros::Duration(2.0)))
    {
        ROS_INFO("waiting for the move_base server to connect");
    }

    while (!points.empty())
    {
        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = points.front().header.frame_id ;
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = points.front().point.x;
        goal.target_pose.pose.position.y = points.front().point.y;
        goal.target_pose.pose.position.z = 0;

        goal.target_pose.pose.orientation.x = 0;
        goal.target_pose.pose.orientation.y = 0;
        goal.target_pose.pose.orientation.z = 0;
        goal.target_pose.pose.orientation.w = 1;


        // movebase_client.sendGoal(goal);
        movebase_client.sendGoalAndWait(goal, ros::Duration(20), ros::Duration(20));
        //add to next list
        waypoints_out.push_front(points.front());
        points.pop_front();
        ROS_INFO_STREAM("the remanent waypoints: "<< points.size());
        // bool finished_within_time = movebase_client.waitForResult(ros::Duration(60));
        // if (!finished_within_time)
        // {
        //     movebase_client.cancelGoal();
        //     ROS_WARN("time out achieving goal");
        //     continue;
        // }
        // else
        // {
        //     actionlib::SimpleClientGoalState status = movebase_client.getState();
        //     if(status == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED)
        //         continue;
        // }
        if(points.empty())
        {
            points = waypoints_out;
            waypoints_out.clear();

        }
    }
}

void wayPointsCallback(const geometry_msgs::PointStampedPtr &point_msg)
{
    geometry_msgs::PointStamped temp_waypoint;
    temp_waypoint.header.frame_id = point_msg->header.frame_id;

    temp_waypoint.point.x = point_msg->point.x;
    temp_waypoint.point.y = point_msg->point.y;
    temp_waypoint.point.z = 0;

    // ROS_INFO_STREAM("next navigation point: " << temp_waypoint.point.x << ", " << temp_waypoint.point.y);
    waypoints_in.push_back(temp_waypoint);

    ROS_INFO_STREAM("select_point_size: " << waypoints_in.size());

    if (waypoints_in.size() >= waypoints_num)
        moveToGoal(waypoints_in);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "auto_move_node");
    ros::NodeHandle nh("~/auto_move");
    nh.param("waypoints_num", waypoints_num, 5); // the number of waypoints

    //sub waypoint
    ros::Subscriber wayPointsSub = nh.subscribe("/clicked_point", 1, &wayPointsCallback);
    ros::spin();

    return 0;
}