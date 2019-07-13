#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib/client/simple_action_client.h> //move_base action client
#include <visualization_msgs/Marker.h>
#include <boost/thread.hpp>
#include <deque>
#include <map>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int waypoints_num;
std::deque<geometry_msgs::PointStamped> waypoints_out;
std::deque<geometry_msgs::PointStamped> waypoints_in;
bool move_flag;
ros::Publisher marker_pub;

// according getting points move to goal
void moveToGoal(std::deque<geometry_msgs::PointStamped> &points)
{
    waypoints_out.clear();
    MoveBaseClient movebase_client("move_base", true);
    while (!movebase_client.waitForServer(ros::Duration(2.0)))
    {
        ROS_INFO("waiting for the move_base server to connect");
    }

    if (!points.empty())
    {
        ROS_INFO("the auto move thread");
        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = points.front().header.frame_id;
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
        ROS_INFO_STREAM("the remanent waypoints: " << points.size());
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
        if (points.empty())
        {
            points = waypoints_out;
            waypoints_out.clear();
        }
    }
}

void visualization_waypoints(const std::deque<geometry_msgs::PointStamped> &waypoints)
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
        point.x = waypoints[i].point.x;
        point.y = waypoints[i].point.y;
        point.z = 0;
        marker.points.push_back(point);
    }
    marker_pub.publish(marker);
}

void wayPointsCallback(const geometry_msgs::PointStampedPtr &point_msg)
{
    geometry_msgs::PointStamped temp_waypoint;
    temp_waypoint.header.frame_id = point_msg->header.frame_id;

    temp_waypoint.point.x = point_msg->point.x;
    temp_waypoint.point.y = point_msg->point.y;
    temp_waypoint.point.z = 0;

    // ROS_INFO_STREAM("next navigation point: " << temp_waypoint.point.x << ", " << temp_waypoint.point.y);
    if (waypoints_in.size() >= waypoints_num)
    {
        waypoints_in.pop_front();
    }
    waypoints_in.push_back(temp_waypoint);

    ROS_INFO_STREAM("select_point_size: " << waypoints_in.size());
    visualization_waypoints(waypoints_in);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "auto_move_node");
    ros::NodeHandle nh("~/auto_move");
    nh.param("waypoints_num", waypoints_num, 5); // the number of waypoints

    marker_pub = nh.advertise<visualization_msgs::Marker>("goal_waypoints", 10);
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

    while (ros::ok())
    {
        if (!waypoints_in.empty())
        {   
            
            ROS_INFO("the auto move thread");
            move_base_msgs::MoveBaseGoal goal;

            goal.target_pose.header.frame_id = waypoints_in.front().header.frame_id;
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose.position.x = waypoints_in.front().point.x;
            goal.target_pose.pose.position.y = waypoints_in.front().point.y;
            goal.target_pose.pose.position.z = 0;

            goal.target_pose.pose.orientation.x = 0;
            goal.target_pose.pose.orientation.y = 0;
            goal.target_pose.pose.orientation.z = 0;
            goal.target_pose.pose.orientation.w = 1;

            // movebase_client.sendGoal(goal);
            movebase_client.sendGoalAndWait(goal, ros::Duration(20), ros::Duration(20));
            //add to next list
            waypoints_out.push_front(waypoints_in.front());
            waypoints_in.pop_front();
            ROS_INFO_STREAM("the remanent waywaypoints_in: " << waypoints_in.size());
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
            if (waypoints_in.empty())
            {
                waypoints_in = waypoints_out;
                waypoints_out.clear();
            }
        }

        rate_loop.sleep();
    }
    return 0;
}