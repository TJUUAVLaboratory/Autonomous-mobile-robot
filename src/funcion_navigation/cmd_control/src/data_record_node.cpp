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

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int waypoints_num;
std::vector<geometry_msgs::PoseStamped> waypoints_in;
std::vector<geometry_msgs::PoseStamped> waypoints_out;
bool move_flag = false;
ros::Publisher marker_pub;
int window_size = 5;

std::string waypoints_file;
std::ofstream save_points;

// yaw angle ==> pose orientation
void set_angle(geometry_msgs::PoseStamped *pose, double angle)
{
    pose->pose.orientation = tf::createQuaternionMsgFromYaw(angle);
}

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
        if(save_points.fail())
            ROS_ERROR_STREAM("can not open the file of waypoints" << waypoints_file );
        //waypoints_in = waypoints_in;
        for (std::size_t i = 0; i < waypoints_in.size(); i++)
        {
            setAngleBasedOnPositionDerivative(waypoints_in, i); //修改航向
            save_points << waypoints_in[i].pose.position.x << " "
                        << waypoints_in[i].pose.position.x << " "
                        << waypoints_in[i].pose.position.x << " "
                        << waypoints_in[i].pose.orientation.x << " "
                        << waypoints_in[i].pose.orientation.x << " "
                        << waypoints_in[i].pose.orientation.x << " "
                        << waypoints_in[i].pose.orientation.x << " "
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
    nh.param("waypoints_num", waypoints_num, 20); // the number of waypoints
    nh.param("waypoints_file", waypoints_file, std::string("waypoints.txt"));

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

    std::vector<geometry_msgs::PoseStamped>::iterator forward_goal = waypoints_in.begin();
    while (ros::ok())
    {
        if ((!waypoints_out.empty()) && move_flag == true)
        {
            ROS_INFO("the auto move thread");
            move_base_msgs::MoveBaseGoal goal;

            goal.target_pose.header.frame_id = (*forward_goal).header.frame_id;
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose.position.x = (*forward_goal).pose.position.x;
            goal.target_pose.pose.position.y = (*forward_goal).pose.position.y;

            goal.target_pose.pose.orientation.x = (*forward_goal).pose.orientation.x;
            goal.target_pose.pose.orientation.y = (*forward_goal).pose.orientation.y;
            goal.target_pose.pose.orientation.z = (*forward_goal).pose.orientation.z;
            goal.target_pose.pose.orientation.w = (*forward_goal).pose.orientation.w;
            movebase_client.sendGoal(goal);
            //movebase_client.sendGoalAndWait(goal, ros::Duration(20), ros::Duration(20));
            bool finished_within_time = movebase_client.waitForResult(ros::Duration(30));
            if (!finished_within_time)
            {
                movebase_client.cancelGoal();
                continue;
            }
            else
            {
                actionlib::SimpleClientGoalState status = movebase_client.getState();
                if (status == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED)
                    continue;
            }

            forward_goal++;
            if (forward_goal == waypoints_in.end())
            {
                move_flag = false;
            }
        }
        rate_loop.sleep();
    }
    return 0;
}
