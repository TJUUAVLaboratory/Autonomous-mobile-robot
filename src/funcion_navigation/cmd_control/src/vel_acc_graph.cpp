

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>
#include <vector>
#include <map>
#include <Eigen/Core>


ros::Time current_time;
ros::Time last_time;
double dalter_t;
int count;
 Eigen::Vector3d linear_vel;
 Eigen::Vector3d angular_vel;
typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> cmd_vel;

// dalter_t  linear_vel angular_vel
std::pair<double, cmd_vel> cmd_info;

std::vector<std::pair<double, cmd_vel>> cmd_list;

void cmd_vel_Callback(const geometry_msgs::TwistPtr& cmd)
{
    count++;
    current_time = ros::Time::now();
    dalter_t = (current_time - last_time).toSec();

    Eigen::Vector3d cmd_linear;
    cmd_linear.x = cmd->linear.x;
    cmd_linear.y = cmd->linear.y;
    cmd_linear.z = cmd->linear.z;

    Eigen::Vector3d cmd_angular;
    cmd_angular.x  =  cmd->angular.x;
    cmd_angular.y  =  cmd->angular.y;
    cmd_angular.z  =  cmd->angular.z;

    cmd_info = std::make_pair<dalter_t, std::make_pair<cmd_linear, cmd_angular> >;


    last_time = current_time;







}

int main(int argc, char* argv[])
{
    ros::init( argc, argv, "vel_cmd_node");
    ros::NodeHandle nh;
    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 1, &cmd_vel_Callback);


}