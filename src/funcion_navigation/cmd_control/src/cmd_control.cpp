
#include<ros/ros.h>
#include "cmd_control/keyDataOrParam.h"
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

ros::Publisher pub_cmd_control;
visualization_msgs::Marker  viz_cmd;

void cmd_vel_Callback(const geometry_msgs::TwistPtr cmd)
{
    ROS_INFO("received cmd_vel ");
    viz_cmd.header.frame_id = "/base_link";
    viz_cmd.header.stamp = ros::Time::now();
    viz_cmd.action = visualization_msgs::Marker::ADD;
    viz_cmd.type = visualization_msgs::Marker::ARROW;
    viz_cmd.ns = "viz_cmd";
    viz_cmd.id = 0;
    viz_cmd.pose.position.x = cmd->linear.x;
    viz_cmd.pose.position.y = cmd->linear.y;
    viz_cmd.pose.position.z = cmd->linear.z;

    viz_cmd.pose.orientation.w = 1;
    viz_cmd.pose.orientation.x = 0;
    viz_cmd.pose.orientation.y = 0;
    viz_cmd.pose.orientation.z = 0;

    viz_cmd.scale.x = 0.1;
    viz_cmd.scale.y = 0.1;
    viz_cmd.scale.z = 0.1;

    viz_cmd.color.r = 1.0f;
    viz_cmd.color.a = 1.0f;

    // viz_cmd.lifetime = ros::Duration();

    // geometry_msgs::Quaternion  quat(cmd->angular.x, cmd->angular.y,cmd->angular.z);
    pub_cmd_control.publish(viz_cmd);

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cmd_control_node");
    ros::NodeHandle nh;

    pub_cmd_control = nh.advertise<cmd_control::keyDataOrParam>("keyDataOrParamU2D", 5, true);
    ros::Subscriber cmd_vel_cmd = nh.subscribe("/cmd_vel", 1, &cmd_vel_Callback);
    
    cmd_control::keyDataOrParam msg_switch;

    msg_switch.function = 0;  
    msg_switch.adress = 0x02000001;  
    msg_switch.data = 1;   //command control  not ipad
    

    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        pub_cmd_control.publish(msg_switch);
        // ros::Duration(0.01);
        // pub_cmd_control.publish(viz_cmd);
        loop_rate.sleep();
    }

    msg_switch.data = 2;// ipad control
    for(int i=0; i<51; i++)
    {
        pub_cmd_control.publish(msg_switch);
    }

    return 0;

}

