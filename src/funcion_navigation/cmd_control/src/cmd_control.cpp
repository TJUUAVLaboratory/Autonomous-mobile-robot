
#include <ros/ros.h>
#include "cmd_control/keyDataOrParam.h"
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <cmd_control/cmd_control.h>


ros::Publisher pub_cmd_control;
cmd_control::keyDataOrParam msg_switch;

bool server_Callback(cmd_control::cmd_control::Request &req,
                     cmd_control::cmd_control::Response &res)
{
    if(req.cmd == true)
    {
        msg_switch.data = 1;// cmd control
        for(int i=0; i<30; i++)
        {
            pub_cmd_control.publish(msg_switch);
        }
        ROS_INFO("commond control  cmd request");
        return true;
    }
    if(req.cmd == false)
    {
        msg_switch.data = 2;// cmd control
        for(int i=0; i<30; i++)
        {
            pub_cmd_control.publish(msg_switch);
        }
        ROS_INFO("commond control  ipad request");
        return true;
    }    
    res.success = true;
    return false;
}




int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cmd_control_node");
    ros::NodeHandle nh;

    pub_cmd_control = nh.advertise<cmd_control::keyDataOrParam>("keyDataOrParamU2D", 5, true);
    ros::ServiceServer server = nh.advertiseService("/cmd_control_srv", server_Callback);
    

    msg_switch.function = 0;  
    msg_switch.adress = 0x02000001;  
    msg_switch.data = 1;   //command control  not ipad
    
    ros::spin();
    // ros::Rate loop_rate(50);
    // while (ros::ok())
    // {
    //     pub_cmd_control.publish(msg_switch);
    //     // ros::Duration(0.01);
    //     // pub_cmd_control.publish(viz_cmd);
    //     loop_rate.sleep();
    // }


    return 0;

}

