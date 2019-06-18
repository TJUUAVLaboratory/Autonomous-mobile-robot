
#include<ros/ros.h>
#include "cmd_control/keyDataOrParam.h"




int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cmd_control_node");
    ros::NodeHandle nh;

    ros::Publisher pub_cmd_control = nh.advertise<cmd_control::keyDataOrParam>("keyDataOrParamU2D", 5, true);
    cmd_control::keyDataOrParam msg_switch;

    msg_switch.function = 0;  
    msg_switch.adress = 0x02000001;  
    msg_switch.data = 1;   //command control  not ipad
    

    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        pub_cmd_control.publish(msg_switch);
        loop_rate.sleep();
    }

    msg_switch.data = 2;// ipad control
    for(int i=0; i<50; i++)
    {
        pub_cmd_control.publish(msg_switch);
    }

    return 0;

}

