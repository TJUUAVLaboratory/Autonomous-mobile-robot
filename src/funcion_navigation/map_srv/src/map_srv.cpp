
#include<ros/ros.h>
#include "map_srv/mapSave.h"
//  request map save to path
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "map_srv_node");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<map_srv::mapSave>("/map_save");
    map_srv::mapSave map_client;
    map_client.request.message = "save_map";
    map_client.request.map_path = "/home/aibee/aibee_navi/aibee_navi_0529/exp0528/pathfind";

    if(client.call(map_client))
    {
        ROS_INFO_STREAM("get the request: "<< map_client.response.message);
        ROS_INFO_STREAM("param: " << map_client.response.parameters[0] 
                            << " "<< map_client.response.parameters[1] 
                            << " "<< map_client.response.parameters[2] 
                            << " "<< map_client.response.parameters[3]);
    }

    else 
        ROS_ERROR("Failed to call the service");

    return 0;

}



