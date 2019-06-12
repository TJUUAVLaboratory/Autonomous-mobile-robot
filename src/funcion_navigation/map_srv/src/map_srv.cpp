
#include<ros/ros.h>
#include "map_srv/mapSave.h"




int main(int argc, char *argv[])
{
    ros::init(argc, argv, "map_srv_node");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<map_srv::mapSave>("/map_save");
    map_srv::mapSave map_client;
    map_client.request.message = "save_map";
    map_client.request.map_path = "pathfind/full_map.png";

    if(client.call(map_client))
        ROS_INFO_STREAM("get the request: "<< map_client.response.message);

    else 
        ROS_ERROR("Failed to call the service");


    return 0;

}

