#include<ros/ros.h>
#include "map_srv/mapSave.h"  // mapsave service message
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>

// + clicked pose button  
// + get the current pose
// request the map save
// opencv read the map and filter
// read the map and pose
// add the current pose in the map
// cv_bridge  publish the picture

struct pixel_point
{
    float x;
    float y;
};

class Check_odom_Node
{

private:
    ros::NodeHandle nh;
    ros::ServiceClient client; //map save request srv client
    map_srv::mapSave map_client; // reruest message
    
    ros::Subscriber current_pose_sub; // subscriber of current pose


    // current odom pose and pixel
    geometry_msgs::PointStamped  current_odom_point;
    pixel_point  current_pixel_point; 

    bool map_srv_flag; // true  reuest the message
    bool map_filter_flag; 


public:
    Check_odom_Node(void)
    {
        // map save service
        client = nh.serviceClient<map_srv::mapSave>("/map_save");
        map_client.request.message = "save_map";
        map_client.request.map_path = "/home/aibee/aibee_navi/aibee_navi_0529/exp0528/pathfind";

        // sub clicked_pose get current pose
        current_pose_sub = nh.subscribe("/clicked_pose", 1, &Check_odom_Node::sub_currentPose_Callback, this);
    }

    void sub_currentPose_Callback(const geometry_msgs::PoseStampedPtr& pose_msg)
    {
        ROS_INFO("received  clicked pose");
        current_odom_point.header.frame_id = "/odom";
        current_odom_point.point.x = pose_msg->pose.position.x;
        current_odom_point.point.y = pose_msg->pose.position.y;
        current_odom_point.point.z = 0;

        //request  map save
        map_srv_flag = true;
    }

void map_operate()
{
    while(ros::ok())
    {
        // map save request 
        if(map_srv_flag == true)
        {
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

            map_srv_flag = false;                    
        }

        //
    }
}    

};  // end of the Check_odom_Node




int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Check_odom_Node");
    // ros::NodeHandle nh;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    Check_odom_Node  check_odom;
    check_odom.map_operate();    

    return 0;

}



