
#include<ros/ros.h>
#include "map_srv/mapSave.h"  // mapsave service message
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Image.h>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include "cJSON.h"
#include <string>

// + clicked pose button  
// + get the current pose
// request the map save
// opencv read the map and filter
// read the map and pose
// add the current pose in the map
// cv_bridge  publish the picture

struct pixel_point
{
    float u;
    float v;
};


// global varient
geometry_msgs::PointStamped  current_odom_point;
geometry_msgs::PointStamped  last_odom_point;
pixel_point  current_pixel_point; 

map_srv::mapSave map_client; // reruest message
ros::ServiceClient client;
cv::Mat  map_image;
image_transport::Publisher pub_map_image;
ros::Publisher fake_goal_pub;
float resolution;

float param[] = {-17.2203, -13.7435, 41, 39.7};


std::string getStringFromFloat(float f)
  {
      std::ostringstream buffer;
      buffer << f;
      return buffer.str();
 }

void path_waypoint_Callback(const std_msgs::StringPtr& path_msg)
{
    std_msgs::String path_data;
    path_data.data = path_msg->data;
    ROS_INFO_STREAM("the waypoints: "<<path_data);

    cJSON*  pathJson = cJSON_Parse(path_data.data.c_str());
    cJSON* json_array = cJSON_GetObjectItem(pathJson, "pathfind_ok");

    for(int i=0; i<cJSON_GetArraySize(json_array); i++)
    {
        cJSON* point = cJSON_GetArrayItem(json_array, i);
        for(int j=0; j<2; j++)
        {
            cJSON* coord = cJSON_GetArrayItem(point,i);
            ROS_INFO_STREAM("test value: "<< coord->valueint);
        }

    }



}


void sub_currentPose_Callback(const geometry_msgs::PoseStampedPtr& pose_msg)
{
    ROS_INFO("received  clicked pose");
    current_odom_point.header.stamp = pose_msg->header.stamp;
    current_odom_point.header.frame_id = "/odom";
    current_odom_point.point.x = pose_msg->pose.position.x;
    current_odom_point.point.y = pose_msg->pose.position.y;
    current_odom_point.point.z = 0;
    ROS_INFO_STREAM("current position: " << current_odom_point.point.x <<" , " << current_odom_point.point.y);
    ROS_INFO_STREAM("last position: " << last_odom_point.point.x <<" , "<< last_odom_point.point.y);
    //request  map save
    if(client.call(map_client))
    {
        ROS_INFO_STREAM("get the request: "<< map_client.response.message);
        ROS_INFO_STREAM("param: " << map_client.response.parameters[0] 
                            << " "<< map_client.response.parameters[1] 
                            << " "<< map_client.response.parameters[2] 
                            << " "<< map_client.response.parameters[3]);

        param[0] = map_client.response.parameters[0];                            
        param[1] = map_client.response.parameters[1];
        param[2] = map_client.response.parameters[2];
        param[3] = map_client.response.parameters[3];

    }

    else 
        ROS_ERROR("Failed to call the service"); 

    std::string image_path = "/home/aibee/aibee_navi/aibee_navi_0529/exp0528/pathfind/full_map.png";
    std::string output_path = "/home/aibee/aibee_navi/aibee_navi_0529/exp0528/pathfind/full_map.png";
    map_image = cv::imread(image_path);
    // cv::Mat  kernel =  cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    // cv::morphologyEx(map_image, map_image, cv::MORPH_CLOSE, kernel);
    // cv::morphologyEx(map_image, map_image, cv::MORPH_OPEN, kernel);

    resolution = param[2]/map_image.rows;
    resolution = (resolution+param[3]/map_image.cols)/2.0;
    ROS_INFO_STREAM("resolution: " << resolution);

    current_pixel_point.u = (current_odom_point.point.x-param[0])/resolution;
    current_pixel_point.v = (current_odom_point.point.y-param[1])/resolution;
    ROS_INFO_STREAM("current pixel point: "<< current_pixel_point.u << " , "<<current_pixel_point.v);
    cv::circle(map_image, cv::Point(current_pixel_point.u, current_pixel_point.v), 10, cv::Scalar(255,255,255));
    cv::imwrite(output_path, map_image);

    // sensor_msgs::ImagePtr image_map_msg;
    // image_map_msg= cv_bridge::CvImage(current_odom_point.header, "mono8", map_image).toImageMsg();
    // pub_map_image.publish(image_map_msg);

    // send target
    std_msgs::String target_msg;
    std::string  target_x, target_y, param_0, param_1, param_2, param_3;
    target_x = getStringFromFloat(last_odom_point.point.x);
    target_y = getStringFromFloat(last_odom_point.point.y);
    param_0 = getStringFromFloat(param[0]);
    param_1 = getStringFromFloat(param[1]);
    param_2 = getStringFromFloat(param[2]);
    param_3 = getStringFromFloat(param[3]);

    std::string msg_string = "[\"start\",\"pathfind/full_map.png\",[" + param_0 +
                                                            ","  + param_1 +
                                                            ","  + param_2 +
                                                            ","  + param_3 + 
                                                            " ],[" + target_x + ","+target_y+"]]";
    cJSON*  json = cJSON_Parse(msg_string.c_str());
    target_msg.data = cJSON_Print(json);
    fake_goal_pub.publish(target_msg);
    last_odom_point = current_odom_point;
       
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Check_odom_Node");
    // ros::NodeHandle nh;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    ROS_INFO("Check_odom_Node");
    // map save service
    ros::NodeHandle nh;
    client = nh.serviceClient<map_srv::mapSave>("/map_save");
    map_client.request.message = "save_map";
    map_client.request.map_path = "/home/aibee/aibee_navi/aibee_navi_0529/exp0528/pathfind";

    // sub clicked_pose get current pose
    ros::Subscriber current_pose_sub = nh.subscribe("/clicked_pose", 1, &sub_currentPose_Callback); 
    ros::Subscriber path_waypoints_sub = nh.subscribe("/aibee_navi_out", 1, &path_waypoint_Callback);
    

    // send  goal
    fake_goal_pub = nh.advertise<std_msgs::String>("/aibee_navi", 1, true);

    //publish map_image
    image_transport::ImageTransport it(nh);
    pub_map_image = it.advertise("/map_image", 2);  

    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        sensor_msgs::ImagePtr image_map_msg;
        image_map_msg= cv_bridge::CvImage(current_odom_point.header, "mono8", map_image).toImageMsg();
        pub_map_image.publish(image_map_msg);
        ros::spinOnce(); 
    }

    return 0;

}



