
#include<ros/ros.h>
#include <opencv2/opencv.hpp>
#include<nav_msgs/Odometry.h>
#include<std_msgs/String.h>

//  request map save to path
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "fake_odom_node");
    ros::NodeHandle nh;

    ros::Publisher fake_odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50, true);
    ros::Publisher fake_goal_pub = nh.advertise<std_msgs::String>("/aibee_navi", 50, true);
    
    std::string image_path = "/home/aibee/aibee_navi/aibee_navi_0529/exp0528/pathfind/full_map.png";
    cv::Mat  map_image = cv::imread(image_path);
    ROS_INFO_STREAM( "the image size: rows: " << map_image.rows << " cols: "<< map_image.cols);

    for(int h=0; h<map_image.rows; ++h)
    {
        for(int w=0; w<map_image.cols; ++w)
        {
            std::cout << " " << *map_image.ptr(h,w);
        }
        std::cout << std::endl;
    }


    

    return 0;

}



