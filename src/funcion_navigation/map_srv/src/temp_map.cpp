
#include<ros/ros.h>
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
#include <vector>
#include <nav_msgs/OccupancyGrid.h>

// + clicked pose button  
// + get the current pose
// request the map save
// opencv read the map and filter
// read the map and pose
// add the current pose in the map
// cv_bridge  publish the picture



// global varient
ros::Subscriber tempCostmap;
ros::Publisher stringMap;

std_msgs::String strObstacleMap;


std::string getStringFromFloat(int f)
  {
      std::ostringstream buffer;
      buffer << f;
      return buffer.str();
 }


void costmap_Callback(const nav_msgs::OccupancyGrid map)
{
    ROS_INFO("received_occupancyGrid");
    // strObstacleMap.data = map.data;
    std::string  data_string;
    for(int i=0; i< map.data.size(); i++)
    {
        data_string = data_string + getStringFromFloat(map.data[i]) + " ";
    }
    strObstacleMap.data = data_string;
    stringMap.publish(strObstacleMap);
    ROS_INFO("publish_obstacle costmap");
}


// void Costmap2DPublisher::tempCostmap_pub()
// {
//   boost::unique_lock<Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
//   nav_msgs::OccupancyGrid grid;
//   double resolution = costmap_->getResolution();
//   grid.info.width = costmap_->getSizeInCellsX();
//   grid.info.height = costmap_->getSizeInCellsY();
//   double wx, wy;
//   costmap_->mapToWorld(0, 0, wx, wy);
//   grid.info.origin.position.x = wx - resolution / 2;
//   grid.info.origin.position.y = wy - resolution / 2;
//   grid.data.resize(grid.info.width * grid.info.height);

//   unsigned char* data = costmap_->getCharMap(); // ros costmap data
//   for (unsigned int i = 0; i < grid.data.size(); i++)
//   {
//     grid.data[i] = cost_translation_table_[ data[ i ]];
//   }

//   if(temp_costmap_pub.getNumSubscribers() == 0)
//     return;

//   else
//   {
//     std_msgs::String  map_string;
//     std::ostringstream string;
//     string << "resolution: " << resolution << ", "
//            << "width: " << grid.info.width << ", "
//            << "height: " << grid.info.height << ", "
//            << "origin_x: " << grid.info.origin.position.x << ", "
//            << "origin_y: " << grid.info.origin.position.y << ", "
//            << "data: ";
//     // for(int i=0; i< grid.data.size(); i++)
//     //   string << grid.data[i] << '';
//     // std::string data_str =  data;
//   }
// }


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_map");
    // ros::NodeHandle nh;
    // map save service
    ros::NodeHandle nh;
   
    tempCostmap = nh.subscribe("/aibee_navi/temp_costmap/costmap", 1, &costmap_Callback);
    stringMap = nh.advertise<std_msgs::String>("/obstalce_map",1, true);

    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        stringMap.publish(strObstacleMap);
        ros::spinOnce(); 
        loop_rate.sleep();
    }

    return 0;

}



