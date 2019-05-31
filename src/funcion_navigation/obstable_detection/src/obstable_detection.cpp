
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h> // velodyne sensor data

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <laser_geometry/laser_geometry.h> // laserScan ==>Pointcloud2

#include <std_msgs/String.h>
#include "point_types.h"
#include "obstable_detection/obstable_detection_msg.h"

#include <string>
#include <stdio.h>
#include <stdlib.h>
#include "cJSON.h"
// #include "json/json.h"
/*
    1. 接收两个laser data
    2. 检测到有障碍物点 小于超出安全阈值, 发送话题消息

    vertical velodyne:探测周围障碍物
    slant welodyne :探测地面的障碍物
*/


class ObstableDetection
{
public:
    ObstableDetection(void)
    {
        //提取参数
        ros::NodeHandle private_nh("~");

        //自定义话题名称  
        //subscribe      
        private_nh.param("horizontal_velodyne_topic", horizontal_velodyne_topic, std::string("/static_velodyne/velodyne_points")); //水平雷达数据
        private_nh.param("vertical_velodyne_topic", vertical_velodyne_topic, std::string("/spinning_velodyne/velodyne_points")); //垂直倾斜雷达

        //publish
        private_nh.param("obstableMsg_topic", obstableMsg_topic, std::string("/aibee_navi"));
        private_nh.param("horizontal_obstable_topic", horizontal_obstable_topic, std::string("/obstable/horizontal_obstable_point"));
        private_nh.param("vertical_obstable_topic", vertical_obstable_topic, std::string("/obstable/vertiacl_obstable_point"));

        private_nh.param("obstable_points_frameId", obstable_points_frameId, std::string("/map"));

        // variable 
        private_nh.param("safety_distance", safety_distance, 0.8); //安全范围
        private_nh.param("safety_tolerance", safety_tolerance, 5); //tolerance点的数量


        //发布障碍物信息
        // obstableMsg_pub = nh.advertise<obstable_detection::obstable_detection_msg>("aibee_navi", 10);
        obstableMsg_pub = nh.advertise<std_msgs::String>(obstableMsg_topic, 10);       

        // horizontal velodyne sensor data sub & pub
        horizontal_velodyneSensor_sub = nh.subscribe(horizontal_velodyne_topic, 10, &ObstableDetection::horizontal_velodyneSensor_Callback, this);
        horizontalObstable_pub = nh.advertise<sensor_msgs::PointCloud>(horizontal_obstable_topic, 20);

        // vertical velodyne sensor data sub & pub
        vertical_velodyneSensor_sub = nh.subscribe(vertical_velodyne_topic, 10, &ObstableDetection::vertical_velodyneSensor_Callback, this);
        verticalObstable_pub = nh.advertise<sensor_msgs::PointCloud>(vertical_obstable_topic, 20);

        //仿真测试用 rplidar
        rplidarSensor_sub  = nh.subscribe("/scan", 1, &ObstableDetection::rplidarSensor_Callback, this);
     
    }

private:   
    ros::NodeHandle nh;
    
    ros::Subscriber rplidarSensor_sub;
    ros::Subscriber vertical_velodyneSensor_sub;  
    ros::Subscriber horizontal_velodyneSensor_sub;

    ros::Publisher obstableMsg_pub; 
    ros::Publisher horizontalObstable_pub; //水平障碍物点的发布
    ros::Publisher verticalObstable_pub;   //倾斜雷达障碍物点的发布
    
    //订阅 laser sensor
    std::string  horizontal_velodyne_topic;  
    std::string  vertical_velodyne_topic;

    //发布 obstable
    std::string  obstableMsg_topic;  // string msg
    std::string  horizontal_obstable_topic; // pointCloud msg
    std::string  vertical_obstable_topic; // 

    std::string   obstable_points_frameId;

    double safety_distance ; //安全阈值
    int  safety_tolerance ; //point数量

    obstable_detection::obstable_detection_msg  obstableMsg; //自定义消息类型
    char*  cString = "[\"stop\"]";
    cJSON*  json;
    std_msgs::String json_str;

    
 public:

    // 仿真测试雷达sensor 回调函数
    void rplidarSensor_Callback(const sensor_msgs::LaserScanConstPtr scan_msg)
    {
        // ROS_INFO("received laserScan message");
            sensor_msgs::LaserScan scan;
            scan.header.stamp = scan_msg->header.stamp;
            scan.header.frame_id = scan_msg->header.frame_id;
            scan.angle_min = scan_msg->angle_min;
            scan.angle_max = scan_msg->angle_max;
            scan.angle_increment = scan_msg->angle_increment;
            scan.scan_time = scan_msg->scan_time;
            scan.range_min = scan_msg->range_min;
            scan.range_max = scan_msg->range_max;

            scan.ranges = scan_msg->ranges;
            scan.intensities = scan_msg->intensities;
            laser_geometry::LaserProjection projector;
            
            sensor_msgs::PointCloud cloud_out;
            projector.projectLaser(scan, cloud_out, -1.0, laser_geometry::channel_option::Index);
            // pointCloud_pub.publish(cloud_out);   
        int count = 0;
        for(int i=0; i<cloud_out.points.size(); i++)
            {
                if(hypot(cloud_out.points[i].x, cloud_out.points[i].y) < safety_distance)
                    count++;
            }
            if(count > safety_tolerance)
            {

                    json = cJSON_Parse(cString);
                    json_str.data = cJSON_Print(json);
                    obstableMsg_pub.publish(json_str);
            }              

    } // end of rplidarSensor_Callback()


// horizontal velodyne:探测周围障碍物
void horizontal_velodyneSensor_Callback(const sensor_msgs::PointCloud2ConstPtr&  velodyneData)
  {
    // ros sensor PointCloud2 ==> pcl pointCloud<XYZR>
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    pcl::fromROSMsg(*velodyneData.get(), *cloud.get());

      int count = 0;
      sensor_msgs::PointCloud obstable_points;
      obstable_points.header.frame_id = obstable_points_frameId;
      obstable_points.header.stamp = ros::Time::now();

       for(int i=0; i<cloud->points.size(); i++)
        {
            geometry_msgs::Point32 point;
            if(hypot(cloud->points[i].x, cloud->points[i].y) < safety_distance)
            {
                count++;
                point.x = cloud->points[i].x;
                point.y = cloud->points[i].y;
                point.z = cloud->points[i].z;
                obstable_points.points.push_back(point);
            }
        }
        if(count > safety_tolerance)
        {
                json = cJSON_Parse(cString);
                json_str.data = cJSON_Print(json);
                obstableMsg_pub.publish(json_str);
                horizontalObstable_pub.publish(obstable_points);
                ROS_DEBUG_STREAM("the horizen points number:" << count);
        }
 
  }




// vertical velodyne:探测前方地面的障碍物
void vertical_velodyneSensor_Callback(const sensor_msgs::PointCloud2ConstPtr&  velodyneData)
  {
    // ros sensor PointCloud2 ==> pcl pointCloud<XYZR>
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    pcl::fromROSMsg(*velodyneData.get(), *cloud.get());

      int count = 0;
      sensor_msgs::PointCloud obstable_points;
      obstable_points.header.frame_id = obstable_points_frameId;
      obstable_points.header.stamp = ros::Time::now();

       for(int i=0; i<cloud->points.size(); i++)
        {
            geometry_msgs::Point32 point;
            if(hypot(cloud->points[i].x, cloud->points[i].y) < safety_distance)
            {
                count++;
                point.x = cloud->points[i].x;
                point.y = cloud->points[i].y;
                point.z = cloud->points[i].z;
                obstable_points.points.push_back(point);
            }
        }
        if(count > safety_tolerance)
        {
                json = cJSON_Parse(cString);
                json_str.data = cJSON_Print(json);
                obstableMsg_pub.publish(json_str);
                horizontalObstable_pub.publish(obstable_points);
                ROS_DEBUG_STREAM("the horizen points number:" << count);
        }
 
  }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstable_detection");
    ROS_INFO("obstable_detection is running ...");
    ObstableDetection obstableDetection;
    // obstableNavigation.update_publish();
    // if(obstableNavigation.selectPoints.size() >=10)
    //     obstableNavigation.moveToGoal(obstableNavigation.selectPoints);
    ros::spin();

    return 0;

}