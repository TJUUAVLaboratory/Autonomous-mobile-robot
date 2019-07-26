/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Eurotec, Netherlands
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * 　-  订阅超声波雷达的数据
 *   -  先转换成各自雷达系坐标的点
 *   - 　再转换成base_link下的点云坐标
 * 
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h> //９个sonar　封装在一起
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>
#include <tf/transform_listener.h>
#include <string>

class Sonar2PointCloud
{
public:
  Sonar2PointCloud(ros::NodeHandle& nh, tf::TransformListener& tf) : nh_(nh), tf_(nh)
  {
    global_frame = "map";
    robot_frame = "base_link";
    sonars_frame = {"m0_sonar", "m1_sonar", "m2_sonar", "m3_sonar", "m4_sonar", "m5_sonar", "m6_sonar", "m7_sonar", "m8_sonar"};

    int concurrency_level = nh_.param("concurrency_level", 2);
    ros::Subscriber sonarDataSub = nh_.subscribe("/ultrasonicScan", 10, &Sonar2PointCloud::sonarDataCallback, this);
    sonarDataPub_0 = nh_.advertise<sensor_msgs::Range>("/sonar_m0", 10, true);
    // sonarDataPub_1 = nh.advertise<sensor_msgs::Range>("/sonar_m1", 10, true);
    // sonarDataPub_2 = nh.advertise<sensor_msgs::Range>("/sonar_m2", 10, true);
    // sonarDataPub_3 = nh.advertise<sensor_msgs::Range>("/sonar_m3", 10, true);
    // sonarDataPub_4 = nh.advertise<sensor_msgs::Range>("/sonar_m4", 10, true);
    // sonarDataPub_5 = nh.advertise<sensor_msgs::Range>("/sonar_m5", 10, true);
    // sonarDataPub_6 = nh.advertise<sensor_msgs::Range>("/sonar_m6", 10, true);
    // sonarDataPub_7 = nh.advertise<sensor_msgs::Range>("/sonar_m7", 10, true);
    // sonarDataPub_8 = nh.advertise<sensor_msgs::Range>("/sonar_m8", 10, true);

    sonarPointcloudPub = nh_.advertise<sensor_msgs::PointCloud>("/sonar_points", 5, true);

    boost::shared_ptr<ros::MultiThreadedSpinner> spinner;
    if (concurrency_level)
    {
      spinner.reset(new ros::MultiThreadedSpinner(static_cast<uint32_t>(concurrency_level)));
    }
    else
    {
      spinner.reset(new ros::MultiThreadedSpinner());
    }
    spinner->spin();
  }

  void sonarDataCallback(const sensor_msgs::LaserScanConstPtr &message);//callback

private:
  ros::NodeHandle nh_;
  std::string global_frame;
  std::string robot_frame;
  std::vector<std::string> sonars_frame;
  tf::TransformListener tf_;

  ros::Publisher sonarDataPub_0;
  ros::Publisher sonarDataPub_1;
  ros::Publisher sonarDataPub_2;
  ros::Publisher sonarDataPub_3;
  ros::Publisher sonarDataPub_4;
  ros::Publisher sonarDataPub_5;
  ros::Publisher sonarDataPub_6;
  ros::Publisher sonarDataPub_7;
  ros::Publisher sonarDataPub_8;
  ros::Publisher sonarPointcloudPub;
};

void Sonar2PointCloud::sonarDataCallback(const sensor_msgs::LaserScanConstPtr &message)
{

  std::vector<sensor_msgs::Range> sonarsValue;
  std::vector<geometry_msgs::PointStamped> sonarsPoints; //各自雷达系坐标系下的点
  //统一到baselink系下
  for (int i = 0; i < message->ranges.size(); i++)
  {
    //封装成sensor_msgs::Range
    sonarsValue[i].header = message->header;
    sonarsValue[i].header.frame_id = sonars_frame[i]; //重新修正sonar_frame_id;
    sonarsValue[i].radiation_type = 0;
    sonarsValue[i].field_of_view = 0.5;
    sonarsValue[i].min_range = 0.1;
    sonarsValue[i].max_range = 1.0;
    sonarsValue[i].range = message->ranges[i];

    sonarDataPub_0.publish(sonarsValue[i]); // publish sonars  range date

    sonarsPoints[i].header = message->header;
    sonarsPoints[i].header.frame_id = sonars_frame[i];
    sonarsPoints[i].point.x = message->ranges[i];
    sonarsPoints[i].point.y = 0;
    sonarsPoints[i].point.z = 0;
  }
  //再将一圈雷达数据组合成base_link下的点云
  if (message->ranges.size() != sonarsPoints.size())
    ROS_ERROR("don't received all the sonars data %d ", message->ranges.size());

  std::vector<geometry_msgs::PointStamped> sonarsPoints_baselink;
      // project the laser into a point cloud
  sensor_msgs::PointCloud cloud;
  cloud.header = message->header;
  cloud.header.frame_id = robot_frame;

  for(std::size_t i=0; i< sonarsPoints.size(); i++)
  {
    sonarsPoints_baselink[i].header =  message->header;
    sonarsPoints_baselink[i].header.frame_id = robot_frame;

    //from mx_sonar to base_link
    tf_.transformPoint(robot_frame, sonarsPoints[i], sonarsPoints_baselink[i]);
    geometry_msgs::Point32  current_point;
    current_point.x = sonarsPoints_baselink[i].point.x;
    current_point.y = sonarsPoints_baselink[i].point.y;
    current_point.z = sonarsPoints_baselink[i].point.z;
    cloud.points.push_back(current_point);
  }
  
  sonarPointcloudPub.publish(cloud);



}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "sonar_to_pointcloud");
    ros::NodeHandle  nh;
    tf::TransformListener tf(ros::Duration(3));
    Sonar2PointCloud sonar2pointclud(nh, tf);

  return 0;
}
