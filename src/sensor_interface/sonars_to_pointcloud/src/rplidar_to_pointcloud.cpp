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
 * 　-  订阅2D laserScan雷达的数据
 *   -  remove the start and end edge data
 *   - 　publish pointCloud of rplidar_link
 * a
 */

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
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Range.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <laser_geometry/laser_geometry.h>
#include <string>

class rplidar2PointCloud
{
public:
  rplidar2PointCloud(ros::NodeHandle &nh, tf2_ros::Buffer *tf2) : nh_(nh), tf2_(tf2)
  {
    global_frame = "map";
    target_frame = "rplidar_link";
    ROS_INFO("init");
    int concurrency_level = nh_.param("concurrency_level", 2);
    ros::Subscriber rplidarDataSub = nh_.subscribe("/singalScan0", 10, &rplidar2PointCloud::rplidarDataCallback, this);
    //rplidarPointcloudPub = nh_.advertise<sensor_msgs::PointCloud>("/rplidar_pointCloud", 10, true);
    rplidarPointPub = nh_.advertise<sensor_msgs::LaserScan>("/rplidar_point", 10, true);
    ROS_INFO("lidar sensor  init finished");
    ros::spin();

    // boost::shared_ptr<ros::MultiThreadedSpinner> spinner;
    // if (concurrency_level)
    // {
    //   spinner.reset(new ros::MultiThreadedSpinner(static_cast<uint32_t>(concurrency_level)));
    // }
    // else
    // {
    //   spinner.reset(new ros::MultiThreadedSpinner());
    // }
    // spinner->spin();
  }

  void rplidarDataCallback(const sensor_msgs::LaserScanConstPtr &message); //callback

private:
  ros::NodeHandle nh_;
  std::string global_frame;
  std::string target_frame;
  tf2_ros::Buffer *tf2_;

  ros::Publisher rplidarPointcloudPub;
  ros::Publisher rplidarPointPub;
};

// sub rplidar laserscan ===> rplidar pointCloud
void rplidar2PointCloud::rplidarDataCallback(const sensor_msgs::LaserScanConstPtr &raw_message)
{
  //ROS_INFO("received rplidar_message");
  static int outlier_index = 0;
  float epsilon = 0.0001; // a tenth of a millimeter
  sensor_msgs::LaserScan message ;
  message.header.frame_id = raw_message->header.frame_id;
  message.header.stamp = raw_message->header.stamp;
  message.angle_max = raw_message->angle_max;
  message.angle_min = raw_message->angle_min;
  message.angle_increment = raw_message->angle_increment;
  message.range_max = raw_message->range_max;
  message.range_min = raw_message->range_min;
  message.intensities = raw_message->intensities;
  message.time_increment = raw_message->time_increment; 

  // remove outlier
  for (size_t i = 0; i < raw_message->ranges.size(); i++)
  {
    float range = raw_message->ranges[i];
    if (!std::isfinite(range) && range > 0)
    {
      range = raw_message->range_max - epsilon;
    }

    if (outlier_index <= 20)
    {
      if (raw_message->ranges[i] <= 10.00)
       range = INT64_MAX;
       
       outlier_index++;
    }

    if (raw_message->ranges[i] <= 1.200 || raw_message->ranges[i] >= 2.5)
      range = INT64_MAX;

    message.ranges.push_back(range);
  }

  // scan ==> PointCloud
  // sensor_msgs::PointCloudPtr scan_cloud;
  // scan_cloud.reset(new sensor_msgs::PointCloud);

  // laser_geometry::LaserProjection projector_;
  // projector_.projectLaser(message, *scan_cloud);

  // // needed tf_frame_transform
  // if (!target_frame.empty() && scan_cloud->header.frame_id != target_frame)
  // {
  //   try
  //   {
  //     *scan_cloud = tf2_->transform(*scan_cloud, target_frame);
  //   }
  //   catch (tf2::TransformException& ex)
  //   {
  //     ROS_ERROR_STREAM("Transform failure: " << ex.what());
  //     return;
  //   }
  // }


  //rplidarPointcloudPub.publish(*scan_cloud);
  //ROS_INFO("publisher rplidar_message");
  rplidarPointPub.publish(message);

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "rplidar_to_pointcloud");
  ros::NodeHandle nh;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ROS_INFO("rplidar scan to pointcloud  running");
  rplidar2PointCloud rplidar2pointcloud(nh, &tfBuffer);

  return 0;
}
