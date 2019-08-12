/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cmath>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cairo/cairo.h"
#include "cartographer/common/mutex.h"
#include "cartographer/common/port.h"
#include "cartographer/io/image.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/id.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/submap.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "gflags/gflags.h"
#include "nav_msgs/OccupancyGrid.h"
#include <std_msgs/String.h>
#include "ros/ros.h"
#include "cartographer_ros_msgs/SaveMap.h"

DEFINE_double(resolution, 0.05,
              "Resolution of a grid cell in the published occupancy grid.");
DEFINE_double(publish_period_sec, 1.0, "OccupancyGrid publishing period.");

namespace cartographer_ros
{
namespace
{

using ::cartographer::io::PaintSubmapSlicesResult;
using ::cartographer::io::SubmapSlice;
using ::cartographer::mapping::SubmapId;

class Node
{
public:
  explicit Node(double resolution, double publish_period_sec);
  ~Node() {}

  Node(const Node &) = delete;
  Node &operator=(const Node &) = delete;

private:
  bool HandleMapSave(
      cartographer_ros_msgs::SaveMap::Request &request,
      cartographer_ros_msgs::SaveMap::Response &response);
  void HandleSubmapList(const cartographer_ros_msgs::SubmapList::ConstPtr &msg);
  void DrawAndPublish(const ::ros::WallTimerEvent &timer_event);
  void PublishOccupancyGrid(const std::string &frame_id, const ros::Time &time,
                            const Eigen::Array2f &origin,
                            cairo_surface_t *surface);
  void WritePgm(const ::cartographer::io::Image &image, const double resolution,
                ::cartographer::io::FileWriter *file_writer);
  ::ros::NodeHandle node_handle_;
  const double resolution_;

  ::cartographer::common::Mutex mutex_;
  ::ros::ServiceServer save_map_ GUARDED_BY(mutex_);
  ::ros::ServiceClient client_ GUARDED_BY(mutex_);
  ::ros::Subscriber submap_list_subscriber_ GUARDED_BY(mutex_);
  ::ros::Publisher occupancy_grid_publisher_ GUARDED_BY(mutex_);
  ::ros::Publisher aibee_navi_publisher_ GUARDED_BY(mutex_);
  std::map<SubmapId, SubmapSlice> submap_slices_ GUARDED_BY(mutex_);
  ::ros::WallTimer occupancy_grid_publisher_timer_;
  std::string last_frame_id_;
  ros::Time last_timestamp_;
  int id;
};

Node::Node(const double resolution, const double publish_period_sec)
    : resolution_(resolution),
      client_(node_handle_.serviceClient<::cartographer_ros_msgs::SubmapQuery>(
          kSubmapQueryServiceName)),
      submap_list_subscriber_(node_handle_.subscribe(
          kSubmapListTopic, kLatestOnlyPublisherQueueSize,
          boost::function<void(
              const cartographer_ros_msgs::SubmapList::ConstPtr &)>(
              [this](const cartographer_ros_msgs::SubmapList::ConstPtr &msg) {
                HandleSubmapList(msg);
              }))),
      occupancy_grid_publisher_(
          node_handle_.advertise<::nav_msgs::OccupancyGrid>(
              kOccupancyGridTopic, kLatestOnlyPublisherQueueSize,
              true /* latched */)),
      aibee_navi_publisher_(node_handle_.advertise<std_msgs::String>("aibee_navi", 10, true)),
      occupancy_grid_publisher_timer_(
          node_handle_.createWallTimer(::ros::WallDuration(publish_period_sec),
                                       &Node::DrawAndPublish, this)),
      id(0)
{
  save_map_ = node_handle_.advertiseService(kSaveMapServiceName, &Node::HandleMapSave, this);
}
void Node::WritePgm(const ::cartographer::io::Image &image, const double resolution,
                    ::cartographer::io::FileWriter *file_writer)
{
  const std::string header = "P5\n# Cartographer map; " +
                             std::to_string(resolution) + " m/pixel\n" +
                             std::to_string(image.width()) + " " +
                             std::to_string(image.height()) + "\n255\n";
  file_writer->Write(header.data(), header.size());
  for (int y = 0; y < image.height(); ++y)
  {
    for (int x = 0; x < image.width(); ++x)
    {
      const unsigned char color = image.GetPixel(x, image.height()-y-1)[0];
      const unsigned char observed = image.GetPixel(x, image.height()-y-1)[1];
      char final_color = 128;
      const float value =
          observed == 0
              ? -1.0
              : ::cartographer::common::RoundToInt(1. - color / 255.);

      // value = occ, 0.65 is occ threshhold
      if (value < 0)
      {
        final_color = 0;
      }
      else if (value > 0.65)
      {
        final_color = 0;
      }
      else if (value < 0.196)
      { // 0.196 is free threshold
        final_color = 255;
      }
      file_writer->Write(&final_color, 1);
    }
  }
}
bool Node::HandleMapSave(
    cartographer_ros_msgs::SaveMap::Request &request,
    cartographer_ros_msgs::SaveMap::Response &response)
{
  response.code = 0;
  std::string status = request.message;
  if (status.compare("save_map") != 0)
  {
    return false;
  }
  if (submap_slices_.empty() || last_frame_id_.empty())
  {
    return false;
  }
  std::string map_path = request.map_path;
  std::string file_name = map_path+"/full_map.pgm";
  std::string png_file_name = map_path+"/full_map.png";
  ::cartographer::common::MutexLocker locker(&mutex_);
  auto painted_slices = PaintSubmapSlices(submap_slices_, resolution_);
  std::unique_ptr<nav_msgs::OccupancyGrid> msg_ptr = CreateOccupancyGridMsg(
      painted_slices, resolution_, last_frame_id_, last_timestamp_);
  // save .pgm image
  ::cartographer::io::StreamFileWriter pgm_writer(file_name);

  ::cartographer::io::Image image(std::move(painted_slices.surface));
  WritePgm(image, resolution_, &pgm_writer);
  pgm_writer.Close();
  //
  double resolution = msg_ptr->info.resolution;
  double width_real = msg_ptr->info.width * resolution;
  double height_real = msg_ptr->info.height * resolution;
  double origin_x = msg_ptr->info.origin.position.x;
  double origin_y = msg_ptr->info.origin.position.y;

  std::string system_command = "convert " + file_name + " " + png_file_name;
  system(system_command.data());
  response.message = png_file_name;
  response.parameters[0] = origin_x;
  response.parameters[1] = origin_y;
  response.parameters[2] = width_real;
  response.parameters[3] = height_real;
  response.code = 1;
  return true;
}
void Node::HandleSubmapList(
    const cartographer_ros_msgs::SubmapList::ConstPtr &msg)
{
  ::cartographer::common::MutexLocker locker(&mutex_);

  // We do not do any work if nobody listens.
  if (occupancy_grid_publisher_.getNumSubscribers() == 0)
  {
    //return;
  }

  // Keep track of submap IDs that don't appear in the message anymore.
  std::set<SubmapId> submap_ids_to_delete;
  for (const auto &pair : submap_slices_)
  {
    submap_ids_to_delete.insert(pair.first);
  }

  for (const auto &submap_msg : msg->submap)
  {
    const SubmapId id{submap_msg.trajectory_id, submap_msg.submap_index};
    submap_ids_to_delete.erase(id);
    SubmapSlice &submap_slice = submap_slices_[id];
    submap_slice.pose = ToRigid3d(submap_msg.pose);
    submap_slice.metadata_version = submap_msg.submap_version;
    if (submap_slice.surface != nullptr &&
        submap_slice.version == submap_msg.submap_version)
    {
      continue;
    }

    auto fetched_textures =
        ::cartographer_ros::FetchSubmapTextures(id, &client_);
    if (fetched_textures == nullptr)
    {
      continue;
    }
    CHECK(!fetched_textures->textures.empty());
    submap_slice.version = fetched_textures->version;

    // We use the first texture only. By convention this is the highest
    // resolution texture and that is the one we want to use to construct the
    // map for ROS.
    const auto fetched_texture = fetched_textures->textures.begin();
    submap_slice.width = fetched_texture->width;
    submap_slice.height = fetched_texture->height;
    submap_slice.slice_pose = fetched_texture->slice_pose;
    submap_slice.resolution = fetched_texture->resolution;
    submap_slice.cairo_data.clear();
    submap_slice.surface = ::cartographer::io::DrawTexture(
        fetched_texture->pixels.intensity, fetched_texture->pixels.alpha,
        fetched_texture->width, fetched_texture->height,
        &submap_slice.cairo_data);
  }

  // Delete all submaps that didn't appear in the message.
  for (const auto &id : submap_ids_to_delete)
  {
    submap_slices_.erase(id);
  }

  last_timestamp_ = msg->header.stamp;
  last_frame_id_ = msg->header.frame_id;
}

void Node::DrawAndPublish(const ::ros::WallTimerEvent &unused_timer_event)
{
  if (submap_slices_.empty() || last_frame_id_.empty())
  {
    return;
  }
  ::cartographer::common::MutexLocker locker(&mutex_);
  auto painted_slices = PaintSubmapSlices(submap_slices_, resolution_);
  std::unique_ptr<nav_msgs::OccupancyGrid> msg_ptr = CreateOccupancyGridMsg(
      painted_slices, resolution_, last_frame_id_, last_timestamp_);
  occupancy_grid_publisher_.publish(*msg_ptr);
}

} // namespace
} // namespace cartographer_ros

int main(int argc, char **argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  ::ros::init(argc, argv, "cartographer_occupancy_grid_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  ::cartographer_ros::Node node(FLAGS_resolution, FLAGS_publish_period_sec);

  ::ros::spin();
  ::ros::shutdown();
}
