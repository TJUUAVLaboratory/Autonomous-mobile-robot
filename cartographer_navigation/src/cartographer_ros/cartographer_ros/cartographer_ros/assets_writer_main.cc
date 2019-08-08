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
#include "cartographer/common/time.h"
#include "cartographer/transform/transform_interpolation_buffer.h"
#include "cartographer_ros/assets_writer.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros/split_string.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include <iomanip>
#include <vector>
#include <fstream>
#include <sstream>
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(
    urdf_filename, "",
    "URDF file that contains static links for your sensor configuration.");
DEFINE_string(bag_filenames, "",
              "Bags to process, must be in the same order as the trajectories "
              "in 'pose_graph_filename'.");
DEFINE_string(pose_graph_filename, "",
              "Proto stream file containing the pose graph.");
DEFINE_bool(use_bag_transforms, true,
            "Whether to read and use the transforms from the bag.");
DEFINE_string(output_file_prefix, "",
              "Will be prefixed to all output file names and can be used to "
              "define the output directory. If empty, the first bag filename "
              "will be used.");
namespace carto = ::cartographer;
void TransformFences(std::string &pose_graph_file, ::cartographer_ros::AssetsWriter &asset_writer)
{
    //
    std::string file_name = pose_graph_file;
    auto p = file_name.find_last_of("/");
    if (p == std::string::npos)
    {
        LOG(ERROR) << "set wrong file path: file_name\n";
        exit(1);
    }
    std::string base_dir_name_ = file_name.substr(0, p) + "/config/fence_points/";

    std::vector<Eigen::Vector3d> global_fences;
    // add by galyean, try to reload fence and transform them to global coordinate
    std::ifstream f_read(base_dir_name_ + "fence.txt");
    if (!f_read.is_open())
    {
        std::cerr << "can not open file: " << base_dir_name_ << " ! and will ignore fences." << std::endl;
        return;
    }

    double time_sec;

    char buffer[1024];
    f_read.getline(buffer, 1024);
    std::stringstream ss;
    ss << buffer;
    ss >> time_sec;
    carto::common::Time time_local = ::cartographer_ros::FromRos(ros::Time(time_sec));
    ::cartographer::mapping::proto::PoseGraph &pose_graph_ = asset_writer.GetPoseGraph();
    size_t trajectory_id = pose_graph_.trajectory_size() - 1;
    const carto::mapping::proto::Trajectory &trajectory_proto =
        pose_graph_.trajectory(trajectory_id);
    const carto::transform::TransformInterpolationBuffer
        transform_interpolation_buffer(trajectory_proto);
    if (!transform_interpolation_buffer.Has(time_local))
    {
        std::cerr << "Local time in file : " << base_dir_name_ << " is wrong! and will ignore fences." << std::endl;
        return;
    }
    const carto::transform::Rigid3d tracking_to_map =
        transform_interpolation_buffer.Lookup(time_local);
    while (f_read.getline(buffer, 1024))
    {
        std::stringstream ss1;
        double point[3];
        ss1 << buffer;
        ss1 >> point[0];
        ss1 >> point[1];
        ss1 >> point[2];
        global_fences.emplace_back(tracking_to_map * Eigen::Vector3d(point[0], point[1], point[2]));
    }
    f_read.close();
    std::ofstream f_write(base_dir_name_ + "fence_global.txt");
    f_write << std::fixed << std::setprecision(9);
    for (auto point : global_fences)
    {
        f_write << point.transpose() << std::endl;
    }
    f_write.close();
}
int main(int argc, char **argv)
{

    FLAGS_alsologtostderr = true;
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    CHECK(!FLAGS_configuration_directory.empty())
        << "-configuration_directory is missing.";
    CHECK(!FLAGS_configuration_basename.empty())
        << "-configuration_basename is missing.";
    CHECK(!FLAGS_bag_filenames.empty()) << "-bag_filenames is missing.";
    CHECK(!FLAGS_pose_graph_filename.empty())
        << "-pose_graph_filename is missing.";

    ::cartographer_ros::AssetsWriter asset_writer(
        FLAGS_pose_graph_filename,
        cartographer_ros::SplitString(FLAGS_bag_filenames, ','),
        FLAGS_output_file_prefix);

    TransformFences(FLAGS_pose_graph_filename, asset_writer);

    asset_writer.Run(FLAGS_configuration_directory, FLAGS_configuration_basename,
                     FLAGS_urdf_filename, FLAGS_use_bag_transforms);
}
