/*
 * Copyright 2017 The Cartographer Authors
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

#include <map>
#include <string>

#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/3d/submap_3d.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/submap.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer_ros/ros_map.h"
#include "cartographer_ros/submap.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "cartographer/io/color.h"

DEFINE_string(pbstream_filename, "",
              "Filename of a pbstream to draw a map from.");
DEFINE_string(map_filestem, "", "Stem of the output files.");
DEFINE_double(resolution, 0.02, "Resolution of a grid cell in the drawn map.");

namespace cartographer_ros {
namespace {

::cartographer::io::Image Run(const std::string& pbstream_filename, const std::string& map_filestem,
         const double resolution , int map_saver_type) {
          
  aibee_map_saver_type=map_saver_type;
  ::cartographer::io::ProtoStreamReader reader(pbstream_filename);
  ::cartographer::io::ProtoStreamDeserializer deserializer(&reader);
   
  const auto& pose_graph = deserializer.pose_graph();
  LOG(INFO) << "Loading submap slices from serialized data.";
  std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>
      submap_slices;
  ::cartographer::mapping::proto::SerializedData proto;
  while (deserializer.ReadNextSerializedData(&proto)) {
    if (proto.has_submap()) {
      const auto& submap = proto.submap();
      const ::cartographer::mapping::SubmapId id{
          submap.submap_id().trajectory_id(),
          submap.submap_id().submap_index()};
      const ::cartographer::transform::Rigid3d global_submap_pose =
          ::cartographer::transform::ToRigid3(
              pose_graph.trajectory(id.trajectory_id)
                  .submap(id.submap_index)
                  .pose());  
      FillSubmapSlice(global_submap_pose, submap, &submap_slices[id]);  
    }
  }
  CHECK(reader.eof());

  LOG(INFO) << "Generating combined map image from submap slices.";
  auto result =
      ::cartographer::io::PaintSubmapSlices(submap_slices, resolution);
  
  ::cartographer::io::StreamFileWriter pgm_writer(map_filestem + ".pgm");

  ::cartographer::io::Image image(std::move(result.surface));
  WritePgm(image, resolution, &pgm_writer);

  const Eigen::Vector2d origin(
      -result.origin.x() * resolution,
      (result.origin.y() - image.height()) * resolution);

  ::cartographer::io::StreamFileWriter yaml_writer(map_filestem + ".yaml");
  WriteYaml(resolution, origin, pgm_writer.GetFilename(), &yaml_writer);
  return image;
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_pbstream_filename.empty()) << "-pbstream_filename is missing.";
  CHECK(!FLAGS_map_filestem.empty()) << "-map_filestem is missing.";
  ::cartographer::io::Image image1 =  ::cartographer_ros::Run(FLAGS_pbstream_filename, FLAGS_map_filestem,
                          FLAGS_resolution,1);
  ::cartographer::io::Image image2 =  ::cartographer_ros::Run(FLAGS_pbstream_filename, FLAGS_map_filestem,
                          FLAGS_resolution,0);
  for (int i=0;i<image1.width();i++){
    for (int j=0;j<image1.height();j++)
    {
      ::cartographer::io::Uint8Color p1=image1.GetPixel(i, j);
      ::cartographer::io::Uint8Color p2=image2.GetPixel(i, j);
      if(p2[0]!=128){
        image1.SetPixel(i,j,p2); 
      }
    }
       
   }
   ::cartographer::io::StreamFileWriter pgm_writer(FLAGS_map_filestem + ".all.pgm");
   ::cartographer_ros::WritePgm(image1, FLAGS_resolution, &pgm_writer);
}
