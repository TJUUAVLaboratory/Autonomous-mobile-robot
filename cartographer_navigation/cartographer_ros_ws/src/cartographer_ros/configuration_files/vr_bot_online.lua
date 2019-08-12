-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 2,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.1,
  trajectory_publish_period_sec = 1,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 150

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 30
POSE_GRAPH.optimization_problem.huber_scale = 10
POSE_GRAPH.optimize_every_n_nodes = 20
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 50
POSE_GRAPH.constraint_builder.min_score = 0.38
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.38
POSE_GRAPH.constraint_builder.max_constraint_distance = 30
MAP_BUILDER.pose_graph.optimization_problem.ceres_solver_options.max_num_iterations = 100

TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 200
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 1.0
TRAJECTORY_BUILDER_3D.max_range = 20
TRAJECTORY_BUILDER_3D.min_range = 0.0
--TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 800
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.04
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.1
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = 8
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 300
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 400
--TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = 5
--TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true
--TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.05
--TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.25
--TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true
--POSE_GRAPH.constraint_builder.min_score = 0.55
--POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60
--POSE_GRAPH.constraint_builder.max_constraint_distance = 30 
--POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 5
--POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 30
--POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_low_resolution_score = 0.55


--TRAJECTORY_BUILDER_3D.ceres_scan_matcher.only_optimize_yaw = true 
--MAP_BUILDER.pose_graph.optimization_problem.use_online_imu_extrinsics_in_3d = false
--MAP_BUILDER.pose_graph.constraint_builder.max_constraint_distance = 250
MAP_BUILDER.pose_graph.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 20
--MAP_BUILDER.pose_graph.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 5
--MAP_BUILDER.pose_graph.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.max_num_iterations = 50


return options
