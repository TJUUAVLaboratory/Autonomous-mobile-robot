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

#include "cartographer/mapping/3d/range_data_inserter_3d.h"

#include "Eigen/Core"
#include "cartographer/mapping/probability_values.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {

void InsertMissesIntoGrid(const std::vector<uint16>& miss_table,
                          const Eigen::Vector3f& origin,
                          const sensor::PointCloud& returns,
                          HybridGrid* hybrid_grid,
                          const int num_free_space_voxels) {
  const Eigen::Array3i origin_cell = hybrid_grid->GetCellIndex(origin);
  for (const Eigen::Vector3f& hit : returns) {
    const Eigen::Array3i hit_cell = hybrid_grid->GetCellIndex(hit);

    const Eigen::Array3i delta = hit_cell - origin_cell;
    const int num_samples = delta.cwiseAbs().maxCoeff();
    CHECK_LT(num_samples, 1 << 15);
    // 'num_samples' is the number of samples we equi-distantly place on the
    // line between 'origin' and 'hit'. (including a fractional part for sub-
    // voxels) It is chosen so that between two samples we change from one voxel
    // to the next on the fastest changing dimension.
    // Only the last 'num_free_space_voxels' are updated for performance.
    //+++aibee_zzx

    if(hybrid_grid->resolution()>=0.03){
      int num = 0.8/hybrid_grid->resolution();
      for (int position = 0;
        position < num_samples - 5; ++position) {
        Eigen::Array3i miss_cell =
        origin_cell + delta * position / num_samples;
        //0.1分辨率 高度限制
        // if(abs(miss_cell[2])<5){
        //   for(int x=-1;x<2;x++){
        //     for(int y=-1;y<2;y++){
        //       //for(int z=-1;z<1;z++){
        //         Eigen::Array3i ifhit;
        //         ifhit[0]=miss_cell[0]+x;
        //         ifhit[1]=miss_cell[1]+y;
        //         //ifhit[2]=miss_cell[2]+z;
        //         float pro=hybrid_grid->GetProbability(ifhit);
        //         if(pro>0.3){
        //            std::cout<<"set!!!"<<std::endl;
        //            hybrid_grid->ApplyLookupTable(ifhit, miss_table);
        //            ifhit[2]+=1;
        //            hybrid_grid->ApplyLookupTable(ifhit, miss_table);
        //            ifhit[2]-=2;
        //            hybrid_grid->ApplyLookupTable(ifhit, miss_table);
        //         }

        //       // }
        //     }
        //   //}
        // }
          if(abs(miss_cell[2])<=num){
              float pro=hybrid_grid->GetProbability(miss_cell);
              if(pro>0.6){
                  hybrid_grid->ApplyLookupTable(miss_cell, miss_table);
                  // miss_cell[2]+=1;
                  // hybrid_grid->ApplyLookupTable(miss_cell, miss_table);
                  // miss_cell[2]-=2;
                  // hybrid_grid->ApplyLookupTable(miss_cell, miss_table);
              }
            }
      }
    }
  }
}
}  // namespace

proto::RangeDataInserterOptions3D CreateRangeDataInserterOptions3D(
    common::LuaParameterDictionary* parameter_dictionary) {
  proto::RangeDataInserterOptions3D options;
  options.set_hit_probability(
      parameter_dictionary->GetDouble("hit_probability"));
  options.set_miss_probability(
      parameter_dictionary->GetDouble("miss_probability"));
  options.set_num_free_space_voxels(
      parameter_dictionary->GetInt("num_free_space_voxels"));
  CHECK_GT(options.hit_probability(), 0.5);
  CHECK_LT(options.miss_probability(), 0.5);
  return options;
}

RangeDataInserter3D::RangeDataInserter3D(
    const proto::RangeDataInserterOptions3D& options)
    : options_(options),
      hit_table_(
          ComputeLookupTableToApplyOdds(Odds(options_.hit_probability()))),
      miss_table_(
          ComputeLookupTableToApplyOdds(Odds(options_.miss_probability()))) {}

void RangeDataInserter3D::Insert(const sensor::RangeData& range_data,
                                 HybridGrid* hybrid_grid) const {
  CHECK_NOTNULL(hybrid_grid);

  for (const Eigen::Vector3f& hit : range_data.returns) {
    const Eigen::Array3i hit_cell = hybrid_grid->GetCellIndex(hit);
    hybrid_grid->ApplyLookupTable(hit_cell, hit_table_);
  }

  // By not starting a new update after hits are inserted, we give hits priority
  // (i.e. no hits will be ignored because of a miss in the same cell).
  InsertMissesIntoGrid(miss_table_, range_data.origin, range_data.returns,
                        hybrid_grid, options_.num_free_space_voxels());
  hybrid_grid->FinishUpdate();
}

}  // namespace mapping
}  // namespace cartographer
