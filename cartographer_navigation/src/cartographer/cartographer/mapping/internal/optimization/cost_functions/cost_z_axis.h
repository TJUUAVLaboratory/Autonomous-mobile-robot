//
// Created by galyean on 19-3-24.
//
#ifndef CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_Z_ONLY_COST_FUNCTION_3D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_Z_ONLY_COST_FUNCTION_3D_H_

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "ceres/ceres.h"
#include <iostream>
namespace cartographer {
    namespace mapping {

// Penalizes differences between IMU data and optimized orientations.
        class ZOnlyCostFunction3D {
        public:
            static ceres::CostFunction* CreateAutoDiffCostFunction(
                    const double scaling_factor) {

                return new ceres::AutoDiffCostFunction<
                        ZOnlyCostFunction3D, 1 /* residuals */,
                        3 /* translation variables */
                >(new ZOnlyCostFunction3D(scaling_factor));
            }

            template <typename T>
            bool operator()(const T* const translation, T* residual) const {
                residual[0] = -scaling_factor_ * translation[2];
                return true;
            }

        private:
            ZOnlyCostFunction3D(const double scaling_factor)
                    : scaling_factor_(scaling_factor){}

            ZOnlyCostFunction3D(const ZOnlyCostFunction3D&) = delete;
            ZOnlyCostFunction3D& operator=(const ZOnlyCostFunction3D&) = delete;

            const double scaling_factor_;
        };

    }  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_Z_ONLY_COST_FUNCTION_3D_H_
