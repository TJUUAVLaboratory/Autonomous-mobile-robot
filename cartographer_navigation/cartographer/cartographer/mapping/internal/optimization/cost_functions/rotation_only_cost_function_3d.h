//
// Created by galyean on 19-3-24.
//
#ifndef CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_ROTATION_ONLY_COST_FUNCTION_3D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_ROTATION_ONLY_COST_FUNCTION_3D_H_

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "ceres/ceres.h"
#include <iostream>
namespace cartographer {
    namespace mapping {

// Penalizes differences between IMU data and optimized orientations.
        class RotationOnlyCostFunction3D {
        public:
            static ceres::CostFunction* CreateAutoDiffCostFunction(
                    const double scaling_factor,
                    const Eigen::Quaterniond& imu_calibration_pre) {

                return new ceres::AutoDiffCostFunction<
                        RotationOnlyCostFunction3D, 3 /* residuals */,
                        4 /* rotation variables */
                >(new RotationOnlyCostFunction3D(scaling_factor, imu_calibration_pre));
            }

            template <typename T>
            bool operator()(const T* const imu_calibration, T* residual) const {

                const Eigen::Quaternion<T> eigen_imu_calibration(
                        imu_calibration[0], imu_calibration[1], imu_calibration[2],
                        imu_calibration[3]);
                const Eigen::Quaternion<T> error =
                        eigen_imu_calibration * imu_calibration_pre_.cast<T>().conjugate();
                residual[0] = scaling_factor_ * error.x();
                residual[1] = scaling_factor_ * error.y();
                residual[2] = scaling_factor_ * error.z();
                return true;
            }

        private:
            RotationOnlyCostFunction3D(const double scaling_factor,
                                   const Eigen::Quaterniond& imu_calibration_pre)
                    : scaling_factor_(scaling_factor),
                      imu_calibration_pre_(imu_calibration_pre) {}

            RotationOnlyCostFunction3D(const RotationOnlyCostFunction3D&) = delete;
            RotationOnlyCostFunction3D& operator=(const RotationOnlyCostFunction3D&) = delete;

            const double scaling_factor_;
            const Eigen::Quaterniond imu_calibration_pre_;
        };

    }  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_ROTATION_ONLY_COST_FUNCTION_3D_H_
