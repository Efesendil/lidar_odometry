/**
 * @file      Parameters.cpp
 * @brief     Implements parameter blocks for Ceres optimization in LiDAR Odometry.
 * @author    Seungwon Choi
 * @date      2025-09-24
 * @copyright Copyright (c) 2025 Seungwon Choi. All rights reserved.
 *
 * @par License
 * This project is released under the MIT License.
 */

#include "Parameters.h"
#include <spdlog/spdlog.h>

namespace lidar_odometry {
namespace optimization {

bool SE3GlobalParameterization::Plus(const double* x,
                                   const double* delta,
                                   double* x_plus_delta) const {
    try {
        // If parameter is fixed, don't apply any updates
        if (m_is_fixed) {
            // Copy original parameters without applying delta
            for (int i = 0; i < 6; ++i) {
                x_plus_delta[i] = x[i];
            }
            return true;
        }
        
        // Convert arrays to Eigen vectors
        Eigen::Map<const Eigen::Vector6d> current_tangent(x);
        Eigen::Map<const Eigen::Vector6d> delta_tangent(delta);
        Eigen::Map<Eigen::Vector6d> result_tangent(x_plus_delta);
        
        // Convert current tangent to SE3
        Sophus::SE3d current_se3 = tangent_to_se3(current_tangent);
        
        // Apply delta as left multiplication: exp(delta) * current  
        // This matches g2o convention for pose updates
        Sophus::SE3d delta_se3 = Sophus::SE3d::exp(delta_tangent);
        Sophus::SE3d result_se3 = current_se3*delta_se3;
        
        // Convert back to tangent space
        result_tangent = se3_to_tangent(result_se3);
        
        return true;
    } catch (const std::exception& e) {
        spdlog::error("[SE3GlobalParameterization::Plus] Exception: {}", e.what());
        return false;
    }
}

bool SE3GlobalParameterization::ComputeJacobian(const double* x,
                                              double* jacobian) const {
    // For small perturbations in SE3, the Jacobian can be approximated as Identity
    // This is much faster than computing the exact right Jacobian
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> jac(jacobian);
    jac.setIdentity();
    return true;
}

Sophus::SE3d SE3GlobalParameterization::tangent_to_se3(const Eigen::Vector6d& tangent) {
    // Ceres order: [t_x, t_y, t_z, so3_x, so3_y, so3_z]
    // Use Sophus SE3::exp for consistent parameterization with V matrix
    return Sophus::SE3d::exp(tangent);
}

Eigen::Vector6d SE3GlobalParameterization::se3_to_tangent(const Sophus::SE3d& se3) {
    // Use SE3::log() for consistency with SE3::exp() in tangent_to_se3()
    // This ensures proper V matrix handling
    return se3.log();
}

Eigen::Vector6d SE3GlobalParameterization::se3f_to_tangent_d(const Sophus::SE3f& se3f) {
    // Convert float SE3 to double SE3, then to tangent space
    Sophus::SE3d se3d = se3f.cast<double>();
    return se3_to_tangent(se3d);
}

Sophus::SE3f SE3GlobalParameterization::tangent_d_to_se3f(const Eigen::Vector6d& tangent_d) {
    // Convert double tangent to SE3, then to float SE3
    Sophus::SE3d se3d = tangent_to_se3(tangent_d);
    return se3d.cast<float>();
}

} // namespace optimization
} // namespace lidar_odometry
