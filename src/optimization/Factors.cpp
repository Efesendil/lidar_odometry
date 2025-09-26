/**
 * @file      Factors.cpp
 * @brief     Implements Ceres cost functions (factors) for LiDAR Odometry optimization.
 * @author    Seungwon Choi
 * @date      2025-09-24
 * @copyright Copyright (c) 2025 Seungwon Choi. All rights reserved.
 *
 * @par License
 * This project is released under the MIT License.
 */

#include "Factors.h"
#include <limits>

namespace lidar_odometry {
namespace optimization {

// PointToPlaneFactor implementation
PointToPlaneFactor::PointToPlaneFactor(const Eigen::Vector3f& source_point,
                                       const Eigen::Vector3f& target_point,
                                       const Eigen::Vector3f& plane_normal,
                                       double information_weight)
    : m_source_point(source_point)
    , m_target_point(target_point)
    , m_plane_normal(plane_normal.normalized())
    , m_information_weight(information_weight)
    , m_robust_weight(1.0)
    , m_is_outlier(false) {
}

bool PointToPlaneFactor::Evaluate(double const* const* parameters,
                                 double* residuals,
                                 double** jacobians) const {
    
    // If marked as outlier, return large residual with zero jacobians
    if (m_is_outlier) {
        residuals[0] = 100.0; // Large residual to indicate outlier
        
        if (jacobians && jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> jac(jacobians[0]);
            jac.setZero();
        }
        return true;
    }

    try {
        // Extract SE3 parameters from tangent space (Ceres order: tx,ty,tz,rx,ry,rz)
        Eigen::Map<const Eigen::Vector6d> se3_tangent(parameters[0]);
        
        // Convert tangent space to SE3 using Sophus exp
        Sophus::SE3d T_correction = Sophus::SE3d::exp(se3_tangent); // local frame correction transform
        
        // Extract rotation and translation
        Eigen::Matrix3d R = T_correction.rotationMatrix();
        Eigen::Vector3d t = T_correction.translation();
        
        // Convert float points to double for computation
        Eigen::Vector3d p = m_source_point.cast<double>();  // source point (current local frame)
        Eigen::Vector3d q = m_target_point.cast<double>();  // target point (current local frame)
        Eigen::Vector3d n = m_plane_normal.cast<double>();  // plane normal (current local frame)
        
        // Apply local correction to source point: p_corrected = R * p + t
        Eigen::Vector3d p_corrected = R * p + t;
        
        // Compute point-to-plane distance: residual = n^T * (p_corrected - q)
        double raw_residual = n.dot(p_corrected - q);
        
        // Apply only information weighting (normalization)
        // Robust weighting is handled by Ceres Loss Function
        residuals[0] = m_information_weight * raw_residual;

        // spdlog::warn("Check residual: raw={:.6f}, weighted={:.6f}, info_weight={:.6f}", 
        //              raw_residual, residuals[0], m_information_weight);
        
        // Compute analytical jacobians if requested
        if (jacobians && jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> jac(jacobians[0]);
            
            // Jacobian w.r.t SE3 tangent space parameters
            // d(residual)/d(tangent) = information_weight * robust_weight * d(n^T * (R*p + t - q))/d(tangent)
            
            // Translation part: d(n^T * (R*p + t - q))/d(t) = n^T
            Eigen::Vector3d jac_translation = n;
            
            // Rotation part: d(n^T * (R*p + t - q))/d(so3) = n^T * d(R*p)/d(so3)
            // Using right perturbation: d(R*p)/d(so3) = -R*p^× (skew-symmetric)
            Eigen::Vector3d Rp = R * p;
            Eigen::Matrix3d Rp_skew;
            Rp_skew << 0, -Rp(2), Rp(1),
                      Rp(2), 0, -Rp(0),
                     -Rp(1), Rp(0), 0;
            Eigen::Vector3d jac_rotation = -n.transpose() * Rp_skew;
            
            // Combine jacobians with weighting (only information weight)
            // Robust weighting is handled by Ceres Loss Function
            double weight = m_information_weight;
            jac.block<1, 3>(0, 0) = weight * jac_translation.transpose(); // translation
            jac.block<1, 3>(0, 3) = weight * jac_rotation.transpose();    // rotation
        }
        
        return true;
        
    } catch (const std::exception& e) {
        spdlog::error("[PointToPlaneFactor::Evaluate] Exception: {}", e.what());
        return false;
    }
}

double PointToPlaneFactor::compute_raw_residual(double const* const* parameters) const {
    if (m_is_outlier) {
        return 0.0; // Don't contribute to robust estimation if outlier
    }
    
    try {
        // Extract SE3 parameters from tangent space
        Eigen::Map<const Eigen::Vector6d> se3_tangent(parameters[0]);
        
        // Convert tangent space to SE3
        Sophus::SE3d T_correction = Sophus::SE3d::exp(se3_tangent);
        
        // Extract rotation and translation
        Eigen::Matrix3d R = T_correction.rotationMatrix();
        Eigen::Vector3d t = T_correction.translation();
        
        // Convert float points to double for computation
        Eigen::Vector3d p = m_source_point.cast<double>();
        Eigen::Vector3d q = m_target_point.cast<double>();
        Eigen::Vector3d n = m_plane_normal.cast<double>();
        
        // Apply local correction to source point
        Eigen::Vector3d p_corrected = R * p + t;
        
        // Compute raw point-to-plane distance (without weighting)
        double raw_residual = n.dot(p_corrected - q);
        
        return std::abs(raw_residual); // Return absolute value for robust estimation
        
    } catch (const std::exception& e) {
        spdlog::error("[PointToPlaneFactor::compute_raw_residual] Exception: {}", e.what());
        return std::numeric_limits<double>::max();
    }
}

} // namespace optimization
} // namespace lidar_odometry
