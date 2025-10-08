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
            Eigen::Vector3d jac_translation = n.transpose()*R;
            
            // Rotation part: d(n^T * (R*p + t - q))/d(so3) = n^T * d(R*p)/d(so3)
            // Using right perturbation: d(R*p)/d(so3) = -R*p^× (skew-symmetric)
            Eigen::Vector3d Rp = R * p;
            Eigen::Matrix3d Rp_skew;
            Rp_skew << 0, -Rp(2), Rp(1),
                      Rp(2), 0, -Rp(0),
                     -Rp(1), Rp(0), 0;
            Eigen::Vector3d jac_rotation = -n.transpose() * R * Rp_skew;
            
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

// PointToPlaneFactorDualFrame implementation
PointToPlaneFactorDualFrame::PointToPlaneFactorDualFrame(const Eigen::Vector3d& p,
                                                           const Eigen::Vector3d& q,
                                                           const Eigen::Vector3d& nq,
                                                           double information_weight)
    : m_p(p)
    , m_q(q)
    , m_nq(nq.normalized())
    , m_information_weight(information_weight)
    , m_robust_weight(1.0)
    , m_is_outlier(false) {
}

bool PointToPlaneFactorDualFrame::Evaluate(double const* const* parameters,
                                           double* residuals,
                                           double** jacobians) const {
    
    // If marked as outlier, return large residual with zero jacobians
    if (m_is_outlier) {
        residuals[0] = 100.0; // Large residual to indicate outlier
        
        if (jacobians) {
            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> jac0(jacobians[0]);
                jac0.setZero();
            }
            if (jacobians[1]) {
                Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> jac1(jacobians[1]);
                jac1.setZero();
            }
        }
        return true;
    }

    try {
        // Extract SE3 parameters from tangent space (Ceres order: tx,ty,tz,rx,ry,rz)
        Eigen::Map<const Eigen::Vector6d> se3_tangent1(parameters[0]); // First pose
        Eigen::Map<const Eigen::Vector6d> se3_tangent2(parameters[1]); // Second pose
        
        // Convert tangent space to SE3 using Sophus exp
        Sophus::SE3d T1 = Sophus::SE3d::exp(se3_tangent1);
        Sophus::SE3d T2 = Sophus::SE3d::exp(se3_tangent2);
        
        // Extract rotation and translation
        Eigen::Matrix3d R1 = T1.rotationMatrix();
        Eigen::Vector3d t1 = T1.translation();
        Eigen::Matrix3d R2 = T2.rotationMatrix();
        Eigen::Vector3d t2 = T2.translation();
        
        // Transform points to world coordinates
        Eigen::Vector3d T1p = R1 * m_p + t1;  // Point p transformed by first pose
        Eigen::Vector3d T2q = R2 * m_q + t2;  // Point q transformed by second pose
        
        // Normal is already in world coordinates (g2o 방식)
        // Compute point-to-plane residual: n_q_world^T * ((R1*p + t1) - (R2*q + t2))
        // g2o EdgeICP uses: error = n^T * (q - T*p), we use n^T * (T1*p - T2*q)
        double raw_residual = m_nq.dot(T1p - T2q);
        
        // Apply information and robust weighting
        residuals[0] = raw_residual * m_information_weight * m_robust_weight;
        
        // Compute Jacobians if requested
        if (jacobians) {
            // Jacobian w.r.t. first pose (parameters[0])
            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> jac0(jacobians[0]);
                
                // Right multiplication인 경우 Jacobian (현재 구현)
                // d(residual)/d(t1) = -m_nq^T * m_information_weight * m_robust_weight
                jac0.block<1, 3>(0, 0) = m_nq.transpose() * R1 * m_information_weight * m_robust_weight;
                
                // d(residual)/d(theta1) = -m_nq^T * R1 * [m_p]_x * m_information_weight * m_robust_weight  
                jac0.block<1, 3>(0, 3) = -m_nq.transpose() * R1 * skew_symmetric(m_p) * m_information_weight * m_robust_weight;
            }
            
            // Jacobian w.r.t. second pose (parameters[1])
            if (jacobians[1]) {
                Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> jac1(jacobians[1]);
                
                // d(residual)/d(t2) = -m_nq^T * m_information_weight * m_robust_weight
                jac1.block<1, 3>(0, 0) = -m_nq.transpose() * R2 * m_information_weight * m_robust_weight;
                
                // d(residual)/d(theta2) = -m_nq^T * R2 * [m_q]_x * m_information_weight * m_robust_weight
                jac1.block<1, 3>(0, 3) = m_nq.transpose() * R2 * skew_symmetric(m_q) * m_information_weight * m_robust_weight;
            }
        }
        
        return true;
        
    } catch (const std::exception& e) {
        spdlog::error("[PointToPlaneFactorDualFrame] Evaluation failed: {}", e.what());
        residuals[0] = std::numeric_limits<double>::max();
        return false;
    }
}

double PointToPlaneFactorDualFrame::compute_raw_residual(double const* const* parameters) const {
    try {
        // Extract SE3 parameters from tangent space
        Eigen::Map<const Eigen::Vector6d> se3_tangent1(parameters[0]);
        Eigen::Map<const Eigen::Vector6d> se3_tangent2(parameters[1]);
        
        // Convert tangent space to SE3
        Sophus::SE3d T1 = Sophus::SE3d::exp(se3_tangent1);
        Sophus::SE3d T2 = Sophus::SE3d::exp(se3_tangent2);
        
        // Extract rotation and translation
        Eigen::Matrix3d R1 = T1.rotationMatrix();
        Eigen::Vector3d t1 = T1.translation();
        Eigen::Matrix3d R2 = T2.rotationMatrix();
        Eigen::Vector3d t2 = T2.translation();
        
        // Transform points to world coordinates
        Eigen::Vector3d T1p = R1 * m_p + t1;
        Eigen::Vector3d T2q = R2 * m_q + t2;
        
        // Return raw residual (without weighting)
        return m_nq.dot(T2q - T1p);
        
    } catch (const std::exception& e) {
        spdlog::error("[PointToPlaneFactorDualFrame] Raw residual computation failed: {}", e.what());
        return std::numeric_limits<double>::max();
    }
}

Eigen::Matrix3d PointToPlaneFactorDualFrame::skew_symmetric(const Eigen::Vector3d& v) const {
    Eigen::Matrix3d S;
    S <<     0, -v.z(),  v.y(),
         v.z(),     0, -v.x(),
        -v.y(),  v.x(),     0;
    return S;
}

} // namespace optimization
} // namespace lidar_odometry
