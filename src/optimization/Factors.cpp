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

// RelativePoseFactor implementation
RelativePoseFactor::RelativePoseFactor(const Sophus::SE3d& relative_measurement,
                                       const Eigen::Matrix<double, 6, 6>& information_matrix)
    : m_relative_measurement(relative_measurement)
    , m_information_matrix(information_matrix)
    , m_robust_weight(1.0)
    , m_is_outlier(false) {
}

RelativePoseFactor::RelativePoseFactor(const Sophus::SE3d& relative_measurement,
                                       double translation_weight,
                                       double rotation_weight)
    : m_relative_measurement(relative_measurement)
    , m_robust_weight(1.0)
    , m_is_outlier(false) {
    // Create diagonal information matrix with separate weights for translation and rotation
    m_information_matrix = Eigen::Matrix<double, 6, 6>::Zero();
    m_information_matrix.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * translation_weight;
    m_information_matrix.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * rotation_weight;
}

bool RelativePoseFactor::Evaluate(double const* const* parameters,
                                  double* residuals,
                                  double** jacobians) const {
    // If marked as outlier, return zero residual with zero jacobians
    if (m_is_outlier) {
        Eigen::Map<Eigen::Vector6d> residual_map(residuals);
        residual_map.setZero();
        
        if (jacobians) {
            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> jac0(jacobians[0]);
                jac0.setZero();
            }
            if (jacobians[1]) {
                Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> jac1(jacobians[1]);
                jac1.setZero();
            }
        }
        return true;
    }

    try {
        // Extract SE3 parameters from tangent space
        Eigen::Map<const Eigen::Vector6d> tangent_i(parameters[0]);
        Eigen::Map<const Eigen::Vector6d> tangent_j(parameters[1]);
        
        // Convert tangent space to SE3
        Sophus::SE3d T_i = SE3GlobalParameterization::tangent_to_se3(tangent_i);
        Sophus::SE3d T_j = SE3GlobalParameterization::tangent_to_se3(tangent_j);
        
        // Compute error: T_error = T_ij^(-1) * T_i^(-1) * T_j
        Sophus::SE3d T_error = m_relative_measurement.inverse() * T_i.inverse() * T_j;
        
        // Convert error to tangent space (log map)
        Eigen::Vector6d error_tangent = T_error.log();
        
        // Apply square root of information matrix for weighting
        // NOTE: Ceres expects residual in the form: sqrt(Information) * error
        // The information matrix is already set correctly (inverse of covariance)
        Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt(m_information_matrix);
        Eigen::Matrix<double, 6, 6> sqrt_info = llt.matrixL();
        
        // Weighted residual: sqrt_info scales the error by sqrt(weight)
        // This is the standard way in Ceres to incorporate information matrix
        Eigen::Map<Eigen::Vector6d> residual_map(residuals);
        residual_map = sqrt_info * error_tangent * m_robust_weight;
        
        // Compute Jacobians if requested
        if (jacobians) {
            // Compute right Jacobian inverse for error (Logmap derivative)
            Eigen::Matrix<double, 6, 6> Jr_inv = right_jacobian_inverse(error_tangent);
            
            // Compute relative transformation (Between operation result)
            // T_relative = T_i^(-1) * T_j
            Sophus::SE3d T_relative = T_i.inverse() * T_j;
            
            // Jacobian w.r.t. pose i (parameters[0])
            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> jac_i(jacobians[0]);
                
                // Correct Jacobian: J_i = sqrt_info * Jr_inv * (-Adjoint(T_relative^(-1))) * m_robust_weight
                // This matches GTSAM's: H1 = -D_v_h * h.inverse().AdjointMap()
                // where h = T_i^(-1) * T_j and D_v_h is the Logmap derivative (Jr_inv)
                jac_i = sqrt_info * Jr_inv * (-T_relative.inverse().Adj()) * m_robust_weight;
            }
            
            // Jacobian w.r.t. pose j (parameters[1])
            if (jacobians[1]) {
                Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> jac_j(jacobians[1]);
                
                // J_j = sqrt_info * Jr_inv * I * m_robust_weight
                // This matches GTSAM's: H2 = D_v_h (identity is absorbed in between operation)
                jac_j = sqrt_info * Jr_inv * m_robust_weight;
            }
            
            // Log Jacobian norms for ALL constraints to analyze gradient flow
            if (constraint_type != "unknown" && jacobians[0] && jacobians[1]) {
                static int call_count = 0;
                call_count++;
                
                // Log first iteration (first ~500 evaluations) to see all constraints
                if (call_count <= 500) {
                    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> jac_i(jacobians[0]);
                    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> jac_j(jacobians[1]);
                    
                    double jac_i_norm = jac_i.norm();
                    double jac_j_norm = jac_j.norm();
                    double residual_norm = residual_map.norm();
                    
                    spdlog::info("[Factor {}] KF {}→{}: residual={:.6f}, jac_i={:.6f}, jac_j={:.6f}", 
                                 constraint_type, from_kf_id, to_kf_id, 
                                 residual_norm, jac_i_norm, jac_j_norm);
                }
            }
        }
        
        return true;
        
    } catch (const std::exception& e) {
        spdlog::error("[RelativePoseFactor] Evaluation failed: {}", e.what());
        Eigen::Map<Eigen::Vector6d> residual_map(residuals);
        residual_map.setConstant(std::numeric_limits<double>::max());
        return false;
    }
}

void RelativePoseFactor::compute_raw_residual(double const* const* parameters, double* residuals) const {
    try {
        // Extract SE3 parameters from tangent space
        Eigen::Map<const Eigen::Vector6d> tangent_i(parameters[0]);
        Eigen::Map<const Eigen::Vector6d> tangent_j(parameters[1]);
        
        // Convert tangent space to SE3
        Sophus::SE3d T_i = SE3GlobalParameterization::tangent_to_se3(tangent_i);
        Sophus::SE3d T_j = SE3GlobalParameterization::tangent_to_se3(tangent_j);
        
        // Compute error: T_error = T_ij^(-1) * T_i^(-1) * T_j
        Sophus::SE3d T_error = m_relative_measurement.inverse() * T_i.inverse() * T_j;
        
        // Convert error to tangent space (log map)
        Eigen::Vector6d error_tangent = T_error.log();
        
        // Return raw residual (without weighting)
        Eigen::Map<Eigen::Vector6d> residual_map(residuals);
        residual_map = error_tangent;
        
    } catch (const std::exception& e) {
        spdlog::error("[RelativePoseFactor] Raw residual computation failed: {}", e.what());
        Eigen::Map<Eigen::Vector6d> residual_map(residuals);
        residual_map.setConstant(std::numeric_limits<double>::max());
    }
}

Eigen::Matrix3d RelativePoseFactor::skew_symmetric(const Eigen::Vector3d& v) const {
    Eigen::Matrix3d S;
    S <<     0, -v.z(),  v.y(),
         v.z(),     0, -v.x(),
        -v.y(),  v.x(),     0;
    return S;
}

Eigen::Matrix<double, 6, 6> RelativePoseFactor::right_jacobian(const Eigen::Matrix<double, 6, 1>& xi) const {
    // Extract rotation and translation parts
    Eigen::Vector3d rho = xi.head<3>();
    Eigen::Vector3d theta = xi.tail<3>();
    
    double theta_norm = theta.norm();
    Eigen::Matrix3d Theta = skew_symmetric(theta);
    Eigen::Matrix3d Rho = skew_symmetric(rho);
    
    Eigen::Matrix3d J_SO3;
    Eigen::Matrix3d Q;
    
    if (theta_norm < 1e-6) {
        // Small angle approximation
        J_SO3 = Eigen::Matrix3d::Identity() + 0.5 * Theta;
        Q = 0.5 * Rho;
    } else {
        double theta2 = theta_norm * theta_norm;
        double theta3 = theta2 * theta_norm;
        
        J_SO3 = Eigen::Matrix3d::Identity() 
              + (1.0 - std::cos(theta_norm)) / theta2 * Theta
              + (theta_norm - std::sin(theta_norm)) / theta3 * Theta * Theta;
        
        Q = 0.5 * Rho 
          + (theta_norm - std::sin(theta_norm)) / theta3 * (Theta * Rho + Rho * Theta + Theta * Rho * Theta)
          - (1.0 - 0.5 * theta2 - std::cos(theta_norm)) / (theta2 * theta2) 
            * (Theta * Theta * Rho + Rho * Theta * Theta - 3.0 * Theta * Rho * Theta)
          - 0.5 * ((1.0 - 0.5 * theta2 - std::cos(theta_norm)) / (theta2 * theta2) 
            - 3.0 * (theta_norm - std::sin(theta_norm) - theta3 / 6.0) / (theta3 * theta2))
            * (Theta * Rho * Theta * Theta + Theta * Theta * Rho * Theta);
    }
    
    Eigen::Matrix<double, 6, 6> Jr;
    Jr.setZero();
    Jr.block<3, 3>(0, 0) = J_SO3;
    Jr.block<3, 3>(0, 3) = Q;
    Jr.block<3, 3>(3, 3) = J_SO3;
    
    return Jr;
}

Eigen::Matrix<double, 6, 6> RelativePoseFactor::right_jacobian_inverse(const Eigen::Matrix<double, 6, 1>& xi) const {
    // Extract rotation and translation parts
    Eigen::Vector3d rho = xi.head<3>();
    Eigen::Vector3d theta = xi.tail<3>();
    
    double theta_norm = theta.norm();
    Eigen::Matrix3d Theta = skew_symmetric(theta);
    Eigen::Matrix3d Rho = skew_symmetric(rho);
    
    Eigen::Matrix3d J_SO3_inv;
    Eigen::Matrix3d Q_inv;
    
    if (theta_norm < 1e-6) {
        // Small angle approximation
        J_SO3_inv = Eigen::Matrix3d::Identity() - 0.5 * Theta;
        Q_inv = -0.5 * Rho;
    } else {
        double theta2 = theta_norm * theta_norm;
        double half_theta = 0.5 * theta_norm;
        
        J_SO3_inv = Eigen::Matrix3d::Identity() 
                  - 0.5 * Theta
                  + (1.0 - half_theta / std::tan(half_theta)) / theta2 * Theta * Theta;
        
        Q_inv = -0.5 * Rho 
              + (1.0 / theta2) * (Theta * Rho + Rho * Theta - Theta * Rho * Theta)
              - (1.0 - half_theta / std::tan(half_theta)) / theta2 
                * (Theta * Theta * Rho + Rho * Theta * Theta - 3.0 * Theta * Rho * Theta);
    }
    
    Eigen::Matrix<double, 6, 6> Jr_inv;
    Jr_inv.setZero();
    Jr_inv.block<3, 3>(0, 0) = J_SO3_inv;
    Jr_inv.block<3, 3>(0, 3) = Q_inv;
    Jr_inv.block<3, 3>(3, 3) = J_SO3_inv;
    
    return Jr_inv;
}


} // namespace optimization
} // namespace lidar_odometry
