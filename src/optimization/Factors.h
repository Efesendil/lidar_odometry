/**
 * @file      Factors.h
 * @brief     Defines Ceres cost functions (factors) for LiDAR Odometry optimization.
 * @author    Seungwon Choi
 * @date      2025-09-24
 * @copyright Copyright (c) 2025 Seungwon Choi. All rights reserved.
 *
 * @par License
 * This project is released under the MIT License.
 */

#pragma once

#include <vector>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <ceres/sized_cost_function.h>
#include <sophus/se3.hpp>
#include <spdlog/spdlog.h>

// Define Vector6d as it's not available in standard Eigen
namespace Eigen {
    typedef Matrix<double, 6, 1> Vector6d;
    typedef Matrix<float, 6, 1> Vector6f;
}

namespace lidar_odometry {
namespace optimization {

/**
 * @brief Point-to-Plane ICP cost function with analytical Jacobian
 * 
 * This factor implements the point-to-plane distance metric used in ICP:
 * residual = n_q^T * (R * p + t - q)
 * 
 * where:
 * - p: point in source frame (current local frame)
 * - q: point on target plane (current local frame, transformed from map) 
 * - n_q: normal vector of target plane (current local frame)
 * - R, t: SE3 correction parameters within current local frame
 * 
 * Parameters: SE3 pose correction in tangent space [6]
 * Residual dimension: [1]
 */
class PointToPlaneFactor : public ceres::SizedCostFunction<1, 6> {
public:
    /**
     * @brief Constructor
     * @param source_point 3D point in source frame (current local frame)
     * @param target_point 3D point on target plane (current local frame)
     * @param plane_normal Normal vector of target plane (current local frame, normalized)
     * @param information_weight Information weight (sqrt of information matrix)
     */
    PointToPlaneFactor(const Eigen::Vector3f& source_point,
                       const Eigen::Vector3f& target_point,
                       const Eigen::Vector3f& plane_normal,
                       double information_weight = 1.0);

    /**
     * @brief Set outlier flag to disable optimization for this factor
     * @param is_outlier If true, this factor will not contribute to optimization
     */
    void set_outlier(bool is_outlier) { m_is_outlier = is_outlier; }
    
    /**
     * @brief Get outlier flag
     * @return true if this factor is marked as outlier
     */
    bool is_outlier() const { return m_is_outlier; }

    /**
     * @brief Set robust weight for this factor
     * @param weight Robust weight computed from loss function
     */
    void set_robust_weight(double weight) { m_robust_weight = weight; }

    /**
     * @brief Get current robust weight
     * @return Current robust weight
     */
    double get_robust_weight() const { return m_robust_weight; }

    /**
     * @brief Evaluate residual and Jacobian
     * @param parameters SE3 pose parameters in tangent space [6]
     * @param residuals Output residual [1]
     * @param jacobians Output Jacobian matrices [1x6] if not nullptr
     * @return true if evaluation successful
     */
    virtual bool Evaluate(double const* const* parameters,
                         double* residuals,
                         double** jacobians) const override;

    /**
     * @brief Compute raw residual without weighting (for robust estimation)
     * @param parameters SE3 pose parameters in tangent space [6]
     * @return Raw residual value
     */
    double compute_raw_residual(double const* const* parameters) const;

    /**
     * @brief Get source point in local coordinates
     */
    const Eigen::Vector3f& get_source_point() const { return m_source_point; }

    /**
     * @brief Get target point in world coordinates
     */
    const Eigen::Vector3f& get_target_point() const { return m_target_point; }

    /**
     * @brief Get plane normal in world coordinates
     */
    const Eigen::Vector3f& get_plane_normal() const { return m_plane_normal; }

private:
    Eigen::Vector3f m_source_point;      // Point in source frame (local)
    Eigen::Vector3f m_target_point;      // Point on target plane (world)
    Eigen::Vector3f m_plane_normal;      // Plane normal (world, normalized)
    double m_information_weight;         // Information weight
    double m_robust_weight;              // Robust weight for outlier handling
    bool m_is_outlier;                   // Outlier flag to disable optimization
};

/**
 * @brief Point-to-Plane ICP cost function between two poses (dual frame)
 * 
 * This factor implements the point-to-plane distance metric for dual frame optimization:
 * residual = n_q^T * ((R2 * q + t2) - (R1 * p + t1))
 * 
 * where:
 * - p: point observed in first pose frame
 * - q: corresponding point observed in second pose frame
 * - n_q: normal vector at point q (normalized)
 * - R1, t1: SE3 transformation parameters for first pose
 * - R2, t2: SE3 transformation parameters for second pose
 * 
 * Parameters: Two SE3 pose corrections in tangent space [6] each
 * Residual dimension: [1]
 * 
 * Note: Parameter order difference between g2o and Ceres:
 * - g2o: [rx, ry, rz, tx, ty, tz]
 * - Ceres: [tx, ty, tz, rx, ry, rz]
 */
class PointToPlaneFactorDualFrame : public ceres::SizedCostFunction<1, 6, 6> {
public:
    /**
     * @brief Constructor
     * @param p Point observed in first pose frame
     * @param q Point observed in second pose frame  
     * @param nq Normal vector at point q (will be normalized)
     * @param information_weight Information weight (sqrt of information matrix)
     */
    PointToPlaneFactorDualFrame(const Eigen::Vector3d& p,
                                const Eigen::Vector3d& q,
                                const Eigen::Vector3d& nq,
                                double information_weight = 1.0);

    /**
     * @brief Set outlier flag to disable optimization for this factor
     * @param is_outlier If true, this factor will not contribute to optimization
     */
    void set_outlier(bool is_outlier) { m_is_outlier = is_outlier; }
    
    /**
     * @brief Get outlier flag
     * @return true if this factor is marked as outlier
     */
    bool is_outlier() const { return m_is_outlier; }

    /**
     * @brief Set robust weight for this factor
     * @param weight Robust weight computed from loss function
     */
    void set_robust_weight(double weight) { m_robust_weight = weight; }

    /**
     * @brief Get current robust weight
     * @return Current robust weight
     */
    double get_robust_weight() const { return m_robust_weight; }

    /**
     * @brief Evaluate residual and Jacobian
     * @param parameters SE3 pose parameters [pose1, pose2] in tangent space [tx,ty,tz,rx,ry,rz]
     * @param residuals Output residual [1]
     * @param jacobians Output Jacobian matrices [1x6, 1x6] if not nullptr
     * @return true if evaluation successful
     */
    virtual bool Evaluate(double const* const* parameters,
                         double* residuals,
                         double** jacobians) const override;

    /**
     * @brief Compute raw residual without weighting (for robust estimation)
     * @param parameters SE3 pose parameters [pose1, pose2] in tangent space
     * @return Raw residual value
     */
    double compute_raw_residual(double const* const* parameters) const;

    /**
     * @brief Get point p (observed in first pose frame)
     */
    const Eigen::Vector3d& get_point_p() const { return m_p; }

    /**
     * @brief Get point q (observed in second pose frame)
     */
    const Eigen::Vector3d& get_point_q() const { return m_q; }

    /**
     * @brief Get normal vector at point q
     */
    const Eigen::Vector3d& get_normal_q() const { return m_nq; }

private:
    /**
     * @brief Create skew-symmetric matrix from 3D vector
     * @param v Input 3D vector
     * @return 3x3 skew-symmetric matrix [v]_x
     */
    Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d& v) const;

    Eigen::Vector3d m_p;                 // Point observed in first pose frame
    Eigen::Vector3d m_q;                 // Point observed in second pose frame
    Eigen::Vector3d m_nq;                // Normal vector at point q (normalized)
    double m_information_weight;         // Information weight
    double m_robust_weight;              // Robust weight for outlier handling
    bool m_is_outlier;                   // Outlier flag to disable optimization
};

/**
 * @brief Relative Pose Factor for Pose Graph Optimization
 * 
 * This factor implements relative pose constraints between two poses in pose graph:
 * residual = log(T_ij^(-1) * T_i^(-1) * T_j)
 * 
 * where:
 * - T_i: SE3 transformation of pose i
 * - T_j: SE3 transformation of pose j  
 * - T_ij: Measured relative transformation from pose i to pose j
 * 
 * Parameters: Two SE3 pose parameters in tangent space [6] each
 * Residual dimension: [6] (SE3 tangent space)
 * 
 * This factor is used for:
 * - Sequential odometry constraints
 * - Loop closure constraints
 * - GPS or other absolute pose measurements
 */
class RelativePoseFactor : public ceres::SizedCostFunction<6, 6, 6> {
public:
    /**
     * @brief Constructor
     * @param relative_measurement Measured relative transformation T_ij from pose i to pose j
     * @param information_matrix 6x6 information matrix (inverse of covariance)
     */
    RelativePoseFactor(const Sophus::SE3d& relative_measurement,
                       const Eigen::Matrix<double, 6, 6>& information_matrix);

    /**
     * @brief Constructor with simplified information matrix
     * @param relative_measurement Measured relative transformation T_ij from pose i to pose j
     * @param translation_weight Weight for translation components (x, y, z)
     * @param rotation_weight Weight for rotation components (rx, ry, rz)
     */
    RelativePoseFactor(const Sophus::SE3d& relative_measurement,
                       double translation_weight = 1.0,
                       double rotation_weight = 1.0);

    /**
     * @brief Set outlier flag to disable optimization for this factor
     * @param is_outlier If true, this factor will not contribute to optimization
     */
    void set_outlier(bool is_outlier) { m_is_outlier = is_outlier; }
    
    /**
     * @brief Get outlier flag
     * @return true if this factor is marked as outlier
     */
    bool is_outlier() const { return m_is_outlier; }

    /**
     * @brief Set robust weight for this factor
     * @param weight Robust weight computed from loss function
     */
    void set_robust_weight(double weight) { m_robust_weight = weight; }

    /**
     * @brief Get current robust weight
     * @return Current robust weight
     */
    double get_robust_weight() const { return m_robust_weight; }

    /**
     * @brief Evaluate residual and Jacobian
     * @param parameters SE3 pose parameters [pose_i, pose_j] in tangent space [tx,ty,tz,rx,ry,rz]
     * @param residuals Output residual [6] in SE3 tangent space
     * @param jacobians Output Jacobian matrices [6x6, 6x6] if not nullptr
     * @return true if evaluation successful
     */
    virtual bool Evaluate(double const* const* parameters,
                         double* residuals,
                         double** jacobians) const override;

    /**
     * @brief Compute raw residual without weighting (for robust estimation)
     * @param parameters SE3 pose parameters [pose_i, pose_j] in tangent space
     * @param residuals Output residual [6] in SE3 tangent space
     */
    void compute_raw_residual(double const* const* parameters, double* residuals) const;

    /**
     * @brief Get measured relative transformation
     */
    const Sophus::SE3d& get_relative_measurement() const { return m_relative_measurement; }

    /**
     * @brief Get information matrix
     */
    const Eigen::Matrix<double, 6, 6>& get_information_matrix() const { return m_information_matrix; }

private:
    /**
     * @brief Create skew-symmetric matrix from 3D vector
     * @param v Input 3D vector
     * @return 3x3 skew-symmetric matrix [v]_x
     */
    Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d& v) const;

    /**
     * @brief Compute right Jacobian for SE(3)
     * @param xi 6D tangent space vector
     * @return 6x6 right Jacobian matrix
     */
    Eigen::Matrix<double, 6, 6> right_jacobian(const Eigen::Matrix<double, 6, 1>& xi) const;

    /**
     * @brief Compute inverse right Jacobian for SE(3)
     * @param xi 6D tangent space vector
     * @return 6x6 inverse right Jacobian matrix
     */
    Eigen::Matrix<double, 6, 6> right_jacobian_inverse(const Eigen::Matrix<double, 6, 1>& xi) const;

    Sophus::SE3d m_relative_measurement;                // Measured relative transformation T_ij
    Eigen::Matrix<double, 6, 6> m_information_matrix;  // Information matrix (6x6)
    double m_robust_weight;                             // Robust weight for outlier handling
    bool m_is_outlier;                                  // Outlier flag to disable optimization
};

} // namespace optimization
} // namespace lidar_odometry
