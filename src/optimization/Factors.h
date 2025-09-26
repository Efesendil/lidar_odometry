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
 * - p: point in source frame (local coordinates)
 * - q: point on target plane (world coordinates) 
 * - n_q: normal vector of target plane (world coordinates)
 * - R, t: SE3 transformation parameters (source to world)
 * 
 * Parameters: SE3 pose in tangent space [6]
 * Residual dimension: [1]
 */
class PointToPlaneFactor : public ceres::SizedCostFunction<1, 6> {
public:
    /**
     * @brief Constructor
     * @param source_point 3D point in source frame (local coordinates)
     * @param target_point 3D point on target plane (world coordinates)
     * @param plane_normal Normal vector of target plane (world coordinates, normalized)
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

} // namespace optimization
} // namespace lidar_odometry
