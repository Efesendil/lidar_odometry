/**
 * @file      Parameters.h
 * @brief     Defines parameter blocks for Ceres optimization in LiDAR Odometry.
 * @author    Seungwon Choi
 * @date      2025-09-24
 * @copyright Copyright (c) 2025 Seungwon Choi. All rights reserved.
 *
 * @par License
 * This project is released under the MIT License.
 */

#pragma once

#include <ceres/local_parameterization.h>
#include <Eigen/Dense>
#include <sophus/se3.hpp>

// Define Vector6d as it's not available in standard Eigen
namespace Eigen {
    typedef Matrix<double, 6, 1> Vector6d;
    typedef Matrix<float, 6, 1> Vector6f;
}

namespace lidar_odometry {
namespace optimization {

/**
 * @brief SE3 Global Parameterization for Ceres optimization
 * Parameterizes SE3 group using 6DoF tangent space representation
 * Parameters: [t_x, t_y, t_z, so3_x, so3_y, so3_z] (Ceres order)
 * 
 * For Twb (body to world transform), we use right multiplication:
 * Twb_new = Twb * exp(delta)
 * 
 * This means the perturbation is applied in the body frame.
 * 
 * NOTE: Uses double precision for Ceres optimization (required)
 * but interfaces with float-based pose storage
 */
class SE3GlobalParameterization : public ceres::LocalParameterization {
public:
    SE3GlobalParameterization() : m_is_fixed(false) {}
    virtual ~SE3GlobalParameterization() = default;

    /**
     * @brief Set parameter as fixed (prevents updates during optimization)
     * @param is_fixed If true, parameters will not be updated
     */
    void set_fixed(bool is_fixed) { m_is_fixed = is_fixed; }
    
    /**
     * @brief Check if parameter is fixed
     * @return true if parameter is fixed
     */
    bool is_fixed() const { return m_is_fixed; }

    /**
     * @brief Plus operation: x_plus_delta = SE3(x) * exp(delta)
     * @param x Current SE3 parameters in tangent space [6]
     * @param delta Update vector in tangent space [6] 
     * @param x_plus_delta Updated SE3 parameters [6]
     * @return true if successful
     */
    virtual bool Plus(const double* x,
                     const double* delta,
                     double* x_plus_delta) const override;
    
    /**
     * @brief Compute Jacobian of Plus operation w.r.t delta
     * @param x Current parameters [6]
     * @param jacobian Output jacobian matrix [6x6] in row-major order
     * @return true if successful
     */
    virtual bool ComputeJacobian(const double* x,
                                double* jacobian) const override;
    
    /**
     * @brief Global size of the parameter (tangent space dimension)
     */
    virtual int GlobalSize() const override { return 6; }
    
    /**
     * @brief Local size of the perturbation (tangent space dimension)
     */
    virtual int LocalSize() const override { return 6; }

    // ===== Utility functions for float/double conversion =====
    
    /**
     * @brief Convert float SE3 to double tangent space for Ceres
     */
    static Eigen::Vector6d se3f_to_tangent_d(const Sophus::SE3f& se3f);
    
    /**
     * @brief Convert double tangent space to float SE3 after optimization
     */
    static Sophus::SE3f tangent_d_to_se3f(const Eigen::Vector6d& tangent_d);

    /**
     * @brief Convert SE3 tangent space vector to SE3 group element
     * @param tangent SE3 tangent space vector [translation, so3] (Ceres order)
     * @return SE3 group element
     */
    static Sophus::SE3d tangent_to_se3(const Eigen::Vector6d& tangent);

    /**
     * @brief Convert SE3 group element to tangent space vector
     * @param se3 SE3 group element
     * @return SE3 tangent space vector [translation, so3] (Ceres order)
     */
    static Eigen::Vector6d se3_to_tangent(const Sophus::SE3d& se3);

private:

    bool m_is_fixed;  // Flag to prevent parameter updates when true
};

} // namespace optimization
} // namespace lidar_odometry
