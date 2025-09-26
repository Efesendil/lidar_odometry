/**
 * @file      MathUtils.cpp
 * @brief     Implementation of math utility functions
 * @author    Seungwon Choi
 * @date      2025-09-24
 * @copyright Copyright (c) 2025 Seungwon Choi. All rights reserved.
 *
 * @par License
 * This project is released under the MIT License.
 */

#include "MathUtils.h"
#include <cmath>
#include <spdlog/spdlog.h>

namespace lidar_odometry {
namespace util {

Eigen::Matrix3f MathUtils::normalize_rotation_matrix(const Eigen::Matrix3f& R) {
    // Perform SVD: R = U * S * V^T
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Matrix3f V = svd.matrixV();
    
    // The closest orthogonal matrix is U * V^T
    Eigen::Matrix3f R_normalized = U * V.transpose();
    
    // Ensure proper rotation (determinant = +1, not reflection)
    if (R_normalized.determinant() < 0) {
        // If determinant is negative, flip the last column of U
        U.col(2) *= -1;
        R_normalized = U * V.transpose();
    }
    
    // Verify the result
    float det = R_normalized.determinant();
    float orthogonality_error = (R_normalized * R_normalized.transpose() - Eigen::Matrix3f::Identity()).norm();
    
    // if (std::abs(det - 1.0f) > 1e-6f || orthogonality_error > 1e-6f) {
    //     spdlog::warn("[MathUtils] Rotation normalization may have failed: det={}, orth_error={}", det, orthogonality_error);
    // }
    
    return R_normalized;
}

Eigen::Matrix3d MathUtils::normalize_rotation_matrix(const Eigen::Matrix3d& R) {
    // Perform SVD: R = U * S * V^T
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    
    // The closest orthogonal matrix is U * V^T
    Eigen::Matrix3d R_normalized = U * V.transpose();
    
    // Ensure proper rotation (determinant = +1, not reflection)
    if (R_normalized.determinant() < 0) {
        // If determinant is negative, flip the last column of U
        U.col(2) *= -1;
        R_normalized = U * V.transpose();
    }
    
    // Verify the result
    double det = R_normalized.determinant();
    double orthogonality_error = (R_normalized * R_normalized.transpose() - Eigen::Matrix3d::Identity()).norm();
    
    // if (std::abs(det - 1.0) > 1e-12 || orthogonality_error > 1e-12) {
    //     spdlog::warn("[MathUtils] Rotation normalization may have failed: det={}, orth_error={}", det, orthogonality_error);
    // }
    
    return R_normalized;
}

bool MathUtils::is_orthogonal(const Eigen::Matrix3f& R, float tolerance) {
    Eigen::Matrix3f should_be_identity = R * R.transpose();
    float error = (should_be_identity - Eigen::Matrix3f::Identity()).norm();
    return error < tolerance && std::abs(R.determinant() - 1.0f) < tolerance;
}

bool MathUtils::is_orthogonal(const Eigen::Matrix3d& R, double tolerance) {
    Eigen::Matrix3d should_be_identity = R * R.transpose();
    double error = (should_be_identity - Eigen::Matrix3d::Identity()).norm();
    return error < tolerance && std::abs(R.determinant() - 1.0) < tolerance;
}

float MathUtils::wrap_to_pi(float angle) {
    while (angle > M_PI) {
        angle -= 2.0f * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0f * M_PI;
    }
    return angle;
}

double MathUtils::wrap_to_pi(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double MathUtils::wrap_angle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double MathUtils::deg_to_rad(double degrees) {
    return degrees * M_PI / 180.0;
}

double MathUtils::rad_to_deg(double radians) {
    return radians * 180.0 / M_PI;
}

Sophus::SE3d MathUtils::se3f_to_se3d(const Sophus::SE3f& se3_f) {
    return Sophus::SE3d(se3_f.so3().cast<double>(), se3_f.translation().cast<double>());
}

Sophus::SE3f MathUtils::se3d_to_se3f(const Sophus::SE3d& se3_d) {
    return Sophus::SE3f(se3_d.so3().cast<float>(), se3_d.translation().cast<float>());
}

// Template implementations
template<typename Scalar>
Sophus::SE3<Scalar> MathUtils::matrix_to_se3(const Eigen::Matrix<Scalar, 4, 4>& matrix) {
    // Extract rotation matrix and normalize using SVD
    Eigen::Matrix<Scalar, 3, 3> rotation = matrix.template block<3, 3>(0, 0);
    Eigen::Matrix<Scalar, 3, 3> normalized_rotation = normalize_rotation_matrix(rotation);
    
    // Extract translation
    Eigen::Matrix<Scalar, 3, 1> translation = matrix.template block<3, 1>(0, 3);
    
    return Sophus::SE3<Scalar>(normalized_rotation, translation);
}

template<typename Scalar>
Eigen::Matrix<Scalar, 4, 4> MathUtils::se3_to_matrix(const Sophus::SE3<Scalar>& se3) {
    return se3.matrix();
}

template<typename T>
bool MathUtils::is_approx_equal(T a, T b, T epsilon) {
    return std::abs(a - b) < epsilon;
}

template<typename T>
T MathUtils::clamp(T value, T min_val, T max_val) {
    return std::max(min_val, std::min(value, max_val));
}

// Explicit template instantiations
template Sophus::SE3<float> MathUtils::matrix_to_se3<float>(const Eigen::Matrix<float, 4, 4>&);
template Sophus::SE3<double> MathUtils::matrix_to_se3<double>(const Eigen::Matrix<double, 4, 4>&);
template Eigen::Matrix<float, 4, 4> MathUtils::se3_to_matrix<float>(const Sophus::SE3<float>&);
template Eigen::Matrix<double, 4, 4> MathUtils::se3_to_matrix<double>(const Sophus::SE3<double>&);

template bool MathUtils::is_approx_equal<float>(float, float, float);
template bool MathUtils::is_approx_equal<double>(double, double, double);
template float MathUtils::clamp<float>(float, float, float);
template double MathUtils::clamp<double>(double, double, double);
template int MathUtils::clamp<int>(int, int, int);

} // namespace util
} // namespace lidar_odometry
