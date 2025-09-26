/**
 * @file      Types.h
 * @brief     Common type definitions for LiDAR odometry system.
 * @author    Your Name
 * @date      2025-01-09
 * @copyright Copyright (c) 2025. All rights reserved.
 *
 * @par License
 * This project is released under the MIT License.
 */

#pragma once

#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <vector>

namespace lidar_odometry {
namespace util {

// ===== Eigen Type Definitions =====

// Float precision (for storage and general computation)
using Vector3f = Eigen::Vector3f;
using Vector4f = Eigen::Vector4f;
using Matrix3f = Eigen::Matrix3f;
using Matrix4f = Eigen::Matrix4f;
using VectorXf = Eigen::VectorXf;
using MatrixXf = Eigen::MatrixXf;

// Double precision (for optimization)
using Vector3d = Eigen::Vector3d;
using Vector4d = Eigen::Vector4d;
using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;
using VectorXd = Eigen::VectorXd;
using MatrixXd = Eigen::MatrixXd;

// ===== Sophus Type Definitions =====

// SE3 transformations (float for storage, double for optimization)
using SE3f = Sophus::SE3f;
using SE3d = Sophus::SE3d;
using SO3f = Sophus::SO3f;
using SO3d = Sophus::SO3d;

// ===== PCL Type Definitions =====

// Point types
using PointType = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointType>;
using PointCloudPtr = PointCloud::Ptr;
using PointCloudConstPtr = PointCloud::ConstPtr;

// ===== Common Constants =====

constexpr float kEpsilonF = 1e-6f;      ///< Small epsilon for float comparisons
constexpr double kEpsilonD = 1e-9;      ///< Small epsilon for double comparisons
constexpr float kInfF = 1e9f;           ///< Large float value
constexpr double kInfD = 1e12;          ///< Large double value

} // namespace util
} // namespace lidar_odometry
