/**
 * @file      IterativeClosestPoint.cpp
 * @brief     Implementation of modern ICP using Ceres optimization.
 * @author    Seungwon Choi
 * @date      2025-09-24
 * @copyright Copyright (c) 2025 Seungwon Choi. All rights reserved.
 *
 * @par License
 * This project is released under the MIT License.
 */

#include "IterativeClosestPoint.h"
#include "../optimization/Factors.h"
#include "../optimization/Parameters.h"
#include "../util/MathUtils.h"
#include <spdlog/spdlog.h>
#include <pcl/common/transforms.h>
#include <algorithm>
#include <numeric>

namespace lidar_odometry {
namespace processing {

IterativeClosestPoint::IterativeClosestPoint(const ICPConfig& config)
    : m_config(config), m_target_kdtree(nullptr), m_adaptive_estimator(nullptr) {
    spdlog::info("IterativeClosestPoint initialized with max_iterations={}", m_config.max_iterations);
}

IterativeClosestPoint::IterativeClosestPoint(const ICPConfig& config, 
                                           std::shared_ptr<optimization::AdaptiveMEstimator> adaptive_estimator)
    : m_config(config), m_target_kdtree(nullptr), m_adaptive_estimator(adaptive_estimator) {
    spdlog::info("IterativeClosestPoint initialized with max_iterations={} and AdaptiveMEstimator", 
                 m_config.max_iterations);
}

void IterativeClosestPoint::update_config(const ICPConfig& config) {
    m_config = config;
    spdlog::debug("ICP configuration updated");
}

bool IterativeClosestPoint::align(ICPPointCloudConstPtr source_cloud,
                                 ICPPointCloudConstPtr target_cloud,
                                 const ICPPose& initial_guess,
                                 ICPPose& result_pose) {
    if (!source_cloud || !target_cloud) {
        spdlog::error("Input clouds are null");
        return false;
    }
    
    if (source_cloud->empty() || target_cloud->empty()) {
        spdlog::error("Input clouds are empty");
        return false;
    }
    
    // Initialize statistics
    m_statistics.reset();
    
    // Build KD-tree for target cloud
    m_target_kdtree = std::make_unique<pcl::KdTreeFLANN<ICPPointType>>();
    m_target_kdtree->setInputCloud(target_cloud);
    
    // Initialize pose
    ICPPose current_pose = initial_guess;
    result_pose = initial_guess;
    
    double prev_cost = std::numeric_limits<double>::max();
    bool converged = false;
    ICPCorrespondenceVector correspondences;  // Move outside the loop
    
    for (int iteration = 0; iteration < m_config.max_iterations && !converged; ++iteration) {
        // Find correspondences
        correspondences.clear();  // Clear previous correspondences
        size_t num_correspondences = find_correspondences(source_cloud, target_cloud, current_pose, correspondences);
        
        if (num_correspondences < static_cast<size_t>(m_config.min_correspondence_points)) {
            spdlog::warn("Insufficient correspondences: {} < {}", 
                        num_correspondences, m_config.min_correspondence_points);
            break;
        }


        // Reject outliers
        size_t num_inliers = reject_outliers(correspondences);
        m_statistics.inlier_count = num_inliers;
        m_statistics.match_ratio = static_cast<double>(num_inliers) / source_cloud->size();
        
        if (num_inliers < static_cast<size_t>(m_config.min_correspondence_points)) {
            spdlog::warn("Insufficient inliers after outlier rejection: {} < {}", 
                        num_inliers, m_config.min_correspondence_points);
            break;
        }
        
        // Optimize pose
        ICPPose optimized_pose;
        if (!optimize_pose(correspondences, current_pose, optimized_pose)) {
            spdlog::error("Pose optimization failed at iteration {}", iteration);
            break;
        }
        
        // Calculate cost and check convergence
        double current_cost = calculate_cost(correspondences, optimized_pose);
        double cost_change = std::abs(prev_cost - current_cost);
        
        if (iteration == 0) {
            m_statistics.initial_cost = current_cost;
        }
        
        // Use cost_change for convergence checking if needed
        converged = check_convergence(current_pose, optimized_pose) || (iteration > 0 && cost_change < 1e-6);

        if (converged) {
            break;
        }        
        // Update pose
        current_pose = optimized_pose;
        prev_cost = current_cost;
        
        spdlog::debug("ICP iteration {}: cost={:.6f}, inliers={}, match_ratio={:.3f}", 
                     iteration, current_cost, num_inliers, m_statistics.match_ratio);
        
        m_statistics.iterations_used = iteration + 1;
    }
    
    // Finalize statistics
    m_statistics.final_cost = (prev_cost == std::numeric_limits<double>::max()) ? 0.0 : prev_cost;
    m_statistics.converged = converged;
    result_pose = current_pose;
    
    // Store final correspondence distances for robust estimation
    m_last_correspondence_distances.clear();
    if (!correspondences.empty()) {
        m_last_correspondence_distances.reserve(correspondences.size());
        for (const auto& corr : correspondences) {
            if (corr.is_valid) {
                m_last_correspondence_distances.push_back(corr.distance);
            }
        }
    }
    
    return m_statistics.converged;
}

size_t IterativeClosestPoint::find_correspondences(ICPPointCloudConstPtr source_cloud,
                                                  ICPPointCloudConstPtr target_cloud,
                                                  const ICPPose& current_pose,
                                                  ICPCorrespondenceVector& correspondences) {
    correspondences.clear();
    correspondences.reserve(source_cloud->size());


    
    // Build KD-tree for target cloud
    pcl::KdTreeFLANN<ICPPointType>::Ptr kdtree(new pcl::KdTreeFLANN<ICPPointType>());
    kdtree->setInputCloud(target_cloud);
    
    // Transform source cloud with current pose
    ICPPointCloud transformed_source;
    pcl::transformPointCloud(*source_cloud, transformed_source, current_pose.matrix());
    
    const int K = 5; // Number of nearest neighbors for plane fitting
    size_t valid_correspondences = 0;
    
    for (size_t i = 0; i < transformed_source.size(); ++i) {
        const auto& p_query = transformed_source.points[i];
        
        std::vector<int> point_indices(K);
        std::vector<float> point_distances(K);
        
        // Find K nearest neighbors in target cloud
        int found = kdtree->nearestKSearch(p_query, K, point_indices, point_distances);
        if (found < 5) {
            continue;
        }
        
        // Select up to 5 non-collinear points for plane fitting
        std::vector<ICPVector3f> selected_points;
        std::vector<int> selected_indices;
        selected_points.reserve(5);
        selected_indices.reserve(5);
        
        bool non_colinear_found = false;
        for (int j = 0; j < found && selected_points.size() < 5; ++j) {
            const auto& pt = target_cloud->points[point_indices[j]];
            ICPVector3f point(pt.x, pt.y, pt.z);
            
            if (selected_points.size() < 2) {
                selected_points.push_back(point);
                selected_indices.push_back(point_indices[j]);
            } else if (!non_colinear_found) {
                // Check collinearity for the third point
                if (isCollinear(selected_points[0], selected_points[1], point, 0.5)) {
                    continue;
                } else {
                    non_colinear_found = true;
                    selected_points.push_back(point);
                    selected_indices.push_back(point_indices[j]);
                }
            } else {
                selected_points.push_back(point);
                selected_indices.push_back(point_indices[j]);
            }
        }
        
        if (selected_points.size() < 5) {
            continue;
        }
        
        // Fit plane using selected points: Ax + By + Cz + D = 0
        // We solve for normal vector [A, B, C] with constraint D = -1
        Eigen::MatrixXf matA(5, 3);
        Eigen::VectorXf matB = -Eigen::VectorXf::Ones(5);
        
        for (int j = 0; j < 5; ++j) {
            matA.row(j) = selected_points[j].transpose();
        }
        
        ICPVector3f plane_normal = matA.colPivHouseholderQr().solve(matB);
        plane_normal.normalize();
        
        // Use first point as plane reference
        ICPVector3f plane_point = selected_points[0];
        
        // Validate plane by checking distance of all points to plane
        bool plane_valid = true;
        for (const auto& pt : selected_points) {
            double dist_to_plane = std::abs(plane_normal.dot(pt - plane_point));
            if (dist_to_plane > m_config.max_correspondence_distance) {
                plane_valid = false;
                break;
            }
        }
        
        if (!plane_valid) {
            continue;
        }
        
        // Calculate point-to-plane residual
        ICPVector3f p_source = source_cloud->points[i].getVector3fMap();
        ICPVector3f p_transformed(p_query.x, p_query.y, p_query.z);
        double residual = std::abs(plane_normal.dot(p_transformed - plane_point));
        
        if (residual > m_config.max_correspondence_distance * 3.0) {
            continue;
        }
        
        // Create correspondence with plane information
        ICPPointCorrespondence corr;
        corr.source_point = p_source;
        corr.target_point = plane_point;
        corr.plane_normal = plane_normal;  // Store plane normal
        corr.distance = residual;
        corr.weight = 1.0;
        corr.is_valid = true;
        
        // Store plane normal in correspondence (we'll need this for PointToPlaneFactor)
        // For now, we'll use a simple correspondence structure
        correspondences.push_back(corr);
        valid_correspondences++;
    }
    
    m_statistics.correspondences_count = valid_correspondences;
    return valid_correspondences;
}

// Helper function to check collinearity (needs to be added)
bool IterativeClosestPoint::isCollinear(const ICPVector3f& p1, const ICPVector3f& p2, const ICPVector3f& p3, float threshold) {
    ICPVector3f v1 = p2 - p1;
    ICPVector3f v2 = p3 - p1;
    
    // Check if cross product is small (vectors are nearly parallel)
    ICPVector3f cross = v1.cross(v2);
    float cross_magnitude = cross.norm();
    
    // Normalize by the magnitudes to get relative measure
    float v1_mag = v1.norm();
    float v2_mag = v2.norm();
    
    if (v1_mag < 1e-6f || v2_mag < 1e-6f) {
        return true; // Points are too close
    }
    
    float normalized_cross = cross_magnitude / (v1_mag * v2_mag);
    return normalized_cross < threshold;
}

size_t IterativeClosestPoint::reject_outliers(ICPCorrespondenceVector& correspondences) {
    if (correspondences.empty()) {
        return 0;
    }
    
    // Sort by distance
    std::sort(correspondences.begin(), correspondences.end(),
              [](const ICPPointCorrespondence& a, const ICPPointCorrespondence& b) {
                  return a.distance < b.distance;
              });
    
    // Keep only the best correspondences based on ratio
    size_t num_keep = static_cast<size_t>(correspondences.size() * m_config.outlier_rejection_ratio);
    correspondences.resize(num_keep);
    
    // Mark all remaining correspondences as valid
    for (auto& corr : correspondences) {
        corr.is_valid = true;
        corr.weight = 1.0; // Equal weight for now
    }
    
    return num_keep;
}

bool IterativeClosestPoint::optimize_pose(const ICPCorrespondenceVector& correspondences,
                                         const ICPPose& initial_pose,
                                         ICPPose& optimized_pose) {
    // Convert SE3 pose to 6D tangent space representation using SE3GlobalParameterization
    // Normalize rotation matrix to ensure orthogonality before creating SE3
    Eigen::Matrix3d rotation_d = initial_pose.rotationMatrix().cast<double>();
    Eigen::Matrix3d normalized_rotation = util::MathUtils::normalize_rotation_matrix(rotation_d);
    
    Sophus::SE3d initial_se3(normalized_rotation, 
                             initial_pose.translation().cast<double>());
    
    // Use SE3GlobalParameterization helper to convert to tangent space
    Eigen::Vector6d pose_tangent = optimization::SE3GlobalParameterization::se3_to_tangent(initial_se3);
    
    // Pose parameters in tangent space [tx, ty, tz, rx, ry, rz]
    double pose_params[6] = {
        pose_tangent[0], pose_tangent[1], pose_tangent[2],
        pose_tangent[3], pose_tangent[4], pose_tangent[5]
    };
    
    // Calculate adaptive Huber loss delta using AdaptiveMEstimator
    double huber_delta = m_config.robust_loss_delta;  // Default delta
    
    if (m_adaptive_estimator && m_adaptive_estimator->get_config().use_adaptive_m_estimator) {
        // Extract residuals (distances) from correspondences
        std::vector<double> residuals;
        residuals.reserve(correspondences.size());
        for (const auto& corr : correspondences) {
            if (corr.is_valid) {
                residuals.push_back(corr.distance);
            }
        }
        
        if (!residuals.empty()) {
            // Calculate scale factor using AdaptiveMEstimator
            double scale_factor = m_adaptive_estimator->calculate_scale_factor(residuals);
            
            // Use scale factor as Huber loss delta
            huber_delta = scale_factor;
        }
    }
    
    // Build Ceres problem
    ceres::Problem problem;
    
    for (const auto& corr : correspondences) {
        if (!corr.is_valid) continue;
        
        // Create PointToPlaneFactor
        auto factor = new optimization::PointToPlaneFactor(
            corr.source_point,
            corr.target_point,
            corr.plane_normal,
            1.0  // Always use unit weight, robustness comes from adaptive Huber delta
        );
        
        if (m_config.use_robust_loss) {
            // Create appropriate loss function based on AdaptiveMEstimator loss type
            ceres::LossFunction* loss_function = nullptr;
            
            if (m_adaptive_estimator) {
                const std::string& loss_type = m_adaptive_estimator->get_config().loss_type;
                
                if (loss_type == "cauchy") {
                    loss_function = new ceres::CauchyLoss(huber_delta);
                } else if (loss_type == "huber") {
                    loss_function = new ceres::HuberLoss(huber_delta);
                } else {
                    // Default to Cauchy loss
                    loss_function = new ceres::CauchyLoss(huber_delta);
                }
                
                spdlog::debug("[ICP] Using {} loss function with delta={:.6f}", loss_type, huber_delta);
            } else {
                // Default to Huber loss if no adaptive estimator
                loss_function = new ceres::HuberLoss(huber_delta);
            }
            
            problem.AddResidualBlock(factor, loss_function, pose_params);
        } else {
            problem.AddResidualBlock(factor, nullptr, pose_params);
        }
    }
    
    // Add SE3 parameterization to handle manifold constraints
    problem.SetParameterization(pose_params, new optimization::SE3GlobalParameterization());
    
    // Solver options
    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 10; // Limited iterations per ICP iteration
    options.function_tolerance = 1e-6; // Use fixed tolerance for Ceres
    
    // Solve
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    
    // Extract optimized pose from tangent space using SE3GlobalParameterization
    Eigen::Map<const Eigen::Vector6d> optimized_tangent(pose_params);
    Sophus::SE3d optimized_se3 = optimization::SE3GlobalParameterization::tangent_to_se3(optimized_tangent);
    
    // Convert back to ICPPose (float)
    optimized_pose = ICPPose(optimized_se3.rotationMatrix().cast<float>(), 
                           optimized_se3.translation().cast<float>());
    
    return true;
}

bool IterativeClosestPoint::check_convergence(const ICPPose& prev_pose,
                                             const ICPPose& current_pose) {
    // Translation change
    double translation_change = (current_pose.translation() - prev_pose.translation()).norm();
    
    // Rotation change (angle between quaternions)
    double rotation_change = (prev_pose.inverse() * current_pose).log().norm();

    
    bool translation_converged = translation_change < m_config.translation_tolerance;
    bool rotation_converged = rotation_change < m_config.rotation_tolerance;
    
    return translation_converged && rotation_converged;
}

double IterativeClosestPoint::calculate_cost(const ICPCorrespondenceVector& correspondences,
                                            const ICPPose& pose) {
    double total_cost = 0.0;
    size_t valid_count = 0;
    
    for (const auto& corr : correspondences) {
        if (!corr.is_valid) continue;
        
        ICPVector3f transformed_source = pose * corr.source_point;
        ICPVector3f diff = transformed_source - corr.target_point;
        total_cost += diff.squaredNorm() * corr.weight;
        valid_count++;
    }
    
    return valid_count > 0 ? total_cost / valid_count : 0.0;
}

} // namespace processing
} // namespace lidar_odometry
