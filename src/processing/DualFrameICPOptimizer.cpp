/**
 * @file      DualFrameICPOptimizer.cpp
 * @brief     Two-frame ICP optimizer implementation
 * @author    Seungwon Choi
 * @date      2025-10-04
 * @copyright Copyright (c) 2025 Seungwon Choi. All rights reserved.
 *
 * @par License
 * This project is released under the MIT License.
 */

#include "DualFrameICPOptimizer.h"
#include "../util/MathUtils.h"
#include "../util/PointCloudUtils.h"
#include <spdlog/spdlog.h>
#include <chrono>
#include <numeric>
#include <algorithm>

namespace lidar_odometry {
namespace processing {

DualFrameICPOptimizer::DualFrameICPOptimizer(const ICPConfig& config)
    : m_config(config), m_adaptive_estimator(nullptr) {
    
    // Initialize voxel filter
    m_voxel_filter = std::make_unique<util::VoxelGrid>();
    m_voxel_filter->setLeafSize(0.4f);  // Default voxel size
    
    spdlog::info("[DualFrameICPOptimizer] Initialized with max_iterations={}, max_correspondence_distance={}", 
                 m_config.max_iterations, m_config.max_correspondence_distance);
}

DualFrameICPOptimizer::DualFrameICPOptimizer(const ICPConfig& config, 
                                            std::shared_ptr<optimization::AdaptiveMEstimator> adaptive_estimator)
    : m_config(config), m_adaptive_estimator(adaptive_estimator) {
    
    // Initialize voxel filter
    m_voxel_filter = std::make_unique<util::VoxelGrid>();
    m_voxel_filter->setLeafSize(0.4f);  // Default voxel size
    
    spdlog::info("[DualFrameICPOptimizer] Initialized with max_iterations={}, max_correspondence_distance={} and AdaptiveMEstimator", 
                 m_config.max_iterations, m_config.max_correspondence_distance);
}

bool DualFrameICPOptimizer::optimize(std::shared_ptr<database::LidarFrame> last_keyframe,
                                     std::shared_ptr<database::LidarFrame> curr_frame,
                                     const Sophus::SE3f& initial_transform,
                                     Sophus::SE3f& optimized_transform) {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Reset stats
    m_last_stats = OptimizationStats();
    
    // Initialize current transform estimate
    Sophus::SE3f current_transform = initial_transform;
    optimized_transform = current_transform;
    
    double total_initial_cost = 0.0;
    double total_final_cost = 0.0;
    int total_iterations = 0;
    double residual_normalization_scale = 1.0;  // Initialize normalization scale
    
    // ICP iteration loop
    for (int icp_iter = 0; icp_iter < m_config.max_iterations; ++icp_iter) {
        
        // Set current poses for correspondence finding
        // Keep last keyframe at its actual pose, update curr frame with current estimate
        // last_keyframe pose should remain unchanged (it's the reference keyframe)
        curr_frame->set_pose(current_transform); // Curr frame at current estimate
        
        // Find correspondences with current transform estimate
        DualFrameCorrespondences correspondences;
        size_t num_correspondences = find_correspondences(last_keyframe, curr_frame, correspondences);
        
        if (num_correspondences < m_config.min_correspondence_points) {
            spdlog::warn("[ICP] Insufficient correspondences: {} < {} at iteration {}", 
                        num_correspondences, m_config.min_correspondence_points, icp_iter + 1);
            return false;
        }
        
        // Analyze residual distribution on first iteration
        if (icp_iter == 0 && !correspondences.residuals.empty()) {
            std::vector<double> residuals = correspondences.residuals;
            std::sort(residuals.begin(), residuals.end());
            
            double mean = std::accumulate(residuals.begin(), residuals.end(), 0.0) / residuals.size();
            double variance = 0.0;
            for (double val : residuals) {
                variance += (val - mean) * (val - mean);
            }
            variance /= residuals.size();
            double std_dev = std::sqrt(variance);
            
            size_t n = residuals.size();
            double median = (n % 2 == 0) ? (residuals[n/2-1] + residuals[n/2]) / 2.0 : residuals[n/2];
            double q25 = residuals[n/4];
            double q75 = residuals[3*n/4];
            double min_val = residuals[0];
            double max_val = residuals[n-1];
            
            // Debug residual distribution (only in debug mode)
            spdlog::debug("[DualFrameICPOptimizer] First iteration residual distribution:");
            spdlog::debug("  Count: {}, Mean: {:.4f}, Std: {:.4f}, Median: {:.4f}", 
                        n, mean, std_dev, median);
            spdlog::debug("  Min: {:.4f}, Q25: {:.4f}, Q75: {:.4f}, Max: {:.4f}", 
                        min_val, q25, q75, max_val);
            
            // Calculate residual normalization scale (same as ICP)
            residual_normalization_scale = std_dev / 6.0;
            spdlog::debug("  Normalization scale (std/6): {:.6f}", residual_normalization_scale);
        }
        
        // Setup optimization problem for this iteration
        ceres::Problem problem;
        
        // SE3 parameters: [tx, ty, tz, rx, ry, rz]
        Eigen::Vector6d se3_tangent = optimization::SE3GlobalParameterization::se3_to_tangent(
            current_transform.cast<double>()
        );
        std::array<double, 6> pose_params;
        std::copy(se3_tangent.data(), se3_tangent.data() + 6, pose_params.data());
        
        // Add parameter blocks for two poses
        // Use actual last keyframe pose instead of identity
        Eigen::Vector6d last_se3_tangent = optimization::SE3GlobalParameterization::se3_to_tangent(
            last_keyframe->get_pose().cast<double>()
        );
        std::array<double, 6> source_pose_params;
        std::copy(last_se3_tangent.data(), last_se3_tangent.data() + 6, source_pose_params.data());
        problem.AddParameterBlock(source_pose_params.data(), 6);
        problem.AddParameterBlock(pose_params.data(), 6);
        
        // Set parameterizations
        problem.SetParameterization(source_pose_params.data(), new optimization::SE3GlobalParameterization());
        problem.SetParameterization(pose_params.data(), new optimization::SE3GlobalParameterization());
        
        // Fix source pose to identity
        problem.SetParameterBlockConstant(source_pose_params.data());
        
        // Calculate adaptive Huber loss delta using AdaptiveMEstimator if available
        double huber_delta = m_config.robust_loss_delta;  // Default delta
        
        if (m_adaptive_estimator && m_adaptive_estimator->get_config().use_adaptive_m_estimator) {
            // Reset adaptive estimator
            m_adaptive_estimator->reset();
            
            // Extract residuals for PKO
            std::vector<double> residuals;
            residuals.reserve(correspondences.residuals.size());
            for (double residual : correspondences.residuals) {
                // Normalize residuals using the scale calculated in first iteration
                double normalized_residual = residual / std::max(residual_normalization_scale, 1e-6);
                residuals.push_back(normalized_residual);
            }
            
            if (!residuals.empty()) {
                // Calculate scale factor using AdaptiveMEstimator on normalized residuals
                double scale_factor = m_adaptive_estimator->calculate_scale_factor(residuals);
                
                // Use scale factor as Huber loss delta
                huber_delta = scale_factor;
                
                spdlog::debug("[DualFrameICPOptimizer] Iter {}: PKO Alpha={:.6f}, norm_scale={:.6f}, delta={:.6f}", 
                             icp_iter + 1, scale_factor, residual_normalization_scale, huber_delta);
            }
        }
        
        // Add residual blocks (point-to-plane factors)
        for (size_t i = 0; i < correspondences.size(); ++i) {
            // Calculate normalization weight for residual
            double normalization_weight = 1.0 / std::max(residual_normalization_scale, 1e-6);
            
            auto factor = new optimization::PointToPlaneFactorDualFrame(
                correspondences.points_last[i].cast<double>(),    // p: last frame point (local)
                correspondences.points_curr[i].cast<double>(),    // q: curr frame point (local)
                correspondences.normals_last[i].cast<double>(),   // nq: normal from last frame (world)
                normalization_weight  // Apply normalization through weight
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
                    
                    // Debug loss function info only for first residual block to avoid spam
                    if (i == 0) {
                        spdlog::debug("[DualFrameICPOptimizer] Iter {}: Using {} loss with delta={:.6f}, norm_weight={:.3f}", 
                                     icp_iter + 1, loss_type, huber_delta, normalization_weight);
                    }
                } else {
                    // Default to Huber loss if no adaptive estimator
                    loss_function = new ceres::HuberLoss(huber_delta);
                }
                
                problem.AddResidualBlock(factor, loss_function, source_pose_params.data(), pose_params.data());
            } else {
                problem.AddResidualBlock(factor, nullptr, source_pose_params.data(), pose_params.data());
            }
        }
        
        // Setup solver options for this iteration
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;  // Reduce output
        options.max_num_iterations = 10;  // Few iterations per ICP step
        options.function_tolerance = 1e-6;
        options.parameter_tolerance = 1e-8;
        
        // Solve for this iteration
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        
        // Extract updated transform
        Eigen::Vector6d updated_tangent = Eigen::Map<const Eigen::Vector6d>(pose_params.data());
        Sophus::SE3d updated_se3_d = optimization::SE3GlobalParameterization::tangent_to_se3(updated_tangent);
        Sophus::SE3f updated_transform = updated_se3_d.cast<float>();
        
        // Calculate transformation delta
        Sophus::SE3f delta_transform = current_transform.inverse() * updated_transform;
        
        // Extract translation and rotation deltas
        Eigen::Vector3f delta_translation = delta_transform.translation();
        float translation_delta = delta_translation.norm();
        float rotation_delta = delta_transform.so3().log().norm();
        
        spdlog::debug("[ICP] Iter {}: {} corr, cost {:.1f}->{:.1f}, Δt={:.3f}m, Δr={:.3f}°", 
                    icp_iter + 1, num_correspondences,
                    summary.initial_cost, summary.final_cost,
                    translation_delta, rotation_delta * 180.0f / M_PI);
        
        // Update current transform
        current_transform = updated_transform;
        
        // Accumulate stats
        if (icp_iter == 0) {
            total_initial_cost = summary.initial_cost;
        }
        total_final_cost = summary.final_cost;
        total_iterations += summary.iterations.size();
        m_last_stats.num_correspondences = num_correspondences;
        
        // Check convergence
        bool converged = (translation_delta < m_config.translation_tolerance) && 
                        (rotation_delta < m_config.rotation_tolerance);
        
        if (converged) {
            break;
        }
    }
    
    // Set final results
    optimized_transform = current_transform;
    m_last_stats.num_iterations = total_iterations;
    m_last_stats.initial_cost = total_initial_cost;
    m_last_stats.final_cost = total_final_cost;
    m_last_stats.converged = true;  // If we got here, it worked
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    m_last_stats.optimization_time_ms = duration.count();
    
    return true;
}

size_t DualFrameICPOptimizer::find_correspondences(std::shared_ptr<database::LidarFrame> last_keyframe,
                                                  std::shared_ptr<database::LidarFrame> curr_frame,
                                                  DualFrameCorrespondences& correspondences) {
    
    correspondences.clear();
    
    // Get point clouds - use local map for keyframe, feature cloud for current frame
    // auto last_cloud = last_keyframe->get_feature_cloud_global();    // Keyframe local map (already in world coordinate
    auto last_cloud = last_keyframe->get_local_map();    // Keyframe local map (already in world coordinates)
    // if(last_cloud->empty()) {
    //     last_cloud = last_keyframe->get_feature_cloud_global();
    // }

    auto curr_cloud = get_frame_cloud(curr_frame);     // Current frame feature cloud (local coordinates)


    Eigen::Matrix4f T_wl = last_keyframe->get_pose().matrix(); // Keyframe pose in world coordinates
    Eigen::Matrix4f T_lw = T_wl.inverse(); // Inverse transform
    
    if (!last_cloud || !curr_cloud || last_cloud->empty() || curr_cloud->empty()) {
        spdlog::warn("[DualFrameICPOptimizer] Empty point clouds - last_map: {}, curr_features: {}", 
                     last_cloud ? last_cloud->size() : 0, 
                     curr_cloud ? curr_cloud->size() : 0);
        return 0;
    }
    
    // Transform current frame cloud to world coordinates using current pose estimate
    auto curr_pose = curr_frame->get_pose();   // Current estimate
    
    PointCloudPtr curr_world(new PointCloud());
    util::transform_point_cloud(curr_cloud, curr_world, curr_pose.matrix());
    
    // Use precomputed KD-tree from keyframe local map
    auto kdtree_ptr = last_keyframe->get_local_map_kdtree();
    if (!kdtree_ptr) {
        spdlog::error("[DualFrameICPOptimizer] Keyframe has no KdTree - this should not happen!");
        return 0;
    }
    util::KdTree* kdtree = kdtree_ptr.get();
    
    const int K = 5;  // Number of neighbors for plane fitting
    
    // Find correspondences: query CURR points, find neighbors in LAST cloud
    for (size_t idx = 0; idx < curr_world->size(); ++idx) {
        
        const auto& curr_point_world = curr_world->at(idx);
        
        // Find K nearest neighbors in LAST cloud
        std::vector<int> neighbor_indices(K);
        std::vector<float> neighbor_distances(K);
        
        int found_neighbors = kdtree->nearestKSearch(curr_point_world, K, neighbor_indices, neighbor_distances);
        
        if (found_neighbors < 5) {
            continue;
        }
        
        // Select points for plane fitting from LAST cloud
        std::vector<Eigen::Vector3d> selected_points_world;
        std::vector<Eigen::Vector3d> selected_points_local;
        
        bool non_collinear_found = false;
        
        for (int k = 0; k < found_neighbors && selected_points_world.size() < 5; ++k) {
            int neighbor_idx = neighbor_indices[k];
            
            // Local map points are already in world coordinates
            Eigen::Vector3d pt_world(last_cloud->at(neighbor_idx).x,
                                   last_cloud->at(neighbor_idx).y,
                                   last_cloud->at(neighbor_idx).z);
            
            // For local coordinates, we use the same world coordinates since local map is in world frame
            Eigen::Vector3d pt_local = T_lw.block<3,3>(0,0).cast<double>()*pt_world + T_lw.block<3,1>(0,3).cast<double>();
            
            if (selected_points_world.size() < 2) {
                selected_points_world.push_back(pt_world);
                selected_points_local.push_back(pt_local);
            } else if (!non_collinear_found) {
                if (is_collinear(selected_points_world[0], selected_points_world[1], pt_world, 0.5)) {
                    continue;
                } else {
                    non_collinear_found = true;
                    selected_points_world.push_back(pt_world);
                    selected_points_local.push_back(pt_local);
                }
            } else {
                selected_points_world.push_back(pt_world);
                selected_points_local.push_back(pt_local);
            }
        }
        
        if (selected_points_world.size() < 5) {
            continue;
        }
        
        // Fit plane to selected points
        size_t n_points = selected_points_world.size();
        Eigen::MatrixXd A(n_points, 3);
        Eigen::VectorXd b(n_points);
        
        for (size_t p = 0; p < n_points; ++p) {
            A(p, 0) = selected_points_world[p].x();
            A(p, 1) = selected_points_world[p].y();
            A(p, 2) = selected_points_world[p].z();
            b(p) = -1.0;
        }
        
        Eigen::Vector3d normal = A.colPivHouseholderQr().solve(b).normalized();
        Eigen::Vector3d plane_point = selected_points_world[0];
        
        // Validate plane consistency
        bool plane_valid = true;
        for (size_t p = 0; p < n_points; ++p) {
            double dist_to_plane = std::abs(normal.dot(selected_points_world[p] - plane_point));
            if (dist_to_plane > 0.4) {  // voxel_size threshold
                plane_valid = false;
                break;
            }
        }
        
        if (!plane_valid) {
            continue;
        }
        
        // Calculate residual
        Eigen::Vector3d curr_point_world_d(curr_point_world.x, curr_point_world.y, curr_point_world.z);
        double residual = std::abs(normal.dot(curr_point_world_d - plane_point));
        
        if (residual > m_config.max_correspondence_distance) {
            continue;
        }
        
        // Store correspondence
        Eigen::Vector3d last_point_local = selected_points_local[0];  // Use first selected point from LAST cloud
        Eigen::Vector3d curr_point_local(curr_cloud->at(idx).x, curr_cloud->at(idx).y, curr_cloud->at(idx).z);
        
        correspondences.points_last.push_back(last_point_local);
        correspondences.points_curr.push_back(curr_point_local);
        correspondences.normals_last.push_back(normal);  // Normal from LAST cloud (world coordinates)
        correspondences.residuals.push_back(residual);
    }
    
    return correspondences.size();
}

PointCloudConstPtr DualFrameICPOptimizer::get_frame_cloud(std::shared_ptr<database::LidarFrame> frame) {
    
    // Try to get feature cloud first, fall back to processed cloud
    auto feature_cloud = frame->get_feature_cloud();
    if (feature_cloud && !feature_cloud->empty()) {
        return feature_cloud;
    }
    
    auto processed_cloud = frame->get_processed_cloud();
    if (processed_cloud && !processed_cloud->empty()) {
        return processed_cloud;
    }
    
    // Last resort: use raw cloud with downsampling
    auto raw_cloud = frame->get_raw_cloud();
    if (raw_cloud && !raw_cloud->empty()) {
        auto downsampled = std::make_shared<PointCloud>();
        m_voxel_filter->setInputCloud(raw_cloud);
        m_voxel_filter->filter(*downsampled);
        return downsampled;
    }
    
    return nullptr;
}

bool DualFrameICPOptimizer::is_collinear(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, 
                                         const Eigen::Vector3d& p3, double threshold) {
    Eigen::Vector3d v1 = (p2 - p1).normalized();
    Eigen::Vector3d v2 = (p3 - p1).normalized();
    
    double cross_norm = v1.cross(v2).norm();
    return cross_norm < threshold;
}

} // namespace processing
} // namespace lidar_odometry