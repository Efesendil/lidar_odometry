/**
 * @file      Estimator.cpp
 * @brief     Implementation of LiDAR odometry estimator.
 * @author    Seungwon Choi
 * @date      2025-09-24
 * @copyright Copyright (c) 2025 Seungwon Choi. All rights reserved.
 *
 * @par License
 * This project is released under the MIT License.
 */

#include "Estimator.h"
#include "DualFrameICPOptimizer.h"
#include "../util/MathUtils.h"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>

namespace lidar_odometry {
namespace processing {

Estimator::Estimator(const util::SystemConfig& config)
    : m_config(config)
    , m_initialized(false)
    , m_T_wl_current()
    , m_velocity()
    , m_feature_map(new PointCloud())
    , m_debug_pre_icp_cloud(new PointCloud())
    , m_debug_post_icp_cloud(new PointCloud())
    , m_last_keyframe_pose()
    , m_total_optimization_iterations(0)
    , m_total_optimization_time_ms(0.0)
    , m_optimization_call_count(0)
{

    
    // Create AdaptiveMEstimator with PKO configuration only
    m_adaptive_estimator = std::make_shared<optimization::AdaptiveMEstimator>(
        config.use_adaptive_m_estimator,
        config.loss_type,
        config.min_scale_factor,
        config.max_scale_factor,
        config.num_alpha_segments,
        config.truncated_threshold,
        config.gmm_components,
        config.gmm_sample_size,
        config.pko_kernel_type
    );
    
        // Initialize DualFrameICPOptimizer with AdaptiveMEstimator and configuration
    ICPConfig dual_frame_config;
    dual_frame_config.max_iterations = config.max_iterations;
    dual_frame_config.translation_tolerance = config.translation_threshold;
    dual_frame_config.rotation_tolerance = config.rotation_threshold;
    dual_frame_config.max_correspondence_distance = config.max_correspondence_distance;
    dual_frame_config.outlier_rejection_ratio = 0.9;
    dual_frame_config.use_robust_loss = true;
    dual_frame_config.robust_loss_delta = 0.1;
    
    m_dual_optimizer = std::make_shared<DualFrameICPOptimizer>(dual_frame_config, m_adaptive_estimator);
    
    // Initialize voxel filter for downsampling
    m_voxel_filter = std::make_unique<util::VoxelGrid>();
    m_voxel_filter->setLeafSize(config.voxel_size);
    
    // Initialize feature extractor
    FeatureExtractorConfig feature_config;
    feature_config.voxel_size = config.feature_voxel_size;
    feature_config.max_neighbor_distance = config.max_neighbor_distance;
    m_feature_extractor = std::make_unique<FeatureExtractor>(feature_config);
}

Estimator::~Estimator() = default;

bool Estimator::process_frame(std::shared_ptr<database::LidarFrame> current_frame) {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (!current_frame || !current_frame->get_raw_cloud()) {
        spdlog::warn("[Estimator] Invalid frame or point cloud");
        return false;
    }
    
    // Step 1: Preprocess frame (downsample + feature extraction)
    if (!preprocess_frame(current_frame)) {
        spdlog::error("[Estimator] Frame preprocessing failed");
        return false;
    }
    
    if (!m_initialized) {
        initialize_first_frame(current_frame);
        return true;
    }


    // Get feature cloud from frame
    auto feature_cloud = current_frame->get_feature_cloud();
    

    // Step 3: Use last keyframe for optimization
    if (!m_last_keyframe) {
        spdlog::warn("[Estimator] No keyframe available, using velocity model only");
        return true;
    }
    
    // Step 4: DualFrameICPOptimizer between current frame and last keyframe
    auto opt_start = std::chrono::high_resolution_clock::now();
    // Calculate initial guess from velocity model: transform from keyframe to current velocity estimate
    SE3f T_keyframe_current_guess = m_previous_frame->get_pose() * m_velocity;
    SE3f T_keyframe_current = estimate_motion_dual_frame(current_frame, m_last_keyframe, T_keyframe_current_guess); 
    auto opt_end = std::chrono::high_resolution_clock::now();
    auto opt_time = std::chrono::duration_cast<std::chrono::milliseconds>(opt_end - opt_start);
    
    // Convert result to world coordinate (keyframe pose is already in world coordinates)
    SE3f optimized_pose = T_keyframe_current;
    
    // Store post-optimization cloud in world coordinates for visualization
    PointCloudPtr post_opt_cloud_world(new PointCloud());
    Eigen::Matrix4f T_wl_final = optimized_pose.matrix();
    util::transform_point_cloud(feature_cloud, post_opt_cloud_world, T_wl_final);

    current_frame->set_feature_cloud_global(post_opt_cloud_world); // Cache world coordinate features
    
    // Update debug clouds and current pose
    // m_debug_pre_icp_cloud = pre_icp_cloud_world;
    // m_debug_post_icp_cloud = post_opt_cloud_world;
    m_T_wl_current = optimized_pose;
    
    // Step 5: Update velocity model
    m_velocity = m_previous_frame->get_pose().inverse() * m_T_wl_current;


    // Update frame pose and trajectory
    current_frame->set_pose(m_T_wl_current);
    m_trajectory.push_back(m_T_wl_current);
    
    // Step 6: Check for keyframe creation
    if (should_create_keyframe(m_T_wl_current)) {
        // Transform feature cloud to world coordinates for keyframe storage
        create_keyframe(current_frame);
    }
    
    // Update for next iteration
    m_previous_frame = current_frame;
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto total_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    spdlog::debug("[Estimator] Frame processed in {}ms (Optimization: {}ms, Features: {})", 
                 total_time.count(), opt_time.count(), feature_cloud->size());
    
    return true;
}

void Estimator::initialize_first_frame(std::shared_ptr<database::LidarFrame> frame) {
    // Get initial pose from frame (could be set by other sensors)
    m_T_wl_current = frame->get_initial_pose();
    m_velocity = SE3f();      // Identity velocity
    frame->set_pose(m_T_wl_current);
    m_trajectory.push_back(m_T_wl_current);

    // Get feature cloud from preprocessed frame
    auto feature_cloud = frame->get_feature_cloud();
    if (!feature_cloud || feature_cloud->empty()) {
        spdlog::error("[Estimator] No feature cloud in first frame");
        m_initialized = true;
        return;
    }

    // Transform features to world coordinates using current pose
    PointCloudPtr feature_cloud_world(new PointCloud());
    Eigen::Matrix4f T_wl = m_T_wl_current.matrix();
    util::transform_point_cloud(feature_cloud, feature_cloud_world, T_wl);
    
    // Set global feature cloud in frame
    frame->set_feature_cloud_global(feature_cloud_world);
    
    // Set as keyframe and initialize local map
    create_keyframe(frame);
    
    m_previous_frame = frame;
    m_last_keyframe_pose = m_T_wl_current;
    
    m_initialized = true;
}

SE3f Estimator::estimate_motion_dual_frame(std::shared_ptr<database::LidarFrame> current_frame,
                                          std::shared_ptr<database::LidarFrame> keyframe,
                                          const SE3f& initial_guess) {
    if (!current_frame || !keyframe) {
        spdlog::warn("[Estimator] Invalid frames for dual frame optimization");
        return initial_guess;
    }
    
    auto current_features = current_frame->get_feature_cloud();
    auto keyframe_local_map = keyframe->get_local_map();
    
    if (!current_features || !keyframe_local_map || current_features->empty() || keyframe_local_map->empty()) {
        spdlog::warn("[Estimator] Invalid feature clouds for dual frame optimization");
        return initial_guess;
    }
    
    // Convert SE3f to Sophus::SE3f for DualFrameICPOptimizer
    Sophus::SE3f initial_transform_sophus(initial_guess.rotationMatrix(), initial_guess.translation());
    Sophus::SE3f optimized_transform_sophus;

    // Debug log for initial guess (only in debug mode)
    spdlog::debug("Check Initial Guess: Translation ({:.3f}, {:.3f}, {:.3f}), Rotation ({:.3f}, {:.3f}, {:.3f})",
                 initial_guess.translation().x(), initial_guess.translation().y(), initial_guess.translation().z(),
                 initial_guess.so3().log().x(), initial_guess.so3().log().y(), initial_guess.so3().log().z());
    
    // Perform dual frame optimization (keyframe as source, current as target)
    bool success = m_dual_optimizer->optimize(
        keyframe,                    // source frame (keyframe)
        current_frame,              // target frame (current)
        initial_transform_sophus,   // initial relative transform
        optimized_transform_sophus  // optimized relative transform (output)
    );
    
    if (!success) {
        spdlog::warn("[Estimator] Dual frame optimization failed, using initial guess");
        return initial_guess;
    }
    
    // Convert back to SE3f
    SE3f T_keyframe_current(optimized_transform_sophus.rotationMatrix(), optimized_transform_sophus.translation());
    
    // Collect optimization statistics
    m_total_optimization_iterations += 10; // TODO: Get actual iterations from DualFrameICPOptimizer
    m_total_optimization_time_ms += 1.0;   // TODO: Get actual time from DualFrameICPOptimizer
    m_optimization_call_count++;
    
    spdlog::debug("[Estimator] Dual frame optimization completed successfully");
    
    return T_keyframe_current;
}

std::shared_ptr<database::LidarFrame> Estimator::select_best_keyframe(const SE3f& current_pose) {
    if (m_keyframes.empty()) {
        return nullptr;
    }
    
    // Use the most recent (latest) keyframe for temporal consistency
    std::shared_ptr<database::LidarFrame> best_keyframe = nullptr;
    
    // Find the most recent keyframe with a valid local map
    for (auto it = m_keyframes.rbegin(); it != m_keyframes.rend(); ++it) {
        if (*it && (*it)->get_local_map() && !(*it)->get_local_map()->empty()) {
            best_keyframe = *it;
            break;  // Use the most recent valid keyframe
        }
    }
    
    if (best_keyframe) {
        Vector3f translation_diff = current_pose.translation() - best_keyframe->get_pose().translation();
        double distance = translation_diff.norm();
        spdlog::debug("[Estimator] Selected most recent keyframe at distance {:.2f}m", distance);
    } else {
        spdlog::debug("[Estimator] No suitable keyframe found");
    }
    
    return best_keyframe;
}

bool Estimator::should_create_keyframe(const SE3f& current_pose) {
   
    if (m_keyframes.empty()) {
        return true;
    }
    
    // Calculate distance and rotation from last keyframe
    Vector3f translation_diff = current_pose.translation() - m_last_keyframe_pose.translation();
    double distance = translation_diff.norm();
    
    Sophus::SO3f rotation_diff = m_last_keyframe_pose.so3().inverse() * current_pose.so3();
    double rotation_angle = rotation_diff.log().norm();

    // Debug log for keyframe check
    spdlog::debug("[Estimator] Keyframe check: Δt={:.2f}m, Δr={:.2f}° (thresholds: {:.2f}m, {:.2f}°)", 
                  distance, rotation_angle * 180.0 / M_PI,
                  m_config.keyframe_distance_threshold, m_config.keyframe_rotation_threshold);
    
    return (distance > m_config.keyframe_distance_threshold || rotation_angle > m_config.keyframe_rotation_threshold);
}

void Estimator::create_keyframe(std::shared_ptr<database::LidarFrame> frame)
{
    m_keyframes.push_back(frame);

    // Check if frame has global feature cloud
    auto global_feature_cloud = frame->get_feature_cloud_global();
    if (!global_feature_cloud) {
        spdlog::warn("[Estimator] Frame has no global feature cloud, using local feature cloud");
        // For first frame, transform local features to global (identity transform)
        auto local_feature_cloud = frame->get_feature_cloud();
        if (local_feature_cloud && !local_feature_cloud->empty()) {
            PointCloudPtr global_cloud = std::make_shared<PointCloud>();
            *global_cloud = *local_feature_cloud;  // Copy for first frame
            frame->set_feature_cloud_global(global_cloud);
            global_feature_cloud = global_cloud;
        } else {
            spdlog::error("[Estimator] Frame has no feature clouds at all!");
            return;
        }
    }

    // Add to feature map
    if (m_feature_map && global_feature_cloud) {
        // First downsample the new features before adding to map
        util::VoxelGrid new_feature_filter;
        float map_voxel_size = static_cast<float>(m_config.map_voxel_size);
        new_feature_filter.setLeafSize(map_voxel_size);
        new_feature_filter.setInputCloud(global_feature_cloud);
        
        PointCloudPtr downsampled_new_features = std::make_shared<PointCloud>();
        new_feature_filter.filter(*downsampled_new_features);
        
        // Add downsampled new features to map
        *m_feature_map += *downsampled_new_features;
        
        spdlog::debug("[Estimator] Added {} -> {} downsampled points to feature map", 
                      global_feature_cloud->size(), downsampled_new_features->size());
    } else {
        spdlog::error("[Estimator] Null pointer in feature map addition!");
        return;
    }

    // Apply voxel grid downsampling to entire map every keyframe
    PointCloudPtr processed_feature_map = m_feature_map;
    
    util::VoxelGrid map_voxel_filter;
    float map_voxel_size = static_cast<float>(m_config.map_voxel_size);
    map_voxel_filter.setLeafSize(map_voxel_size);
    map_voxel_filter.setInputCloud(m_feature_map);

    auto new_feature_map = std::make_shared<util::PointCloud>();
    map_voxel_filter.filter(*new_feature_map);
    
    spdlog::debug("[Estimator] Resampled entire map: {} -> {} points", 
                  m_feature_map->size(), new_feature_map->size());
    
    processed_feature_map = new_feature_map;

    // Apply radius-based filtering around current pose (LiDAR circular pattern)
    Eigen::Vector3f current_position = frame->get_pose().translation();
    float filter_radius = static_cast<float>(m_config.max_range * 1.2);
    
    auto filtered_feature_map = std::make_shared<util::PointCloud>();
    filtered_feature_map->reserve(processed_feature_map->size());
    
    // Filter points within radius from current position
    for (const auto& point : *processed_feature_map) {
        Eigen::Vector3f point_pos(point.x, point.y, point.z);
        float distance = (point_pos - current_position).norm();
        
        if (distance <= filter_radius) {
            filtered_feature_map->push_back(point);
        }
    }
    
    // Store current local map in keyframe for DualFrameICPOptimizer
    auto local_map_copy = std::make_shared<util::PointCloud>();
    *local_map_copy = *filtered_feature_map;  // Copy the local map at keyframe creation time
    frame->set_local_map(local_map_copy);
    
    // Build KdTree for the local map at keyframe creation
    frame->build_local_map_kdtree();
    
    // Update global feature map
    m_feature_map = processed_feature_map;
    
    // Update last keyframe reference for optimization
    m_last_keyframe = frame;

    m_last_keyframe_pose = m_last_keyframe->get_pose();
    
    spdlog::debug("[Estimator] Keyframe created: input={} -> local_map={} points, global_map={} points", 
                  global_feature_cloud->size(), local_map_copy->size(), m_feature_map->size());
}


void Estimator::update_config(const util::SystemConfig& config) {
    m_config = config;
    
    // Update voxel filter
    m_voxel_filter->setLeafSize(m_config.voxel_size);
}

const util::SystemConfig& Estimator::get_config() const {
    return m_config;
}

PointCloudConstPtr Estimator::get_local_map() const {
    return m_feature_map;
}

PointCloudConstPtr Estimator::get_last_keyframe_map() const {
    if (m_last_keyframe) {
        return m_last_keyframe->get_local_map();
    }
    return nullptr;
}

void Estimator::get_debug_clouds(PointCloudConstPtr& pre_icp_cloud, PointCloudConstPtr& post_icp_cloud) const {
    pre_icp_cloud = m_debug_pre_icp_cloud;
    post_icp_cloud = m_debug_post_icp_cloud;
}

bool Estimator::preprocess_frame(std::shared_ptr<database::LidarFrame> frame) {
    auto raw_cloud = frame->get_raw_cloud();
    if (!raw_cloud || raw_cloud->empty()) {
        spdlog::error("[Estimator] Invalid raw cloud");
        return false;
    }
    
    // Step 1: Downsample the raw cloud
    PointCloudPtr downsampled_cloud = std::make_shared<PointCloud>();
    m_voxel_filter->setInputCloud(raw_cloud);
    m_voxel_filter->filter(*downsampled_cloud);
    
    if (downsampled_cloud->empty()) {
        spdlog::error("[Estimator] Downsampled cloud is empty");
        return false;
    }
    
    // Step 2: Extract features from downsampled cloud
    PointCloudPtr feature_cloud = std::make_shared<PointCloud>();
    size_t num_features = m_feature_extractor->extract_features(downsampled_cloud, feature_cloud);
    
    if (num_features == 0) {
        spdlog::warn("[Estimator] No features extracted, using downsampled cloud as features");
        feature_cloud = downsampled_cloud;
    }
    
    // Step 3: Set processed clouds in the frame
    frame->set_processed_cloud(downsampled_cloud);
    frame->set_feature_cloud(feature_cloud);
    
    spdlog::debug("[Estimator] Preprocessing: {} -> {} points, {} features", raw_cloud->size(), downsampled_cloud->size(), feature_cloud->size());
    
    return true;
}

void Estimator::get_optimization_statistics(double& avg_iterations, double& avg_time_ms) const {
    if (m_optimization_call_count > 0) {
        avg_iterations = static_cast<double>(m_total_optimization_iterations) / m_optimization_call_count;
        avg_time_ms = m_total_optimization_time_ms / m_optimization_call_count;
    } else {
        avg_iterations = 0.0;
        avg_time_ms = 0.0;
    }
}

const SE3f& Estimator::get_current_pose() const {
    return m_T_wl_current;
}

size_t Estimator::get_keyframe_count() const {
    return m_keyframes.size();
}

std::shared_ptr<database::LidarFrame> Estimator::get_keyframe(size_t index) const {
    if (index >= m_keyframes.size()) {
        return nullptr;
    }
    return m_keyframes[index];
}

} // namespace processing
} // namespace lidar_odometry
