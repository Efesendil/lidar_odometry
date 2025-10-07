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
#include "IterativeClosestPoint.h"
#include "../util/MathUtils.h"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>

namespace lidar_odometry {
namespace processing {

Estimator::Estimator(const EstimatorConfig& config)
    : m_config(config)
    , m_initialized(false)
    , m_T_wl_current()
    , m_velocity()
    , m_feature_map(new PointCloud())
    , m_debug_pre_icp_cloud(new PointCloud())
    , m_debug_post_icp_cloud(new PointCloud())
    , m_last_keyframe_pose()
    , m_total_icp_iterations(0)
    , m_total_icp_time_ms(0.0)
    , m_icp_call_count(0)
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
    
    // Initialize modern ICP with Ceres and AdaptiveMEstimator using YAML configuration
    ICPConfig icp_config;
    icp_config.max_iterations = config.max_icp_iterations;
    icp_config.translation_tolerance = config.icp_translation_threshold;  // From YAML odometry.translation_threshold
    icp_config.rotation_tolerance = config.icp_rotation_threshold;        // From YAML odometry.rotation_threshold  
    icp_config.max_correspondence_distance = config.correspondence_distance;
    icp_config.outlier_rejection_ratio = 0.9;
    icp_config.use_robust_loss = true;
    icp_config.robust_loss_delta = 0.1;
    
    // Create ICP with AdaptiveMEstimator
    m_icp = std::make_shared<IterativeClosestPoint>(icp_config, m_adaptive_estimator);
    
    // Initialize voxel filter for downsampling
    m_voxel_filter = std::make_unique<util::VoxelGrid>();
    m_voxel_filter->setLeafSize(static_cast<float>(m_config.voxel_size));
    
    // Initialize feature extractor
    FeatureExtractorConfig feature_config;
    feature_config.voxel_size = 0.1f;
    feature_config.max_neighbor_distance = 1.0f;
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
    
    // Step 2: Initial motion estimate using constant velocity model
    m_T_wl_current *= m_velocity;
    
    // Store pre-ICP cloud in world coordinates for visualization
    PointCloudPtr pre_icp_cloud_world(new PointCloud());
    Eigen::Matrix4f T_wl_initial = m_T_wl_current.matrix();
    util::transform_point_cloud(feature_cloud, pre_icp_cloud_world, T_wl_initial);
    
    // Step 3: Transform previous features to current frame estimate
    PointCloudPtr feature_map_local(new PointCloud());


    if (!m_feature_map->empty()) {
        // Transform from world to current frame coordinate
        Eigen::Matrix4f T_lw = m_T_wl_current.inverse().matrix();
        util::transform_point_cloud(m_feature_map, feature_map_local, T_lw);
    }
    
    // Step 4: ICP between current features and previous features
    auto icp_start = std::chrono::high_resolution_clock::now();
    SE3f T_li_lj = estimate_motion_icp(feature_cloud, feature_map_local, SE3f()); // Transform from li frame to lj frame
    auto icp_end = std::chrono::high_resolution_clock::now();
    auto icp_time = std::chrono::duration_cast<std::chrono::milliseconds>(icp_end - icp_start);
    
    // Collect ICP statistics
    const auto& stats = m_icp->get_statistics();
    m_total_icp_iterations += stats.iterations_used;
    m_total_icp_time_ms += icp_time.count();
    m_icp_call_count++;
    
    // Convert ICP result to world coordinate
    SE3f icp_pose_estimate = m_T_wl_current * T_li_lj;
    
    // Store post-ICP cloud in world coordinates for visualization
    PointCloudPtr post_icp_cloud_world(new PointCloud());
    Eigen::Matrix4f T_wl_final = icp_pose_estimate.matrix();
    util::transform_point_cloud(feature_cloud, post_icp_cloud_world, T_wl_final);

    current_frame->set_feature_cloud_global(post_icp_cloud_world); // Cache world coordinate features
    
    // Update debug clouds and current pose
    m_debug_pre_icp_cloud = pre_icp_cloud_world;
    m_debug_post_icp_cloud = post_icp_cloud_world;
    m_T_wl_current = icp_pose_estimate;
    
    // Step 5: Update velocity model
    SE3f previous_pose = m_trajectory.empty() ? SE3f() : m_trajectory.back();
    SE3f current_velocity = previous_pose.inverse() * m_T_wl_current;
    
    // Smooth velocity update
    const float alpha = 0.7f;
    if (m_trajectory.size() > 1) {
        Eigen::Vector3f prev_trans = m_velocity.translation();
        Eigen::Vector3f curr_trans = current_velocity.translation();
        Eigen::Vector3f smooth_trans = alpha * curr_trans + (1.0f - alpha) * prev_trans;
        
        Sophus::SO3f prev_rot = m_velocity.so3();
        Sophus::SO3f curr_rot = current_velocity.so3();
        Sophus::SO3f smooth_rot = prev_rot * Sophus::SO3f::exp(alpha * (prev_rot.inverse() * curr_rot).log());
        
        m_velocity = SE3f(smooth_rot.matrix(), smooth_trans);
    } else {
        m_velocity = current_velocity;
    }
    
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
    
    spdlog::debug("[Estimator] Frame processed in {}ms (ICP: {}ms, Features: {})", 
                 total_time.count(), icp_time.count(), feature_cloud->size());
    
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

SE3f Estimator::estimate_motion_icp(PointCloudConstPtr current_features,
                                   PointCloudConstPtr previous_features,
                                   const SE3f& initial_guess) {
    if (!current_features || !previous_features || current_features->empty() || previous_features->empty()) {
        spdlog::warn("[Estimator] Invalid feature clouds for ICP");
        return initial_guess;
    }
    
    // Use our point cloud implementation for ICP
    auto current_xyz = std::make_shared<util::PointCloud>();
    auto previous_xyz = std::make_shared<util::PointCloud>();
    
    // Copy point clouds
    util::copy_point_cloud(current_features, current_xyz);
    util::copy_point_cloud(previous_features, previous_xyz);
    
    // Convert Sophus pose to our ICP pose type
    Eigen::Matrix3f initial_rotation = util::MathUtils::normalize_rotation_matrix(initial_guess.rotationMatrix());
    Sophus::SE3f initial_pose_se3f(initial_rotation, initial_guess.translation());
    
    // Perform ICP alignment
    Sophus::SE3f result_pose;
    m_icp->align(current_xyz, previous_xyz, initial_pose_se3f, result_pose);
    
    // Get ICP statistics
    const auto& stats = m_icp->get_statistics();
    spdlog::debug("[Estimator] ICP completed: iterations={}, cost={:.6f}, match_ratio={:.3f}",
                 stats.iterations_used, stats.final_cost, stats.match_ratio);
    
    // Convert back to our pose type
    Eigen::Matrix3f result_rotation = util::MathUtils::normalize_rotation_matrix(result_pose.rotationMatrix());
    return SE3f(result_rotation, result_pose.translation());
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
        *m_feature_map += *global_feature_cloud;
    } else {
        spdlog::error("[Estimator] Null pointer in feature map addition!");
        return;
    }

    // Apply voxel grid downsampling with configurable map voxel size
    util::VoxelGrid map_voxel_filter;
    float map_voxel_size = static_cast<float>(m_config.map_voxel_size);
    map_voxel_filter.setLeafSize(map_voxel_size);
    map_voxel_filter.setInputCloud(m_feature_map);

    auto new_feature_map = std::make_shared<util::PointCloud>();
    map_voxel_filter.filter(*new_feature_map);

    // Apply crop box filter around current pose
    util::CropBox crop_box;
    crop_box.setInputCloud(new_feature_map);
    
    // Get current pose center
    Eigen::Vector3f current_position = frame->get_pose().translation();
    
    // Set crop box size based on max_range from config
    float crop_radius = static_cast<float>(m_config.max_range * 1.2);
    Point3D min_point(current_position.x() - crop_radius,
                     current_position.y() - crop_radius,
                     current_position.z() - crop_radius);
    Point3D max_point(current_position.x() + crop_radius,
                     current_position.y() + crop_radius,
                     current_position.z() + crop_radius);
    
    Eigen::Vector4f min_vec(min_point.x, min_point.y, min_point.z, 1.0f);
    Eigen::Vector4f max_vec(max_point.x, max_point.y, max_point.z, 1.0f);
    
    crop_box.setMin(min_vec);
    crop_box.setMax(max_vec);
    
    auto cropped_feature_map = std::make_shared<util::PointCloud>();
    crop_box.filter(*cropped_feature_map);
    
    m_feature_map = cropped_feature_map;

    m_last_keyframe_pose = frame->get_pose();
    
    spdlog::debug("[Estimator] Keyframe created: {} -> {} -> {} points after voxel+crop", 
                  global_feature_cloud->size(), new_feature_map->size(), m_feature_map->size());
}


void Estimator::update_config(const EstimatorConfig& config) {
    m_config = config;
    
    // Update voxel filter
    m_voxel_filter->setLeafSize(static_cast<float>(m_config.voxel_size));
}

const EstimatorConfig& Estimator::get_config() const {
    return m_config;
}

PointCloudConstPtr Estimator::get_local_map() const {
    return m_feature_map;
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

void Estimator::get_icp_statistics(double& avg_iterations, double& avg_time_ms) const {
    if (m_icp_call_count > 0) {
        avg_iterations = static_cast<double>(m_total_icp_iterations) / m_icp_call_count;
        avg_time_ms = m_total_icp_time_ms / m_icp_call_count;
    } else {
        avg_iterations = 0.0;
        avg_time_ms = 0.0;
    }
}

const SE3f& Estimator::get_current_pose() const {
    return m_T_wl_current;
}

} // namespace processing
} // namespace lidar_odometry
