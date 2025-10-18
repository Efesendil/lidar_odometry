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
    , m_next_keyframe_id(0)
    , m_feature_map(new PointCloud())
    , m_debug_pre_icp_cloud(new PointCloud())
    , m_debug_post_icp_cloud(new PointCloud())
    , m_last_successful_loop_keyframe_id(-1)  // Initialize to -1 (no successful loop closure yet)
    , m_last_keyframe_pose()
    , m_total_optimization_iterations(0)
    , m_total_optimization_time_ms(0.0)
    , m_optimization_call_count(0)
{
    // Initialize pose graph optimizer (Ceres only)
    m_pose_graph_optimizer = std::make_shared<optimization::PoseGraphOptimizer>();
    
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
    
    // Initialize IterativeClosestPointOptimizer with AdaptiveMEstimator and configuration
    ICPConfig dual_frame_config;
    dual_frame_config.max_iterations = config.max_iterations;
    dual_frame_config.translation_tolerance = config.translation_threshold;
    dual_frame_config.rotation_tolerance = config.rotation_threshold;
    dual_frame_config.max_correspondence_distance = config.max_correspondence_distance;
    dual_frame_config.outlier_rejection_ratio = 0.9;
    dual_frame_config.use_robust_loss = true;
    dual_frame_config.robust_loss_delta = 0.1;
    
    m_icp_optimizer = std::make_shared<optimization::IterativeClosestPointOptimizer>(dual_frame_config, m_adaptive_estimator);
    
    // Initialize voxel filter for downsampling
    m_voxel_filter = std::make_unique<util::VoxelGrid>();
    m_voxel_filter->setLeafSize(config.voxel_size);
    
    // Initialize feature extractor
    FeatureExtractorConfig feature_config;
    feature_config.voxel_size = config.feature_voxel_size;
    feature_config.max_neighbor_distance = config.max_neighbor_distance;
    m_feature_extractor = std::make_unique<FeatureExtractor>(feature_config);
    
    // Initialize loop closure detector
    LoopClosureConfig loop_config;
    loop_config.enable_loop_detection = config.loop_enable_loop_detection;
    loop_config.similarity_threshold = config.loop_similarity_threshold;
    loop_config.min_keyframe_gap = config.loop_min_keyframe_gap;
    loop_config.max_search_distance = config.loop_max_search_distance;
    loop_config.enable_debug_output = config.loop_enable_debug_output;
    // Iris parameters are now automatically calculated
    m_loop_detector = std::make_unique<LoopClosureDetector>(loop_config);
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
    
    // Step 4: optimization::IterativeClosestPointOptimizer between current frame and last keyframe
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
    
    m_T_wl_current = optimized_pose;
    
    // Step 5: Update velocity model
    m_velocity = m_previous_frame->get_pose().inverse() * m_T_wl_current;


    // Update frame pose and trajectory
    current_frame->set_pose(m_T_wl_current);
    m_trajectory.push_back(m_T_wl_current);
    
    // Set previous keyframe reference for dynamic pose calculation
    if (m_last_keyframe) {
        current_frame->set_previous_keyframe(m_last_keyframe);
        // Calculate and store relative pose from keyframe to current frame
        SE3f relative_pose = m_last_keyframe->get_stored_pose().inverse() * m_T_wl_current;
        current_frame->set_relative_pose(relative_pose);
    }
    
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
    
    // First frame has no previous keyframe (it will become the first keyframe)
    frame->set_previous_keyframe(nullptr);
    frame->set_relative_pose(SE3f());  // Identity relative pose

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
    
    // Convert SE3f to Sophus::SE3f for optimization::IterativeClosestPointOptimizer
    Sophus::SE3f initial_transform_sophus(initial_guess.rotationMatrix(), initial_guess.translation());
    Sophus::SE3f optimized_transform_sophus;

    // Debug log for initial guess (only in debug mode)
    spdlog::debug("Check Initial Guess: Translation ({:.3f}, {:.3f}, {:.3f}), Rotation ({:.3f}, {:.3f}, {:.3f})",
                 initial_guess.translation().x(), initial_guess.translation().y(), initial_guess.translation().z(),
                 initial_guess.so3().log().x(), initial_guess.so3().log().y(), initial_guess.so3().log().z());
    
    // Perform dual frame optimization (keyframe as source, current as target)
    bool success = m_icp_optimizer->optimize(
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
    m_total_optimization_iterations += 10; // TODO: Get actual iterations from optimization::IterativeClosestPointOptimizer
    m_total_optimization_time_ms += 1.0;   // TODO: Get actual time from optimization::IterativeClosestPointOptimizer
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
    // Set keyframe ID
    frame->set_keyframe_id(m_next_keyframe_id++);
    
    // Calculate and store relative pose from previous keyframe
    if (!m_keyframes.empty()) {
        auto previous_keyframe = m_keyframes.back();
        SE3f prev_pose = previous_keyframe->get_pose();
        SE3f curr_pose = frame->get_pose();
        
        // Compute relative pose: T_prev_curr = T_prev^-1 * T_curr
        SE3f relative_pose_raw = prev_pose.inverse() * curr_pose;
        
        // Normalize rotation matrix for numerical stability
        Eigen::Matrix3f rotation_matrix = relative_pose_raw.rotationMatrix();
        Eigen::Matrix3f normalized_rotation = util::MathUtils::normalize_rotation_matrix(rotation_matrix);
        SE3f relative_pose(normalized_rotation, relative_pose_raw.translation());
        
        frame->set_relative_pose(relative_pose);
        
        spdlog::debug("[Estimator] Set relative pose for keyframe {}: t_norm={:.3f}, r_norm={:.3f}°", 
                     frame->get_keyframe_id(), 
                     relative_pose.translation().norm(),
                     relative_pose.so3().log().norm() * 180.0f / M_PI);
        
        // Odometry constraint will be added to PGO later, right before optimization
    } else {
        // First keyframe: set identity relative pose
        frame->set_relative_pose(SE3f());
        spdlog::debug("[Estimator] First keyframe: set identity relative pose");
    }
    
    // Add to keyframes list
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

    // Build local map by accumulating last keyframe's local map + current global features
    PointCloudPtr accumulated_map = std::make_shared<util::PointCloud>();
    
    // Add last keyframe's local map if it exists
    if (m_last_keyframe && m_last_keyframe->get_local_map()) {
        *accumulated_map += *m_last_keyframe->get_local_map();
        spdlog::debug("[Estimator] Added {} points from last keyframe's local map", 
                      m_last_keyframe->get_local_map()->size());
    }
    
    // Add current keyframe's global features
    if (global_feature_cloud) {
        *accumulated_map += *global_feature_cloud;
        spdlog::debug("[Estimator] Added {} points from current global features", 
                      global_feature_cloud->size());
    } else {
        spdlog::error("[Estimator] Null global feature cloud!");
        return;
    }
    
    // Downsample the accumulated map
    util::VoxelGrid map_voxel_filter;
    float map_voxel_size = static_cast<float>(m_config.map_voxel_size);
    map_voxel_filter.setLeafSize(map_voxel_size);
    map_voxel_filter.setInputCloud(accumulated_map);
    
    auto downsampled_map = std::make_shared<util::PointCloud>();
    map_voxel_filter.filter(*downsampled_map);
    
    spdlog::debug("[Estimator] Downsampled accumulated map: {} -> {} points", 
                  accumulated_map->size(), downsampled_map->size());
    
    // Apply radius-based filtering around current pose (LiDAR circular pattern)
    Eigen::Vector3f current_position = frame->get_pose().translation();
    float filter_radius = static_cast<float>(m_config.max_range * 1.2);
    
    auto filtered_local_map = std::make_shared<util::PointCloud>();
    filtered_local_map->reserve(downsampled_map->size());
    
    // Filter points within radius from current position
    for (const auto& point : *downsampled_map) {
        Eigen::Vector3f point_pos(point.x, point.y, point.z);
        float distance = (point_pos - current_position).norm();
        
        if (distance <= filter_radius) {
            filtered_local_map->push_back(point);
        }
    }
    
    // Store filtered local map in keyframe for optimization::IterativeClosestPointOptimizer
    frame->set_local_map(filtered_local_map);
    
    // Build KdTree for the local map at keyframe creation
    frame->build_local_map_kdtree();
    
    // Clean up previous last keyframe's kdtree to save memory
    if (m_last_keyframe && m_last_keyframe != frame) {
        m_last_keyframe->clear_local_map_kdtree();
        spdlog::debug("[Estimator] Cleared kdtree for previous keyframe {}", m_last_keyframe->get_keyframe_id());
    }
    
    // Update last keyframe reference for optimization
    m_last_keyframe = frame;

    m_last_keyframe_pose = m_last_keyframe->get_pose();
    
    spdlog::debug("[Estimator] Keyframe created: input={} -> local_map={} points", 
                  global_feature_cloud->size(), filtered_local_map->size());
    
    // Loop closure detection
    if (m_loop_detector && m_config.loop_enable_loop_detection) {
        // Always add keyframe to database regardless of cooldown
        m_loop_detector->add_keyframe(frame);
        
        // Check cooldown: only perform loop detection if enough keyframes have passed since last successful loop closure
        int current_keyframe_id = frame->get_keyframe_id();
        
        // For first attempt, allow when keyframe ID >= min_keyframe_gap
        // For subsequent attempts, check gap from last successful loop
        bool allow_detection;
        int keyframes_since_last_loop = 0;  // Declare in broader scope for debug message
        

        // Subsequent attempts: check cooldown from last successful loop
        keyframes_since_last_loop = current_keyframe_id - m_last_successful_loop_keyframe_id;
        allow_detection = (keyframes_since_last_loop >= m_config.loop_min_keyframe_gap);
        
        if (allow_detection) {
            // Detect loop closure candidates
            auto loop_candidates = m_loop_detector->detect_loop_closures(frame);

            
            if (!loop_candidates.empty()) {
                
                // Process loop closures and compute relative poses
                process_loop_closures(frame, loop_candidates);
            }
        } else {
            spdlog::debug("[Estimator] Loop detection skipped: only {} keyframes since last successful loop closure (need {})",
                         keyframes_since_last_loop, m_config.loop_min_keyframe_gap);
        }
    }
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

void Estimator::enable_loop_closure(bool enable) {
    if (m_loop_detector) {
        LoopClosureConfig config = m_loop_detector->get_config();
        config.enable_loop_detection = enable;
        m_loop_detector->update_config(config);
        spdlog::info("[Estimator] Loop closure detection {}", enable ? "enabled" : "disabled");
    }
}

void Estimator::set_loop_closure_config(const LoopClosureConfig& config) {
    if (m_loop_detector) {
        m_loop_detector->update_config(config);
    }
}

size_t Estimator::get_loop_closure_count() const {
    // For now, return 0 since we haven't implemented PGO yet
    // This will be updated when we add pose graph optimization
    return 0;
}

std::map<int, Eigen::Matrix4f> Estimator::get_optimized_trajectory() const {
    std::map<int, Eigen::Matrix4f> result;
    for (const auto& [id, pose] : m_optimized_poses) {
        result[id] = pose.matrix();
    }
    return result;
}

void Estimator::process_loop_closures(std::shared_ptr<database::LidarFrame> current_keyframe, 
                                     const std::vector<LoopCandidate>& loop_candidates) {
    
    if (loop_candidates.empty()) {
        return;
    }
    
    // Use only the best candidate (first one, already sorted by similarity score)
    const auto& candidate = loop_candidates[0];
    
    spdlog::info("[Estimator] Processing best loop closure candidate for ICP optimization");
    
    // Find the matched keyframe in our database
    std::shared_ptr<database::LidarFrame> matched_keyframe = nullptr;
    
    for (const auto& kf : m_keyframes) {
        if (static_cast<size_t>(kf->get_keyframe_id()) == candidate.match_keyframe_id) {
            matched_keyframe = kf;
            break;
        }
    }
    
    if (!matched_keyframe) {
        spdlog::warn("[Estimator] Could not find matched keyframe {} in database", candidate.match_keyframe_id);
        return;
    }
    
    // Get local maps from both keyframes
    auto current_local_map = current_keyframe->get_local_map();
    auto matched_local_map = matched_keyframe->get_local_map();
    
    if (!current_local_map || !matched_local_map || 
        current_local_map->empty() || matched_local_map->empty()) {
        spdlog::warn("[Estimator] Empty local maps for loop {} <-> {}", 
                    candidate.query_keyframe_id, candidate.match_keyframe_id);
        return;
    }

    Sophus::SE3f T_current_l2l;
    float inlier_ratio = 0.0f;

    bool icp_success = m_icp_optimizer->optimize_loop(
        current_keyframe,             // source frame (has fresh kdtree built)
        matched_keyframe,             // target frame (will use local map as features)
        T_current_l2l,                // optimized relative transform (output)
        inlier_ratio                  // inlier ratio (output)
    );

    if (!icp_success) {
        spdlog::warn("[Estimator] Loop closure ICP failed for {} <-> {}", 
                    candidate.query_keyframe_id, candidate.match_keyframe_id);
        return;
    }
    
    // Validate loop closure using inlier ratio
    const float min_inlier_ratio = 0.3f;  // Minimum 30% inliers required
    if (inlier_ratio < min_inlier_ratio) {
        spdlog::warn("[Estimator] Loop closure rejected: inlier ratio {:.2f}% < {:.2f}% for {} <-> {}", 
                    inlier_ratio * 100.0f, min_inlier_ratio * 100.0f,
                    candidate.query_keyframe_id, candidate.match_keyframe_id);
        return;
    }

    // T_current_l2l is the ICP correction: T_original^-1 * T_corrected
    // ICP optimizes curr_keyframe pose, returns correction transform
    
    // Get current poses (with drift)
    Sophus::SE3f T_world_current = current_keyframe->get_pose();
    Sophus::SE3f T_world_matched = matched_keyframe->get_pose();
    
    // Apply ICP correction: T_corrected = T_correction * T_original
    // ICP returns: T_correction = T_original^-1 * T_optimized
    // So: T_corrected = (T_original^-1 * T_optimized) * T_original = T_optimized
    Sophus::SE3f T_current_corrected = T_world_current * T_current_l2l;
    
    // Calculate pose difference for logging (how much correction ICP suggests)
    SE3f pose_diff = T_world_current.inverse() * T_current_corrected;
    float translation_diff = pose_diff.translation().norm();
    float rotation_diff = pose_diff.so3().log().norm() * 180.0f / M_PI;
    
    spdlog::info("[Estimator] Loop closure ICP success {} <-> {}: Δt={:.3f}m, Δr={:.2f}°, inliers={:.1f}%",
                candidate.query_keyframe_id, candidate.match_keyframe_id,
                translation_diff, rotation_diff, inlier_ratio * 100.0f);
    
    // Compute relative pose constraint: from matched to current
    // Using GTSAM's between() logic: poseFrom.between(poseTo) = poseFrom^-1 * poseTo
    Sophus::SE3f T_matched_to_current = T_world_matched.inverse() * T_current_corrected;
    
    // Check if PGO is enabled
    if (!m_config.pgo_enable_pgo) {
        spdlog::info("[Estimator] PGO disabled, skipping pose graph optimization");
        return;
    }
    
    // Build pose graph from scratch with all keyframes and odometry constraints
    spdlog::info("[PGO] Using Ceres optimizer");
    
    // Store pre-PGO poses for visualization (before optimization)
    std::map<int, SE3f> pre_pgo_poses;
    for (const auto& kf : m_keyframes) {
        // Use get_stored_pose() to get the actual stored value, not dynamic calculation
        pre_pgo_poses[kf->get_keyframe_id()] = kf->get_stored_pose();
    }
    
    // Clear previous graph
    m_pose_graph_optimizer->clear();
    
    // Add all keyframes and odometry constraints
    spdlog::info("[Estimator] Building pose graph with {} keyframes", m_keyframes.size());
    
    // Add all keyframes as optimization variables
    for (size_t i = 0; i < m_keyframes.size(); ++i) {
        auto& kf = m_keyframes[i];
        
        if (i == 0) {
            // First keyframe: add with prior factor (constant)
            m_pose_graph_optimizer->add_keyframe_pose(
                kf->get_keyframe_id(),
                pre_pgo_poses[kf->get_keyframe_id()],
                true);
        } else {
            // Non-first keyframe: add as variable
            m_pose_graph_optimizer->add_keyframe_pose(
                kf->get_keyframe_id(),
                pre_pgo_poses[kf->get_keyframe_id()],
                false);
            
            // Add odometry constraint from previous keyframe
            auto& prev_kf = m_keyframes[i-1];
            SE3f relative_pose = kf->get_relative_pose();
            
            double odom_trans_noise = m_config.pgo_odometry_translation_noise;
            double odom_rot_noise = m_config.pgo_odometry_rotation_noise;
            double trans_weight = 1.0 / (odom_trans_noise * odom_trans_noise);
            double rot_weight = 1.0 / (odom_rot_noise * odom_rot_noise);
            m_pose_graph_optimizer->add_odometry_constraint(
                prev_kf->get_keyframe_id(),
                kf->get_keyframe_id(),
                relative_pose,
                trans_weight,
                rot_weight
            );
        }
    }
    
    // Store and add loop closure constraint
    LoopConstraint loop_constraint;
    loop_constraint.from_keyframe_id = matched_keyframe->get_keyframe_id();
    loop_constraint.to_keyframe_id = current_keyframe->get_keyframe_id();
    loop_constraint.relative_pose = T_matched_to_current;
    loop_constraint.translation_noise = m_config.pgo_loop_translation_noise;
    loop_constraint.rotation_noise = m_config.pgo_loop_rotation_noise;
    
    // Accumulate all loop constraints
    m_loop_constraints.push_back(loop_constraint);
    spdlog::info("[Estimator] Stored loop closure constraint: {} -> {} (total loops: {})",
                loop_constraint.from_keyframe_id, loop_constraint.to_keyframe_id, 
                m_loop_constraints.size());
    
    // Add ALL accumulated loop constraints to pose graph
    spdlog::info("[Estimator] Adding {} stored loop constraints to PGO", m_loop_constraints.size());
    for (const auto& lc : m_loop_constraints) {
        double trans_weight = 1.0 / (lc.translation_noise * lc.translation_noise);
        double rot_weight = 1.0 / (lc.rotation_noise * lc.rotation_noise);
        spdlog::debug("[Estimator]   Loop {}->{}: trans_weight={}, rot_weight={}", 
                     lc.from_keyframe_id, lc.to_keyframe_id, trans_weight, rot_weight);
        m_pose_graph_optimizer->add_loop_closure_constraint(
            lc.from_keyframe_id,
            lc.to_keyframe_id,
            lc.relative_pose,
            trans_weight,
            rot_weight
        );
    }
    
    // Perform pose graph optimization
    spdlog::info("[Estimator] Running Ceres pose graph optimization with {} loop closures...", m_loop_constraints.size());
    
    bool opt_success = m_pose_graph_optimizer->optimize();
        
        if (opt_success) {
            // Print pose corrections without applying them
            auto optimized_poses = m_pose_graph_optimizer->get_all_optimized_poses();
            
            spdlog::info("[Estimator] ========== PGO Results (NOT APPLIED) ==========");
            spdlog::info("[Estimator] Total keyframes optimized: {}", optimized_poses.size());
            spdlog::info("[Estimator]");
            spdlog::info("[Estimator] Per-Keyframe Pose Corrections:");
            spdlog::info("[Estimator] KF_ID | Δt (m)  | Δr (deg) | Position Before -> After");
            spdlog::info("[Estimator] ------|---------|----------|------------------------");
            
            float max_translation_diff = 0.0f;
            float max_rotation_diff = 0.0f;
            float avg_translation_diff = 0.0f;
            float avg_rotation_diff = 0.0f;
            int count = 0;
            
            for (size_t i = 0; i < m_keyframes.size(); i++) {
                int kf_id = m_keyframes[i]->get_keyframe_id();
                auto it = optimized_poses.find(kf_id);
                
                if (it != optimized_poses.end()) {
                    SE3f old_pose = m_keyframes[i]->get_pose();
                    SE3f new_pose = it->second;
                    
                    float translation_diff = (new_pose.translation() - old_pose.translation()).norm();
                    float rotation_diff = (new_pose.so3().log() - old_pose.so3().log()).norm() * 180.0f / M_PI;
                    
                    // Print each keyframe's correction (debug level)
                    Eigen::Vector3f pos_before = old_pose.translation();
                    Eigen::Vector3f pos_after = new_pose.translation();
                    spdlog::debug("[Estimator] {:5d} | {:7.4f} | {:8.3f} | ({:6.2f},{:6.2f},{:6.2f}) -> ({:6.2f},{:6.2f},{:6.2f})",
                                kf_id, 
                                translation_diff, 
                                rotation_diff,
                                pos_before.x(), pos_before.y(), pos_before.z(),
                                pos_after.x(), pos_after.y(), pos_after.z());
                    
                    max_translation_diff = std::max(max_translation_diff, translation_diff);
                    max_rotation_diff = std::max(max_rotation_diff, rotation_diff);
                    avg_translation_diff += translation_diff;
                    avg_rotation_diff += rotation_diff;
                    count++;
                    
                    // Calculate odometry constraint cost
                    float odom_cost_before = 0.0f;
                    float odom_cost_after = 0.0f;
                    
                    if (i > 0) {
                        SE3f prev_old = m_keyframes[i-1]->get_pose();
                        SE3f prev_new = optimized_poses[m_keyframes[i-1]->get_keyframe_id()];
                        SE3f relative = prev_old.inverse() * old_pose;
                        
                        SE3f error_before = relative.inverse() * prev_old.inverse() * old_pose;
                        SE3f error_after = relative.inverse() * prev_new.inverse() * new_pose;
                        
                        Eigen::Matrix<float, 6, 1> log_error_before = error_before.log();
                        Eigen::Matrix<float, 6, 1> log_error_after = error_after.log();
                        
                        float t_weight = 1.0f / (0.1f * 0.1f);
                        float r_weight = 1.0f / (0.1f * 0.1f);
                        
                        odom_cost_before = t_weight * log_error_before.head<3>().squaredNorm() + 
                                          r_weight * log_error_before.tail<3>().squaredNorm();
                        odom_cost_after = t_weight * log_error_after.head<3>().squaredNorm() + 
                                         r_weight * log_error_after.tail<3>().squaredNorm();
                    }
                    
                    // Calculate loop closure constraint cost
                    float loop_cost_before = 0.0f;
                    float loop_cost_after = 0.0f;
                    
                    if ((matched_keyframe && kf_id == matched_keyframe->get_keyframe_id()) ||
                        (current_keyframe && kf_id == current_keyframe->get_keyframe_id())) {
                        
                        SE3f matched_old = matched_keyframe->get_pose();
                        SE3f matched_new = optimized_poses[matched_keyframe->get_keyframe_id()];
                        SE3f current_old = current_keyframe->get_pose();
                        SE3f current_new = optimized_poses[current_keyframe->get_keyframe_id()];
                        
                        SE3f error_before = T_matched_to_current.inverse() * matched_old.inverse() * current_old;
                        SE3f error_after = T_matched_to_current.inverse() * matched_new.inverse() * current_new;
                        
                        Eigen::Matrix<float, 6, 1> log_error_before = error_before.log();
                        Eigen::Matrix<float, 6, 1> log_error_after = error_after.log();
                        
                        float loop_t_weight = 1.0f / (0.01f * 0.01f);
                        float loop_r_weight = 1.0f / (0.01f * 0.01f);
                        
                        loop_cost_before = loop_t_weight * log_error_before.head<3>().squaredNorm() + 
                                          loop_r_weight * log_error_before.tail<3>().squaredNorm();
                        loop_cost_after = loop_t_weight * log_error_after.head<3>().squaredNorm() + 
                                         loop_r_weight * log_error_after.tail<3>().squaredNorm();
                    }
                    
                    float total_cost_before = odom_cost_before + loop_cost_before;
                    float total_cost_after = odom_cost_after + loop_cost_after;
                    
                }
            }
            
            if (count > 0) {
                avg_translation_diff /= count;
                avg_rotation_diff /= count;
                
                spdlog::info("[Estimator] ========== PGO Statistics =========="); 
                spdlog::info("[Estimator] Average correction: Δt={:.3f}m, Δr={:.2f}°", avg_translation_diff, avg_rotation_diff);
                spdlog::info("[Estimator] Maximum correction: Δt={:.3f}m, Δr={:.2f}°", max_translation_diff, max_rotation_diff);
            }
            
            spdlog::info("[Estimator] =========================================");
            
            // Store optimized poses for visualization
            m_optimized_poses = optimized_poses;
            
            // Apply pose graph optimization results to all keyframes
            spdlog::info("[Estimator] Applying PGO corrections to all keyframes...");
            apply_pose_graph_optimization();
            
            // Update cooldown
            m_last_successful_loop_keyframe_id = current_keyframe->get_keyframe_id();
            spdlog::info("[Estimator] Loop closure cooldown activated: next detection after keyframe {}",
                        m_last_successful_loop_keyframe_id + m_config.loop_min_keyframe_gap);
        } else {
            spdlog::error("[Estimator] Pose graph optimization failed!");
        }
}

void Estimator::apply_pose_graph_optimization() {
    // Get all optimized poses from Ceres pose graph optimizer
    auto optimized_poses = m_pose_graph_optimizer->get_all_optimized_poses();
    
    if (optimized_poses.empty()) {
        spdlog::warn("[Estimator] No optimized poses available from pose graph!");
        return;
    }
    
    spdlog::info("[Estimator] Applying PGO results to {} keyframes", optimized_poses.size());

    auto last_keyframe = m_keyframes.back();

    Sophus::SE3f last_keyframe_pose_before_opt = last_keyframe->get_pose();
    Sophus::SE3f last_keyframe_pose_after_opt = optimized_poses[last_keyframe->get_keyframe_id()];
    Sophus::SE3f total_correction = last_keyframe_pose_after_opt * last_keyframe_pose_before_opt.inverse();
    
    // Update all keyframe poses (absolute poses only)
    // NOTE: We do NOT recalculate relative poses after PGO!
    // The PGO optimization already considers relative pose constraints,
    // so recalculating and normalizing them would distort the optimization result.
    for (auto& keyframe : m_keyframes) {
        int kf_id = keyframe->get_keyframe_id();
        
        auto it = optimized_poses.find(kf_id);
        if (it == optimized_poses.end()) {
            spdlog::warn("[Estimator] No optimized pose for keyframe {}", kf_id);
            continue;
        }
        
        SE3f old_pose = keyframe->get_pose();
        SE3f new_pose = it->second;
        
        float translation_diff = (new_pose.translation() - old_pose.translation()).norm();
        float rotation_diff = (new_pose.so3().log() - old_pose.so3().log()).norm() * 180.0f / M_PI;
        
        spdlog::debug("[Estimator] Keyframe {}: Δt={:.3f}m, Δr={:.2f}°", 
                     kf_id, translation_diff, rotation_diff);
        
        // Update keyframe pose (absolute pose)
        keyframe->set_pose(new_pose);
    }
    
    // Relative poses are kept as-is (not recalculated from absolute poses)
    // This preserves the optimization result from PGO

    // transform last keyframe's local map
    auto last_local_map = last_keyframe->get_local_map();

    util::PointCloudPtr transformed_local_map = std::make_shared<util::PointCloud>();

    util::transform_point_cloud(
        last_local_map,
        transformed_local_map,
        total_correction.matrix()
    );

    // Update last keyframe's local map
    last_keyframe->set_local_map(transformed_local_map);
    last_keyframe->build_local_map_kdtree();
}

} // namespace processing
} // namespace lidar_odometry
