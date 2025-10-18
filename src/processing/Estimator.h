/**
 * @file      Estimator.h
 * @brief     LiDAR odometry estimator with ICP and mapping capabilities.
 * @author    Seungwon Choi
 * @date      2025-09-24
 * @copyright Copyright (c) 2025 Seungwon Choi. All rights reserved.
 *
 * @par License
 * This project is released under the MIT License.
 */

#pragma once

#include "../util/Types.h"
#include "../util/Config.h"
#include "../database/LidarFrame.h"
#include "FeatureExtractor.h"
#include "../optimization/Factors.h"
#include "../optimization/Parameters.h"
#include "../optimization/AdaptiveMEstimator.h"
#include "../optimization/PoseGraphOptimizerGTSAM.h"
#include "../optimization/PoseGraphOptimizerCeres.h"
#include "DualFrameICPOptimizer.h"
#include "LoopClosureDetector.h"
#include "../util/MathUtils.h"

#include <ceres/ceres.h>
#include <sophus/se3.hpp>

#include <memory>
#include <vector>
#include <map>
#include <deque>

namespace lidar_odometry {
namespace processing {

// Import types from util namespace  
using namespace lidar_odometry::util;

/**
 * @brief LiDAR odometry estimator
 * 
 * This class performs:
 * - Frame-to-frame ICP registration
 * - Feature-based optimization with Ceres
 * - Local mapping and keyframe management
 * - Pose graph optimization
 */
class Estimator {
public:
    /**
     * @brief Constructor
     * @param config Configuration from YAML file
     */
    explicit Estimator(const util::SystemConfig& config);
    
    /**
     * @brief Destructor
     */
    ~Estimator();
    
    // ===== Main Processing =====
    
    /**
     * @brief Process new LiDAR frame
     * @param current_frame Current LiDAR frame
     * @return True if processing successful
     */
    bool process_frame(std::shared_ptr<database::LidarFrame> current_frame);
    
    /**
     * @brief Update configuration
     * @param config New configuration
     */
    void update_config(const util::SystemConfig& config);
    
    /**
     * @brief Get current configuration
     * @return Current configuration
     */
    const util::SystemConfig& get_config() const;
    
    /**
     * @brief Get local map for visualization
     * @return Const pointer to local map
     */
    PointCloudConstPtr get_local_map() const;
    
    /**
     * @brief Get last keyframe's local map for visualization
     * @return Const pointer to last keyframe's local map, nullptr if no keyframes
     */
    PointCloudConstPtr get_last_keyframe_map() const;

    /**
    * @brief Get current pose
    * @return Current pose as SE3f
    */
    const SE3f& get_current_pose() const;

    /**
     * @brief Get dual frame optimization statistics
     * @param avg_iterations Average iterations per optimization call
     * @param avg_time_ms Average time per optimization call in milliseconds
     */
    void get_optimization_statistics(double& avg_iterations, double& avg_time_ms) const;
    
    /**
     * @brief Get debug clouds for visualization
     * @param pre_icp_cloud Output pre-ICP cloud
     * @param post_icp_cloud Output post-ICP cloud
     */
    void get_debug_clouds(PointCloudConstPtr& pre_icp_cloud, PointCloudConstPtr& post_icp_cloud) const;
    
    /**
     * @brief Get number of keyframes
     * @return Number of keyframes created so far
     */
    size_t get_keyframe_count() const;
    
    /**
     * @brief Get keyframe by index
     * @param index Keyframe index
     * @return Keyframe at the given index, nullptr if index out of bounds
     */
    std::shared_ptr<database::LidarFrame> get_keyframe(size_t index) const;
    
    /**
     * @brief Enable/disable loop closure detection
     * @param enable Enable loop closure detection
     */
    void enable_loop_closure(bool enable);
    
    /**
     * @brief Set loop closure detection configuration
     * @param config Loop closure configuration
     */
    void set_loop_closure_config(const LoopClosureConfig& config);
    
    /**
     * @brief Get loop closure detection statistics
     * @return Number of loop closures detected
     */
    size_t get_loop_closure_count() const;
    
    /**
     * @brief Get optimized trajectory from pose graph optimization (for debugging)
     * @return Map of keyframe ID to optimized pose (as Eigen::Matrix4f)
     */
    std::map<int, Eigen::Matrix4f> get_optimized_trajectory() const;

private:
    // ===== Internal Processing =====
    
    /**
     * @brief Preprocess frame: downsample and extract features
     * @param frame Frame to preprocess (will be modified in-place)
     * @return True if preprocessing successful
     */
    bool preprocess_frame(std::shared_ptr<database::LidarFrame> frame);
    
    /**
     * @brief Initialize with first frame
     * @param frame First frame
     */
    void initialize_first_frame(std::shared_ptr<database::LidarFrame> frame);
    
    /**
     * @brief Estimate motion between frames using DualFrameICPOptimizer
     * @param current_frame Current frame with features
     * @param keyframe Reference keyframe with local map
     * @param initial_guess Initial transformation guess
     * @return Estimated transformation from keyframe to current
     */
    SE3f estimate_motion_dual_frame(std::shared_ptr<database::LidarFrame> current_frame,
                                   std::shared_ptr<database::LidarFrame> keyframe,
                                   const SE3f& initial_guess = SE3f());
    
    /**
     * @brief Select best keyframe for current frame
     * @param current_pose Current frame pose
     * @return Best keyframe for optimization, nullptr if none suitable
     */
    std::shared_ptr<database::LidarFrame> select_best_keyframe(const SE3f& current_pose);
    
    /**
     * @brief Check if current frame should be a keyframe
     * @param current_pose Current pose
     * @return True if should be keyframe
     */
    bool should_create_keyframe(const SE3f& current_pose);
    
    /**
     * @brief Create new keyframe
     * @param frame Frame to use as keyframe
     * @param feature_cloud Features of the frame as point cloud
     */
    void create_keyframe(std::shared_ptr<database::LidarFrame> frame);
    
    /**
     * @brief Process loop closure candidates and compute relative poses
     * @param current_keyframe Current keyframe
     * @param loop_candidates List of loop closure candidates
     */
    void process_loop_closures(std::shared_ptr<database::LidarFrame> current_keyframe, 
                              const std::vector<LoopCandidate>& loop_candidates);

    /**
     * @brief Apply pose graph optimization results to all keyframes
     */
    void apply_pose_graph_optimization();

    /**
     * @brief Rebuild local map for current keyframe after loop closure
     * Clears existing local map and rebuilds it from the updated global feature map
     */
    void rebuild_current_keyframe_local_map();

    
    

private:
    // Configuration
    util::SystemConfig m_config;
    
    // State
    bool m_initialized;
    SE3f m_T_wl_current;
    SE3f m_velocity;  // Velocity model: T_current = T_previous * m_velocity
    std::vector<SE3f> m_trajectory;
    int m_next_keyframe_id;  // Next keyframe ID to assign
    
    // Frames and features
    std::shared_ptr<database::LidarFrame> m_previous_frame;
    std::deque<std::shared_ptr<database::LidarFrame>> m_keyframes;
    
    // Local map
    PointCloudPtr m_feature_map;
    
    // Debug visualization clouds
    PointCloudPtr m_debug_pre_icp_cloud;
    PointCloudPtr m_debug_post_icp_cloud;
    
    // Processing tools
    std::shared_ptr<DualFrameICPOptimizer> m_dual_optimizer;
    std::unique_ptr<util::VoxelGrid> m_voxel_filter;
    std::unique_ptr<FeatureExtractor> m_feature_extractor;
    std::shared_ptr<optimization::AdaptiveMEstimator> m_adaptive_estimator;
    
    // Loop closure detection and pose graph optimization
    std::unique_ptr<LoopClosureDetector> m_loop_detector;
    std::shared_ptr<optimization::PoseGraphOptimizerGTSAM> m_pose_graph_optimizer_gtsam;
    std::shared_ptr<optimization::PoseGraphOptimizerCeres> m_pose_graph_optimizer_ceres;
    int m_last_successful_loop_keyframe_id;  // Last keyframe ID where loop closure succeeded
    bool m_use_ceres_pgo;  // Flag to switch between GTSAM and Ceres PGO
    std::map<int, SE3f> m_optimized_poses;  // Optimized poses from PGO (for debugging visualization)
    
    // Loop closure storage (to reuse in every PGO)
    struct LoopConstraint {
        int from_keyframe_id;
        int to_keyframe_id;
        SE3f relative_pose;
        double translation_noise;
        double rotation_noise;
    };
    std::vector<LoopConstraint> m_loop_constraints;  // All detected loop closures
    
    // Last keyframe for optimization
    std::shared_ptr<database::LidarFrame> m_last_keyframe;
    
    // Last keyframe pose for keyframe decision
    SE3f m_last_keyframe_pose;
    
    // Optimization statistics (changed from ICP to dual frame)
    mutable size_t m_total_optimization_iterations;
    mutable double m_total_optimization_time_ms;
    mutable size_t m_optimization_call_count;

};

} // namespace processing
} // namespace lidar_odometry
