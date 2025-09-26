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
#include "../database/LidarFrame.h"
#include "FeatureExtractor.h"
#include "../optimization/Factors.h"
#include "../optimization/Parameters.h"
#include "../optimization/AdaptiveMEstimator.h"  // Add this include
#include "IterativeClosestPoint.h"  // Include instead of forward declaration

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>

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
 * @brief Configuration for the Estimator
 */
struct EstimatorConfig {
    // ICP parameters (from YAML odometry section)
    size_t max_icp_iterations = 50;
    double icp_convergence_threshold = 0.001;  // Used for both translation and rotation tolerance
    double correspondence_distance = 1.0;
    double transformation_epsilon = 1e-6;
    double euclidean_fitness_epsilon = 1e-6;
    
    // Robust estimation parameters (from YAML robust_estimation section)
    bool use_adaptive_m_estimator = true;
    std::string loss_type = "huber";
    std::string scale_method = "MAD";
    double fixed_scale_factor = 1.0;
    double mad_multiplier = 1.4826;
    double min_scale_factor = 0.01;                // Also used as PKO alpha lower bound
    double max_scale_factor = 10.0;               // Also used as PKO alpha upper bound
    
    // PKO (Probabilistic Kernel Optimization) parameters
    int num_alpha_segments = 1000;
    double truncated_threshold = 10.0;
    int gmm_components = 3;
    int gmm_sample_size = 100;
    std::string pko_kernel_type = "cauchy";
    
    // Mapping parameters
    double voxel_size = 0.4;  // Voxel size for all downsampling operations
    double max_range = 100.0;  // Max range for crop box filter
    size_t max_map_frames = 50;
    double keyframe_distance_threshold = 1.0;
    double keyframe_rotation_threshold = 0.2;  // radians
    
    // Ceres optimization
    size_t max_solver_iterations = 100;
    double parameter_tolerance = 1e-8;
    double function_tolerance = 1e-8;
    
    // Feature matching
    double max_correspondence_distance = 2.0;
    size_t min_correspondence_points = 10;
    
    // Local map parameters
    size_t local_map_size = 20;
    double local_map_radius = 50.0;
};

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
     * @param config Estimator configuration
     */
    explicit Estimator(const EstimatorConfig& config = EstimatorConfig());
    
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
    void update_config(const EstimatorConfig& config);
    
    /**
     * @brief Get current configuration
     * @return Current configuration
     */
    const EstimatorConfig& get_config() const;
    
    /**
     * @brief Get local map for visualization
     * @return Const pointer to local map
     */
    PointCloudConstPtr get_local_map() const;
    
    /**
     * @brief Get ICP statistics
     * @param avg_iterations Average iterations per ICP call
     * @param avg_time_ms Average time per ICP call in milliseconds
     */
    void get_icp_statistics(double& avg_iterations, double& avg_time_ms) const;
    
    /**
     * @brief Get debug clouds for visualization
     * @param pre_icp_cloud Output pre-ICP cloud
     * @param post_icp_cloud Output post-ICP cloud
     */
    void get_debug_clouds(PointCloudConstPtr& pre_icp_cloud, PointCloudConstPtr& post_icp_cloud) const;

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
     * @brief Estimate motion between frames using ICP
     * @param current_features Current feature cloud
     * @param previous_features Previous feature cloud
     * @param initial_guess Initial transformation guess
     * @return Estimated transformation
     */
    SE3f estimate_motion_icp(PointCloudConstPtr current_features,
                            PointCloudConstPtr previous_features,
                            const SE3f& initial_guess = SE3f());
    
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
    

private:
    // Configuration
    EstimatorConfig m_config;
    
    // State
    bool m_initialized;
    SE3f m_T_wl_current;
    SE3f m_velocity;  // Velocity model: T_current = T_previous * m_velocity
    std::vector<SE3f> m_trajectory;
    
    // Frames and features
    std::shared_ptr<database::LidarFrame> m_previous_frame;
    std::deque<std::shared_ptr<database::LidarFrame>> m_keyframes;
    
    // Local map
    PointCloudPtr m_feature_map;
    
    // Debug visualization clouds
    PointCloudPtr m_debug_pre_icp_cloud;
    PointCloudPtr m_debug_post_icp_cloud;
    
    // Processing tools
    std::shared_ptr<IterativeClosestPoint> m_icp;
    std::unique_ptr<pcl::VoxelGrid<PointType>> m_voxel_filter;
    std::unique_ptr<FeatureExtractor> m_feature_extractor;
    std::shared_ptr<optimization::AdaptiveMEstimator> m_adaptive_estimator;
    
    // Last keyframe pose for keyframe decision
    SE3f m_last_keyframe_pose;
    
    // ICP statistics
    mutable size_t m_total_icp_iterations;
    mutable double m_total_icp_time_ms;
    mutable size_t m_icp_call_count;
};

} // namespace processing
} // namespace lidar_odometry
