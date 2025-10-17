/**
 * @file      PoseGraphOptimizerGTSAM.h
 * @brief     GTSAM-based pose graph optimization for loop closure.
 * @author    Seungwon Choi
 * @date      2025-10-17
 * @copyright Copyright (c) 2025 Seungwon Choi. All rights reserved.
 *
 * @par License
 * This project is released under the MIT License.
 */

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/inference/Symbol.h>

#include "../util/Types.h"
#include <sophus/se3.hpp>
#include <memory>
#include <vector>
#include <map>

namespace lidar_odometry {
namespace optimization {

/**
 * @brief GTSAM-based pose graph optimizer for loop closure optimization
 * 
 * This class uses GTSAM's NonlinearFactorGraph for pose graph optimization:
 * - Keyframe poses as optimization variables
 * - Odometry constraints (BetweenFactor between consecutive keyframes)
 * - Loop closure constraints (BetweenFactor between non-consecutive keyframes)
 * - Prior factor on first pose
 * 
 * GTSAM advantages:
 * - Incremental optimization with iSAM2
 * - Automatic covariance propagation
 * - Robust outlier rejection
 * - Well-tested and optimized
 */
class PoseGraphOptimizerGTSAM {
public:
    using SE3f = Sophus::SE3f;
    
    /**
     * @brief Constructor
     */
    PoseGraphOptimizerGTSAM();
    
    /**
     * @brief Destructor
     */
    ~PoseGraphOptimizerGTSAM();
    
    /**
     * @brief Add a keyframe pose to the pose graph
     * @param keyframe_id Keyframe ID
     * @param pose Pose in world coordinates (initial estimate with drift)
     * @param should_fix Whether to fix this pose (will not be optimized)
     */
    void add_keyframe_pose(int keyframe_id, const SE3f& pose, bool should_fix = false);
    
    /**
     * @brief Add an odometry constraint between consecutive keyframes
     * @param from_keyframe_id Source keyframe ID
     * @param to_keyframe_id Target keyframe ID
     * @param relative_pose Relative pose from source to target
     * @param translation_noise Translation noise standard deviation (meters)
     * @param rotation_noise Rotation noise standard deviation (radians)
     */
    void add_odometry_constraint(int from_keyframe_id, int to_keyframe_id,
                                 const SE3f& relative_pose,
                                 double translation_noise = 1.0,
                                 double rotation_noise = 1.0);
    
    /**
     * @brief Add a loop closure constraint between non-consecutive keyframes
     * @param from_keyframe_id Source keyframe ID
     * @param to_keyframe_id Target keyframe ID
     * @param relative_pose Relative pose from source to target (from ICP)
     * @param translation_noise Translation noise standard deviation (meters)
     * @param rotation_noise Rotation noise standard deviation (radians)
     */
    void add_loop_closure_constraint(int from_keyframe_id, int to_keyframe_id,
                                     const SE3f& relative_pose,
                                     double translation_noise = 1.0,
                                     double rotation_noise = 1.0);
    
    /**
     * @brief Perform pose graph optimization
     * @return True if optimization succeeded
     */
    bool optimize();
    
    /**
     * @brief Get optimized pose for a keyframe
     * @param keyframe_id Keyframe ID
     * @param optimized_pose Output optimized pose
     * @return True if pose was found
     */
    bool get_optimized_pose(int keyframe_id, SE3f& optimized_pose) const;
    
    /**
     * @brief Get all optimized poses
     * @return Map from keyframe ID to optimized pose
     */
    std::map<int, SE3f> get_all_optimized_poses() const;
    
    /**
     * @brief Clear the pose graph (remove all poses and constraints)
     */
    void clear();
    
    /**
     * @brief Get number of keyframes in the graph
     */
    size_t get_keyframe_count() const { return m_initial_estimates.size(); }
    
    /**
     * @brief Get number of loop closure constraints
     */
    size_t get_loop_closure_count() const { return m_loop_closure_count; }
    
private:
    /**
     * @brief Convert Sophus SE3 to GTSAM Pose3
     */
    static gtsam::Pose3 sophus_to_gtsam(const Sophus::SE3d& pose);
    
    /**
     * @brief Convert GTSAM Pose3 to Sophus SE3
     */
    static Sophus::SE3d gtsam_to_sophus(const gtsam::Pose3& pose);
    
    /**
     * @brief Create noise model from standard deviations
     */
    static gtsam::noiseModel::Diagonal::shared_ptr create_noise_model(
        double translation_noise, double rotation_noise);
    
    gtsam::NonlinearFactorGraph m_graph;        ///< GTSAM factor graph
    gtsam::Values m_initial_estimates;          ///< Initial pose estimates
    gtsam::Values m_optimized_values;           ///< Optimized pose values
    
    std::set<int> m_fixed_keyframes;            ///< Set of fixed keyframe IDs
    size_t m_loop_closure_count;                ///< Number of loop closure constraints
    bool m_is_optimized;                        ///< Whether optimization has been performed
};

} // namespace optimization
} // namespace lidar_odometry
