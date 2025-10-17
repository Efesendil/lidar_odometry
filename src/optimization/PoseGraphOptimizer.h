/**
 * @file      PoseGraphOptimizer.h
 * @brief     Ceres-based pose graph optimization for loop closure.
 * @author    Seungwon Choi
 * @date      2025-10-17
 * @copyright Copyright (c) 2025 Seungwon Choi. All rights reserved.
 *
 * @par License
 * This project is released under the MIT License.
 */

#pragma once

#include <ceres/ceres.h>
#include "../util/Types.h"
#include "Factors.h"
#include "Parameters.h"
#include <sophus/se3.hpp>
#include <memory>
#include <vector>
#include <map>

namespace lidar_odometry {
namespace optimization {

/**
 * @brief Pose graph optimizer using Ceres for loop closure optimization
 * 
 * This class manages a pose graph with:
 * - Keyframe poses as optimization variables
 * - Odometry constraints (between consecutive keyframes)
 * - Loop closure constraints (between non-consecutive keyframes)
 * 
 * Uses RelativePoseFactor from Factors.h for all constraints.
 */
class PoseGraphOptimizer {
public:
    using SE3f = Sophus::SE3f;
    
    /**
     * @brief Constraint type for debugging/logging
     */
    enum class ConstraintType {
        PRIOR,          ///< Prior constraint on first pose
        ODOMETRY,       ///< Sequential odometry constraint
        LOOP_CLOSURE    ///< Loop closure constraint
    };
    
    /**
     * @brief Constructor
     */
    PoseGraphOptimizer();
    
    /**
     * @brief Destructor
     */
    ~PoseGraphOptimizer();
    
    /**
     * @brief Add a keyframe pose to the pose graph
     * @param keyframe_id Keyframe ID
     * @param pose Pose in world coordinates (initial estimate with drift)
     * @param is_first_keyframe Whether this is the first keyframe (will fix this pose)
     */
    void add_keyframe_pose(int keyframe_id, const SE3f& pose, bool is_first_keyframe = false);
    
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
    size_t get_keyframe_count() const { return m_poses.size(); }
    
    /**
     * @brief Get number of loop closure constraints
     */
    size_t get_loop_closure_count() const { return m_loop_closure_count; }
    
private:
    /**
     * @brief Pose parameter block (SE3 in tangent space [6])
     */
    struct PoseBlock {
        std::array<double, 6> parameters;  // [tx, ty, tz, rx, ry, rz]
        bool is_fixed;                     // Whether this pose is fixed (first keyframe)
    };
    
    std::map<int, PoseBlock> m_poses;              ///< Keyframe ID -> pose parameters
    std::unique_ptr<ceres::Problem> m_problem;     ///< Ceres optimization problem
    
    size_t m_loop_closure_count;                   ///< Number of loop closure constraints
    bool m_is_optimized;                           ///< Whether optimization has been performed
};

} // namespace optimization
} // namespace lidar_odometry