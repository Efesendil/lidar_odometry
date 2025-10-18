/**
 * @file      PoseGraphOptimizer.h
 * @brief     Ceres-based pose graph optimization for loop closure.
 * @author    Seungwon Choi
 * @date      2025-10-18
 * @copyright Copyright (c) 2025 Seungwon Choi. All rights reserved.
 *
 * @par License
 * This project is released under the MIT License.
 */

#pragma once

#include "../util/Types.h"
#include "Parameters.h"
#include "Factors.h"
#include <sophus/se3.hpp>
#include <memory>
#include <vector>
#include <map>

namespace lidar_odometry {
namespace optimization {

/**
 * @brief Ceres-based pose graph optimizer for loop closure optimization
 * 
 * This class uses Ceres Solver with custom RelativePoseFactor for pose graph optimization:
 * - Keyframe poses as optimization variables (SE3 tangent space)
 * - Odometry constraints (RelativePoseFactor between consecutive keyframes)
 * - Loop closure constraints (RelativePoseFactor between non-consecutive keyframes)
 * - Prior factor on first pose (strong constraint)
 * 
 * Uses the corrected RelativePoseFactor with proper Jacobian calculation.
 */
class PoseGraphOptimizer {
public:
    using SE3f = Sophus::SE3f;
    
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
     * @param should_fix Whether to fix this pose (will not be optimized)
     */
    void add_keyframe_pose(int keyframe_id, const SE3f& pose, bool should_fix = false);
    
    /**
     * @brief Add an odometry constraint between consecutive keyframes
     * @param from_keyframe_id Source keyframe ID
     * @param to_keyframe_id Target keyframe ID
     * @param relative_pose Relative pose from source to target
     * @param translation_weight Translation information weight (inverse variance)
     * @param rotation_weight Rotation information weight (inverse variance)
     */
    void add_odometry_constraint(int from_keyframe_id, int to_keyframe_id,
                                 const SE3f& relative_pose,
                                 double translation_weight = 1.0,
                                 double rotation_weight = 1.0);
    
    /**
     * @brief Add a loop closure constraint between non-consecutive keyframes
     * @param from_keyframe_id Source keyframe ID
     * @param to_keyframe_id Target keyframe ID
     * @param relative_pose Relative pose from source to target (from ICP)
     * @param translation_weight Translation information weight (inverse variance)
     * @param rotation_weight Rotation information weight (inverse variance)
     */
    void add_loop_closure_constraint(int from_keyframe_id, int to_keyframe_id,
                                     const SE3f& relative_pose,
                                     double translation_weight = 1.0,
                                     double rotation_weight = 1.0);
    
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
    size_t get_keyframe_count() const { return m_keyframe_poses.size(); }
    
    /**
     * @brief Get number of loop closure constraints
     */
    size_t get_loop_closure_count() const { return m_loop_closure_count; }
    
private:
    /**
     * @brief Structure to hold keyframe pose data
     */
    struct KeyframePose {
        int id;
        Eigen::Vector6d tangent_pose;  // SE3 in tangent space (tx,ty,tz,rx,ry,rz)
        bool is_fixed;
        
        KeyframePose(int id_, const Eigen::Vector6d& tangent_, bool fixed_)
            : id(id_), tangent_pose(tangent_), is_fixed(fixed_) {}
    };
    
    /**
     * @brief Structure to hold constraint data
     */
    struct Constraint {
        int from_id;
        int to_id;
        Sophus::SE3d relative_pose;
        double translation_weight;
        double rotation_weight;
        bool is_loop_closure;
        
        Constraint(int from, int to, const Sophus::SE3d& rel_pose,
                  double t_weight, double r_weight, bool is_loop)
            : from_id(from), to_id(to), relative_pose(rel_pose)
            , translation_weight(t_weight), rotation_weight(r_weight)
            , is_loop_closure(is_loop) {}
    };
    
    std::map<int, KeyframePose> m_keyframe_poses;  ///< Map from keyframe ID to pose
    std::vector<Constraint> m_constraints;         ///< All constraints (odometry + loop)
    
    size_t m_loop_closure_count;                   ///< Number of loop closure constraints
    bool m_is_optimized;                           ///< Whether optimization has been performed
};

} // namespace optimization
} // namespace lidar_odometry
