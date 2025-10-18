/**
 * @file      PoseGraphOptimizerCeres.cpp
 * @brief     Implementation of Ceres-based pose graph optimization.
 * @author    Seungwon Choi
 * @date      2025-10-18
 * @copyright Copyright (c) 2025 Seungwon Choi. All rights reserved.
 *
 * @par License
 * This project is released under the MIT License.
 */

#include "PoseGraphOptimizerCeres.h"
#include "AdaptiveMEstimator.h"
#include <spdlog/spdlog.h>
#include <ceres/ceres.h>

namespace lidar_odometry {
namespace optimization {

PoseGraphOptimizerCeres::PoseGraphOptimizerCeres()
    : m_loop_closure_count(0)
    , m_is_optimized(false) {
}

PoseGraphOptimizerCeres::~PoseGraphOptimizerCeres() = default;

void PoseGraphOptimizerCeres::add_keyframe_pose(int keyframe_id, const SE3f& pose, bool should_fix) {
    // Convert SE3 to tangent space
    Sophus::SE3d pose_d = pose.cast<double>();
    Eigen::Vector6d tangent = pose_d.log();
    
    // Add to keyframe poses map
    m_keyframe_poses.emplace(keyframe_id, KeyframePose(keyframe_id, tangent, should_fix));
    
    if (should_fix) {
        spdlog::debug("[PGO-Ceres] Added keyframe {} (fixed)", keyframe_id);
    } else {
        spdlog::debug("[PGO-Ceres] Added keyframe {} (optimizable)", keyframe_id);
    }
    
    m_is_optimized = false;
}

void PoseGraphOptimizerCeres::add_odometry_constraint(int from_keyframe_id, int to_keyframe_id,
                                                      const SE3f& relative_pose,
                                                      double translation_weight,
                                                      double rotation_weight) {
    // Check if both keyframes exist
    if (m_keyframe_poses.find(from_keyframe_id) == m_keyframe_poses.end() ||
        m_keyframe_poses.find(to_keyframe_id) == m_keyframe_poses.end()) {
        spdlog::error("[PGO-Ceres] Cannot add odometry constraint: keyframe {} or {} not found",
                     from_keyframe_id, to_keyframe_id);
        return;
    }
    
    // Add constraint
    m_constraints.emplace_back(
        from_keyframe_id, to_keyframe_id, relative_pose.cast<double>(),
        translation_weight, rotation_weight, false
    );
    
    spdlog::debug("[PGO-Ceres] Added odometry constraint: {} -> {} (t_weight={:.3f}, r_weight={:.3f})",
                 from_keyframe_id, to_keyframe_id, translation_weight, rotation_weight);
    
    m_is_optimized = false;
}

void PoseGraphOptimizerCeres::add_loop_closure_constraint(int from_keyframe_id, int to_keyframe_id,
                                                          const SE3f& relative_pose,
                                                          double translation_weight,
                                                          double rotation_weight) {
    // Check if both keyframes exist
    if (m_keyframe_poses.find(from_keyframe_id) == m_keyframe_poses.end() ||
        m_keyframe_poses.find(to_keyframe_id) == m_keyframe_poses.end()) {
        spdlog::error("[PGO-Ceres] Cannot add loop closure constraint: keyframe {} or {} not found",
                     from_keyframe_id, to_keyframe_id);
        return;
    }
    
    // Add constraint with loop closure flag
    m_constraints.emplace_back(
        from_keyframe_id, to_keyframe_id, relative_pose.cast<double>(),
        translation_weight, rotation_weight, true
    );
    
    m_loop_closure_count++;
    
    spdlog::info("[PGO-Ceres] Added loop closure constraint: {} -> {} (total loops: {})",
                from_keyframe_id, to_keyframe_id, m_loop_closure_count);
    
    m_is_optimized = false;
}

bool PoseGraphOptimizerCeres::optimize() {
    if (m_keyframe_poses.empty()) {
        spdlog::warn("[PGO-Ceres] Cannot optimize: no keyframes added");
        return false;
    }
    
    if (m_constraints.empty()) {
        spdlog::warn("[PGO-Ceres] Cannot optimize: no constraints added");
        return false;
    }
    
    spdlog::info("[PGO-Ceres] Starting pose graph optimization:");
    spdlog::info("  - Keyframes: {}", m_keyframe_poses.size());
    spdlog::info("  - Constraints: {} (odometry: {}, loop: {})",
                 m_constraints.size(), m_constraints.size() - m_loop_closure_count, m_loop_closure_count);
    
    // Build Ceres problem
    ceres::Problem problem;
    
    // Set up parameterization (use global SE3 parameterization)
    auto* se3_parameterization = new SE3GlobalParameterization();
    
    // Add all keyframe poses as parameters
    for (auto& [id, kf] : m_keyframe_poses) {
        problem.AddParameterBlock(kf.tangent_pose.data(), 6, se3_parameterization);
        
        // Fix if needed
        if (kf.is_fixed) {
            problem.SetParameterBlockConstant(kf.tangent_pose.data());
        }
    }
    
    // Add all constraints as residual blocks
    int odometry_count = 0;
    int loop_count = 0;
    
    for (const auto& constraint : m_constraints) {
        // Get pointers to pose parameters
        double* pose_i = m_keyframe_poses.at(constraint.from_id).tangent_pose.data();
        double* pose_j = m_keyframe_poses.at(constraint.to_id).tangent_pose.data();
        
        // Create RelativePoseFactor cost function
        auto* cost_function = new RelativePoseFactor(
            constraint.relative_pose,
            constraint.translation_weight,
            constraint.rotation_weight
        );
        
        // Add residual block without robust loss function
        if (constraint.is_loop_closure) {
            // No loss function for loop closures (trust ICP result directly)
            problem.AddResidualBlock(cost_function, nullptr, pose_i, pose_j);
            loop_count++;
        } else {
            // No loss function for odometry (trust odometry more)
            problem.AddResidualBlock(cost_function, nullptr, pose_i, pose_j);
            odometry_count++;
        }
    }
    
    spdlog::info("[PGO-Ceres] Built problem with {} odometry and {} loop constraints",
                 odometry_count, loop_count);
    
    // Configure solver
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.max_num_iterations = 100;
    options.function_tolerance = 1e-6;
    options.gradient_tolerance = 1e-10;
    options.parameter_tolerance = 1e-8;
    options.minimizer_progress_to_stdout = false;
    options.num_threads = 8;
    
    // Solve
    ceres::Solver::Summary summary;
    auto start = std::chrono::high_resolution_clock::now();
    ceres::Solve(options, &problem, &summary);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    // Log results
    spdlog::info("[PGO-Ceres] Optimization finished in {:.3f} seconds", duration.count() / 1000.0);
    spdlog::info("  - Initial cost: {:.6f}", summary.initial_cost);
    spdlog::info("  - Final cost: {:.6f}", summary.final_cost);
    spdlog::info("  - Cost reduction: {:.2f}%", 
                 100.0 * (1.0 - summary.final_cost / summary.initial_cost));
    spdlog::info("  - Iterations: {}", summary.iterations.size());
    spdlog::info("  - Termination: {}", ceres::TerminationTypeToString(summary.termination_type));
    
    m_is_optimized = true;
    
    return summary.IsSolutionUsable();
}

bool PoseGraphOptimizerCeres::get_optimized_pose(int keyframe_id, SE3f& optimized_pose) const {
    auto it = m_keyframe_poses.find(keyframe_id);
    if (it == m_keyframe_poses.end()) {
        return false;
    }
    
    // Convert tangent space back to SE3
    Sophus::SE3d pose_d = Sophus::SE3d::exp(it->second.tangent_pose);
    optimized_pose = pose_d.cast<float>();
    
    return true;
}

std::map<int, PoseGraphOptimizerCeres::SE3f> PoseGraphOptimizerCeres::get_all_optimized_poses() const {
    std::map<int, SE3f> result;
    
    for (const auto& [id, kf] : m_keyframe_poses) {
        Sophus::SE3d pose_d = Sophus::SE3d::exp(kf.tangent_pose);
        result[id] = pose_d.cast<float>();
    }
    
    return result;
}

void PoseGraphOptimizerCeres::clear() {
    m_keyframe_poses.clear();
    m_constraints.clear();
    m_loop_closure_count = 0;
    m_is_optimized = false;
    
    spdlog::debug("[PGO-Ceres] Cleared pose graph");
}

} // namespace optimization
} // namespace lidar_odometry
