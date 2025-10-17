/**
 * @file      PoseGraphOptimizer.cpp
 * @brief     Implementation of Ceres-based pose graph optimization (Analytic Jacobian version).
 * @author    Seungwon Choi
 * @date      2025-10-17
 * @copyright Copyright (c) 2025 Seungwon Choi. All rights reserved.
 *
 * @par License
 * This project is released under the MIT License.
 */

#include "PoseGraphOptimizer.h"
#include <spdlog/spdlog.h>

namespace lidar_odometry {
namespace optimization {

using SE3f = Sophus::SE3f;

PoseGraphOptimizer::PoseGraphOptimizer()
    : m_loop_closure_count(0)
    , m_is_optimized(false) {
    m_problem = std::make_unique<ceres::Problem>();
}

PoseGraphOptimizer::~PoseGraphOptimizer() = default;

void PoseGraphOptimizer::add_keyframe_pose(int keyframe_id, const SE3f& pose, bool is_first_keyframe) {
    // Convert Sophus SE3f to tangent space parameters
    Eigen::Vector6d tangent = SE3GlobalParameterization::se3_to_tangent(pose.cast<double>());
    
    PoseBlock pose_block;
    std::copy(tangent.data(), tangent.data() + 6, pose_block.parameters.data());
    pose_block.is_fixed = is_first_keyframe;
    
    m_poses[keyframe_id] = pose_block;
    
    // Add parameter block to Ceres problem
    m_problem->AddParameterBlock(m_poses[keyframe_id].parameters.data(), 6);
    m_problem->SetParameterization(m_poses[keyframe_id].parameters.data(), 
                                   new SE3GlobalParameterization());
    
    // Fix first keyframe (acts as prior)
    if (is_first_keyframe) {
        m_problem->SetParameterBlockConstant(m_poses[keyframe_id].parameters.data());
        spdlog::info("[PGO] Added first keyframe {} (fixed as prior)", keyframe_id);
    } else {
        spdlog::debug("[PGO] Added keyframe {} with initial estimate", keyframe_id);
    }
    
    m_is_optimized = false;
}

void PoseGraphOptimizer::add_odometry_constraint(int from_keyframe_id, int to_keyframe_id,
                                                const SE3f& relative_pose,
                                                double translation_noise,
                                                double rotation_noise) {
    // Check if both keyframes exist
    if (m_poses.find(from_keyframe_id) == m_poses.end() ||
        m_poses.find(to_keyframe_id) == m_poses.end()) {
        spdlog::error("[PGO] Cannot add odometry constraint: keyframe {} or {} not found",
                     from_keyframe_id, to_keyframe_id);
        return;
    }
    
    // Create analytic Jacobian cost function
    RelativePoseFactor* factor = 
        new RelativePoseFactor(
            relative_pose.cast<double>(),
            translation_noise,
            rotation_noise
        );
    
    // Set constraint type for debugging
    if (std::abs(to_keyframe_id - from_keyframe_id) == 1) {
        factor->constraint_type = "odometry";
    } else {
        factor->constraint_type = "skip";
    }
    factor->from_kf_id = from_keyframe_id;
    factor->to_kf_id = to_keyframe_id;
    
    // Add residual block (no robust loss for odometry)
    m_problem->AddResidualBlock(
        factor,
        nullptr,
        m_poses[from_keyframe_id].parameters.data(),
        m_poses[to_keyframe_id].parameters.data()
    );
    
    spdlog::debug("[PGO] Added odometry constraint: {} -> {} (t_noise={:.3f}, r_noise={:.3f})",
                 from_keyframe_id, to_keyframe_id, translation_noise, rotation_noise);
    
    m_is_optimized = false;
}

void PoseGraphOptimizer::add_loop_closure_constraint(int from_keyframe_id, int to_keyframe_id,
                                                    const SE3f& relative_pose,
                                                    double translation_noise,
                                                    double rotation_noise) {
    // Check if both keyframes exist
    if (m_poses.find(from_keyframe_id) == m_poses.end() ||
        m_poses.find(to_keyframe_id) == m_poses.end()) {
        spdlog::error("[PGO] Cannot add loop closure constraint: keyframe {} or {} not found",
                     from_keyframe_id, to_keyframe_id);
        return;
    }
    
    // Create analytic Jacobian cost function
    RelativePoseFactor* factor = 
        new RelativePoseFactor(
            relative_pose.cast<double>(),
            translation_noise,
            rotation_noise
        );
    
    // Set constraint type for debugging
    factor->constraint_type = "loop";
    factor->from_kf_id = from_keyframe_id;
    factor->to_kf_id = to_keyframe_id;
    
    // Add residual block with Huber loss
    m_problem->AddResidualBlock(
        factor,
        new ceres::HuberLoss(1.0),
        m_poses[from_keyframe_id].parameters.data(),
        m_poses[to_keyframe_id].parameters.data()
    );
    
    m_loop_closure_count++;
    
    spdlog::info("[PGO] Added loop closure constraint: {} -> {} (total loops: {})",
                from_keyframe_id, to_keyframe_id, m_loop_closure_count);
    
    m_is_optimized = false;
}

bool PoseGraphOptimizer::optimize() {
    if (m_poses.empty()) {
        spdlog::warn("[PGO] Cannot optimize: no poses in graph");
        return false;
    }
    
    // Count constraints
    size_t num_residual_blocks = m_problem->NumResidualBlocks();
    size_t num_parameters = m_problem->NumParameters();
    size_t num_odometry = num_residual_blocks - m_loop_closure_count;
    
    spdlog::info("[PGO] Starting pose graph optimization with {} factors and {} poses",
                num_residual_blocks, num_parameters);
    spdlog::info("[PGO]   - Odometry constraints: {}", num_odometry);
    spdlog::info("[PGO]   - Loop closure constraints: {}", m_loop_closure_count);
    
    // Configure solver
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 100;
    options.minimizer_progress_to_stdout = false;  // We'll log ourselves
    options.num_threads = 4;
    
    // Solve
    ceres::Solver::Summary summary;
    ceres::Solve(options, m_problem.get(), &summary);
    
    // Log results with detailed cost information
    spdlog::info("[PGO] Optimization completed: initial cost={:.6f}, final cost={:.6f}, reduction={:.2f}%",
                summary.initial_cost, summary.final_cost,
                (summary.initial_cost > 0) ? (1.0 - summary.final_cost / summary.initial_cost) * 100.0 : 0.0);
    spdlog::info("[PGO]   - Cost change: {:.6f} -> {:.6f} (Δ={:.6f})",
                summary.initial_cost, summary.final_cost, summary.initial_cost - summary.final_cost);
    spdlog::info("[PGO]   - Iterations: {}, Total time: {:.3f}s",
                summary.iterations.size(), summary.total_time_in_seconds);
    
    if (!summary.IsSolutionUsable()) {
        spdlog::error("[PGO] Optimization failed: {}", summary.message);
        return false;
    }
    
    m_is_optimized = true;
    return true;
}

bool PoseGraphOptimizer::get_optimized_pose(int keyframe_id, SE3f& optimized_pose) const {
    auto it = m_poses.find(keyframe_id);
    if (it == m_poses.end()) {
        return false;
    }
    
    // Convert tangent space back to SE3
    Eigen::Vector6d tangent;
    std::copy(it->second.parameters.data(), 
              it->second.parameters.data() + 6, 
              tangent.data());
    
    Sophus::SE3d pose_d = SE3GlobalParameterization::tangent_to_se3(tangent);
    optimized_pose = pose_d.cast<float>();
    
    return true;
}

std::map<int, PoseGraphOptimizer::SE3f> PoseGraphOptimizer::get_all_optimized_poses() const {
    std::map<int, SE3f> result;
    
    for (const auto& pair : m_poses) {
        SE3f pose;
        if (get_optimized_pose(pair.first, pose)) {
            result[pair.first] = pose;
        }
    }
    
    return result;
}

void PoseGraphOptimizer::clear() {
    m_poses.clear();
    m_problem = std::make_unique<ceres::Problem>();
    m_loop_closure_count = 0;
    m_is_optimized = false;
    
    spdlog::info("[PGO] Pose graph cleared");
}

} // namespace optimization
} // namespace lidar_odometry
