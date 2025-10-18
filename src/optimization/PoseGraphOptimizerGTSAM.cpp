/**
 * @file      PoseGraphOptimizerGTSAM.cpp
 * @brief     Implementation of GTSAM-based pose graph optimization.
 * @author    Seungwon Choi
 * @date      2025-10-17
 * @copyright Copyright (c) 2025 Seungwon Choi. All rights reserved.
 *
 * @par License
 * This project is released under the MIT License.
 */

#include "PoseGraphOptimizerGTSAM.h"
#include <spdlog/spdlog.h>
#include <gtsam/inference/Symbol.h>

namespace lidar_odometry {
namespace optimization {

using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

PoseGraphOptimizerGTSAM::PoseGraphOptimizerGTSAM()
    : m_loop_closure_count(0)
    , m_is_optimized(false) {
}

PoseGraphOptimizerGTSAM::~PoseGraphOptimizerGTSAM() = default;

gtsam::Pose3 PoseGraphOptimizerGTSAM::sophus_to_gtsam(const Sophus::SE3d& pose) {
    // Extract rotation (quaternion) and translation
    Eigen::Quaterniond quat(pose.unit_quaternion());
    Eigen::Vector3d trans = pose.translation();
    
    // Create GTSAM Pose3
    gtsam::Rot3 rotation(quat.w(), quat.x(), quat.y(), quat.z());
    gtsam::Point3 translation(trans.x(), trans.y(), trans.z());
    
    return gtsam::Pose3(rotation, translation);
}

Sophus::SE3d PoseGraphOptimizerGTSAM::gtsam_to_sophus(const gtsam::Pose3& pose) {
    // Extract rotation and translation from GTSAM Pose3
    gtsam::Quaternion gtsam_quat = pose.rotation().toQuaternion();
    gtsam::Point3 gtsam_trans = pose.translation();
    
    // Create Sophus SE3
    Eigen::Quaterniond quat(gtsam_quat.w(), gtsam_quat.x(), gtsam_quat.y(), gtsam_quat.z());
    Eigen::Vector3d trans(gtsam_trans.x(), gtsam_trans.y(), gtsam_trans.z());
    
    return Sophus::SE3d(quat, trans);
}

gtsam::noiseModel::Diagonal::shared_ptr PoseGraphOptimizerGTSAM::create_noise_model(
    double translation_noise, double rotation_noise) {
    // GTSAM uses standard deviation directly
    // Create 6D noise: [roll, pitch, yaw, x, y, z]
    gtsam::Vector6 sigmas;
    sigmas << rotation_noise, rotation_noise, rotation_noise,
              translation_noise, translation_noise, translation_noise;
    
    return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

void PoseGraphOptimizerGTSAM::add_keyframe_pose(int keyframe_id, const SE3f& pose, bool should_fix) {
    // Convert to GTSAM Pose3
    gtsam::Pose3 gtsam_pose = sophus_to_gtsam(pose.cast<double>());
    
    // Add to initial estimates
    m_initial_estimates.insert(X(keyframe_id), gtsam_pose);
    
    // Track fixed keyframes
    if (should_fix) {
        m_fixed_keyframes.insert(keyframe_id);
        
        // Add strong prior factor to fix the pose
        auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished()
        );
        m_graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
            X(keyframe_id), gtsam_pose, prior_noise
        );
        
        spdlog::debug("[PGO-GTSAM] Added keyframe {} (fixed)", keyframe_id);
    } else {
        spdlog::debug("[PGO-GTSAM] Added keyframe {} (optimizable)", keyframe_id);
    }
    
    m_is_optimized = false;
}

void PoseGraphOptimizerGTSAM::add_odometry_constraint(int from_keyframe_id, int to_keyframe_id,
                                                      const SE3f& relative_pose,
                                                      double translation_noise,
                                                      double rotation_noise) {
    // Check if both keyframes exist
    if (!m_initial_estimates.exists(X(from_keyframe_id)) ||
        !m_initial_estimates.exists(X(to_keyframe_id))) {
        spdlog::error("[PGO-GTSAM] Cannot add odometry constraint: keyframe {} or {} not found",
                     from_keyframe_id, to_keyframe_id);
        return;
    }
    
    // Convert relative pose to GTSAM
    gtsam::Pose3 gtsam_relative = sophus_to_gtsam(relative_pose.cast<double>());
    
    // Create noise model
    auto noise = create_noise_model(translation_noise, rotation_noise);
    
    // Add BetweenFactor
    m_graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        X(from_keyframe_id), X(to_keyframe_id), gtsam_relative, noise
    );
    
    spdlog::debug("[PGO-GTSAM] Added odometry constraint: {} -> {} (t_noise={:.3f}, r_noise={:.3f})",
                 from_keyframe_id, to_keyframe_id, translation_noise, rotation_noise);
    
    m_is_optimized = false;
}

void PoseGraphOptimizerGTSAM::add_loop_closure_constraint(int from_keyframe_id, int to_keyframe_id,
                                                          const SE3f& relative_pose,
                                                          double translation_noise,
                                                          double rotation_noise) {
    // Check if both keyframes exist
    if (!m_initial_estimates.exists(X(from_keyframe_id)) ||
        !m_initial_estimates.exists(X(to_keyframe_id))) {
        spdlog::error("[PGO-GTSAM] Cannot add loop closure constraint: keyframe {} or {} not found",
                     from_keyframe_id, to_keyframe_id);
        return;
    }
    
    // Convert relative pose to GTSAM
    gtsam::Pose3 gtsam_relative = sophus_to_gtsam(relative_pose.cast<double>());
    
    // Create noise model (can be different from odometry)
    auto noise = create_noise_model(translation_noise, rotation_noise);
    
    // Add BetweenFactor with robust kernel
    auto robust_noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.0),
        noise
    );
    
    m_graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        X(from_keyframe_id), X(to_keyframe_id), gtsam_relative, robust_noise
    );
    
    m_loop_closure_count++;
    
    spdlog::info("[PGO-GTSAM] Added loop closure constraint: {} -> {} (total loops: {})",
                from_keyframe_id, to_keyframe_id, m_loop_closure_count);
    
    m_is_optimized = false;
}

bool PoseGraphOptimizerGTSAM::optimize() {
    if (m_initial_estimates.empty()) {
        spdlog::warn("[PGO-GTSAM] Cannot optimize: no poses in graph");
        return false;
    }
    
    // Count constraints
    size_t num_factors = m_graph.size();
    size_t num_poses = m_initial_estimates.size();
    size_t num_odometry = num_factors - m_loop_closure_count - m_fixed_keyframes.size();
    
    spdlog::info("[PGO-GTSAM] Starting pose graph optimization with {} factors and {} poses",
                num_factors, num_poses);
    spdlog::info("[PGO-GTSAM]   - Fixed poses: {}", m_fixed_keyframes.size());
    spdlog::info("[PGO-GTSAM]   - Odometry constraints: {}", num_odometry);
    spdlog::info("[PGO-GTSAM]   - Loop closure constraints: {}", m_loop_closure_count);
    
    // Calculate initial error
    double initial_error = m_graph.error(m_initial_estimates);
    
    // Optimize using Levenberg-Marquardt
    gtsam::LevenbergMarquardtParams params;
    params.setVerbosity("SILENT");  // Completely suppress GTSAM output
    params.setMaxIterations(100);
    params.setRelativeErrorTol(1e-5);
    params.setAbsoluteErrorTol(1e-5);
    
    try {
        gtsam::LevenbergMarquardtOptimizer optimizer(m_graph, m_initial_estimates, params);
        m_optimized_values = optimizer.optimize();
        
        // Calculate final error
        double final_error = m_graph.error(m_optimized_values);
        double reduction = (initial_error > 0) ? (1.0 - final_error / initial_error) * 100.0 : 0.0;
        
        spdlog::info("[PGO-GTSAM] Optimization completed: initial error={:.6f}, final error={:.6f}, reduction={:.2f}%",
                    initial_error, final_error, reduction);
        spdlog::info("[PGO-GTSAM]   - Error change: {:.6f} -> {:.6f} (Δ={:.6f})",
                    initial_error, final_error, initial_error - final_error);
        spdlog::info("[PGO-GTSAM]   - Iterations: {}", optimizer.iterations());
        
        m_is_optimized = true;
        return true;
        
    } catch (const std::exception& e) {
        spdlog::error("[PGO-GTSAM] Optimization failed: {}", e.what());
        return false;
    }
}

bool PoseGraphOptimizerGTSAM::get_optimized_pose(int keyframe_id, SE3f& optimized_pose) const {
    if (!m_is_optimized || !m_optimized_values.exists(X(keyframe_id))) {
        return false;
    }
    
    // Extract optimized pose
    gtsam::Pose3 gtsam_pose = m_optimized_values.at<gtsam::Pose3>(X(keyframe_id));
    
    // Convert to Sophus and cast to float
    Sophus::SE3d pose_d = gtsam_to_sophus(gtsam_pose);
    optimized_pose = pose_d.cast<float>();
    
    return true;
}

std::map<int, PoseGraphOptimizerGTSAM::SE3f> PoseGraphOptimizerGTSAM::get_all_optimized_poses() const {
    std::map<int, SE3f> result;
    
    if (!m_is_optimized) {
        return result;
    }
    
    // Iterate through all optimized values
    for (const auto& key_value : m_optimized_values) {
        gtsam::Key key = key_value.key;
        
        // Check if this is a pose key (symbol 'x')
        if (gtsam::Symbol(key).chr() == 'x') {
            int keyframe_id = gtsam::Symbol(key).index();
            SE3f pose;
            if (get_optimized_pose(keyframe_id, pose)) {
                result[keyframe_id] = pose;
            }
        }
    }
    
    return result;
}

void PoseGraphOptimizerGTSAM::clear() {
    m_graph = gtsam::NonlinearFactorGraph();
    m_initial_estimates.clear();
    m_optimized_values.clear();
    m_fixed_keyframes.clear();
    m_loop_closure_count = 0;
    m_is_optimized = false;
    
    spdlog::info("[PGO-GTSAM] Pose graph cleared");
}

} // namespace optimization
} // namespace lidar_odometry
