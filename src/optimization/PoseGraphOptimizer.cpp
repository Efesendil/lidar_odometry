/**
 * @file      PoseGraphOptimizer.cpp
 * @brief     Implementation of GTSAM-based pose graph optimization.
 * @author    Seungwon Choi
 * @date      2025-10-17
 * @copyright Copyright (c) 2025 Seungwon Choi. All rights reserved.
 *
 * @par License
 * This project is released under the MIT License.
 */

#include "PoseGraphOptimizer.h"
#include <spdlog/spdlog.h>
#include <gtsam/geometry/Rot3.h>

namespace lidar_odometry {
namespace optimization {

using SE3f = Sophus::SE3f;

PoseGraphOptimizer::PoseGraphOptimizer()
    : m_loop_closure_count(0)
    , m_is_optimized(false)
{
}

PoseGraphOptimizer::~PoseGraphOptimizer() = default;

gtsam::Pose3 PoseGraphOptimizer::sophus_to_gtsam(const SE3f& pose) const {
    // Convert Sophus SE3f to GTSAM Pose3
    Eigen::Matrix3d R = pose.rotationMatrix().cast<double>();
    Eigen::Vector3d t = pose.translation().cast<double>();
    
    gtsam::Rot3 rotation(R);
    gtsam::Point3 translation(t.x(), t.y(), t.z());
    
    return gtsam::Pose3(rotation, translation);
}

SE3f PoseGraphOptimizer::gtsam_to_sophus(const gtsam::Pose3& pose) const {
    // Convert GTSAM Pose3 to Sophus SE3f
    Eigen::Matrix3d R = pose.rotation().matrix();
    Eigen::Vector3d t = pose.translation().vector();
    
    Eigen::Matrix3f R_f = R.cast<float>();
    Eigen::Vector3f t_f = t.cast<float>();
    
    return SE3f(R_f, t_f);
}

void PoseGraphOptimizer::add_keyframe_pose(int keyframe_id, const SE3f& pose, bool is_first_keyframe) {
    gtsam::Symbol symbol('x', keyframe_id);
    gtsam::Pose3 gtsam_pose = sophus_to_gtsam(pose);
    
    // Add initial estimate
    m_initial_estimates.insert(symbol, gtsam_pose);
    
    // Add prior factor for the first keyframe (anchor the pose graph)
    if (is_first_keyframe) {
        auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished()
        );
        m_graph.add(gtsam::PriorFactor<gtsam::Pose3>(symbol, gtsam_pose, prior_noise));
        
        spdlog::info("[PGO] Added first keyframe {} with prior factor", keyframe_id);
    }
    
    m_is_optimized = false;
}

void PoseGraphOptimizer::add_odometry_constraint(int from_keyframe_id, int to_keyframe_id,
                                                const SE3f& relative_pose,
                                                double translation_noise,
                                                double rotation_noise) {
    gtsam::Symbol from_symbol('x', from_keyframe_id);
    gtsam::Symbol to_symbol('x', to_keyframe_id);
    
    gtsam::Pose3 gtsam_relative_pose = sophus_to_gtsam(relative_pose);
    
    // Create noise model (6D: 3 for rotation, 3 for translation)
    auto noise_model = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << rotation_noise, rotation_noise, rotation_noise,
                            translation_noise, translation_noise, translation_noise).finished()
    );
    
    // Add BetweenFactor (odometry constraint)
    m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
        from_symbol, to_symbol, gtsam_relative_pose, noise_model
    ));
    
    spdlog::debug("[PGO] Added odometry constraint: {} -> {}", from_keyframe_id, to_keyframe_id);
    
    m_is_optimized = false;
}

void PoseGraphOptimizer::add_loop_closure_constraint(int from_keyframe_id, int to_keyframe_id,
                                                    const SE3f& corrected_to_keyframe_pose,
                                                    double translation_noise,
                                                    double rotation_noise) {
    gtsam::Symbol from_symbol('x', from_keyframe_id);
    gtsam::Symbol to_symbol('x', to_keyframe_id);
    
    // Get the current estimate for from_keyframe
    gtsam::Pose3 from_pose = m_initial_estimates.at<gtsam::Pose3>(from_symbol);
    
    // Convert corrected pose to GTSAM
    gtsam::Pose3 corrected_to_pose = sophus_to_gtsam(corrected_to_keyframe_pose);
    
    // Compute relative pose: T_from_to = T_from^-1 * T_to_corrected
    gtsam::Pose3 relative_pose = from_pose.inverse() * corrected_to_pose;
    
    // Create noise model with tighter constraints (loop closures are more accurate from ICP)
    auto noise_model = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << rotation_noise, rotation_noise, rotation_noise,
                            translation_noise, translation_noise, translation_noise).finished()
    );
    
    // Add BetweenFactor (loop closure constraint)
    m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
        from_symbol, to_symbol, relative_pose, noise_model
    ));
    
    // Update the initial estimate for to_keyframe with corrected pose
    m_initial_estimates.update(to_symbol, corrected_to_pose);
    
    m_loop_closure_count++;
    
    spdlog::info("[PGO] Added loop closure constraint: {} -> {} (total loops: {})",
                from_keyframe_id, to_keyframe_id, m_loop_closure_count);
    
    m_is_optimized = false;
}

bool PoseGraphOptimizer::optimize() {
    if (m_graph.empty() || m_initial_estimates.empty()) {
        spdlog::warn("[PGO] Cannot optimize: empty graph or initial estimates");
        return false;
    }
    
    spdlog::info("[PGO] Starting pose graph optimization with {} factors and {} poses",
                m_graph.size(), m_initial_estimates.size());
    
    try {
        // Create Levenberg-Marquardt optimizer
        gtsam::LevenbergMarquardtParams params;
        params.setVerbosity("ERROR");  // Suppress GTSAM output
        params.setMaxIterations(100);
        params.setRelativeErrorTol(1e-5);
        params.setAbsoluteErrorTol(1e-5);
        
        gtsam::LevenbergMarquardtOptimizer optimizer(m_graph, m_initial_estimates, params);
        
        // Optimize
        m_optimized_estimates = optimizer.optimize();
        
        // Calculate error reduction
        double initial_error = m_graph.error(m_initial_estimates);
        double final_error = m_graph.error(m_optimized_estimates);
        
        spdlog::info("[PGO] Optimization completed: initial error={:.6f}, final error={:.6f}, reduction={:.2f}%",
                    initial_error, final_error, (1.0 - final_error / initial_error) * 100.0);
        
        m_is_optimized = true;
        return true;
        
    } catch (const std::exception& e) {
        spdlog::error("[PGO] Optimization failed: {}", e.what());
        return false;
    }
}

bool PoseGraphOptimizer::get_optimized_pose(int keyframe_id, SE3f& optimized_pose) const {
    if (!m_is_optimized) {
        spdlog::warn("[PGO] No optimized poses available (optimization not performed)");
        return false;
    }
    
    gtsam::Symbol symbol('x', keyframe_id);
    
    try {
        gtsam::Pose3 gtsam_pose = m_optimized_estimates.at<gtsam::Pose3>(symbol);
        optimized_pose = gtsam_to_sophus(gtsam_pose);
        return true;
    } catch (const std::exception& e) {
        spdlog::warn("[PGO] Failed to get optimized pose for keyframe {}: {}", keyframe_id, e.what());
        return false;
    }
}

std::map<int, SE3f> PoseGraphOptimizer::get_all_optimized_poses() const {
    std::map<int, SE3f> optimized_poses;
    
    if (!m_is_optimized) {
        spdlog::warn("[PGO] No optimized poses available (optimization not performed)");
        return optimized_poses;
    }
    
    // Iterate through all keys in optimized estimates
    for (const auto& key_value : m_optimized_estimates) {
        gtsam::Symbol symbol(key_value.key);
        
        // Check if this is a pose symbol (character 'x')
        if (symbol.chr() == 'x') {
            int keyframe_id = symbol.index();
            gtsam::Pose3 gtsam_pose = m_optimized_estimates.at<gtsam::Pose3>(key_value.key);
            optimized_poses[keyframe_id] = gtsam_to_sophus(gtsam_pose);
        }
    }
    
    return optimized_poses;
}

void PoseGraphOptimizer::clear() {
    m_graph = gtsam::NonlinearFactorGraph();
    m_initial_estimates.clear();
    m_optimized_estimates.clear();
    m_loop_closure_count = 0;
    m_is_optimized = false;
    
    spdlog::info("[PGO] Pose graph cleared");
}

size_t PoseGraphOptimizer::get_keyframe_count() const {
    return m_initial_estimates.size();
}

} // namespace optimization
} // namespace lidar_odometry
