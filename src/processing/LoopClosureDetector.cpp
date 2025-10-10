/**
 * @file      LoopClosureDetector.cpp
 * @brief     Loop closure detection using LiDAR Iris features
 * @author    Seungwon Choi
 * @date      2025-10-10
 * @copyright Copyright (c) 2025 Seungwon Choi. All rights reserved.
 *
 * @par License
 * This project is released under the MIT License.
 */

#include "LoopClosureDetector.h"
#include <algorithm>
#include <chrono>

namespace lidar_odometry {
namespace processing {

LoopClosureDetector::LoopClosureDetector(const LoopClosureConfig& config)
    : m_config(config) {
    
    // Initialize LiDAR Iris detector with standard parameters
    // Note: range filtering will be done in the point cloud processing stage
    m_iris = std::make_unique<LidarIris>(
        4,    // nscale: number of filter scales
        18,   // minWaveLength: minimum wavelength
        2.1f, // mult: wavelength multiplier
        0.75f,// sigmaOnf: bandwidth parameter
        2     // matchNum: both forward and reverse directions
    );
    
    spdlog::info("[LoopClosureDetector] Initialized with similarity_threshold={:.3f}, min_gap={}, max_distance={:.1f}m",
                 m_config.similarity_threshold, m_config.min_keyframe_gap, m_config.max_search_distance);
}

LoopClosureDetector::~LoopClosureDetector() {
    spdlog::info("[LoopClosureDetector] Statistics: {} queries, {} candidates found",
                 m_total_queries, m_total_candidates);
}

bool LoopClosureDetector::add_keyframe(std::shared_ptr<database::LidarFrame> keyframe) {
    if (!keyframe) {
        spdlog::warn("[LoopClosureDetector] Null keyframe provided");
        return false;
    }
    
    try {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Convert to SimplePointCloud format
        auto simple_cloud = convert_to_simple_cloud(keyframe);
        
        if (simple_cloud.empty()) {
            spdlog::warn("[LoopClosureDetector] Empty point cloud for keyframe {}", keyframe->get_keyframe_id());
            return false;
        }
        
        // Extract LiDAR Iris feature
        auto feature = extract_iris_feature(simple_cloud);
        
        // Store feature, keyframe ID, and position
        m_feature_database.push_back(feature);
        m_keyframe_ids.push_back(keyframe->get_keyframe_id());  // Use keyframe ID, not frame ID
        m_keyframe_positions.push_back(keyframe->get_pose().translation());
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        
        if (m_config.enable_debug_output) {
            spdlog::debug("[LoopClosureDetector] Added keyframe {} - feature extraction: {}ms, total features: {}",
                         keyframe->get_keyframe_id(), duration, m_feature_database.size());
        }
        
        return true;
        
    } catch (const std::exception& e) {
        spdlog::error("[LoopClosureDetector] Exception adding keyframe {}: {}", 
                     keyframe->get_keyframe_id(), e.what());
        return false;
    }
}

std::vector<LoopCandidate> LoopClosureDetector::detect_loop_closures(
    std::shared_ptr<database::LidarFrame> current_keyframe) {
    
    std::vector<LoopCandidate> candidates;
    
    if (!m_config.enable_loop_detection) {
        return candidates;
    }
    
    if (!current_keyframe) {
        spdlog::warn("[LoopClosureDetector] Null current keyframe provided");
        return candidates;
    }
    
    m_total_queries++;
    
    try {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Convert current keyframe to SimplePointCloud format
        auto simple_cloud = convert_to_simple_cloud(current_keyframe);
        
        if (simple_cloud.empty()) {
            spdlog::warn("[LoopClosureDetector] Empty point cloud for current keyframe {}", 
                        current_keyframe->get_keyframe_id());
            return candidates;
        }
        
        // Extract feature for current keyframe
        auto current_feature = extract_iris_feature(simple_cloud);
        
        // Search for loop closure candidates
        size_t current_id = current_keyframe->get_keyframe_id();  // Use keyframe ID, not frame ID
        Eigen::Vector3f current_position = current_keyframe->get_pose().translation();
        float min_similarity = 999.0f;
        std::vector<std::pair<float, size_t>> similarity_scores;
        
        for (size_t i = 0; i < m_feature_database.size(); ++i) {
            size_t candidate_id = m_keyframe_ids[i];
            

            // Check minimum keyframe gap
            if (static_cast<int>(current_id) - static_cast<int>(candidate_id) < m_config.min_keyframe_gap) {
                continue;
            }
            
            // Check distance constraint
            const Eigen::Vector3f& candidate_position = m_keyframe_positions[i];
            float distance = (current_position - candidate_position).norm();
            if (distance > m_config.max_search_distance) {

                continue;
            }
            
            // Compare features
            int bias = 0;
            float similarity = m_iris->Compare(current_feature, m_feature_database[i], &bias);


            
            similarity_scores.push_back({similarity, i});
            min_similarity = std::min(min_similarity, similarity);
        }
        
        // Sort by similarity (lower is better)
        std::sort(similarity_scores.begin(), similarity_scores.end());
        
        // Select only the best candidate that meets threshold
        for (const auto& score_pair : similarity_scores) {
            float similarity = score_pair.first;
            size_t db_index = score_pair.second;
            
            if (similarity > m_config.similarity_threshold) {
                break; // No valid candidates
            }
            
            // Get rotational bias
            int bias = 0;
            m_iris->Compare(current_feature, m_feature_database[db_index], &bias);
            
            LoopCandidate candidate(current_id, m_keyframe_ids[db_index], similarity, bias);
            candidates.push_back(candidate);
            break; // Only take the best candidate
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        
        m_total_candidates += candidates.size();
        
        if (!candidates.empty()) {
            spdlog::info("[LoopClosureDetector] Found {} loop candidates for keyframe {} (search time: {}ms)",
                        candidates.size(), current_id, duration);
            
            for (const auto& candidate : candidates) {
                spdlog::info("  -> Candidate: {} <-> {} (distance: {:.4f}, bias: {})",
                           candidate.query_keyframe_id, candidate.match_keyframe_id,
                           candidate.similarity_score, candidate.bias);
            }
        } else if (m_config.enable_debug_output) {
            spdlog::debug("[LoopClosureDetector] No loop candidates found for keyframe {} (min_distance: {:.4f}, search time: {}ms)",
                         current_id, min_similarity, duration);
        }
        
    } catch (const std::exception& e) {
        spdlog::error("[LoopClosureDetector] Exception detecting loops for keyframe {}: {}", 
                     current_keyframe->get_keyframe_id(), e.what());
    }
    
    return candidates;
}

void LoopClosureDetector::update_config(const LoopClosureConfig& config) {
    m_config = config;
    spdlog::info("[LoopClosureDetector] Configuration updated: threshold={:.3f}, min_gap={}",
                 m_config.similarity_threshold, m_config.min_keyframe_gap);
}

void LoopClosureDetector::clear() {
    m_feature_database.clear();
    m_keyframe_ids.clear();
    m_total_queries = 0;
    m_total_candidates = 0;
    spdlog::info("[LoopClosureDetector] Database cleared");
}

SimplePointCloud LoopClosureDetector::convert_to_simple_cloud(
    std::shared_ptr<database::LidarFrame> lidar_frame) {
    
    SimplePointCloud simple_cloud;
    
    // Use local map instead of raw cloud for more refined and consistent features
    auto local_map = lidar_frame->get_local_map();
    if (!local_map || local_map->empty()) {
        spdlog::warn("[LoopClosureDetector] No local map available for keyframe {}, falling back to feature cloud", 
                    lidar_frame->get_keyframe_id());
        
        // Fallback to feature cloud if local map is not available
        auto feature_cloud = lidar_frame->get_feature_cloud_global();
        if (!feature_cloud || feature_cloud->empty()) {
            return simple_cloud;
        }
        
        simple_cloud.reserve(feature_cloud->size());
        for (const auto& point : *feature_cloud) {
            simple_cloud.emplace_back(point.x, point.y, point.z);
        }
    } else {
        simple_cloud.reserve(local_map->size());
        for (const auto& point : *local_map) {
            simple_cloud.emplace_back(point.x, point.y, point.z);
        }
    }
    
    return simple_cloud;
}

LidarIris::FeatureDesc LoopClosureDetector::extract_iris_feature(
    const SimplePointCloud& point_cloud) {
    
    // Generate LiDAR Iris image directly from SimplePointCloud
    cv::Mat1b iris_image = LidarIris::GetIris(point_cloud);
    
    // Extract feature descriptor
    LidarIris::FeatureDesc feature = m_iris->GetFeature(iris_image);
    
    return feature;
}

} // namespace processing
} // namespace lidar_odometry