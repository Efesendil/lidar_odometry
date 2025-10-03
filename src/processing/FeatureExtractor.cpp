/**
 * @file      FeatureExtractor.cpp
 * @brief     Implementation of feature extraction for LiDAR point clouds.
 * @author    Seungwon Choi
 * @date      2025-09-24
 * @copyright Copyright (c) 2025 Seungwon Choi. All rights reserved.
 *
 * @par License
 * This project is released under the MIT License.
 */

#include "FeatureExtractor.h"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cmath>
#include <chrono>

namespace lidar_odometry {
namespace processing {

FeatureExtractor::FeatureExtractor(const FeatureExtractorConfig& config)
    : m_config(config) {
    spdlog::info("[FeatureExtractor] Initialized with max_neighbors: {}, plane_distance: {}", 
                 m_config.max_neighbors, m_config.max_plane_distance);
}

// ===== Feature Extraction Methods =====

size_t FeatureExtractor::extract_features(const PointCloudConstPtr& input_cloud,
                                         PointCloudPtr& output_features) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    if (!input_cloud || input_cloud->empty()) {
        spdlog::warn("[FeatureExtractor] Invalid input cloud for feature extraction");
        return 0;
    }
    
    // Initialize output cloud
    output_features = PointCloudPtr(new PointCloud());
    output_features->reserve(input_cloud->size() / 10); // Rough estimate
    
    // Build KdTree for input cloud
    util::KdTree kdtree;
    kdtree.setInputCloud(input_cloud);
    
    size_t valid_feature_count = 0;
    std::vector<bool> processed(input_cloud->size(), false);
    
    // Extract plane features from point cloud
    for (size_t i = 0; i < input_cloud->size(); ++i) {
        if (processed[i]) {
            continue; // Skip already processed points
        }
        
        const PointType& query_point = input_cloud->at(i);
        
        // Find neighbors for current point
        std::vector<int> neighbor_indices;
        std::vector<float> neighbor_distances;
        
        int neighbor_count = kdtree.nearestKSearch(query_point, m_config.max_neighbors, 
                                                   neighbor_indices, neighbor_distances);
        
        if (neighbor_count < static_cast<int>(m_config.min_plane_points)) {
            processed[i] = true;
            continue; // Not enough neighbors for plane fitting
        }
        
        // Extract neighbor points
        std::vector<Vector3f> neighbor_points;
        for (int j = 0; j < neighbor_count; ++j) {
            const PointType& pt = input_cloud->at(neighbor_indices[j]);
            neighbor_points.emplace_back(pt.x, pt.y, pt.z);
        }
        
        // Fit plane to neighbor points
        Vector3f plane_normal, plane_center;
        if (fit_plane_to_points(neighbor_points, plane_normal, plane_center)) {
            // Validate plane quality
            float plane_fitness = compute_plane_fitness(neighbor_points, plane_normal, plane_center);
            
            if (plane_fitness < m_config.max_plane_distance) {
                // Add representative point to output features
                output_features->push_back(query_point);
                valid_feature_count++;
                
                // Mark processed points to avoid duplicate features
                for (int idx : neighbor_indices) {
                    if (idx >= 0 && idx < static_cast<int>(processed.size())) {
                        processed[idx] = true;
                    }
                }
            }
        }
        
        processed[i] = true;
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    // spdlog::info("[FeatureExtractor] Extracted {} features from {} points in {}ms", 
    //              valid_feature_count, input_cloud->size(), duration.count());
    
    return valid_feature_count;
}

// ===== Plane Fitting Methods =====

bool FeatureExtractor::fit_plane_to_points(const std::vector<Vector3f>& points,
                                          Vector3f& plane_normal,
                                          Vector3f& plane_center) {
    if (points.size() < 3) {
        return false;
    }
    
    // Compute centroid
    plane_center = Vector3f::Zero();
    for (const auto& point : points) {
        plane_center += point;
    }
    plane_center /= static_cast<float>(points.size());
    
    // Build covariance matrix
    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    for (const auto& point : points) {
        Vector3f centered = point - plane_center;
        covariance += centered * centered.transpose();
    }
    
    // Find normal via eigenvalue decomposition
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(covariance, Eigen::ComputeFullU);
    plane_normal = svd.matrixU().col(2); // Smallest eigenvalue corresponds to normal
    
    // Ensure normal is normalized
    plane_normal.normalize();
    
    return true;
}

float FeatureExtractor::compute_plane_fitness(const std::vector<Vector3f>& points,
                                             const Vector3f& plane_normal,
                                             const Vector3f& plane_center) {
    if (points.empty()) {
        return std::numeric_limits<float>::max();
    }
    
    float total_distance = 0.0f;
    for (const auto& point : points) {
        Vector3f to_point = point - plane_center;
        float distance = std::abs(to_point.dot(plane_normal));
        total_distance += distance;
    }
    
    return total_distance / static_cast<float>(points.size());
}

} // namespace processing
} // namespace lidar_odometry