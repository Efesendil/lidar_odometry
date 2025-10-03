/**
 * @file      FeatureExtractor.h
 * @brief     Feature extraction for LiDAR point clouds.
 * @author    Seungwon Choi
 * @date      2025-09-24
 * @copyright Copyright (c) 2025 Seungwon Choi. All rights reserved.
 *
 * @par License
 * This project is released under the MIT License.
 */

#pragma once

#include "../util/Types.h"

#include <memory>
#include <vector>
#include <Eigen/Dense>
// #include <pcl/point_cloud.h>      // Removed PCL includes
// #include <pcl/point_types.h>
// #include <pcl/kdtree/kdtree_flann.h>

namespace lidar_odometry {
namespace processing {

// Import types from util namespace
using namespace lidar_odometry::util;

// ===== Configuration =====
struct FeatureExtractorConfig {
    // Plane fitting parameters
    size_t min_plane_points = 3;           ///< Minimum points for plane fitting
    size_t max_neighbors = 10;             ///< Maximum neighbors to consider
    float max_plane_distance = 0.05f;      ///< Maximum distance from plane (m)
    float collinearity_threshold = 0.05f;  ///< Collinearity threshold for point selection
    
    // Neighbor search parameters
    float max_neighbor_distance = 1.0f;    ///< Maximum neighbor search distance (m)
    float voxel_size = 0.1f;               ///< Voxel size for downsampling
    
    // Feature extraction parameters
    size_t min_points_per_feature = 5;     ///< Minimum points to form a feature
    float feature_quality_threshold = 0.1f; ///< Minimum quality for valid feature
};

// ===== Data Structures =====

/**
 * @brief Statistics for feature extraction
 */
struct Statistics {
    size_t total_points_processed = 0;     ///< Total points processed
    size_t features_extracted = 0;         ///< Number of features extracted
    float extraction_time_ms = 0.0f;       ///< Extraction time in milliseconds
    
    Statistics() = default;
};

/**
 * @brief Feature extractor for LiDAR point clouds
 * 
 * This class extracts planar features from point clouds for use in odometry.
 * It focuses purely on feature extraction without correspondence finding.
 */
class FeatureExtractor {
public:
    /**
     * @brief Constructor
     * @param config Feature extraction configuration
     */
    explicit FeatureExtractor(const FeatureExtractorConfig& config = FeatureExtractorConfig{});
    
    /**
     * @brief Extract features from point cloud
     * @param input_cloud Input point cloud
     * @param output_features Output feature point cloud
     * @return Number of features extracted
     */
    size_t extract_features(const PointCloudConstPtr& input_cloud,
                           PointCloudPtr& output_features);
    
    // ===== Utility Methods =====
    
    /**
     * @brief Get statistics from last feature extraction
     * @return Statistics structure
     */
    const Statistics& get_last_statistics() const { return m_last_stats; }

private:
    // ===== Configuration =====
    FeatureExtractorConfig m_config;
    
    // ===== Statistics =====
    Statistics m_last_stats;
    
    // ===== Internal Methods =====
    
    /**
     * @brief Fit plane to a set of points and get center point
     * @param points Vector of 3D points
     * @param plane_normal Output normal vector of the plane
     * @param plane_center Output center point of the plane
     * @return True if plane fitting succeeded
     */
    bool fit_plane_to_points(const std::vector<Vector3f>& points,
                            Vector3f& plane_normal,
                            Vector3f& plane_center);
    
    /**
     * @brief Compute plane fitness (average distance to plane)
     * @param points Vector of 3D points
     * @param plane_normal Plane normal vector
     * @param plane_center Plane center point
     * @return Average distance of points to the plane
     */
    float compute_plane_fitness(const std::vector<Vector3f>& points,
                               const Vector3f& plane_normal,
                               const Vector3f& plane_center);
    
    /**
     * @brief Check if points form a valid plane feature
     * @param points Points to check
     * @param normal Plane normal
     * @return True if plane is valid
     */
    bool is_valid_plane(const std::vector<Vector3f>& points, const Vector3f& normal);
    
    /**
     * @brief Find neighbors for plane fitting
     * @param query_point Query point
     * @param cloud Point cloud to search in
     * @param neighbor_points Output neighbor points
     * @return Number of neighbors found
     */
    size_t find_neighbors_for_plane(const Vector3f& query_point,
                                   const PointCloudConstPtr& cloud,
                                   std::vector<Vector3f>& neighbor_points);
};

} // namespace processing
} // namespace lidar_odometry
