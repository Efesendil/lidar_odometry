/**
 * @file      PangolinViewer.h
 * @brief     Pangolin-based 3D viewer for LiDAR odometry visualization.
 * @author    Seungwon Choi
 * @date      2025-09-24
 * @copyright Copyright (c) 2025 Seungwon Choi. All rights reserved.
 *
 * @par License
 * This project is released under the MIT License.
 */

#pragma once

#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/handler/handler.h>
#include <pangolin/gl/glinclude.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/var/var.h>
#include <pangolin/var/varextra.h>
#include <pangolin/display/widgets.h>
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Forward declarations
namespace lidar_odometry {
namespace database {
class LidarFrame;
}
}

namespace lidar_odometry {
namespace viewer {

using PointType = pcl::PointXYZ;  // Use consistent point type
using PointCloud = pcl::PointCloud<PointType>;
using PointCloudPtr = PointCloud::Ptr;
using PointCloudConstPtr = PointCloud::ConstPtr;
using Vector3f = Eigen::Vector3f;
using Matrix4f = Eigen::Matrix4f;

/**
 * @brief Pangolin-based 3D viewer for LiDAR odometry visualization
 * 
 * This viewer displays:
 * - Current LiDAR frame point cloud (gray)
 * - Extracted plane features (red, larger points)
 * - Camera trajectory
 * - Coordinate frames and axes
 */
class PangolinViewer {
public:
    PangolinViewer();
    ~PangolinViewer();

    // ===== Initialization and Control =====
    
    /**
     * @brief Initialize the viewer window and start render thread
     * @param width Window width
     * @param height Window height
     * @return True if initialization succeeded
     */
    bool initialize(int width = 1280, int height = 960);
    
    /**
     * @brief Shutdown the viewer and stop render thread
     */
    void shutdown();
    
    /**
     * @brief Check if viewer should close
     * @return True if close requested
     */
    bool should_close() const;
    
    /**
     * @brief Check if viewer is ready
     * @return True if initialized and ready
     */
    bool is_ready() const;
    
    /**
     * @brief Main render loop - removed from public (now runs in thread)
     */
    
    /**
     * @brief Reset camera to default view
     */
    void reset_camera();

    // ===== Data Updates =====
    
    /**
     * @brief Update current LiDAR frame  
     * @param lidar_frame Current LiDAR frame
     */
    void update_current_frame(std::shared_ptr<database::LidarFrame> lidar_frame);
    
    /**
     * @brief Update map points for visualization
     * @param map_cloud Map point cloud in world coordinates
     */
    void update_map_points(PointCloudConstPtr map_cloud);
    
    /**
     * @brief Update ICP debug clouds for visualization
     * @param pre_icp_cloud LiDAR features before ICP (world coordinates)
     * @param post_icp_cloud LiDAR features after ICP (world coordinates)
     */
    void update_icp_debug_clouds(PointCloudConstPtr pre_icp_cloud, 
                                PointCloudConstPtr post_icp_cloud);
    
    /**
     * @brief Add pose to trajectory
     * @param pose Pose to add to trajectory
     */
    void add_trajectory_pose(const Matrix4f& pose);
    // ===== UI Controls =====
    
    /**
     * @brief Check if auto mode is enabled
     * @return True if auto mode enabled
     */
    bool is_auto_mode_enabled() const;
    
    /**
     * @brief Check if finish was requested
     * @return True if finish button pressed
     */
    bool is_finish_requested() const;
    
    /**
     * @brief Check if step forward was requested
     * @return True if step forward button was pressed
     */
    bool is_step_forward_requested();
    
    /**
     * @brief Process keyboard input
     * @param auto_play Auto play mode flag
     * @param step_mode Step mode flag
     * @param advance_frame Advance frame flag
     */
    void process_keyboard_input(bool& auto_play, bool& step_mode, bool& advance_frame);

private:
    // ===== Pangolin Components =====
    pangolin::OpenGlRenderState m_cam_state;
    pangolin::View m_display_3d;
    pangolin::View m_display_panel;
    
    // ===== Data Storage =====
    std::shared_ptr<database::LidarFrame> m_current_frame;  ///< Current LiDAR frame
    std::vector<Matrix4f> m_trajectory;                     ///< Estimated camera trajectory
    
    // ===== New Visualization Data =====
    PointCloudConstPtr m_map_cloud;                        ///< Map points (world coordinate) - Gray
    PointCloudConstPtr m_pre_icp_cloud;                    ///< Pre-ICP lidar features (world) - Blue  
    PointCloudConstPtr m_post_icp_cloud;                   ///< Post-ICP lidar features (world) - Red
    
    // ===== Thread Safety =====
    mutable std::mutex m_data_mutex;
    
    // ===== Thread Management =====
    std::thread m_render_thread;                       ///< Render thread
    std::atomic<bool> m_should_stop;                   ///< Thread stop flag
    std::atomic<bool> m_thread_ready;                  ///< Thread ready flag
    
    // ===== UI Variables =====
    pangolin::Var<bool> m_auto_mode;                   ///< Auto mode checkbox
    pangolin::Var<bool> m_show_map_points;             ///< Show map points checkbox (Gray)
    pangolin::Var<bool> m_show_point_cloud;            ///< Show LiDAR point cloud (Red-Blue)
    pangolin::Var<bool> m_show_features;               ///< Show LiDAR features (Mint)
    pangolin::Var<bool> m_show_trajectory;             ///< Show estimated trajectory checkbox
    pangolin::Var<bool> m_show_coordinate_frame;       ///< Show coordinate frame checkbox
    pangolin::Var<bool> m_top_view_follow;             ///< Top-down view follow mode checkbox
    pangolin::Var<bool> m_step_forward_button;         ///< Step forward button
    pangolin::Var<bool> m_finish_button;               ///< Finish button
    pangolin::Var<int> m_frame_id;                     ///< Current frame ID
    pangolin::Var<int> m_total_points;                 ///< Total points in frame
    pangolin::Var<int> m_feature_count;                ///< Number of features
    
    // ===== Display Settings =====
    float m_point_size;                                 ///< Point cloud point size
    float m_feature_point_size;                        ///< Feature point size
    float m_trajectory_width;                          ///< Trajectory line width
    float m_coordinate_frame_size;                     ///< Coordinate frame axis length
    float m_coordinate_frame_width;                    ///< Coordinate frame line width
    
    // ===== State =====
    bool m_initialized;                                 ///< Initialization state
    bool m_finish_pressed;                             ///< Finish button state
    bool m_step_forward_pressed;                       ///< Step forward button state
    
    // ===== Internal Methods =====
    
    /**
     * @brief Setup UI panels
     */
    void setup_panels();
    
    /**
     * @brief Main render loop - runs in separate thread
     */
    void render_loop();
    
    /**
     * @brief Draw 3D grid
     */
    void draw_grid();
    
    /**
     * @brief Draw coordinate axes
     */
    void draw_coordinate_axes();
    
    /**
     * @brief Draw map points (world coordinate) in gray
     */
    void draw_map_points();
    
    /**
     * @brief Draw pre-ICP LiDAR features (world coordinate) in blue
     */
    void draw_pre_icp_features();
    
    /**
     * @brief Draw post-ICP LiDAR features (world coordinate) in red
     */
    void draw_post_icp_features();
    
    /**
     * @brief Draw point cloud
     */
    void draw_point_cloud();
    
    /**
     * @brief Draw point cloud with specific frame data
     * @param frame LiDAR frame containing point cloud data
     */
    void draw_point_cloud_with_frame(std::shared_ptr<database::LidarFrame> frame);
    
    /**
     * @brief Draw plane features
     */
    void draw_plane_features();
    
    /**
     * @brief Draw trajectory
     */
    void draw_trajectory();
    
    /**
     * @brief Draw trajectory with specific trajectory data
     * @param trajectory Vector of poses forming the trajectory
     */
    void draw_trajectory_with_data(const std::vector<Matrix4f>& trajectory);
    
    /**
     * @brief Draw current pose
     */
    void draw_current_pose();
    
    /**
     * @brief Draw current pose with specific frame data
     * @param frame LiDAR frame containing pose data
     */
    void draw_current_pose_with_frame(std::shared_ptr<database::LidarFrame> frame);
    
    /**
     * @brief Convert PCL point to Eigen vector
     * @param pcl_point PCL point
     * @return Eigen vector
     */
    Vector3f pcl_to_eigen(const PointType& pcl_point) const;
};

} // namespace viewer
} // namespace lidar_odometry
