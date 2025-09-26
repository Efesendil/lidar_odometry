/**
 * @file      kitti_lidar_odometry.cpp
 * @brief     Main application for KITTI LiDAR odometry
 * @author    Seungwon Choi
 * @date      2025-09-25
 * @copyright Copyright (c) 2025 Seungwon Choi. All rights reserved.
 *
 * @par License
 * This project is released under the MIT License.
 */

#include <iostream>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include "player/kitti_player.h"

int main(int argc, char** argv) {
    // Initialize logging
    auto console = spdlog::stdout_color_mt("console");
    spdlog::set_default_logger(console);
    spdlog::set_level(spdlog::level::info);
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
    
    // Parse command line arguments
    std::string config_path = "./config/kitti.yaml";
    
    if (argc > 1) {
        config_path = argv[1];
    }
    
    spdlog::info("════════════════════════════════════════════════════════════════════");
    spdlog::info("                    KITTI LiDAR Odometry System                     ");
    spdlog::info("════════════════════════════════════════════════════════════════════");
    spdlog::info("Using configuration file: {}", config_path);
    spdlog::info("");
    
    try {
        // Create and run KITTI player
        lidar_odometry::app::KittiPlayer player;
        auto result = player.run_from_yaml(config_path);
        
        if (result.success) {
            spdlog::info("");
            spdlog::info("════════════════════════════════════════════════════════════════════");
            spdlog::info("                        PROCESSING COMPLETED                        ");
            spdlog::info("════════════════════════════════════════════════════════════════════");
            spdlog::info(" Successfully processed {} frames", result.processed_frames);
            spdlog::info(" Average processing time: {:.2f}ms", result.average_processing_time_ms);
            spdlog::info(" Average frame rate: {:.1f}fps", 1000.0 / result.average_processing_time_ms);
            
            if (result.error_stats.available) {
                spdlog::info("");
                spdlog::info(" Trajectory Error Analysis:");
                spdlog::info("   ATE RMSE: {:.4f}m", result.error_stats.ate_rmse);
                spdlog::info("   ATE Mean: {:.4f}m", result.error_stats.ate_mean);
                spdlog::info("   Rotation RMSE: {:.4f}°", result.error_stats.rotation_rmse);
                spdlog::info("   Translation RMSE: {:.4f}m", result.error_stats.translation_rmse);
            }
            
            spdlog::info("════════════════════════════════════════════════════════════════════");
            
            return 0;
        } else {
            spdlog::error("Processing failed: {}", result.error_message);
            return 1;
        }
        
    } catch (const std::exception& e) {
        spdlog::error("Fatal error: {}", e.what());
        return 1;
    }
}
