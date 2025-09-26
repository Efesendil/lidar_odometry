# LiDAR Odometry

This is a real-time LiDAR odometry system designed for SLAM applications. It utilizes feature extraction from point clouds, iterative closest point (ICP) registration, sliding window optimization with Ceres Solver, and Pangolin for 3D visualization.

## License

This project is licensed under the 🚀MIT License🚀 - see the [LICENSE](LICENSE) file for details.

## Features

- ⚡ Real-time LiDAR odometry processing
- 🎯 Feature-based point cloud registration
- 🔧 Ceres Solver-based optimization
- 📊 3D trajectory visualization with Pangolin
- 🗂️ KITTI dataset support
- 🔄 Iterative Closest Point (ICP) algorithm
- 📈 Adaptive M-estimator for robust estimation

## Installation

📋 **[Installation Guide](docs/Install.md)** - Complete installation instructions for both Docker and native builds

## Dataset Download

📁 **[Dataset Download Guide](docs/Download_Dataset.md)** - KITTI dataset download and preparation

## Running the Application

🚀 **[Running Examples](docs/Running_Example.md)** - Usage examples and configuration

## Performance Analysis and Evaluation

📊 **[Performance Analysis Guide](docs/Performance_Analysis.md)** - Comprehensive performance evaluation and benchmarking

## Project Structure

The source code is organized into the following directories:

- `app/`: Main application entry points
  - `kitti_lidar_odometry.cpp`: Main LiDAR odometry application
  - `player/`: KITTI dataset player utilities
- `src/`:
  - `database/`: Data structures for `LidarFrame` and point cloud management
  - `processing/`: Core odometry modules, including `Estimator`, `FeatureExtractor`, and `IterativeClosestPoint`
  - `optimization/`: Ceres Solver cost functions and optimization parameters
  - `viewer/`: Pangolin-based 3D visualization
  - `util/`: Utility functions for configuration, math operations, and data types
- `thirdparty/`: External libraries (Ceres, Pangolin, Sophus, spdlog)
- `config/`: Configuration files for KITTI dataset processing
- `docs/`: Detailed documentation guides

## Quick Start

1. **Clone and Build**:
   ```bash
   git clone <repository-url>
   cd lidar_odometry
   chmod +x build.sh
   ./build.sh
   ```

2. **Download Sample Data**:
   Download the sample KITTI sequence 07 from [Google Drive](https://drive.google.com/drive/folders/13YL4H9EIfL8oq1bVp0Csm0B7cMF3wT_0?usp=sharing) and extract to `data/kitti/`

3. **Run LiDAR Odometry**:
   ```bash
   cd build
   ./lidar_odometry ../config/kitti.yaml
   ```

## Configuration

The system uses YAML configuration files located in the `config/` directory:
- `kitti.yaml`: Configuration for KITTI dataset processing

## Dependencies

- **PCL** (Point Cloud Library) - Point cloud processing
- **Eigen3** - Linear algebra operations
- **Ceres Solver** - Non-linear optimization
- **Pangolin** - 3D visualization
- **OpenGL** - Graphics rendering
- **YAML-cpp** - Configuration file parsing

## System Requirements

- **Ubuntu 20.04/22.04** (recommended)
- **C++17 Compiler** (g++ or clang++)
- **CMake** (>= 3.16)
- **CUDA** (optional, for GPU acceleration)

## Contributing

Contributions are welcome! Please feel free to submit issues and pull requests.

## References

This project implements various LiDAR odometry techniques based on the following concepts:

- Feature extraction from LiDAR point clouds
- Iterative Closest Point (ICP) registration
- Sliding window optimization
- Robust estimation with M-estimators

## Troubleshooting

For common issues and solutions, please refer to the [Installation Guide](docs/Install.md) or open an issue.

## Acknowledgments

Special thanks to the open-source community and the developers of the third-party libraries used in this project.
