# LiDAR Odometry with Probabilistic Kernel Optimization (PKO)

This is a real-time LiDAR odometry system designed for SLAM applications. It utilizes feature extraction from point clouds, iterative closest point (ICP) registration, sliding window optimization with Ceres Solver, and Pangolin for 3D visualization.

The system incorporates **Probabilistic Kernel Optimization (PKO)** for robust state estimation, as described in:

> S. Choi and T.-W. Kim, "Probabilistic Kernel Optimization for Robust State Estimation," *IEEE Robotics and Automation Letters*, vol. 10, no. 3, pp. 2998-3005, 2025, doi: 10.1109/LRA.2025.3536294.
> 
> **Paper**: [https://ieeexplore.ieee.org/document/10857458](https://ieeexplore.ieee.org/document/10857458)

ROS Wrapper: https://github.com/93won/lidar_odometry_ros_wrapper


## Features

- ⚡ Real-time LiDAR odometry processing
- 🎯 Feature-based point cloud registration
- 🔧 Ceres Solver-based optimization
- 📈 Adaptive M-estimator for robust estimation (PKO)

## Demo

[![LiDAR Odometry Demo](https://img.youtube.com/vi/FANz9mhIAQQ/0.jpg)](https://www.youtube.com/watch?v=FANz9mhIAQQ)

*Click to watch the demo video showing real-time LiDAR odometry on KITTI dataset*

## Quick Start

### 1. Build Options

#### Native Build (Ubuntu 22.04)
```bash
git clone https://github.com/93won/lidar_odometry
cd lidar_odometry
chmod +x build.sh
./build.sh
```

### 2. Download Sample Data

Download the sample KITTI sequence 07 from [Google Drive](https://drive.google.com/drive/folders/13YL4H9EIfL8oq1bVp0Csm0B7cMF3wT_0?usp=sharing) and extract to `data/kitti/`

### 3. Update Configuration

Edit `config/kitti.yaml` to set your dataset paths:
```yaml
# Data paths - Update these paths to your dataset location
data_directory: "/path/to/your/kitti_dataset/sequences"
ground_truth_directory: "/path/to/your/kitti_dataset/poses"  
output_directory: "/path/to/your/output/directory"
seq: "07"  # Change this to your sequence number
```

### 4. Run LiDAR Odometry

```bash
cd build
./lidar_odometry ../config/kitti.yaml
```

## Full KITTI Dataset

For complete evaluation, download the full KITTI dataset from:
- **Official Website**: [http://www.cvlibs.net/datasets/kitti/](http://www.cvlibs.net/datasets/kitti/)
- **Odometry Dataset**: [http://www.cvlibs.net/datasets/kitti/eval_odometry.php](http://www.cvlibs.net/datasets/kitti/eval_odometry.php)

## Project Structure

- `app/`: Main application and KITTI dataset player
- `src/`: Core modules (database, processing, optimization, viewer, util)
- `thirdparty/`: External libraries (Ceres, Pangolin, Sophus, spdlog)
- `config/`: Configuration files
- `build.sh`: Build script for native compilation

## Dependencies

- **PCL** (Point Cloud Library)
- **Eigen3** - Linear algebra
- **Ceres Solver** - Non-linear optimization
- **Pangolin** - 3D visualization
- **OpenGL** - Graphics rendering
- **YAML-cpp** - Configuration parsing

## System Requirements

- **Ubuntu 20.04/22.04** (recommended)
- **C++17 Compiler** (g++ or clang++)
- **CMake** (>= 3.16)

## License

This project is released under the MIT License.

## References

```bibtex
@ARTICLE{10857458,
  author={Choi, Seungwon and Kim, Tae-Wan},
  journal={IEEE Robotics and Automation Letters}, 
  title={Probabilistic Kernel Optimization for Robust State Estimation}, 
  year={2025},
  volume={10},
  number={3},
  pages={2998-3005},
  keywords={Kernel;Optimization;State estimation;Probabilistic logic;Tuning;Robustness;Cost function;Point cloud compression;Oceans;Histograms;Robust state estimation;SLAM},
  doi={10.1109/LRA.2025.3536294}
}
```
