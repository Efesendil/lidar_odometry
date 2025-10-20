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
- 🚗 Support for KITTI dataset (outdoor/vehicle scenarios)
- 🏠 Support for MID360 LiDAR (indoor/handheld scenarios)

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

Choose one of the sample datasets:

#### Option A: KITTI Dataset (Outdoor/Vehicle)
Download the sample KITTI sequence 07 from [Google Drive](https://drive.google.com/drive/folders/13YL4H9EIfL8oq1bVp0Csm0B7cMF3wT_0?usp=sharing) and extract to `data/kitti/`

#### Option B: MID360 Dataset (Indoor/Handheld)
Download the sample MID360 dataset from [Google Drive](https://drive.google.com/file/d/1psjoqrX9CtMvNCUskczUlsmaysh823CO/view?usp=sharing) and extract to `data/MID360/`

*MID360 dataset source: https://www.youtube.com/watch?v=u8siB0KLFLc*

### 3. Update Configuration

Choose the appropriate configuration file for your dataset:

#### For KITTI Dataset
Edit `config/kitti.yaml` to set your dataset paths:
```yaml
# Data paths - Update these paths to your dataset location
data_directory: "/path/to/your/kitti_dataset/sequences"
ground_truth_directory: "/path/to/your/kitti_dataset/poses"  
output_directory: "/path/to/your/output/directory"
seq: "07"  # Change this to your sequence number
```

#### For MID360 Dataset  
Edit `config/mid360.yaml` to set your dataset paths:
```yaml
# Data paths - Update these paths to your dataset location
data_directory: "/path/to/your/MID360_dataset"
output_directory: "/path/to/your/output/directory"
seq: "slam"  # Subdirectory name containing PLY files
```

### 4. Run LiDAR Odometry

Choose the appropriate executable for your dataset:

#### For KITTI Dataset (Outdoor/Vehicle)
```bash
cd build
./kitti_lidar_odometry ../config/kitti.yaml
```

#### For MID360 Dataset (Indoor/Handheld)
```bash
cd build
./mid360_lidar_odometry ../config/mid360.yaml
```

## Full KITTI Dataset

For complete evaluation, download the full KITTI dataset from:
- **Official Website**: [http://www.cvlibs.net/datasets/kitti/](http://www.cvlibs.net/datasets/kitti/)
- **Odometry Dataset**: [http://www.cvlibs.net/datasets/kitti/eval_odometry.php](http://www.cvlibs.net/datasets/kitti/eval_odometry.php)

## Project Structure

- `app/`: Main applications and dataset players
  - `kitti_lidar_odometry.cpp`: KITTI dataset application  
  - `mid360_lidar_odometry.cpp`: MID360 dataset application
  - `player/`: Dataset-specific player implementations
- `src/`: Core modules (database, processing, optimization, viewer, util)
- `thirdparty/`: External libraries (Ceres, Pangolin, Sophus, spdlog)
- `config/`: Configuration files for different datasets
- `build.sh`: Build script for native compilation

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
