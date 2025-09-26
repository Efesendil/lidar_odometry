# LiDAR Odometry

This is a real-time LiDAR odometry system designed for SLAM applications. It utilizes feature extraction from point clouds, iterative closest point (ICP) registration, sliding window optimization with Ceres Solver, and Pangolin for 3D visualization.

## Features

- ⚡ Real-time LiDAR odometry processing
- 🎯 Feature-based point cloud registration
- 🔧 Ceres Solver-based optimization
- 📊 3D trajectory visualization with Pangolin
- 🗂️ KITTI dataset support
- 🔄 Iterative Closest Point (ICP) algorithm
- 📈 Adaptive M-estimator for robust estimation

## Quick Start

### 1. Build Options

#### Native Build (Ubuntu 22.04)
```bash
git clone <repository-url>
cd lidar_odometry
chmod +x build.sh
./build.sh
```

#### Docker Build
```bash
git clone <repository-url>
cd lidar_odometry
chmod +x docker.sh
./docker.sh

# Run with Docker
docker run -it --rm \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    -v $(pwd):/workspace \
    lidar_odometry:latest
```

### 2. Download Sample Data

Download the sample KITTI sequence 07 from [Google Drive](https://drive.google.com/drive/folders/13YL4H9EIfL8oq1bVp0Csm0B7cMF3wT_0?usp=sharing) and extract to `data/kitti/`

### 3. Update Configuration

Edit `config/kitti.yaml` to set your dataset path:
```yaml
dataset:
  path: "/path/to/your/kitti_dataset"
  sequence: "07"  # Change this to your sequence
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
