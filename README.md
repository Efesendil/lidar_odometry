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
- 🔧 Asynchronous loop closure detection and pose graph optimization (PGO)
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

## Configuration

### Loop Closure Detection

The system supports automatic loop closure detection using LiDAR Iris descriptors. Configure these settings in `config/*.yaml`:

```yaml
loop_detector:
  enable_loop_detection: true        # Enable/disable loop closure detection
  similarity_threshold: 0.3          # Descriptor similarity threshold (0.0 = identical, lower is stricter)
  min_keyframe_gap: 50               # Dual purpose:
                                     #   1) Minimum keyframe ID gap for candidate search (prevents false positives from nearby frames)
                                     #   2) Cooldown gap after successful loop closure (prevents repeated detections)
  max_search_distance: 5.0           # Maximum Euclidean distance (meters) to search for loop candidates
  enable_debug_output: true          # Enable detailed debug logging for loop detection
```

**Parameter Explanation:**

- **`enable_loop_detection`**: Master switch for loop closure detection. When disabled, the system runs odometry-only mode.

- **`similarity_threshold`**: Controls how similar two place descriptors must be to consider them a loop closure candidate. Range: [0.0, 2.0]
  - Lower values (e.g., 0.2-0.3): More strict, fewer false positives, may miss some valid loops
  - Higher values (e.g., 0.4-0.5): More permissive, more loop candidates, higher false positive risk
  - Recommended: 0.3 for most scenarios

- **`min_keyframe_gap`**: Serves two purposes:
  1. **Candidate Search**: Only considers keyframes with ID difference ≥ this value as loop candidates (prevents matching with nearby frames in the trajectory)
  2. **Cooldown Period**: After a successful loop closure, waits for this many new keyframes before allowing another loop detection (prevents redundant closures)
  - Smaller values (e.g., 30-50): More frequent loop detection, suitable for slow-moving robots
  - Larger values (e.g., 100-200): Less frequent but more robust loop detection, suitable for fast motion
  - Recommended: 50 for typical scenarios

- **`max_search_distance`**: Only considers keyframes within this Euclidean distance as loop candidates (uses odometry pose estimates for distance calculation). Helps reduce computational cost and false positives from distant places.
  - Smaller values (e.g., 3-5m): Faster search, suitable for small environments
  - Larger values (e.g., 10-20m): More comprehensive search, suitable for large-scale mapping
  - Recommended: 5.0 meters for most scenarios

- **`enable_debug_output`**: When enabled, prints detailed information about loop detection process, including similarity scores, ICP results, and PGO corrections.

### Pose Graph Optimization (PGO)

When a loop closure is detected, the system performs pose graph optimization to correct accumulated drift. This runs asynchronously in a background thread without blocking odometry.

```yaml
pose_graph_optimization:
  enable_pgo: true                   # Enable pose graph optimization
  odometry_translation_noise: 1.0    # Odometry constraint translation noise (lower = more trust)
  odometry_rotation_noise: 1.0       # Odometry constraint rotation noise (lower = more trust)
  loop_translation_noise: 1.0        # Loop closure constraint translation noise (lower = more trust)
  loop_rotation_noise: 1.0           # Loop closure constraint rotation noise (lower = more trust)
```

**Parameter Explanation:**

- **`enable_pgo`**: Master switch for pose graph optimization. When disabled, loop closures are detected but not used to correct the trajectory.

- **Noise Parameters**: These values represent the standard deviation (uncertainty) of each constraint type in the pose graph. They control the relative trust between odometry and loop closure constraints during optimization.

  - **Lower noise values** (e.g., 0.1-0.5): More trust in the constraint, stronger influence on optimization
  - **Higher noise values** (e.g., 2.0-5.0): Less trust in the constraint, weaker influence on optimization
  
  **Odometry Noise** (`odometry_translation_noise`, `odometry_rotation_noise`):
  - Controls trust in frame-to-frame odometry estimates
  - Lower values: Trust odometry more, less correction from loop closures
  - Higher values: Trust odometry less, allow more correction from loop closures
  - Recommended: 1.0 for balanced optimization
  
  **Loop Closure Noise** (`loop_translation_noise`, `loop_rotation_noise`):
  - Controls trust in loop closure ICP alignment results
  - Lower values: Trust loop closures more, stronger drift correction
  - Higher values: Trust loop closures less, more conservative correction
  - Recommended: 1.0 for balanced optimization
  
  **Tuning Strategy**:
  - If odometry is very accurate but loop closures have false positives: Decrease odometry noise, increase loop noise
  - If odometry drifts significantly: Increase odometry noise, decrease loop noise
  - For most scenarios, equal noise values (1.0) provide balanced optimization

**Asynchronous Processing:**

Loop closure detection and PGO run in a background thread to maintain real-time odometry performance (~35 FPS). The main thread:
1. Adds new keyframes to a query queue
2. Continues odometry processing without waiting
3. Applies PGO results when available (non-blocking check)

The background thread:
1. Processes loop detection queries from the queue
2. Performs ICP refinement for detected loops
3. Builds and optimizes the pose graph
4. Sends optimized poses back to the main thread

This architecture ensures smooth odometry even when PGO takes several seconds to complete.

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
