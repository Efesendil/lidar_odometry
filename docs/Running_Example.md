# Running Examples

This guide provides detailed instructions on how to run the LiDAR Odometry system with various configurations and datasets.

## Prerequisites

Before running the examples, make sure you have:
1. Successfully built the project (see [Installation Guide](Install.md))
2. Downloaded and prepared the KITTI dataset (see [Dataset Guide](Download_Dataset.md))

## Basic Usage

### Running with Default Configuration

```bash
cd build
./lidar_odometry ../config/kitti.yaml
```

This will:
- Load the KITTI dataset configuration
- Start processing LiDAR frames
- Display real-time 3D visualization
- Save trajectory results

### Command Line Options

```bash
./lidar_odometry [options] <config_file>

Options:
  -h, --help              Show help message
  -v, --verbose           Enable verbose logging
  -s, --sequence <num>    Override sequence number
  -o, --output <path>     Override output directory
  --no-viz               Disable visualization
  --save-frames          Save processed frames
```

## Configuration

### Basic Configuration (config/kitti.yaml)

```yaml
# Dataset settings
dataset:
  path: "/path/to/kitti_dataset"
  sequence: "00"
  start_frame: 0
  max_frames: -1  # -1 for all frames

# Processing parameters
processing:
  voxel_size: 0.5
  max_range: 80.0
  min_range: 3.0
  
# Feature extraction
features:
  max_features: 1000
  edge_threshold: 0.1
  plane_threshold: 0.2

# Optimization
optimization:
  max_iterations: 10
  convergence_threshold: 1e-6
  
# Visualization
visualization:
  enabled: true
  point_size: 2
  trajectory_color: [1.0, 0.0, 0.0]  # RGB
```

### Advanced Configuration

For more advanced usage, you can modify:

```yaml
# ICP parameters
icp:
  max_iterations: 30
  transformation_epsilon: 1e-8
  euclidean_fitness_epsilon: 1
  max_correspondence_distance: 2.0

# Robust estimation
robust_estimation:
  use_adaptive_m_estimator: true
  huber_threshold: 1.0
  
# Output settings
output:
  save_trajectory: true
  save_pointclouds: false
  save_poses: true
  output_format: "TUM"  # TUM, KITTI, or CUSTOM
```

## Example Runs

### Example 1: Basic Odometry on Sequence 00

```bash
cd build
./lidar_odometry ../config/kitti.yaml
```

**Expected Output:**
```
[INFO] Loading KITTI sequence 00...
[INFO] Found 4541 frames
[INFO] Starting LiDAR odometry...
[INFO] Processing frame 0/4541...
[INFO] Features extracted: 857
[INFO] ICP converged in 12 iterations
[INFO] Processing frame 1/4541...
...
[INFO] Odometry completed successfully
[INFO] Final trajectory length: 3724.2m
[INFO] Results saved to: results/kitti_00_trajectory.txt
```

### Example 2: Run Specific Sequence Range

```bash
./lidar_odometry ../config/kitti.yaml --sequence 02 --start-frame 100 --max-frames 500
```

### Example 3: Headless Mode (No Visualization)

```bash
./lidar_odometry ../config/kitti.yaml --no-viz --output results/headless_run/
```

### Example 4: Verbose Logging

```bash
./lidar_odometry ../config/kitti.yaml --verbose
```

**Expected Verbose Output:**
```
[DEBUG] Loading configuration from: ../config/kitti.yaml
[DEBUG] Dataset path: /home/user/kitti_dataset
[DEBUG] Sequence: 00, Frames: 0 to 4540
[DEBUG] Voxel filter size: 0.5m
[DEBUG] Point cloud loaded: 124536 points
[DEBUG] After voxel filtering: 89234 points
[DEBUG] Feature extraction: 12.3ms
[DEBUG] ICP registration: 45.7ms
[DEBUG] Optimization: 8.1ms
[DEBUG] Total frame processing: 66.1ms
...
```

## Visualization Controls

When the 3D visualization window is open, you can use:

### Mouse Controls
- **Left Click + Drag**: Rotate view
- **Right Click + Drag**: Zoom in/out
- **Middle Click + Drag**: Pan view
- **Scroll Wheel**: Zoom

### Keyboard Shortcuts
- **SPACE**: Pause/Resume processing
- **R**: Reset view
- **S**: Save current view as screenshot
- **T**: Toggle trajectory display
- **P**: Toggle point cloud display
- **F**: Toggle feature points display
- **ESC/Q**: Exit application

### Panel Controls
- **Play/Pause**: Control playback
- **Speed**: Adjust processing speed (0.1x to 10x)
- **Frame Slider**: Jump to specific frame
- **View Options**: Toggle different visual elements

## Output Files

The system generates several output files:

### Trajectory Files
```
results/
├── trajectory_tum.txt          # TUM format trajectory
├── trajectory_kitti.txt        # KITTI format trajectory  
├── poses.txt                   # Raw poses (4x4 matrices)
└── timestamps.txt              # Frame timestamps
```

### Log Files
```
logs/
├── odometry.log                # General log messages
├── performance.log             # Timing statistics
└── debug.log                   # Debug information (if enabled)
```

### Optional Output (if enabled)
```
frames/
├── pointclouds/
│   ├── frame_000000.pcd
│   ├── frame_000001.pcd
│   └── ...
└── features/
    ├── frame_000000_features.pcd
    ├── frame_000001_features.pcd
    └── ...
```

## Performance Tips

### Optimization for Real-time Performance

1. **Reduce point cloud density**:
   ```yaml
   processing:
     voxel_size: 1.0  # Larger voxel size for faster processing
     max_range: 50.0  # Reduce processing range
   ```

2. **Limit feature extraction**:
   ```yaml
   features:
     max_features: 500  # Reduce for faster processing
   ```

3. **Disable visualization**:
   ```bash
   ./lidar_odometry config.yaml --no-viz
   ```

### Memory Optimization

1. **Process in batches**:
   ```bash
   ./lidar_odometry config.yaml --max-frames 1000
   ```

2. **Disable point cloud saving**:
   ```yaml
   output:
     save_pointclouds: false
   ```

## Troubleshooting

### Common Issues

1. **"Dataset not found" error**:
   - Check the dataset path in your config file
   - Verify the dataset structure matches expected format

2. **Visualization window not appearing**:
   - Check X11 forwarding if using SSH
   - Try running with `--no-viz` to test core functionality

3. **Slow performance**:
   - Reduce voxel size or max range
   - Check system resources (CPU, memory)
   - Consider using GPU acceleration if available

4. **Memory errors**:
   - Process smaller batches of frames
   - Increase system swap space
   - Disable unnecessary output saving

### Debug Mode

For debugging issues, run with maximum verbosity:

```bash
./lidar_odometry config.yaml --verbose --debug
```

This will:
- Enable detailed logging
- Save intermediate results
- Display timing information
- Show memory usage statistics

### Performance Monitoring

Monitor system performance during execution:

```bash
# In another terminal
htop  # Monitor CPU and memory usage
nvidia-smi  # Monitor GPU usage (if applicable)
```

## Integration with Other Tools

### ROS Integration

Convert trajectories to ROS bag format:

```bash
python3 scripts/trajectory_to_rosbag.py results/trajectory_tum.txt output.bag
```

### MATLAB/Python Analysis

Load results in MATLAB or Python:

```matlab
% MATLAB
trajectory = load('results/trajectory_tum.txt');
plot3(trajectory(:,2), trajectory(:,3), trajectory(:,4));
```

```python
# Python
import numpy as np
import matplotlib.pyplot as plt

traj = np.loadtxt('results/trajectory_tum.txt')
plt.plot(traj[:, 1], traj[:, 2])
plt.show()
```
