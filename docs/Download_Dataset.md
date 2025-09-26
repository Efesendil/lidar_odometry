# Dataset Download Guide

This guide explains how to download and prepare the KITTI dataset for use with the LiDAR Odometry system.

## Quick Start (Sample Data)

For quick testing, we provide a sample KITTI sequence 07 dataset:

**📁 [Sample Dataset (Google Drive)](https://drive.google.com/drive/folders/13YL4H9EIfL8oq1bVp0Csm0B7cMF3wT_0?usp=sharing)**

This includes:
- KITTI sequence 07 LiDAR data
- Ground truth poses
- Ready-to-use format

Simply download and extract to get started immediately!

## Full KITTI Dataset

The KITTI dataset is a widely-used benchmark for autonomous driving research, containing synchronized LiDAR point clouds, camera images, GPS/IMU data, and ground truth trajectories.

### Official KITTI Website

**🌐 [KITTI Dataset Homepage](http://www.cvlibs.net/datasets/kitti/)**

For full dataset access:
1. Visit the official KITTI website
2. Navigate to **"Raw Data"** or **"Odometry"** section
3. Download the Velodyne point clouds and ground truth poses
4. Follow the data preparation steps below

## Dataset Structure

After downloading, organize your dataset as follows:

```
kitti_dataset/
├── sequences/
│   ├── 00/
│   │   ├── velodyne/
│   │   │   ├── 000000.bin
│   │   │   ├── 000001.bin
│   │   │   └── ...
│   │   └── poses.txt
│   ├── 01/
│   │   ├── velodyne/
│   │   └── poses.txt
│   └── ...
└── calibration/
    ├── calib.txt
    └── times.txt
```

## Data Preparation

### Step 1: Extract Downloaded Archives

```bash
# Extract point cloud data
tar -xzf data_odometry_velodyne.tar.gz

# Extract ground truth poses
tar -xzf data_odometry_poses.tar.gz
```

### Step 2: Verify Data Structure

Run the verification script:

```bash
cd scripts
python3 verify_kitti_data.py --dataset_path /path/to/kitti_dataset
```

### Step 3: Update Configuration

Update the dataset path in your configuration file:

```yaml
# config/kitti.yaml
dataset:
  path: "/path/to/your/kitti_dataset"
  sequence: "00"  # Change this to your desired sequence
```

## Recommended Sequences

For testing and evaluation, we recommend starting with these sequences:

| Sequence | Length | Environment | Difficulty |
|----------|--------|-------------|------------|
| 00 | 4541 frames | Urban | Easy |
| 01 | 1101 frames | Highway | Easy |
| 02 | 4661 frames | Urban | Medium |
| 03 | 801 frames | Country | Medium |
| 04 | 271 frames | Urban | Easy |
| 05 | 2761 frames | Urban | Medium |
| 06 | 1101 frames | Urban | Medium |
| 07 | 1101 frames | Urban | Hard |
| 08 | 4071 frames | Urban | Hard |
| 09 | 1591 frames | Urban | Medium |
| 10 | 1201 frames | Urban | Hard |

## Data Format

### Point Cloud Format (.bin files)

Each `.bin` file contains point cloud data in binary format:
- **Format**: Float32 array
- **Structure**: [x, y, z, intensity] per point
- **Coordinate System**: 
  - X: forward
  - Y: left
  - Z: up
- **Units**: meters

### Poses Format (poses.txt)

Ground truth poses are stored as 3x4 transformation matrices (one per line):
```
r11 r12 r13 t1
r21 r22 r23 t2
r31 r32 r33 t3
```

Where R is the 3x3 rotation matrix and t is the 3x1 translation vector.

## Storage Requirements

- **Full KITTI Raw Dataset**: ~180 GB
- **KITTI Odometry Dataset**: ~80 GB
- **Recommended sequences (00-10)**: ~60 GB

## Alternative Datasets

If KITTI is not available, the system also supports:

1. **Custom Point Cloud Data**: 
   - Format: PCD, PLY, or custom binary
   - See `docs/Custom_Dataset.md` for details

2. **Simulated Data**:
   - CARLA simulator
   - AirSim
   - Custom ROS bag files

## Troubleshooting

### Download Issues

1. **Slow download speeds**: Use a download manager or split downloads
2. **Corrupted files**: Verify checksums and re-download if necessary
3. **Permission issues**: Ensure proper write permissions in target directory

### Data Issues

1. **Missing sequences**: Check if the sequence number exists in your dataset
2. **Incorrect paths**: Verify the dataset path in your configuration file
3. **Format errors**: Ensure binary files are not corrupted

### Verification Script

Use our verification script to check your dataset:

```bash
python3 scripts/verify_dataset.py --path /path/to/kitti --sequence 00
```

This will:
- Check file existence and structure
- Verify data format
- Count frames and report statistics
- Test data loading

## Usage with Docker

If using Docker, mount your dataset directory:

```bash
docker run -it --rm \
    -v /path/to/kitti_dataset:/data/kitti \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    lidar_odometry:latest
```

Update the config file to use `/data/kitti` as the dataset path.
