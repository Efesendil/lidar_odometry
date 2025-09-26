# Installation Guide

This guide provides detailed instructions for installing and building the LiDAR Odometry system.

## Installation Methods

We provide two installation methods:
1. **Native Build (Ubuntu 22.04)** - Direct installation on your system
2. **Docker** - Containerized environment (Recommended for cross-platform)

---

## Method 1: Native Build (Ubuntu 22.04)

This method builds the project and all its dependencies directly on your system. It has been tested on Ubuntu 22.04.

### Prerequisites

- **Ubuntu 20.04/22.04 LTS**
- **C++17 Compiler** (g++ >= 7.0)
- **CMake** (>= 3.16)
- **Git**

### Step 1: Clone the Repository

```bash
git clone <repository-url>
cd lidar_odometry
```

### Step 2: Run the Build Script

The provided `build.sh` script will automatically install system dependencies and build the project with all third-party libraries.

```bash
chmod +x build.sh
./build.sh
```

The build script will:
1. Install system dependencies (PCL, Eigen3, OpenGL, etc.)
2. Build third-party libraries (Ceres Solver, Pangolin, Sophus, spdlog)
3. Configure and build the main application

### Step 3: Verify Installation

```bash
cd build
./lidar_odometry --help
```

---

## Method 2: Docker Installation (Recommended)

Using Docker is the recommended method as it provides a self-contained, consistent environment across different systems.

### Prerequisites

- **Docker** installed on your system
- **X11 forwarding** support (for visualization on Linux)
- **XQuartz** (for macOS users)

### Step 1: Clone the Repository

```bash
git clone <repository-url>
cd lidar_odometry
```

### Step 2: Build Docker Image

```bash
chmod +x docker.sh
./docker.sh
```

### Step 3: Run with Docker

```bash
# For Linux with X11 forwarding
docker run -it --rm \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    -v $(pwd):/workspace \
    lidar_odometry:latest

# For macOS with XQuartz
docker run -it --rm \
    -e DISPLAY=host.docker.internal:0 \
    -v $(pwd):/workspace \
    lidar_odometry:latest
```

---

## Manual Installation (Advanced Users)

If you prefer to install dependencies manually:

### System Dependencies

```bash
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    libpcl-dev \
    libeigen3-dev \
    libglu1-mesa-dev \
    libgl1-mesa-dev \
    libglew-dev \
    libyaml-cpp-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libatlas-base-dev \
    libsuitesparse-dev
```

### Building Third-party Libraries

The project includes scripts to build the required third-party libraries:

1. **Ceres Solver**:
   ```bash
   cd thirdparty/ceres-solver
   mkdir build && cd build
   cmake .. -DCMAKE_BUILD_TYPE=Release
   make -j$(nproc)
   ```

2. **Pangolin**:
   ```bash
   cd thirdparty/pangolin
   mkdir build && cd build
   cmake ..
   make -j$(nproc)
   ```

### Building Main Project

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

---

## Troubleshooting

### Common Issues

1. **CMake version too old**:
   ```bash
   # Install newer CMake
   sudo apt remove cmake
   sudo snap install cmake --classic
   ```

2. **PCL not found**:
   ```bash
   sudo apt install libpcl-dev
   ```

3. **OpenGL issues**:
   ```bash
   sudo apt install libgl1-mesa-dev libglu1-mesa-dev libglew-dev
   ```

4. **Eigen3 not found**:
   ```bash
   sudo apt install libeigen3-dev
   ```

### Build Warnings

If you encounter compiler warnings during build, they have been configured to be suppressed in the CMakeLists.txt. The build should complete successfully despite any warnings.

### Memory Requirements

- Minimum: 4GB RAM
- Recommended: 8GB RAM (for faster compilation)
- Build time: ~10-30 minutes depending on your system

### GPU Support (Optional)

For GPU-accelerated point cloud processing:

```bash
# Install CUDA toolkit
sudo apt install nvidia-cuda-toolkit
```

Make sure to rebuild the project after installing CUDA support.

---

## Verification

After successful installation, you can verify the build by running:

```bash
cd build
./lidar_odometry ../config/kitti.yaml
```

This should start the LiDAR odometry system with the default KITTI configuration. If you see the 3D visualization window, the installation was successful!
