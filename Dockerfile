# Use Ubuntu 22.04 LTS as base image (has PCL 1.12)
FROM ubuntu:jammy

# Set environment variables to avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul

# Set the working directory
WORKDIR /workspace

# Install system dependencies including PCL 1.12
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libpcl-dev \
    libeigen3-dev \
    libglu1-mesa-dev \
    libgl1-mesa-dev \
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libblas-dev \
    liblapack-dev \
    libsuitesparse-dev \
    pkg-config \
    wget \
    unzip \
    && rm -rf /var/lib/apt/lists/*

# Install additional X11 and GUI dependencies
RUN apt-get update && apt-get install -y \
    xauth \
    x11-apps \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Copy project files
COPY . /workspace/

# Build third-party dependencies and main project
ENV DOCKER_CONTAINER=1
RUN chmod +x build.sh && ./build.sh

# Set environment variables
ENV LD_LIBRARY_PATH=/workspace/build/ceres/lib:/workspace/build/pangolin/lib:/workspace/build/spdlog:$LD_LIBRARY_PATH
ENV PKG_CONFIG_PATH=/workspace/build/ceres/lib/pkgconfig:/workspace/build/pangolin/lib/pkgconfig:$PKG_CONFIG_PATH

# Set the entry point
CMD ["./build/lidar_odometry"]
