cmake_minimum_required(VERSION 3.16)
project(lidar_odometry)

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build" FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set OpenGL preference to avoid warnings
set(OpenGL_GL_PREFERENCE GLVND)

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Find system packages
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(Threads REQUIRED)

# Build third-party dependencies from source
set(THIRDPARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)

# Build Pangolin from thirdparty
set(BUILD_PANGOLIN_EXAMPLES OFF CACHE BOOL "Build Pangolin examples" FORCE)
set(BUILD_PANGOLIN_TESTS OFF CACHE BOOL "Build Pangolin tests" FORCE)
set(BUILD_PANGOLIN_TOOLS OFF CACHE BOOL "Build Pangolin tools" FORCE)
add_subdirectory(${THIRDPARTY_DIR}/pangolin ${CMAKE_BINARY_DIR}/pangolin EXCLUDE_FROM_ALL)

# Sophus is header-only, no need to build
# Just include the headers

# spdlog is header-only, no need to build
# Just include the headers

# Build Ceres from thirdparty
set(CERES_DIR ${THIRDPARTY_DIR}/ceres-solver)
set(BUILD_EXAMPLES OFF CACHE BOOL "Build Ceres examples" FORCE)
set(BUILD_BENCHMARKS OFF CACHE BOOL "Build Ceres benchmarks" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "Build Ceres tests" FORCE)
set(SUITESPARSE OFF CACHE BOOL "Enable SuiteSparse" FORCE)
set(PROVIDE_UNINSTALL_TARGET OFF CACHE BOOL "Disable Ceres uninstall target" FORCE)

# Prevent finding system Ceres
set(CMAKE_DISABLE_FIND_PACKAGE_Ceres ON)
add_subdirectory(${CERES_DIR} ${CMAKE_BINARY_DIR}/ceres EXCLUDE_FROM_ALL)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIRS}
    ${THIRDPARTY_DIR}/pangolin/include
    ${CMAKE_BINARY_DIR}/thirdparty/pangolin/include
    ${THIRDPARTY_DIR}/Sophus
    ${THIRDPARTY_DIR}/spdlog/include
    ${THIRDPARTY_DIR}/ceres-solver/include
    ${CMAKE_BINARY_DIR}/thirdparty/ceres-solver/include
    ${THIRDPARTY_DIR}/nanoflann/include
)

# Source files
set(LIDAR_ODOMETRY_SOURCES
    # Database module
    src/database/LidarFrame.cpp
    
    # Processing module
    src/processing/FeatureExtractor.cpp
    src/processing/Estimator.cpp
    src/processing/IterativeClosestPoint.cpp
    
    # Optimization module
    src/optimization/Parameters.cpp
    src/optimization/Factors.cpp
    src/optimization/AdaptiveMEstimator.cpp
    
    # Viewer module
    src/viewer/PangolinViewer.cpp
    
    # Utility module
    src/util/Config.cpp
    src/util/MathUtils.cpp
    src/util/PointCloudUtils.cpp
    
    # Main application
    app/kitti_lidar_odometry.cpp
    app/player/kitti_player.cpp
)

# Create executable
add_executable(lidar_odometry ${LIDAR_ODOMETRY_SOURCES})

# Link Eigen3 to target
if(TARGET Eigen3::Eigen)
    target_link_libraries(lidar_odometry Eigen3::Eigen)
else()
    # Fallback for older CMake versions
    target_include_directories(lidar_odometry PRIVATE ${EIGEN3_INCLUDE_DIR})
endif()

# Link libraries
target_link_libraries(lidar_odometry
    OpenGL::GL
    GLEW::GLEW
    Threads::Threads
    ceres
    pango_display
    pango_windowing
    pango_opengl
    pango_core
    pango_image
    m  # Math library for M_PI
    stdc++fs  # For std::filesystem
)

# Install target
install(TARGETS lidar_odometry
    RUNTIME DESTINATION bin
)

# Install configuration files
install(DIRECTORY config/
    DESTINATION share/lidar_odometry/config
    FILES_MATCHING PATTERN "*.yaml"
)

# Print configuration info
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "ThirdParty directory: ${THIRDPARTY_DIR}")
