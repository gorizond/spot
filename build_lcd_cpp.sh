#!/bin/bash
# build_lcd_cpp.sh - Build script for LCD C++ application on the current system

echo "=== Building LCD C++ Application ==="

# Get the current working directory
CURRENT_DIR=$(pwd)
PROJECT_ROOT="$CURRENT_DIR"

echo "Project root: $PROJECT_ROOT"

# Check if we're on a Raspberry Pi or similar ARM system
ARCH=$(uname -m)
if [[ $ARCH == "arm"* || $ARCH == "aarch64" ]]; then
    IS_RASPBERRY_PI=true
    echo "Detected ARM architecture: $ARCH - enabling GPIO support"
else
    IS_RASPBERRY_PI=false
    echo "Detected architecture: $ARCH - GPIO support will be disabled"
fi

# Navigate to the project directory
cd "$PROJECT_ROOT/spot_lcd_cpp"

# Create build directory
BUILD_DIR="build"
if [ -d "$BUILD_DIR" ]; then
    echo "Removing existing build directory..."
    rm -rf "$BUILD_DIR"
fi

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Determine build options based on platform
if [ "$IS_RASPBERRY_PI" = true ]; then
    # On Raspberry Pi, enable GPIO support
    echo "Configuring for Raspberry Pi with GPIO support..."
    cmake .. -DENABLE_GPIO=ON -DENABLE_ROS2=OFF -DCMAKE_BUILD_TYPE=Release
else
    # On non-Raspberry Pi systems, disable GPIO support
    echo "Configuring for non-Raspberry Pi system without GPIO support..."
    cmake .. -DENABLE_GPIO=OFF -DENABLE_ROS2=OFF -DCMAKE_BUILD_TYPE=Release
fi

if [ $? -ne 0 ]; then
    echo "Error: CMake configuration failed!"
    exit 1
fi

# Build the project
echo "Building the project..."
JOBS=$(nproc)
echo "Using $JOBS parallel jobs..."
make -j$JOBS

if [ $? -ne 0 ]; then
    echo "Error: Build failed!"
    exit 1
fi

echo "Build completed successfully!"
echo "Executable location:"
ls -la ./spot_lcd_cpp_node

echo "=== Build process finished ==="