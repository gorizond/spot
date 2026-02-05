#!/bin/bash
# setup_dev_environment.sh - Complete setup for Gorizond Spot LCD development environment

set -e  # Exit on any error

echo "==========================================="
echo "Setting up Gorizond Spot LCD Development Environment"
echo "==========================================="

# Function to print section headers
print_header() {
    echo ""
    echo ">>> $1 <<<"
    echo ""
}

# Check if we're in the right directory
print_header "Verifying project structure"

if [ ! -d "/root/Yandex.Disk/Projects/gorizond/spot" ]; then
    echo "Error: Expected project directory not found!"
    echo "Expected: /root/Yandex.Disk/Projects/gorizond/spot"
    exit 1
fi

cd /root/Yandex.Disk/Projects/gorizond/spot
echo "Current directory: $(pwd)"

# Install system dependencies
print_header "Installing system dependencies"

# Update package lists
apt-get update

# Install build tools and dependencies
apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    libwiringpi-dev \
    wiringpi \
    docker.io \
    curl \
    wget

if [ $? -ne 0 ]; then
    echo "Error: Failed to install system dependencies"
    exit 1
fi

echo "System dependencies installed successfully"

# Verify that wiringPi is accessible via pkg-config
if pkg-config --exists wiringpi; then
    echo "✓ wiringPi is available via pkg-config"
else
    echo "Creating pkg-config file for wiringPi..."
    cat > /tmp/wiringpi.pc << EOF
prefix=/usr
exec_prefix=\${prefix}
libdir=\${exec_prefix}/lib/x86_64-linux-gnu
includedir=\${prefix}/include

Name: wiringPi
Description: GPIO Interface library for the Raspberry Pi
Version: 2.50
Libs: -L\${libdir} -lwiringPi
Cflags: -I\${includedir}
EOF
    cp /tmp/wiringpi.pc /usr/lib/pkgconfig/wiringpi.pc
    echo "Created pkg-config file for wiringPi"
fi

# Create necessary directories
print_header "Creating project directories"

mkdir -p k8s
mkdir -p docker/lcd
mkdir -p spot_lcd_cpp/config
mkdir -p spot_lcd_cpp/src
mkdir -p spot_lcd_cpp/include

echo "Project directories created"

# Create updated configuration files if they don't exist
print_header "Creating configuration files"

# Create a sample config file if it doesn't exist
CONFIG_FILE="spot_lcd_cpp/config/lcd_config.toml"
if [ ! -f "$CONFIG_FILE" ]; then
    cat > "$CONFIG_FILE" << EOF
# Configuration for LCD display
[lcd]
pin_rs = 25
pin_en = 24
pins_data = [23, 17, 18, 22]
cols = 16
rows = 2

[temp_monitor]
interval = 5

[uptime_monitor]
interval = 10
EOF
    echo "Created sample LCD configuration file"
else
    echo "LCD configuration file already exists"
fi

# Verify all required files exist
print_header "Verifying project files"

REQUIRED_FILES=(
    "spot_lcd_cpp/CMakeLists.txt"
    "spot_lcd_cpp/src/main.cpp"
    "spot_lcd_cpp/include/spot_lcd_node.h"
    "k8s/lcd_deployment.yaml"
    "BUILD_INSTRUCTIONS.md"
    "DEVELOPMENT.md"
    "build_lcd_cpp.sh"
    "build_and_push_docker.sh"
    "deploy_to_rancher.sh"
)

MISSING_FILES=()
for file in "${REQUIRED_FILES[@]}"; do
    if [ ! -f "$file" ]; then
        MISSING_FILES+=("$file")
    fi
done

if [ ${#MISSING_FILES[@]} -gt 0 ]; then
    echo "Warning: The following required files are missing:"
    for file in "${MISSING_FILES[@]}"; do
        echo "  - $file"
    done
    echo "Some functionality may be limited."
else
    echo "All required files are present"
fi

# Test build the application
print_header "Testing build process"

BUILD_SCRIPT="./build_lcd_cpp.sh"
if [ -f "$BUILD_SCRIPT" ]; then
    echo "Running build test..."
    chmod +x "$BUILD_SCRIPT"
    
    # Create a temporary directory for testing build
    TEMP_BUILD_DIR="/tmp/test_lcd_build_$$"
    mkdir -p "$TEMP_BUILD_DIR"
    cd "$TEMP_BUILD_DIR"
    
    # Copy the source files for testing
    cp -r /root/Yandex.Disk/Projects/gorizond/spot/spot_lcd_cpp .
    
    # Create a simple build test
    cd spot_lcd_cpp
    mkdir -p build
    cd build
    
    # Try to configure the project (without building to save time)
    cmake .. -DENABLE_GPIO=OFF -DENABLE_ROS2=OFF -DCMAKE_BUILD_TYPE=Debug
    
    if [ $? -eq 0 ]; then
        echo "✓ CMake configuration successful"
    else
        echo "✗ CMake configuration failed"
    fi
    
    # Clean up
    cd /
    rm -rf "$TEMP_BUILD_DIR"
    
    cd /root/Yandex.Disk/Projects/gorizond/spot
else
    echo "Build script not found, skipping build test"
fi

# Check Docker availability
print_header "Checking Docker setup"

if command -v docker &> /dev/null; then
    echo "✓ Docker is available"
    
    # Check if Docker daemon is running
    if docker info &> /dev/null; then
        echo "✓ Docker daemon is running"
    else
        echo "⚠ Docker daemon is not running. You may need to start it:"
        echo "  sudo systemctl start docker"
    fi
else
    echo "⚠ Docker is not available, some functionality will be limited"
fi

# Check kubectl availability
print_header "Checking Kubernetes tools"

if command -v kubectl &> /dev/null; then
    echo "✓ kubectl is available"
else
    echo "⚠ kubectl is not available, deployment functionality will be limited"
fi

# Create a summary
print_header "Development Environment Setup Summary"

echo "Directory: /root/Yandex.Disk/Projects/gorizond/spot"
echo ""
echo "Available tools:"
echo "  - Build: ./build_lcd_cpp.sh"
echo "  - Docker build: ./build_and_push_docker.sh"
echo "  - Deploy: ./deploy_to_rancher.sh"
echo "  - Documentation: DEVELOPMENT.md, BUILD_INSTRUCTIONS.md"
echo ""
echo "Source code location: spot_lcd_cpp/"
echo "Deployment configs: k8s/, deployment.yaml, lcd.yaml"
echo ""
echo "Configuration file: spot_lcd_cpp/config/lcd_config.toml"

echo ""
echo "==========================================="
echo "Setup completed successfully!"
echo "==========================================="

echo ""
echo "Next steps:"
echo "1. Review the configuration in spot_lcd_cpp/config/lcd_config.toml"
echo "2. Build the application: ./build_lcd_cpp.sh"
echo "3. For Docker build: ./build_and_push_docker.sh"
echo "4. For deployment: ./deploy_to_rancher.sh"
echo ""