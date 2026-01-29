#!/bin/bash

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Source helper functions
source "$PROJECT_ROOT/install/check_jetson_orin_nano.sh"
source "$PROJECT_ROOT/install/install_packages.sh"
source "$PROJECT_ROOT/install/install_opencv_jetson.sh"
source "$PROJECT_ROOT/install/install_ros_connector.sh"
source "$PROJECT_ROOT/install/build_cpp_project.sh"

# Run checks and installation
check_jetson_orin_nano
# Step 0: Install TensorRT runtime (before CMake)
install_packages "$PROJECT_ROOT/install/jetson_packages.txt"
install_packages "$PROJECT_ROOT/install/base_packages.txt"
install_opencv_jetson
install_ros_connector
build_cpp_project "$@"
