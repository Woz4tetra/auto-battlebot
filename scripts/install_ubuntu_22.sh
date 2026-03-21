#!/bin/bash

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Source helper functions
source "$PROJECT_ROOT/install/check_os_ubuntu_22.sh"
source "$PROJECT_ROOT/install/install_packages.sh"
source "$PROJECT_ROOT/install/install_tensorrt_runtime_ubuntu.sh"
source "$PROJECT_ROOT/install/install_docker_ubuntu.sh"
source "$PROJECT_ROOT/install/install_ros_connector.sh"
source "$PROJECT_ROOT/install/install_mcap_cli.sh"
source "$PROJECT_ROOT/install/build_cpp_project.sh"
source "$PROJECT_ROOT/install/install_platformio.sh"

# Run checks and installation
check_os_ubuntu_22
# Step 0: Install TensorRT runtime (before CMake)
install_tensorrt_runtime_ubuntu
install_packages "$PROJECT_ROOT/install/base_packages.txt"
install_packages "$PROJECT_ROOT/install/ubuntu_22_packages.txt"
install_docker_ubuntu
install_ros_connector
install_mcap_cli
build_cpp_project "$@"
install_platformio
