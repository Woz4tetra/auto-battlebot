#!/bin/bash

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Source helper functions
source "$PROJECT_ROOT/install/check_jetson_orin_nano.sh"
source "$PROJECT_ROOT/install/install_packages.sh"
source "$PROJECT_ROOT/install/install_opencv_jetson.sh"
source "$PROJECT_ROOT/install/install_pytorch_jetson.sh"
source "$PROJECT_ROOT/install/install_ros_connector.sh"
source "$PROJECT_ROOT/install/build_cpp_project.sh"
source "$PROJECT_ROOT/install/install_mcap_cli.sh"
source "$PROJECT_ROOT/install/install_taplo.sh"
source "$PROJECT_ROOT/install/install_udev_rules.sh"
source "$PROJECT_ROOT/install/install_ds3231_rtc.sh"
source "$PROJECT_ROOT/install/install_pstore_ramoops.sh"
source "$PROJECT_ROOT/install/install_jetson_stability.sh"
source "$PROJECT_ROOT/install/install_jetson_clocks.sh"

# Run checks and installation
check_jetson_orin_nano
# Step 0: Install TensorRT runtime (before CMake)
install_packages "$PROJECT_ROOT/install/jetson_packages.txt"
install_packages "$PROJECT_ROOT/install/base_packages.txt"
install_opencv_jetson
# PyTorch for Jetson (NVIDIA wheel into project venv)
install_pytorch_jetson
install_ros_connector
install_mcap_cli
install_taplo
build_cpp_project "$@"
install_udev_rules
install_ds3231_rtc
install_pstore_ramoops
install_jetson_stability
install_jetson_clocks
