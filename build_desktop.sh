#!/bin/bash

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source helper functions
source "$SCRIPT_DIR/install/check_os_ubuntu.sh"
source "$SCRIPT_DIR/install/install_packages.sh"
source "$SCRIPT_DIR/install/build_cpp_project.sh"

# Run checks and installation
check_os_ubuntu
install_packages "$SCRIPT_DIR/install/base_packages.txt"
build_cpp_project
