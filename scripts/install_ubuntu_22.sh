#!/bin/bash

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Source helper functions
source "$PROJECT_ROOT/install/check_os_ubuntu_22.sh"
source "$PROJECT_ROOT/install/install_packages.sh"
source "$PROJECT_ROOT/install/install_libtorch_ubuntu.sh"
source "$PROJECT_ROOT/install/build_cpp_project.sh"

# Run checks and installation
check_os_ubuntu_22
install_packages "$PROJECT_ROOT/install/base_packages.txt"
install_packages "$PROJECT_ROOT/install/ubuntu_22_packages.txt"
install_libtorch_ubuntu 'https://download.pytorch.org/libtorch/cu130/libtorch-shared-with-deps-2.9.1%2Bcu130.zip'
build_cpp_project "$@"
