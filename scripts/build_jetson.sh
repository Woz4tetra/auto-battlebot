#!/bin/bash

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Source helper functions
source "$PROJECT_ROOT/install/check_jetson_orin_nano.sh"
source "$PROJECT_ROOT/install/install_packages.sh"

# Run checks and installation
check_jetson_orin_nano
install_packages "$PROJECT_ROOT/install/base_packages.txt"
