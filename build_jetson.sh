#!/bin/bash

set -e

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source helper functions
source "$SCRIPT_DIR/install/check_jetson_orin_nano.sh"
source "$SCRIPT_DIR/install/install_packages.sh"

# Run checks and installation
check_jetson_orin_nano
install_packages "$SCRIPT_DIR/install/base_packages.txt"
