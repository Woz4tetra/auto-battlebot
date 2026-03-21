#!/bin/bash

# Main script to set up Python environment for auto-battlebot project

set -e  # Exit on error

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Source helper functions
source "$PROJECT_ROOT/install/install_python_environment.sh"
source "$PROJECT_ROOT/install/install_platformio.sh"

# Run the installation (pass through flags: --recreate / -y, --no-recreate / -n)
install_python_environment "$@"
install_platformio
