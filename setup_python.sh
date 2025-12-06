#!/bin/bash

# Main script to set up Python environment for auto-battlebot project

set -e  # Exit on error

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source the install_python_environment function
source "$SCRIPT_DIR/install/install_python_environment.sh"

# Run the installation
install_python_environment