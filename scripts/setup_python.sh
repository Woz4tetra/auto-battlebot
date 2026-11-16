#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

source "$PROJECT_ROOT/install/install_python_environment.sh"
source "$PROJECT_ROOT/install/install_platformio.sh"

install_python_environment "$@"
install_platformio
