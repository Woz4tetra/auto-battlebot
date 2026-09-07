#!/bin/bash

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="${PROJECT_ROOT}/build"

# Start the Foxglove relay if it is not already up. The app does not wait on it.
${SCRIPT_DIR}/run_viz_relay.sh

# Run the executable with any additional arguments passed to this script
${BUILD_DIR}/auto_battlebot "$@"
