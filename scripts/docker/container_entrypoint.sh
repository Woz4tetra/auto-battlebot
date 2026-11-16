#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$(dirname "${SCRIPT_DIR}")")"
BUILD_DIR="${PROJECT_ROOT}/${AUTO_BATTLEBOT_BUILD_DIR:-build-docker}"

"${PROJECT_ROOT}/scripts/build.sh"
"${PROJECT_ROOT}/scripts/run_viz_relay.sh"

echo ""
echo "Starting auto_battlebot..."
echo "========================================"
exec "${BUILD_DIR}/auto_battlebot" "$@"
