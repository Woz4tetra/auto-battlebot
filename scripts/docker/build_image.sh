#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/docker_common.sh"

echo "Building the playback image..."
compose build "$@" playback

echo ""
echo "Built auto-battlebot-playback:latest"
echo "Run playback with: scripts/docker/run_playback.sh"
