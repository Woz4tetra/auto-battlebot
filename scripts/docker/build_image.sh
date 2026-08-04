#!/bin/bash

# Build the playback toolchain image.
#
# The image contains build tools and system libraries only, so this should be rare.
# Rebuild triggers are install/base_packages.txt, install/ubuntu_24_packages.txt,
# .llvm-version, and the version pins in docker/playback.Dockerfile. Editing code under
# src/ or include/ leaves every layer cached.
#
# Extra arguments go to `docker compose build`, so --no-cache and --progress=plain work.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/docker_common.sh"

echo "Building the playback image..."
compose build "$@" playback

echo ""
echo "Built auto-battlebot-playback:latest"
echo "Run playback with: scripts/docker/run_playback.sh"
