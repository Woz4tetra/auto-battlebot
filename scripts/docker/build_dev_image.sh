#!/bin/bash

# Build the dev image: the playback toolchain plus personal devtools
# (docker/dev.Dockerfile, https://github.com/aalbaali/workstation_setup).
#
# Builds `playback` first: dev.Dockerfile is FROM auto-battlebot-playback:latest by
# local tag, not a registry image, so it has to exist already.
#
# Extra arguments go to `docker compose build`, so --no-cache and --progress=plain work.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/docker_common.sh"

echo "Building the playback image (dev.Dockerfile builds on top of it)..."
compose build "$@" playback

echo "Building the dev image..."
compose build "$@" dev

echo ""
echo "Built auto-battlebot-dev:latest"
echo "Open a shell with: scripts/docker/dev_shell.sh"
