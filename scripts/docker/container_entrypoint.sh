#!/bin/bash

# Runs inside the playback container. Bind-mounted with the repo, never baked into the
# image, so editing it takes effect on the next run with no rebuild.
#
# Compiles into build-docker/ (AUTO_BATTLEBOT_BUILD_DIR is set by docker_common.sh) and
# execs the binary. Arguments are forwarded straight through to auto_battlebot.
#
# scripts/run.sh is not reused here because it starts the ros-connector container via
# docker compose. That runs on the host instead, launched by run_playback.sh, so the
# container needs no docker socket.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$(dirname "${SCRIPT_DIR}")")"
BUILD_DIR="${PROJECT_ROOT}/${AUTO_BATTLEBOT_BUILD_DIR:-build-docker}"

"${PROJECT_ROOT}/scripts/build.sh"

echo ""
echo "Starting auto_battlebot..."
echo "========================================"
exec "${BUILD_DIR}/auto_battlebot" "$@"
