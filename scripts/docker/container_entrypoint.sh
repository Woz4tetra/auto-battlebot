#!/bin/bash

# Runs inside the playback container. Bind-mounted with the repo, never baked into the
# image, so editing it takes effect on the next run with no rebuild.
#
# Compiles into build-docker/ (AUTO_BATTLEBOT_BUILD_DIR is set by docker_common.sh) and
# execs the binary. Arguments are forwarded straight through to auto_battlebot.
#
# The Foxglove relay runs inside this container next to the app (host networking exposes
# ws://0.0.0.0:8765 to the desktop), so no second container or docker socket is needed.

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
