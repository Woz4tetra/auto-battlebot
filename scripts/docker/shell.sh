#!/bin/bash

# Interactive shell in the playback container, with the same mounts run_playback.sh
# uses. The repo is at its host path and AUTO_BATTLEBOT_BUILD_DIR is already set, so
# scripts/build.sh and scripts/lint work as they do on the host.
#
# Any arguments are run as a command instead of an interactive shell:
#   scripts/docker/shell.sh ./scripts/build.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/docker_common.sh"

if [ "$#" -eq 0 ]; then
    set -- bash
fi

# --no-deps skips the Foxglove bridge; a shell rarely needs it.
# Not exec'd: compose is a shell function from docker_common.sh, and exec needs a binary.
compose run --rm --no-deps --build playback "$@"
