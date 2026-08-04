#!/bin/bash

# Build and run SVO playback in the container.
#
# Arguments are forwarded to the auto_battlebot binary, matching scripts/run.sh. With no
# arguments the default config in docker/docker-compose.playback.yml is used.
#
#   scripts/docker/run_playback.sh
#   scripts/docker/run_playback.sh -c config/playback/mr_stabs_mk2_playback.toml

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/docker_common.sh"

warn_first_run

# compose builds the playback and ros-connector images if they are missing, so a fresh
# machine needs no separate install step. depends_on starts the Foxglove bridge.
# Not exec'd: compose is a shell function from docker_common.sh, and exec needs a binary.
if [ "$#" -eq 0 ]; then
    compose run --rm --build playback
else
    compose run --rm --build playback \
        ./scripts/docker/container_entrypoint.sh "$@"
fi
