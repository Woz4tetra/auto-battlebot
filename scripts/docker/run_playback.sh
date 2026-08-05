#!/bin/bash

# Build and run SVO playback in the container.
#
# Arguments are forwarded to the auto_battlebot binary, matching scripts/run.sh. With no
# arguments the default config in docker/docker-compose.playback.yml is used.
#
#   scripts/docker/run_playback.sh
#   scripts/docker/run_playback.sh -c config/playback/mr_stabs_mk2_playback.toml
#   scripts/docker/run_playback.sh --no-display
#
# --no-display drops X11 forwarding and passes --no-ui to the binary, so the UI is
# skipped whatever the config says. For headless machines and SSH sessions.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Parsed before sourcing docker_common.sh, which reads AUTO_BATTLEBOT_NO_DISPLAY when it
# decides what to do with DISPLAY.
BINARY_ARGS=()
for arg in "$@"; do
    if [ "${arg}" = "--no-display" ]; then
        export AUTO_BATTLEBOT_NO_DISPLAY=1
    else
        BINARY_ARGS+=("${arg}")
    fi
done

source "${SCRIPT_DIR}/docker_common.sh"

warn_first_run

if [ -n "${AUTO_BATTLEBOT_NO_DISPLAY:-}" ]; then
    BINARY_ARGS+=("--no-ui")
fi

# Overriding the command drops the compose file's default -c, so re-supply it unless the
# caller named a config themselves.
config_given=false
for arg in "${BINARY_ARGS[@]}"; do
    if [ "${arg}" = "-c" ] || [ "${arg}" = "--config" ]; then
        config_given=true
        break
    fi
done
if [ "${config_given}" = false ]; then
    BINARY_ARGS=(-c "${PLAYBACK_CONFIG}" "${BINARY_ARGS[@]}")
fi

# compose builds the playback and ros-connector images if they are missing, so a fresh
# machine needs no separate install step. depends_on starts the Foxglove bridge.
# Not exec'd: compose is a shell function from docker_common.sh, and exec needs a binary.
compose run --rm --build playback \
    ./scripts/docker/container_entrypoint.sh "${BINARY_ARGS[@]}"
