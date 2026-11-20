#!/bin/bash

set -e

usage() {
    cat <<'EOF'
Usage: scripts/docker/run_playback.sh [--no-display] [auto_battlebot args]

Build and run SVO playback in the container.

Arguments are forwarded to the auto_battlebot binary, matching scripts/run.sh. With no
arguments the default config in docker/docker-compose.playback.yml is used.

  scripts/docker/run_playback.sh
  scripts/docker/run_playback.sh -c config/playback/mr_stabs_mk2_playback.toml
  scripts/docker/run_playback.sh --no-display

--no-display drops X11 forwarding. Playback opens no window either way; it only silences the
DISPLAY notes on headless machines and SSH sessions.
EOF
}

case "${1:-}" in
    -h | --help)
        usage
        exit 0
        ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

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

compose run --rm --build playback \
    ./scripts/docker/container_entrypoint.sh "${BINARY_ARGS[@]}"
