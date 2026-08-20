#!/bin/bash

# Interactive shell in the dev container: the playback toolchain plus personal devtools
# (docker/dev.Dockerfile), with the same repo/ZED/X11 mounts run_playback.sh and shell.sh
# use, but its own image and HOME volume (auto-battlebot-dev-home) so plain playback runs
# are unaffected. The repo is at its host path and AUTO_BATTLEBOT_BUILD_DIR is already
# set, so scripts/build.sh and scripts/lint work as they do on the host.
#
# Any arguments are run as a command instead of an interactive shell:
#   scripts/docker/dev_shell.sh ./scripts/build.sh
#
# --no-display drops X11 forwarding, for headless machines and SSH sessions:
#   scripts/docker/dev_shell.sh --no-display

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Parsed before sourcing docker_common.sh, which reads AUTO_BATTLEBOT_NO_DISPLAY when it
# decides what to do with DISPLAY.
COMMAND_ARGS=()
for arg in "$@"; do
    if [ "${arg}" = "--no-display" ]; then
        export AUTO_BATTLEBOT_NO_DISPLAY=1
    else
        COMMAND_ARGS+=("${arg}")
    fi
done

source "${SCRIPT_DIR}/docker_common.sh"

if [ "${#COMMAND_ARGS[@]}" -eq 0 ]; then
    COMMAND_ARGS=(zsh)
fi

# --no-deps skips the Foxglove bridge; a shell rarely needs it.
# Not exec'd: compose is a shell function from docker_common.sh, and exec needs a binary.
compose run --rm --no-deps --build dev "${COMMAND_ARGS[@]}"
