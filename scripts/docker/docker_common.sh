#!/bin/bash

# Shared environment for the playback compose project.
#
# docker/docker-compose.playback.yml owns the container configuration. This file only
# resolves and exports the variables that file interpolates, so there is one place to
# change settings rather than the 25 scripts the true_battlebot attempt accumulated.

# Repo root, exported because compose mounts it at the same path inside the container.
REPO="$(git -C "$(dirname "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
export REPO

COMPOSE_FILE="${REPO}/docker/docker-compose.playback.yml"

# Compose has no $(id -u) equivalent, so pass them in. UID is set by bash but not
# exported, and GID is not set at all under some shells.
HOST_UID="$(id -u)"
HOST_GID="$(id -g)"
export HOST_UID HOST_GID

# Compose cannot express a conditional mount. XAUTHORITY must always resolve to a real
# path or the "${XAUTHORITY}:${XAUTHORITY}:ro" volume becomes ":" and compose rejects
# the file. Fall back to the conventional location and create it if absent.
XAUTHORITY="${XAUTHORITY:-${HOME}/.Xauthority}"
if [ ! -e "${XAUTHORITY}" ]; then
    touch "${XAUTHORITY}" 2>/dev/null || XAUTHORITY=/dev/null
fi
export XAUTHORITY

export DISPLAY="${DISPLAY:-}"
if [ -z "${DISPLAY}" ]; then
    echo "warning: DISPLAY is unset, so the LVGL window cannot open." >&2
    echo "         Run from a graphical session, or set ui.enable = false in config." >&2
fi

compose() {
    docker compose -f "${COMPOSE_FILE}" "$@"
}

# First run GPU-optimizes the ZED neural depth model, which takes several minutes and
# looks like a hang. Say so rather than let it look broken.
warn_first_run() {
    if ! docker volume inspect auto-battlebot-zed-resources >/dev/null 2>&1; then
        echo ""
        echo "First run: the ZED SDK will GPU-optimize its neural depth model."
        echo "This takes several minutes and needs network access. It is cached in the"
        echo "auto-battlebot-zed-resources volume, so later runs skip it."
        echo ""
    fi
}
