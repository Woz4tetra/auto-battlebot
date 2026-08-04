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

# docker-compose.playback.yml uses the `include:` key, added in Compose v2.20. Older
# versions reject the file with "(root) Additional property include is not allowed",
# which does not hint at the cause. Fail with something actionable instead.
require_compose_version() {
    local min="2.20"
    local version
    version=$(docker compose version --short 2>/dev/null | sed 's/^v//')

    if [ -z "$version" ]; then
        echo "Error: 'docker compose' is unavailable. Run install/install_docker_ubuntu.sh." >&2
        return 1
    fi
    if [ "$(printf '%s\n%s\n' "$min" "$version" | sort -V | head -1)" != "$min" ]; then
        echo "Error: docker compose $version is too old; $min or newer is required." >&2
        echo "       A stale plugin in /usr/local/lib/docker/cli-plugins can shadow the" >&2
        echo "       packaged one. See install/install_docker_ubuntu.sh for the fix." >&2
        return 1
    fi
    return 0
}

compose() {
    require_compose_version || return 1
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
