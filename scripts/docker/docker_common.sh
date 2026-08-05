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

# Default playback profile. Interpolated into the compose file's command and reused by
# run_playback.sh, so the default lives in exactly one place.
export PLAYBACK_CONFIG="${PLAYBACK_CONFIG:-config/playback/mrs_buff_mk3_playback.toml}"

# Compose has no $(id -u) equivalent, so pass them in. UID is set by bash but not
# exported, and GID is not set at all under some shells.
HOST_UID="$(id -u)"
HOST_GID="$(id -g)"
export HOST_UID HOST_GID

# Compose cannot express a conditional mount. XAUTHORITY must always resolve to a real
# path or the "${XAUTHORITY}:${XAUTHORITY}:ro" volume becomes ":" and compose rejects
# the file. Fall back to the conventional location and create it if absent.
if [ -z "${XAUTHORITY:-}" ]; then
    # A shell that lost DISPLAY has usually lost XAUTHORITY too. gdm keeps the cookie
    # under /run/user, which is where it lives on a normal desktop login.
    for candidate in "/run/user/$(id -u)/gdm/Xauthority" "${HOME}/.Xauthority"; do
        if [ -r "${candidate}" ]; then
            XAUTHORITY="${candidate}"
            break
        fi
    done
fi
# Nothing readable found. Point at /dev/null rather than creating a file in $HOME: the
# mount only has to resolve to something real, and an empty cookie authorizes nothing
# anyway.
if [ ! -r "${XAUTHORITY:-}" ]; then
    XAUTHORITY=/dev/null
fi
export XAUTHORITY

# Set by the --no-display flag in run_playback.sh and shell.sh. Blanking DISPLAY is what
# actually stops the container reaching an X server; the socket and cookie stay mounted
# because compose has no conditional mounts, and they are inert without DISPLAY.
if [ -n "${AUTO_BATTLEBOT_NO_DISPLAY:-}" ]; then
    export DISPLAY=""
    echo "--no-display: X11 forwarding off, running headless." >&2
else
    export DISPLAY="${DISPLAY:-}"

    # A terminal can easily end up without DISPLAY even while a perfectly good X server
    # is running: tmux and screen sessions outliving a login, or a shell started outside
    # the graphical session. Rather than silently drop the UI, adopt the local display
    # when there is exactly one and it is unambiguous.
    if [ -z "${DISPLAY}" ]; then
        x_sockets=()
        for socket in /tmp/.X11-unix/X*; do
            [ -S "${socket}" ] && x_sockets+=(":${socket##*/X}")
        done

        if [ "${#x_sockets[@]}" -eq 1 ]; then
            export DISPLAY="${x_sockets[0]}"
            echo "note: DISPLAY was unset; using the only local X display, ${DISPLAY}." >&2
            echo "      Export DISPLAY yourself to pick a different one, or pass" >&2
            echo "      --no-display to run headless." >&2
        elif [ "${#x_sockets[@]}" -gt 1 ]; then
            echo "note: DISPLAY is unset and several X displays exist (${x_sockets[*]})." >&2
            echo "      Export the one you want, or pass --no-display to run headless." >&2
        else
            # Nothing to connect to. Not fatal: SDL_Init fails, the UI thread returns,
            # and playback, Foxglove, and mcap recording all carry on.
            echo "note: DISPLAY is unset and no local X display was found, so no window" >&2
            echo "      will open. Playback, Foxglove, and mcap recording are unaffected." >&2
            echo "      Pass --no-display to skip the UI and silence the SDL error." >&2
        fi
    fi
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
