#!/bin/bash

REPO="$(git -C "$(dirname "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
export REPO

COMPOSE_FILE="${REPO}/docker/docker-compose.playback.yml"

export PLAYBACK_CONFIG="${PLAYBACK_CONFIG:-config/playback/mrs_buff_mk3_playback.toml}"

HOST_UID="$(id -u)"
HOST_GID="$(id -g)"
export HOST_UID HOST_GID

if [ -z "${XAUTHORITY:-}" ]; then
    for candidate in "/run/user/$(id -u)/gdm/Xauthority" "${HOME}/.Xauthority"; do
        if [ -r "${candidate}" ]; then
            XAUTHORITY="${candidate}"
            break
        fi
    done
fi
if [ ! -r "${XAUTHORITY:-}" ]; then
    XAUTHORITY=/dev/null
fi
export XAUTHORITY

adopt_local_display() {
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
            echo "note: DISPLAY is unset and no local X display was found, so GUI tools in" >&2
            echo "      the container cannot open windows. Playback, Foxglove, and mcap" >&2
            echo "      recording are unaffected. Pass --no-display to silence this." >&2
        fi
    fi
}

if [ -n "${AUTO_BATTLEBOT_NO_DISPLAY:-}" ]; then
    export DISPLAY=""
    echo "--no-display: X11 forwarding off, running headless." >&2
else
    export DISPLAY="${DISPLAY:-}"

    adopt_local_display
fi

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

warn_first_run() {
    if ! docker volume inspect auto-battlebot-zed-resources >/dev/null 2>&1; then
        echo ""
        echo "First run: the ZED SDK will GPU-optimize its neural depth model."
        echo "This takes several minutes and needs network access. It is cached in the"
        echo "auto-battlebot-zed-resources volume, so later runs skip it."
        echo ""
    fi
}
