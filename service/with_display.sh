#!/bin/bash

set -e

X_WAIT_SECONDS="${X_WAIT_SECONDS:-60}"
X11_SOCKET_DIR="${X11_SOCKET_DIR:-/tmp/.X11-unix}"

boots_a_desktop() {
    systemctl is-enabled display-manager.service >/dev/null 2>&1 &&
        [ "$(systemctl get-default 2>/dev/null || true)" = "graphical.target" ]
}

socket_for_display() {
    local number="${1#:}"
    echo "${X11_SOCKET_DIR}/X${number%%.*}"
}

first_x_display() {
    local socket
    for socket in "${X11_SOCKET_DIR}"/X*; do
        [ -e "$socket" ] || continue
        echo ":${socket##*/X}"
        return 0
    done
    return 1
}

if [ -n "$DISPLAY" ] && [ -e "$(socket_for_display "$DISPLAY")" ]; then
    exec "$@"
fi

if found="$(first_x_display)"; then
    export DISPLAY="$found"
elif boots_a_desktop; then
    polls_left=$((X_WAIT_SECONDS * 2))
    while ! found="$(first_x_display)"; do
        if [ "$polls_left" -le 0 ]; then
            echo "with_display: no X server after ${X_WAIT_SECONDS}s; running without DISPLAY." >&2
            break
        fi
        sleep 0.5
        polls_left=$((polls_left - 1))
    done
    if [ -n "$found" ]; then export DISPLAY="$found"; fi
else
    unset DISPLAY
fi

exec "$@"
