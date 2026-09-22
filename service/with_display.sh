#!/bin/bash
# Run a command with DISPLAY pointing at the X server of the logged-in session, on the boxes
# that have one. Headless boxes run the command straight away.
#
# Two things went wrong before this existed, and both looked like a crashed UI:
#
# 1. The unit hardcoded DISPLAY=:0. GDM hands the session whichever display number is free, so
#    after a manual login it is :1 and the app was aimed at a server that is not there.
# 2. At boot the service starts before GDM's autologin session exists. Measured on the handheld:
#    the service starts at t=11.8s and the session's Xorg at t=13.6s.
#
# SDL answers a missing display by falling back to its "offscreen" video driver instead of
# failing: every call succeeds, and the window silently never appears. Nothing crashes, so
# Restart= never retries and the app stays headless for the rest of the session. Hence the wait
# below rather than a start-and-hope.
#
# The wait is gated on the machine booting a desktop at all, so a headless deploy never sits
# waiting for a session that is never coming. Both conditions have to hold: a display manager
# enabled (that is what creates the X session) and graphical.target as the boot target (that is
# what runs it). Anything else is treated as headless, so an unclear answer costs no startup time.
#
# No XAUTHORITY handling needed: GDM grants the session owner by uid (xhost shows
# SI:localuser:<user>), so a process running as that user connects without a cookie.

set -e

X_WAIT_SECONDS=60

boots_a_desktop() {
    systemctl is-enabled display-manager.service >/dev/null 2>&1 &&
        [ "$(systemctl get-default 2>/dev/null || true)" = "graphical.target" ]
}

socket_for_display() {
    local number="${1#:}"
    echo "/tmp/.X11-unix/X${number%%.*}"
}

first_x_display() {
    local socket
    for socket in /tmp/.X11-unix/X*; do
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
    deadline=$((SECONDS + X_WAIT_SECONDS))
    while ! found="$(first_x_display)"; do
        if [ "$SECONDS" -ge "$deadline" ]; then
            echo "with_display: no X server after ${X_WAIT_SECONDS}s; running without DISPLAY." >&2
            break
        fi
        sleep 0.5
    done
    if [ -n "$found" ]; then export DISPLAY="$found"; fi
else
    unset DISPLAY
fi

exec "$@"
