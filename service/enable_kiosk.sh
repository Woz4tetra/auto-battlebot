#!/bin/bash
# Enable kiosk mode: disable screen blanking, screensaver, and the GNOME
# on-screen keyboard so the UI stays visible and unobstructed.

set -e

echo "Enabling kiosk mode (disabling screen blanking and on-screen keyboard)..."

# Disable the GNOME on-screen keyboard so it doesn't cover the UI on the touchscreen.
# Needs the session dbus; GLib finds it at $XDG_RUNTIME_DIR/bus without
# DBUS_SESSION_BUS_ADDRESS being set.
if command -v gsettings &>/dev/null; then
    gsettings set org.gnome.desktop.a11y.applications screen-keyboard-enabled false
    echo "  gsettings: on-screen keyboard disabled"
else
    echo "  gsettings not found; skipping"
fi

# Disable X screen blanking and power management (run in X session)
if command -v xset &>/dev/null; then
    xset s off
    xset -dpms
    xset s noblank
    echo "  xset: screen blanking disabled"
else
    echo "  xset not found (not in X session?); skipping"
fi

echo "Kiosk mode enabled."
