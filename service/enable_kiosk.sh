#!/bin/bash

set -e

if [ -z "$DISPLAY" ]; then
    echo "Kiosk mode: no DISPLAY, so nothing to configure. Skipping."
    exit 0
fi

echo "Enabling kiosk mode (disabling screen blanking and on-screen keyboard)..."

if command -v gsettings &>/dev/null; then
    gsettings set org.gnome.desktop.a11y.applications screen-keyboard-enabled false
    echo "  gsettings: on-screen keyboard disabled"
else
    echo "  gsettings not found; skipping"
fi

if command -v xset &>/dev/null; then
    xset s off
    xset -dpms
    xset s noblank
    echo "  xset: screen blanking disabled"
else
    echo "  xset not found (not in X session?); skipping"
fi

echo "Kiosk mode enabled."
