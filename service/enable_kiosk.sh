#!/bin/bash
# Enable kiosk mode: disable screen blanking and screensaver so the UI stays visible.

set -e

echo "Enabling kiosk mode (disabling screen blanking)..."

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
