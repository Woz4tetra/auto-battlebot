#!/bin/bash
# Disable kiosk mode: restore normal screen blanking, power management, and the
# on-screen keyboard setting.

set -e

echo "Disabling kiosk mode (restoring screen blanking and on-screen keyboard)..."

if command -v gsettings &>/dev/null; then
    gsettings reset org.gnome.desktop.a11y.applications screen-keyboard-enabled
    echo "  gsettings: on-screen keyboard setting restored to default"
else
    echo "  gsettings not found; skipping"
fi

if command -v xset &>/dev/null; then
    xset s default
    xset +dpms
    echo "  xset: screen blanking restored"
else
    echo "  xset not found; skipping"
fi

echo "Kiosk mode disabled."
