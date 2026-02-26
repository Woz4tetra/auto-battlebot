#!/bin/bash
# Disable kiosk mode: restore normal screen blanking and power management.

set -e

echo "Disabling kiosk mode (restoring screen blanking)..."

if command -v xset &>/dev/null; then
    xset s default
    xset +dpms
    echo "  xset: screen blanking restored"
else
    echo "  xset not found; skipping"
fi

echo "Kiosk mode disabled."
