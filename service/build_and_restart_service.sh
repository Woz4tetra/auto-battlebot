#!/bin/bash
# Build, install, and restart the auto_battlebot systemd service (if present).

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
SERVICE_NAME="auto_battlebot"

"$PROJECT_ROOT/scripts/build_and_install.sh" "$@"

echo "Restarting $SERVICE_NAME service..."
if sudo systemctl restart "$SERVICE_NAME" 2>/dev/null; then
    echo "Service restarted."
else
    echo "Could not restart. Install with: sudo cp $PROJECT_ROOT/service/auto_battlebot.service /etc/systemd/system/ && sudo systemctl daemon-reload && sudo systemctl enable $SERVICE_NAME"
fi
