#!/bin/bash
# Build, install, and restart the auto_battlebot systemd service (if present).

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
SERVICE_NAME="auto_battlebot"

"$PROJECT_ROOT/scripts/build_and_install.sh" "$@"

SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
SOURCE_FILE="$SCRIPT_DIR/${SERVICE_NAME}.service"

# Resolve the real user even when the script is invoked with sudo.
REAL_USER="${SUDO_USER:-$USER}"

# Expand __USER__ placeholder into the invoking user's name.
STAGED_FILE="$(mktemp)"
sed "s/__USER__/${REAL_USER}/g" "$SOURCE_FILE" > "$STAGED_FILE"

# Install the service unit if it isn't present or is out of date.
if [ ! -f "$SERVICE_FILE" ] || ! diff -q "$STAGED_FILE" "$SERVICE_FILE" > /dev/null 2>&1; then
    echo "Installing $SERVICE_NAME service (User=${REAL_USER})..."
    sudo cp "$STAGED_FILE" "$SERVICE_FILE"
    sudo systemctl daemon-reload
    sudo systemctl enable "$SERVICE_NAME"
    echo "Service installed and enabled."
fi
rm -f "$STAGED_FILE"

echo "Restarting $SERVICE_NAME service..."
sudo systemctl restart "$SERVICE_NAME"
echo "Service restarted."
