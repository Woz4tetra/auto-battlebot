#!/bin/bash
# Build, install, and restart the auto_battlebot systemd service (if present).

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
SERVICE_NAME="auto_battlebot"
JOURNALD_DIR="/etc/systemd/journald.conf.d"
JOURNALD_FILE="${JOURNALD_DIR}/99-auto-battlebot-persistent.conf"

if systemctl is-active --quiet "$SERVICE_NAME" 2>/dev/null; then
    echo "Stopping $SERVICE_NAME (was running)..."
    sudo systemctl stop "$SERVICE_NAME"
fi

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

# Ensure journald keeps logs on disk so service logs survive reboot.
if [ ! -f "$JOURNALD_FILE" ] || ! rg -q '^Storage=persistent$' "$JOURNALD_FILE"; then
    echo "Configuring persistent journald storage..."
    sudo mkdir -p "$JOURNALD_DIR"
    sudo tee "$JOURNALD_FILE" >/dev/null <<'EOF'
[Journal]
Storage=persistent
SystemMaxUse=200M
RuntimeMaxUse=50M
EOF
    sudo mkdir -p /var/log/journal
    sudo systemctl restart systemd-journald
fi

echo "Restarting $SERVICE_NAME service..."
sudo systemctl restart "$SERVICE_NAME"
echo "Service restarted."
