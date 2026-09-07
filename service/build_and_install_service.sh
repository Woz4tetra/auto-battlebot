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

# Resolve the real user even when the script is invoked with sudo.
REAL_USER="${SUDO_USER:-$USER}"

# Install a unit from service/<name>.service if it isn't present or is out of date,
# expanding the __USER__ placeholder into the invoking user's name.
install_unit() {
    local name="$1"
    local service_file="/etc/systemd/system/${name}.service"
    local source_file="$SCRIPT_DIR/${name}.service"
    local staged_file
    staged_file="$(mktemp)"
    sed "s/__USER__/${REAL_USER}/g" "$source_file" > "$staged_file"
    if [ ! -f "$service_file" ] || ! diff -q "$staged_file" "$service_file" > /dev/null 2>&1; then
        echo "Installing $name service (User=${REAL_USER})..."
        sudo cp "$staged_file" "$service_file"
        sudo systemctl daemon-reload
        sudo systemctl enable "$name"
        echo "Service installed and enabled."
    fi
    rm -f "$staged_file"
}

# The relay owns the Foxglove WebSocket server and restarts on its own; the app unit only
# orders After= it and never waits on it.
install_unit viz_relay
install_unit "$SERVICE_NAME"

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

echo "Restarting viz_relay and $SERVICE_NAME services..."
sudo systemctl restart viz_relay
sudo systemctl restart "$SERVICE_NAME"
echo "Services restarted."
