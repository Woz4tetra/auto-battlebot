#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
SERVICE_NAME="auto-battlebot"
JOURNALD_DIR="/etc/systemd/journald.conf.d"
JOURNALD_FILE="${JOURNALD_DIR}/99-auto-battlebot-persistent.conf"

"$PROJECT_ROOT/install/rename_legacy_services.sh"

if systemctl is-active --quiet "$SERVICE_NAME" 2>/dev/null; then
    echo "Stopping $SERVICE_NAME (was running)..."
    sudo systemctl stop "$SERVICE_NAME"
fi

"$PROJECT_ROOT/scripts/build_and_install.sh" "$@"

REAL_USER="${SUDO_USER:-$USER}"
# The unit needs the numeric uid for XDG_RUNTIME_DIR. systemd's %U specifier resolves to 0 in a
# system unit even with User= set, which sent dconf to /run/user/0 and broke the kiosk setup.
REAL_UID="$(id -u "$REAL_USER")"

boots_a_desktop() {
    systemctl is-enabled display-manager.service >/dev/null 2>&1 &&
        [ "$(systemctl get-default 2>/dev/null || true)" = "graphical.target" ]
}

# Install a unit from service/<name>.service if it isn't present or is out of date,
# expanding the __USER__ placeholder into the invoking user's name. --no-enable installs the
# file without enabling it, and never disables a unit someone enabled by hand.
install_unit() {
    local name="$1"
    local enable=true
    [ "${2:-}" = "--no-enable" ] && enable=false
    local service_file="/etc/systemd/system/${name}.service"
    local source_file="$SCRIPT_DIR/${name}.service"
    local staged_file
    staged_file="$(mktemp)"
    sed -e "s/__USER__/${REAL_USER}/g" -e "s/__UID__/${REAL_UID}/g" "$source_file" > "$staged_file"
    if [ ! -f "$service_file" ] || ! diff -q "$staged_file" "$service_file" > /dev/null 2>&1; then
        echo "Installing $name service (User=${REAL_USER})..."
        sudo cp "$staged_file" "$service_file"
        sudo systemctl daemon-reload
        if [ "$enable" = true ]; then
            sudo systemctl enable "$name"
            echo "Service installed and enabled."
        else
            echo "Service installed, not enabled."
        fi
    fi
    rm -f "$staged_file"
}

install_unit viz-relay
install_unit "$SERVICE_NAME"
if boots_a_desktop; then
    install_unit dashboard-kiosk
else
    install_unit dashboard-kiosk --no-enable
fi

STATE_DIR="$(getent passwd "$REAL_USER" | cut -d: -f6)/.local/state/auto_battlebot"
sudo -u "$REAL_USER" mkdir -p "$STATE_DIR"

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

echo "Restarting viz-relay and $SERVICE_NAME services..."
sudo systemctl restart viz-relay
sudo systemctl restart "$SERVICE_NAME"
if systemctl is-enabled --quiet dashboard-kiosk 2>/dev/null; then
    echo "Restarting dashboard-kiosk..."
    sudo systemctl restart dashboard-kiosk
fi
echo "Services restarted."
