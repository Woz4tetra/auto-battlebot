#!/bin/bash

# Install a systemd oneshot unit that runs `jetson_clocks` at boot, lifting
# DVFS gating so the CPU/GPU/EMC stay at their max frequency. RemainAfterExit
# keeps the unit "active" after the script returns so the kept settings are
# clearly attributed to this service.

install_jetson_clocks() {
    local service_name="auto-battlebot-jetson-clocks.service"
    local service_file="/etc/systemd/system/${service_name}"
    local jetson_clocks_bin="/usr/bin/jetson_clocks"

    if [ ! -x "$jetson_clocks_bin" ]; then
        echo "${jetson_clocks_bin} not found; skipping jetson_clocks service (not a Jetson?)."
        return
    fi

    if [ -f "$service_file" ] \
        && systemctl is-enabled --quiet "$service_name" 2>/dev/null \
        && systemctl is-active --quiet "$service_name" 2>/dev/null; then
        echo "${service_name} already installed, enabled, and active; skipping."
        return
    fi

    echo "Installing ${service_name}..."
    sudo tee "$service_file" >/dev/null <<EOF
[Unit]
Description=Jetson Clocks Service
After=multi-user.target

[Service]
Type=oneshot
ExecStart=${jetson_clocks_bin}
RemainAfterExit=true

[Install]
WantedBy=multi-user.target
EOF

    sudo systemctl daemon-reload
    sudo systemctl enable --now "$service_name"

    echo ""
    echo "${service_name} setup complete."
    echo "Verification commands:"
    echo "  - systemctl status ${service_name}"
    echo "  - sudo ${jetson_clocks_bin} --show"
}
