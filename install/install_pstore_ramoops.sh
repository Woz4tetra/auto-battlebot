#!/bin/bash

# Configure crash-survivable debugging on Jetson:
# - pstore/ramoops kernel support checks
# - panic/sysrq sysctls
# - persistent journald storage
# - systemd-pstore service (if available)
install_pstore_ramoops() {
    local sysctl_file="/etc/sysctl.d/99-auto-battlebot-crash-debug.conf"
    local journald_dir="/etc/systemd/journald.conf.d"
    local journald_file="${journald_dir}/99-auto-battlebot-persistent.conf"
    local supports_pstore=true
    local supports_ramoops=true
    local sysctl_ready=false
    local journald_ready=false
    local pstore_service_known=false
    local pstore_service_ready=true
    local panic_value panic_oops_value sysrq_value
    local needs_reboot=false

    panic_value=$(sysctl -n kernel.panic 2>/dev/null || echo "")
    panic_oops_value=$(sysctl -n kernel.panic_on_oops 2>/dev/null || echo "")
    sysrq_value=$(sysctl -n kernel.sysrq 2>/dev/null || echo "")

    if [ -f "$sysctl_file" ] \
        && rg -q '^kernel\.panic=10$' "$sysctl_file" \
        && rg -q '^kernel\.panic_on_oops=1$' "$sysctl_file" \
        && rg -q '^kernel\.sysrq=1$' "$sysctl_file" \
        && [ "$panic_value" = "10" ] \
        && [ "$panic_oops_value" = "1" ] \
        && [ "$sysrq_value" = "1" ]; then
        sysctl_ready=true
    fi

    if [ -f "$journald_file" ] \
        && rg -q '^Storage=persistent$' "$journald_file" \
        && rg -q '^SystemMaxUse=200M$' "$journald_file" \
        && rg -q '^RuntimeMaxUse=50M$' "$journald_file" \
        && [ -d /var/log/journal ]; then
        journald_ready=true
    fi

    if systemctl list-unit-files | rg -q '^systemd-pstore\.service'; then
        pstore_service_known=true
        if ! systemctl is-enabled --quiet systemd-pstore.service; then
            pstore_service_ready=false
        fi
    fi

    if [ "$sysctl_ready" = true ] && [ "$journald_ready" = true ] && [ "$pstore_service_ready" = true ]; then
        echo "pstore/ramoops crash-debug setup already configured; skipping."
        return
    fi

    echo "Configuring pstore/ramoops crash-debug support..."

    if [ ! -r /proc/config.gz ]; then
        echo "Warning: /proc/config.gz not readable; cannot verify kernel pstore options."
    else
        if ! zgrep -qE '^CONFIG_PSTORE=(y|m)$' /proc/config.gz; then
            supports_pstore=false
        fi
        if ! zgrep -qE '^CONFIG_(PSTORE_RAM|RAMOOPS)=(y|m)$' /proc/config.gz; then
            supports_ramoops=false
        fi
    fi

    if [ "$supports_pstore" = false ]; then
        echo "Warning: kernel is missing CONFIG_PSTORE; pstore will not work until kernel config is updated."
    fi
    if [ "$supports_ramoops" = false ]; then
        echo "Warning: kernel is missing CONFIG_PSTORE_RAM/CONFIG_RAMOOPS; ramoops will not work until kernel config is updated."
    fi

    if modprobe -n ramoops >/dev/null 2>&1; then
        if sudo modprobe ramoops; then
            echo "Loaded ramoops module."
        else
            echo "Warning: failed to load ramoops module. A DT reserved-memory node may be required."
        fi
    else
        echo "Warning: ramoops module is not available in this kernel build."
    fi

    echo "Installing crash sysctl settings: ${sysctl_file}"
    sudo tee "$sysctl_file" >/dev/null <<'EOF'
kernel.panic=10
kernel.panic_on_oops=1
kernel.sysrq=1
EOF
    sudo sysctl --system >/dev/null || true

    echo "Configuring persistent journald storage: ${journald_file}"
    sudo mkdir -p "$journald_dir"
    sudo tee "$journald_file" >/dev/null <<'EOF'
[Journal]
Storage=persistent
SystemMaxUse=200M
RuntimeMaxUse=50M
EOF
    sudo mkdir -p /var/log/journal
    sudo systemctl restart systemd-journald

    if [ "$pstore_service_known" = true ]; then
        echo "Enabling systemd-pstore.service..."
        sudo systemctl enable --now systemd-pstore.service
    else
        echo "Warning: systemd-pstore.service not found; install/enable pstore tooling for your distro."
    fi

    if [ -d /sys/fs/pstore ]; then
        echo "pstore filesystem path detected: /sys/fs/pstore"
    else
        echo "Warning: /sys/fs/pstore is missing. pstore may not be enabled in this kernel/device-tree."
    fi

    needs_reboot=true
    echo ""
    echo "Crash-debug setup complete."
    if [ "$needs_reboot" = true ]; then
        echo "Reboot recommended to ensure all settings are active."
    fi
    echo "Verification commands:"
    echo "  - dmesg | rg -i 'pstore|ramoops'"
    echo "  - mount | rg pstore"
    echo "  - ls -lah /sys/fs/pstore"
    echo "  - sysctl kernel.panic kernel.panic_on_oops kernel.sysrq"
    echo "  - journalctl --disk-usage"
}
