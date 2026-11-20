#!/bin/bash


install_jetson_stability() {
    install_cgroup_disable_memory
    install_jetson_service_masks
}

install_cgroup_disable_memory() {
    local extlinux_conf="${1:-/boot/extlinux/extlinux.conf}"
    local arg="cgroup_disable=memory"
    local backup

    local kernel_major
    kernel_major=$(uname -r | cut -d. -f1)
    if [ "${kernel_major:-0}" -ge 6 ]; then
        echo "Kernel $(uname -r) does not need ${arg}; skipping (JetPack 6 / 5.15 only)."
        return
    fi

    if [ ! -f "$extlinux_conf" ]; then
        echo "${extlinux_conf} not found; skipping cgroup_disable=memory step (not a Jetson?)."
        return
    fi

    if rg -q "^\s*APPEND\b.*\b${arg}\b" "$extlinux_conf"; then
        echo "${arg} already present on extlinux APPEND line; skipping."
        return
    fi

    backup="${extlinux_conf}.bak.$(date +%Y%m%d-%H%M%S)"
    echo "Adding ${arg} to ${extlinux_conf} APPEND line..."
    echo "  Backup: ${backup}"
    sudo cp -a "$extlinux_conf" "$backup"
    sudo sed -i "/^\s*APPEND\s/ s/$/ ${arg}/" "$extlinux_conf"

    if ! rg -q "^\s*APPEND\b.*\b${arg}\b" "$extlinux_conf"; then
        echo "Error: failed to add ${arg}; restoring ${backup}."
        sudo cp -a "$backup" "$extlinux_conf"
        return 1
    fi

    echo "  -> Reboot required for ${arg} to take effect."
    echo "     Verify after reboot with: cat /proc/cmdline; cat /proc/cgroups"
}

install_jetson_service_masks() {
    local system_services=(
        bluetooth.service
    )
    local user_services=(
        evolution-addressbook-factory.service
        evolution-calendar-factory.service
        evolution-source-registry.service
        tracker-miner-fs-3.service
        tracker-extract-3.service
        tracker-xdg-portal-3.service
        gvfs-afc-volume-monitor.service
        gvfs-gphoto2-volume-monitor.service
        gvfs-mtp-volume-monitor.service
        gvfs-goa-volume-monitor.service
        org.gnome.SettingsDaemon.PrintNotifications.service
        org.gnome.SettingsDaemon.Smartcard.service
        org.gnome.SettingsDaemon.Wacom.service
        org.gnome.SettingsDaemon.Sharing.service
    )

    local s state
    for s in "${system_services[@]}"; do
        if ! systemctl list-unit-files --no-legend 2>/dev/null \
                | rg -q "^${s}\b"; then
            echo "system service '${s}' not present; skipping."
            continue
        fi
        state=$(systemctl is-enabled "$s" 2>/dev/null || true)
        if [ "$state" = "masked" ]; then
            echo "system service '${s}' already masked; skipping."
        else
            echo "Masking system service '${s}'..."
            sudo systemctl mask --no-block "$s" >/dev/null
        fi
    done

    if [ "${USER:-root}" = "root" ]; then
        echo "Running as root; skipping user-level service masks."
        echo "  -> Re-run install as the user that owns the auto_battlebot session."
        return
    fi

    local user_units_dir="${HOME}/.config/systemd/user"
    mkdir -p "$user_units_dir"

    local target
    for s in "${user_services[@]}"; do
        target="${user_units_dir}/${s}"
        if [ -L "$target" ] && [ "$(readlink "$target")" = "/dev/null" ]; then
            echo "user service '${s}' already masked; skipping."
        else
            echo "Masking user service '${s}' for ${USER}..."
            ln -sfn /dev/null "$target"
        fi
    done

    local runtime_dir="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
    if [ -S "${runtime_dir}/systemd/private" ] || [ -d "${runtime_dir}/systemd" ]; then
        XDG_RUNTIME_DIR="$runtime_dir" systemctl --user daemon-reload \
            >/dev/null 2>&1 || true
    fi
}
