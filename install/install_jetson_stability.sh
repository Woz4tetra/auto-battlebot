#!/bin/bash

# Stability workarounds for the Jetson.
#
# Three independent steps, all idempotent:
#   1. Add "cgroup_disable=memory" to the extlinux APPEND line, on the L4T 36.4 /
#      5.15.148-tegra kernel only. Works around a NULL-deref panic in kswapd0 ->
#      workingset_update_node -> list_lru_add that is reachable through
#      memory-cgroup-aware list_lru entries on that kernel. With memcg disabled at
#      boot, the buggy code path is unreachable. Skipped on kernel 6.x (JetPack 7),
#      where the bug is fixed and disabling memcg would cost real accounting.
#   2. Pin the ZED's USB link: disable USB autosuspend globally and apply
#      USB_QUIRK_NO_LPM to the camera, both via the extlinux APPEND line. Chasing
#      frames that arrive cyclically rotated by 256 px on JetPack 7.
#   3. Mask non-essential desktop services that create background cgroup churn
#      on a kiosk (Bluetooth, GNOME Tracker, Evolution data daemons, several
#      gvfs volume monitors, and unused gnome-settings-daemon services).
#
# Notes:
#   - Steps 1 and 2 require a reboot to take effect.
#   - Step 3 keeps gnome-shell, gdm, the screensaver-proxy daemon, audio, and
#     core gvfs running so the X session and the auto_battlebot UI continue
#     to work.

install_jetson_stability() {
    install_cgroup_disable_memory
    install_usb_zed_quirks
    install_jetson_service_masks
}

# Adds one kernel argument to the extlinux APPEND line, once. Backs the file up first and
# restores it if the edit does not take.
append_extlinux_arg() {
    local arg="$1"
    local extlinux_conf="/boot/extlinux/extlinux.conf"
    local backup

    if [ ! -f "$extlinux_conf" ]; then
        echo "${extlinux_conf} not found; skipping ${arg} (not a Jetson?)."
        return 0
    fi

    if rg -N "^\s*APPEND\s" "$extlinux_conf" | rg -qF -- "$arg"; then
        echo "${arg} already on the extlinux APPEND line; skipping."
        return 0
    fi

    backup="${extlinux_conf}.bak.$(date +%Y%m%d-%H%M%S)"
    echo "Adding ${arg} to ${extlinux_conf} APPEND line..."
    echo "  Backup: ${backup}"
    sudo cp -a "$extlinux_conf" "$backup"
    sudo sed -i "/^\s*APPEND\s/ s|\$| ${arg}|" "$extlinux_conf"

    if ! rg -N "^\s*APPEND\s" "$extlinux_conf" | rg -qF -- "$arg"; then
        echo "Error: failed to add ${arg}; restoring ${backup}."
        sudo cp -a "$backup" "$extlinux_conf"
        return 1
    fi
    return 0
}

install_usb_zed_quirks() {
    # These do NOT fix the frame-rotation bug. Measured on 2026-09-22 with both settings verified
    # active: 7 rotated frames in 3000 (0.23%) against 2 in 625 (0.32%) before, which is the same
    # rate. Kept anyway, because runtime-suspending a camera that streams continuously is wrong on
    # its own terms and Stereolabs' own udev rule asks for it. The rotation is still open; see the
    # column-1024 seam test for how to measure it.
    #
    # Neither setting can be applied at runtime, which is the only reason they are here:
    #
    #   * power/control reverts to "auto" on every re-enumeration. Stereolabs' 99-slabs.rules
    #     writes "on" but only matches ACTION=="add", which a re-authorize does not emit.
    #     usbcore.autosuspend=-1 settles it globally instead of racing udev.
    #   * usb3_hardware_lpm_u1/u2 are read-only in sysfs, and the quirk list is only consulted
    #     during full enumeration (usb_detect_quirks in usb_new_device), so USB_QUIRK_NO_LPM has
    #     to be on the command line at boot. Writing /sys/module/usbcore/parameters/quirks only
    #     affects devices connected afterwards.
    #
    # 2b03:f880 is the ZED camera, 2b03:f881 its HID interface. Harmless on a box without one.
    local arg
    for arg in "usbcore.autosuspend=-1" "usbcore.quirks=2b03:f880:k,2b03:f881:k"; do
        append_extlinux_arg "$arg" || return 1
    done

    echo "  -> Reboot required for the USB settings to take effect."
    echo "     Verify after reboot (2-1.2 is the ZED's path on the handheld; check lsusb -t):"
    echo "       cat /proc/cmdline                                          # both args present"
    echo "       cat /sys/bus/usb/devices/2-1.2/power/autosuspend_delay_ms  # negative: never suspends"
    echo "       ls /sys/bus/usb/devices/2-1.2/power/ | grep lpm            # NO output: quirk applied"
    echo "     power/control still reads 'auto' and the lpm attributes are absent rather than"
    echo "     'disabled'; the kernel only creates them when lpm_capable is 1."
}

install_cgroup_disable_memory() {
    local extlinux_conf="/boot/extlinux/extlinux.conf"
    local arg="cgroup_disable=memory"
    local backup

    # The panic this works around is specific to the 5.15 Tegra kernel that JetPack 6
    # shipped. JetPack 7 runs 6.8, where it is fixed, and disabling memcg there is not
    # free: it also takes out cgroup v2 memory accounting, systemd MemoryMax=, and
    # systemd-oomd. Only apply it on the kernel that needs it.
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
    # User-level GNOME / GVFS services that are not needed on a kiosk and
    # contribute to background cgroup churn.
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
