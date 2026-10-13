#!/bin/bash
# Renames the systemd units earlier installs created under their old names. Each unit keeps its
# enabled and running state, and the old and new app units never run side by side.
#
# Idempotent: on a box with no old unit it does nothing. service/build_and_install_service.sh runs
# it before installing anything, so a deploy migrates a box on its own. Delete it once no box has
# an old name left.
#
# A unit file is copied with every old unit name in it rewritten (dashboard-wifi Requires= the
# firewall unit, the kiosk orders After= the relay), and the polkit rule that lets the app start
# the Wi-Fi unit is rewritten the same way. The viz_relay port 80 drop-in is deleted, not moved:
# service/viz-relay.service carries --http-port 80 itself now. Helper scripts under
# /usr/local/sbin keep their names; only units were renamed.
#
# Usage: install/rename_legacy_services.sh

RENAMES=(
    "auto_battlebot.service auto-battlebot.service"
    "viz_relay.service viz-relay.service"
    "dashboard_kiosk.service dashboard-kiosk.service"
    "auto-battlebot-jetson-clocks.service jetson-clocks.service"
    "auto-battlebot-dashboard-firewall.service dashboard-firewall.service"
    "auto-battlebot-dashboard-wifi.service dashboard-wifi.service"
    "auto-battlebot-ds3231.service ds3231.service"
    "auto-battlebot-uvcvideo-rt.service uvcvideo-rt.service"
)

rename_legacy_services() {
    local unit_dir="/etc/systemd/system"
    local polkit_rule="/etc/polkit-1/rules.d/50-auto-battlebot-dashboard.rules"
    local pair old new

    local sed_args=()
    for pair in "${RENAMES[@]}"; do
        read -r old new <<<"$pair"
        sed_args+=(-e "s/${old//./\\.}/${new}/g")
    done

    # 1. Record states before stopping anything. Stopping the firewall unit also stops the Wi-Fi
    #    unit that Requires= it, so reading states during the renames would lose that one.
    local present=() to_enable=() to_start=()
    for pair in "${RENAMES[@]}"; do
        read -r old new <<<"$pair"
        [ -f "$unit_dir/$old" ] || continue
        present+=("$pair")
        [ "$(systemctl is-enabled "$old" 2>/dev/null)" = "enabled" ] && to_enable+=("$new")
        systemctl is-active --quiet "$old" 2>/dev/null && to_start+=("$new")
    done
    if [ ${#present[@]} -eq 0 ]; then
        return 0
    fi
    echo "Renaming ${#present[@]} legacy systemd unit(s)..."

    # 2. Stop and disable every old unit.
    for pair in "${present[@]}"; do
        read -r old new <<<"$pair"
        sudo systemctl stop "$old" 2>/dev/null || true
        sudo systemctl disable "$old" >/dev/null 2>&1 || true
    done

    # 3. Copy each file to its new name with the references rewritten, then remove the old one.
    for pair in "${present[@]}"; do
        read -r old new <<<"$pair"
        echo "  $old -> $new"
        sudo sed "${sed_args[@]}" "$unit_dir/$old" | sudo tee "$unit_dir/$new" >/dev/null
        sudo rm -f "$unit_dir/$old"
        if [ -d "$unit_dir/$old.d" ]; then
            if [ "$old" = "viz_relay.service" ]; then
                sudo rm -rf "$unit_dir/$old.d"
            else
                sudo mv "$unit_dir/$old.d" "$unit_dir/$new.d"
            fi
        fi
    done
    if sudo test -f "$polkit_rule"; then
        sudo sed -i "${sed_args[@]}" "$polkit_rule"
    fi
    sudo systemctl daemon-reload

    # 4. Restore the recorded states. RENAMES lists the firewall before the Wi-Fi unit and the
    #    relay before the app, so starting in this order satisfies Requires= and After=.
    local unit
    for unit in "${to_enable[@]}"; do
        sudo systemctl enable "$unit" >/dev/null 2>&1
    done
    for unit in "${to_start[@]}"; do
        sudo systemctl start "$unit" || echo "Warning: $unit did not start; check journalctl -u $unit"
    done
    echo "Enabled: ${to_enable[*]:-none}"
    echo "Started: ${to_start[*]:-none}"

    local leftovers
    leftovers=$(sudo grep -rlE "auto_battlebot\.service|viz_relay\.service|dashboard_kiosk\.service|auto-battlebot-[a-z0-9-]+\.service" \
        "$unit_dir" /etc/polkit-1 2>/dev/null || true)
    if [ -n "$leftovers" ]; then
        echo "Warning: old unit names still referenced in:"
        echo "$leftovers"
    fi
}

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    set -e
    rename_legacy_services "$@"
fi
