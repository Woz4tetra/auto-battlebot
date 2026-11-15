#!/bin/bash

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

record_unit_states() {
    for pair in "${RENAMES[@]}"; do
        read -r old new <<<"$pair"
        [ -f "$unit_dir/$old" ] || continue
        present+=("$pair")
        [ "$(systemctl is-enabled "$old" 2>/dev/null)" = "enabled" ] && to_enable+=("$new")
        systemctl is-active --quiet "$old" 2>/dev/null && to_start+=("$new")
    done
}

stop_legacy_units() {
    for pair in "${present[@]}"; do
        read -r old new <<<"$pair"
        sudo systemctl stop "$old" 2>/dev/null || true
        sudo systemctl disable "$old" >/dev/null 2>&1 || true
    done
}

copy_units_with_new_names() {
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
}

restore_unit_states() {
    local unit
    for unit in "${to_enable[@]}"; do
        sudo systemctl enable "$unit" >/dev/null 2>&1
    done
    for unit in "${to_start[@]}"; do
        sudo systemctl start "$unit" || echo "Warning: $unit did not start; check journalctl -u $unit"
    done
    echo "Enabled: ${to_enable[*]:-none}"
    echo "Started: ${to_start[*]:-none}"
}

rename_legacy_services() {
    local unit_dir="/etc/systemd/system"
    local polkit_rule="/etc/polkit-1/rules.d/50-auto-battlebot-dashboard.rules"
    local pair old new

    local sed_args=()
    for pair in "${RENAMES[@]}"; do
        read -r old new <<<"$pair"
        sed_args+=(-e "s/${old//./\\.}/${new}/g")
    done

    local present=() to_enable=() to_start=()
    record_unit_states
    if [ ${#present[@]} -eq 0 ]; then
        return 0
    fi
    echo "Renaming ${#present[@]} legacy systemd unit(s)..."

    stop_legacy_units
    copy_units_with_new_names
    restore_unit_states

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
