#!/bin/bash
# Serve the web dashboard at http://<hostname>.local over the Ethernet cable. The box keeps the
# hostname it already has; this script never renames it.
#
# The iPad plugs into the box with a USB-C Ethernet dongle. Nothing on that link hands out
# addresses: the box takes the fixed IPv4 link-local address 169.254.42.1/16, and the iPad
# self-assigns a 169.254.x.x address a few seconds after the cable goes in. A DHCP server would
# work too, but it would hand out addresses to a whole shop LAN the day the box is plugged into a
# router; link-local can't do that.
#
# The name is the box's hostname, published by Avahi over mDNS. Avahi answers a hostname query on
# each interface with that interface's own address, so the iPad on the cable and a phone on Wi-Fi
# both resolve it to an address they can reach.
#
# The dashboard ports (80 and Foxglove's 8765) are cable-only by default: an nftables table accepts
# them on loopback and the Ethernet port and drops them everywhere else. The app opens them on
# Wi-Fi when someone turns on "Allow on Wi-Fi" in the System tab, by starting
# auto-battlebot-dashboard-wifi.service, which a polkit rule lets the app user do for that one
# unit only.
#
# Usage (also sourced by scripts/install_jetson.sh):
#   install/install_dashboard_network.sh [--interface <dev>] [--wifi-interface <dev>]
#                                        [--tailscale-interface <dev>] [--allow-all-interfaces]
#
#   --interface <dev>         Ethernet port the iPad uses. Default: first ethernet device nmcli
#                             lists.
#   --wifi-interface <dev>    Interface the Wi-Fi access toggle opens. Default: first wifi device.
#   --tailscale-interface <dev>
#                             Tailscale interface, always allowed. Default: tailscale0. The rule
#                             matches by name, so it works whether or not tailscaled is up yet.
#   --allow-all-interfaces    No firewall table: the dashboard and Foxglove are open on every
#                             interface all the time. Removes the table, units, and polkit rule an
#                             earlier run installed.

install_dashboard_network() {
    local hostname
    hostname=$(hostname)
    local profile="auto-battlebot-dashboard"
    local address="169.254.42.1/16"
    local table="auto_battlebot_dashboard"
    local nft_file="/etc/auto-battlebot/dashboard.nft"
    local firewall_unit="auto-battlebot-dashboard-firewall.service"
    local wifi_unit="auto-battlebot-dashboard-wifi.service"
    local polkit_rule="/etc/polkit-1/rules.d/50-auto-battlebot-dashboard.rules"
    local dropin_dir="/etc/systemd/system/viz_relay.service.d"
    local dropin="$dropin_dir/dashboard.conf"
    local app_user="${SUDO_USER:-$USER}"
    local app_home
    app_home=$(getent passwd "$app_user" | cut -d: -f6)
    local project_root
    project_root=$(dirname "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")")

    local iface=""
    local wifi_iface=""
    local tailscale_iface="tailscale0"
    local allow_all=false
    while [ $# -gt 0 ]; do
        case "$1" in
            --interface)
                iface="$2"
                shift 2
                ;;
            --wifi-interface)
                wifi_iface="$2"
                shift 2
                ;;
            --tailscale-interface)
                tailscale_iface="$2"
                shift 2
                ;;
            --allow-all-interfaces)
                allow_all=true
                shift
                ;;
            *)
                echo "Unknown option: $1" >&2
                return 2
                ;;
        esac
    done

    # ---- 1. Checks ----
    if ! command -v nmcli >/dev/null 2>&1; then
        echo "Error: nmcli not found. The dashboard network needs NetworkManager." >&2
        return 1
    fi
    if [ -z "$iface" ]; then
        iface=$(nmcli -t -f DEVICE,TYPE device status | awk -F: '$2 == "ethernet" { print $1; exit }')
    fi
    if [ -z "$wifi_iface" ]; then
        wifi_iface=$(nmcli -t -f DEVICE,TYPE device status | awk -F: '$2 == "wifi" { print $1; exit }')
    fi
    if [ -z "$iface" ]; then
        echo "Error: no Ethernet device found. Pass --interface <dev>." >&2
        return 1
    fi
    if ip route show default | grep -qw "dev $iface"; then
        echo "Error: $iface carries the default route. Converting it would cut the box off the" >&2
        echo "network and drop any SSH session on it. Pick another port with --interface, or move" >&2
        echo "the uplink to Wi-Fi first." >&2
        return 1
    fi
    echo "Dashboard cable port: $iface"
    echo "Wi-Fi access interface: ${wifi_iface:-none}"
    echo "Tailscale interface (always allowed): $tailscale_iface"

    # ---- 2. Packages ----
    sudo apt-get install -y avahi-daemon avahi-utils nftables

    # ---- 3. NetworkManager profile ----
    # Deleted and re-added, so rerunning is safe. mdns off keeps systemd-resolved off port 5353
    # for Avahi. Priority 100 beats the distro's "Wired connection 1" at 0.
    sudo nmcli connection delete "$profile" >/dev/null 2>&1 || true
    sudo nmcli connection add type ethernet ifname "$iface" con-name "$profile" \
        ipv4.method manual ipv4.addresses "$address" ipv4.never-default yes \
        ipv6.method link-local connection.mdns no \
        connection.autoconnect yes connection.autoconnect-priority 100
    # Fails with no cable plugged in, which is fine: NetworkManager brings it up on carrier.
    sudo nmcli connection up "$profile" >/dev/null 2>&1 ||
        echo "No carrier on $iface yet; the profile comes up when the cable goes in."

    # ---- 4. Avahi ----
    sudo systemctl enable --now avahi-daemon
    local allow_line
    allow_line=$(grep -E '^\s*allow-interfaces\s*=' /etc/avahi/avahi-daemon.conf 2>/dev/null || true)
    if [ -n "$allow_line" ]; then
        if ! echo "$allow_line" | grep -qw "$iface" ||
            { [ -n "$wifi_iface" ] && ! echo "$allow_line" | grep -qw "$wifi_iface"; }; then
            echo "Warning: /etc/avahi/avahi-daemon.conf has '$allow_line', which leaves out $iface" \
                "or ${wifi_iface:-the Wi-Fi}. Not editing it; add both for the name to resolve."
        fi
    fi

    # ---- 5. Firewall ----
    if [ "$allow_all" = true ]; then
        echo "--allow-all-interfaces: removing the dashboard firewall"
        sudo systemctl disable --now "$wifi_unit" >/dev/null 2>&1 || true
        sudo systemctl disable --now "$firewall_unit" >/dev/null 2>&1 || true
        sudo rm -f "/etc/systemd/system/$wifi_unit" "/etc/systemd/system/$firewall_unit" \
            "$polkit_rule" "$nft_file"
        sudo nft delete table inet "$table" >/dev/null 2>&1 || true
    else
        sudo mkdir -p "$(dirname "$nft_file")"
        # Declare-then-delete, so loading it again replaces the table instead of failing on the
        # one that exists. Its own table, so no other ruleset is touched. 8080 is the relay's
        # default HTTP port, dropped too in case the port 80 drop-in is missing.
        sudo tee "$nft_file" >/dev/null <<EOF
table inet $table
delete table inet $table
table inet $table {
    set allowed_ifaces {
        type ifname
        elements = { "lo", "$iface", "$tailscale_iface" }
    }
    chain input {
        type filter hook input priority filter; policy accept;
        iifname @allowed_ifaces accept
        tcp dport { 80, 8080, 8765 } drop
    }
}
EOF
        sudo tee "/etc/systemd/system/$firewall_unit" >/dev/null <<EOF
[Unit]
Description=Limit the auto_battlebot dashboard ports to the cable
Before=network-pre.target
Wants=network-pre.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/sbin/nft -f $nft_file
ExecStop=/usr/sbin/nft delete table inet $table

[Install]
WantedBy=multi-user.target
EOF
        if [ -n "$wifi_iface" ]; then
            # Not enabled: the app starts it at boot when the saved setting says so.
            sudo tee "/etc/systemd/system/$wifi_unit" >/dev/null <<EOF
[Unit]
Description=Allow the auto_battlebot dashboard on Wi-Fi ($wifi_iface)
Requires=$firewall_unit
After=$firewall_unit

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/sbin/nft 'add element inet $table allowed_ifaces { "$wifi_iface" }'
ExecStop=-/usr/sbin/nft 'delete element inet $table allowed_ifaces { "$wifi_iface" }'
EOF
            # Ubuntu 24.04's polkit reads JavaScript rules. Start and stop on this unit only.
            sudo tee "$polkit_rule" >/dev/null <<EOF
// Installed by auto-battlebot install/install_dashboard_network.sh.
polkit.addRule(function (action, subject) {
    if (action.id == "org.freedesktop.systemd1.manage-units" &&
        action.lookup("unit") == "$wifi_unit" &&
        (action.lookup("verb") == "start" || action.lookup("verb") == "stop") &&
        subject.user == "$app_user") {
        return polkit.Result.YES;
    }
});
EOF
        else
            echo "No Wi-Fi interface: skipping the Wi-Fi access unit."
        fi
    fi

    # ---- 6. Port 80 for viz_relay ----
    # The capability lives in the unit, so rebuilding or reinstalling viz_relay keeps it; setcap
    # on the binary would not survive `cmake --install`. The relay never runs as root.
    local relay_bin="$app_home/.local/bin/viz_relay"
    local help_bin="$relay_bin"
    [ -x "$help_bin" ] || help_bin="$project_root/build/viz_relay"
    if [ -x "$help_bin" ] && "$help_bin" --help 2>/dev/null | grep -q -- "--http-port"; then
        sudo mkdir -p "$dropin_dir"
        sudo tee "$dropin" >/dev/null <<EOF
[Service]
AmbientCapabilities=CAP_NET_BIND_SERVICE
ExecStart=
ExecStart=$relay_bin --http-port 80
EOF
    else
        # An unknown flag would put the relay in a crash loop.
        echo "viz_relay has no --http-port yet (or is not built). Skipping the port 80 drop-in;"
        echo "rerun this script after building."
    fi

    # ---- 7. Enable and restart ----
    sudo systemctl daemon-reload
    if [ "$allow_all" != true ]; then
        sudo systemctl enable "$firewall_unit"
        sudo systemctl restart "$firewall_unit"
    fi
    sudo systemctl restart avahi-daemon
    if systemctl is-active --quiet viz_relay.service; then
        sudo systemctl restart viz_relay.service
    fi

    # ---- 8. Verification ----
    echo ""
    echo "Dashboard network setup complete. Dashboard: http://$hostname.local"
    echo "Verification:"
    echo "  - avahi-resolve -4 -n $hostname.local          # prints the box's addresses"
    if [ "$allow_all" != true ]; then
        echo "  - sudo nft list set inet $table allowed_ifaces   # lo, $iface, $tailscale_iface"
        [ -n "$wifi_iface" ] &&
            echo "  - sudo -u $app_user systemctl start $wifi_unit   # no password prompt"
    fi
    echo "  - On the iPad: Settings > Ethernet shows a self-assigned 169.254.x.x address, and"
    echo "    Safari opens http://$hostname.local"
}

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    set -e
    install_dashboard_network "$@"
fi
