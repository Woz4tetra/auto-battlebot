# Dashboard network

- The dashboard cable subnet is 10.42.42.0/24, set up by `install/install_dashboard_network.sh`. The box is 10.42.42.1. It is a routable private subnet rather than 169.254/16 because iOS routes link-local traffic out its primary interface: with Wi-Fi on, the iPad sent the dashboard requests over Wi-Fi instead of the cable.
- `HostServices` runs the jobs the loop must never wait on: opening the dashboard to Wi-Fi and reading the network addresses the System tab shows. It has one worker thread. `set_wifi_access()` queues the request and returns; the worker runs `systemctl start|stop dashboard-wifi.service`, which a polkit rule from the install script lets the app user do for that unit only. The setting persists in the state file and is applied again at startup, so a phone reaches the box after a reboot without the iPad. The worker publishes `/status/network` after each change and every `refresh_period`.
# Dashboard network

`install/install_dashboard_network.sh` sets up the cable port, firewall, and Wi-Fi access for the web dashboard. It serves the dashboard at `http://<hostname>.local` over the Ethernet cable and never renames the box.

## Addressing

The iPad plugs into the box with a USB-C Ethernet dongle. Nothing on that link hands out addresses: the box takes the fixed address `10.42.42.1/24`, and the iPad is set by hand to `10.42.42.2/24` with no router (Settings > Ethernet > the dongle > Configure IP > Manual). A DHCP server would save that step, but it would hand out addresses to a whole shop LAN the day the box is plugged into a router.

The cable used to be link-local (`169.254.42.1/16`). iOS routes link-local traffic out its primary interface, which is Wi-Fi whenever the iPad has both. After a replug Safari sent the dashboard requests over Wi-Fi, where the firewall drops them. Only the cable has a route to `10.42.42.0/24`. Keep this subnet in sync with `include/host/dashboard_network.hpp`. `tests/python/test_scripts_misc.py` checks that the two agree.

## Name resolution

The name is the box's hostname, published by Avahi over mDNS. Avahi answers a hostname query on each interface with that interface's own address, so the iPad on the cable and a phone on Wi-Fi both resolve it to an address they can reach.

## Firewall

The dashboard ports (80 and Foxglove's 8765) are cable-only by default. An nftables table accepts them on loopback and the Ethernet port and drops them everywhere else. The app opens them on Wi-Fi when someone turns on "Allow on Wi-Fi" in the System tab, by starting `dashboard-wifi.service`, which a polkit rule lets the app user do for that one unit only. `--allow-all-interfaces` installs no table and removes the table, units, and polkit rule an earlier run installed.

## nftables table

The firewall rules are written as declare-then-delete, so loading them again replaces the table instead of failing on the one that exists. They live in their own table, so no other ruleset is touched. `service/viz-relay.service` serves the dashboard on port 80.

## Wi-Fi unit

The Wi-Fi access unit is installed but not enabled. The app starts it at boot when the saved setting says so.

## polkit

The polkit rule is a JavaScript `.rules` file, which Ubuntu 24.04's polkit reads. The rule allows start and stop on the Wi-Fi unit only. Unverified: Ubuntu 22.04 (JetPack 6, ZED Box Mini) ships polkit 0.105, which ignores `.rules` JS files. Check that the Wi-Fi toggle works there.
