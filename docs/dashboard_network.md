# Dashboard network

`install/install_dashboard_network.sh` sets up the cable port, firewall, and Wi-Fi access for the web dashboard.

## nftables table

The firewall rules are written as declare-then-delete, so loading them again replaces the table instead of failing on the one that exists. They live in their own table, so no other ruleset is touched. `service/viz-relay.service` serves the dashboard on port 80.

## Wi-Fi unit

The Wi-Fi access unit is installed but not enabled. The app starts it at boot when the saved setting says so.

## polkit

The polkit rule is a JavaScript `.rules` file, which Ubuntu 24.04's polkit reads. The rule allows start and stop on the Wi-Fi unit only. Unverified: Ubuntu 22.04 (JetPack 6, ZED Box Mini) ships polkit 0.105, which ignores `.rules` JS files. Check that the Wi-Fi toggle works there.
