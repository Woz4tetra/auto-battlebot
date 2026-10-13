#!/bin/bash

# Install udev rules for USB serial devices used by auto-battlebot.
# - OpenTX / EdgeTX transmitter (VID=0483, PID=5740): creates /dev/opentx symlink
# - ESP-NOW dongle (docs/plans/espnow_link_plan.md): creates /dev/espnow_dongle symlink
# - mr_stabs_mk2 on USB: creates /dev/mr_stabs symlink
# - Adds the current user to the dialout group
# - Disables ModemManager probing of the transmitter (prevents CRSF frame corruption)
# - Ensures the cdc_acm kernel module loads on boot

install_udev_rules() {
    local RULES_FILE="/etc/udev/rules.d/99-auto-battlebot.rules"
    local MODULES_FILE="/etc/modules-load.d/cdc_acm.conf"

    # -------------------------------------------------------------------------
    # 1. dialout group membership
    # -------------------------------------------------------------------------
    if groups "$USER" | grep -qw dialout; then
        echo "User '$USER' is already in the dialout group"
    else
        echo "Adding '$USER' to the dialout group..."
        sudo usermod -aG dialout "$USER"
        echo "  -> You must log out and back in (or reboot) for this to take effect"
    fi

    # -------------------------------------------------------------------------
    # 2. udev rules
    # -------------------------------------------------------------------------
    echo "Writing udev rules to $RULES_FILE..."
    sudo tee "$RULES_FILE" > /dev/null <<'EOF'
# auto-battlebot USB serial device rules

# OpenTX / EdgeTX transmitter (STM32 USB CDC, VID=0483 PID=5740)
# Creates a stable /dev/opentx symlink regardless of enumeration order.
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", \
    SYMLINK+="opentx", \
    GROUP="dialout", MODE="0660", \
    TAG+="systemd"

# Prevent ModemManager from probing the transmitter on plug-in.
# Without this, MM sends AT commands that corrupt CRSF framing for ~10 s.
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", \
    ENV{ID_MM_DEVICE_IGNORE}="1"

# ESP-NOW dongle (firmware/espnow_dongle). ESP32-S3 USB-Serial/JTAG reports 303a:1001 and its MAC as
# the serial number. Matched by serial because mr_stabs_mk2 is the same chip and shows the same
# VID/PID when it is plugged in for a USB flash. One line per dongle: add the bench and match
# dongles here as they are built (the dongle prints its MAC at boot).
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", \
    ATTRS{serial}=="D8:3B:DA:8E:ED:2C", \
    SYMLINK+="espnow_dongle", ENV{ID_MM_DEVICE_IGNORE}="1", \
    GROUP="dialout", MODE="0660"

# mr_stabs_mk2 itself on USB (built with ARDUINO_USB_MODE=1), for scripts/upload and USB rescue.
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", \
    ATTRS{serial}=="F4:12:FA:59:5C:B4", \
    SYMLINK+="mr_stabs", ENV{ID_MM_DEVICE_IGNORE}="1", \
    GROUP="dialout", MODE="0660"

# Generic fallback: ensure all ttyACM/ttyUSB nodes are group-accessible
SUBSYSTEM=="tty", KERNEL=="ttyACM[0-9]*", GROUP="dialout", MODE="0660"
SUBSYSTEM=="tty", KERNEL=="ttyUSB[0-9]*", GROUP="dialout", MODE="0660"
EOF

    # -------------------------------------------------------------------------
    # 3. Ensure cdc_acm module loads on boot
    # -------------------------------------------------------------------------
    if [ -f "$MODULES_FILE" ] && grep -q "^cdc_acm" "$MODULES_FILE"; then
        echo "cdc_acm already listed in $MODULES_FILE"
    else
        echo "Enabling cdc_acm module on boot ($MODULES_FILE)..."
        echo "cdc_acm" | sudo tee "$MODULES_FILE" > /dev/null
    fi

    # Load the module now if it isn't already
    if ! lsmod | grep -q "^cdc_acm"; then
        echo "Loading cdc_acm kernel module..."
        sudo modprobe cdc_acm
    else
        echo "cdc_acm module already loaded"
    fi

    # -------------------------------------------------------------------------
    # 4. Reload udev and re-trigger devices
    # -------------------------------------------------------------------------
    echo "Reloading udev rules..."
    sudo udevadm control --reload-rules
    sudo udevadm trigger --subsystem-match=tty

    # -------------------------------------------------------------------------
    # 5. Verify
    # -------------------------------------------------------------------------
    echo ""
    echo "--- Verification ---"
    if [ -e /dev/opentx ]; then
        echo "/dev/opentx -> $(readlink /dev/opentx)  (transmitter detected)"
    else
        echo "/dev/opentx not present (plug in the transmitter to verify)"
    fi

    if [ -e /dev/espnow_dongle ]; then
        echo "/dev/espnow_dongle -> $(readlink /dev/espnow_dongle)  (dongle detected)"
    else
        echo "/dev/espnow_dongle not present (plug in the dongle to verify)"
    fi

    echo ""
    echo "udev rules installed successfully."
    echo "If you were just added to the dialout group, please log out and back in."
}
