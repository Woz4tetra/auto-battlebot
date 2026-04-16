#!/bin/bash

# Configure DS3231 RTC on Jetson so system time is restored on boot.
# Usage:
#   install_ds3231_rtc [i2c_bus] [i2c_address]
# Example:
#   install_ds3231_rtc 7 0x68

install_ds3231_rtc() {
    local i2c_bus="${1:-7}"
    local i2c_address="${2:-0x68}"
    local modules_file="/etc/modules-load.d/auto-battlebot-rtc.conf"
    local init_script="/usr/local/sbin/auto-battlebot-ds3231-init.sh"
    local service_file="/etc/systemd/system/auto-battlebot-ds3231.service"
    local i2c_adapter="/sys/class/i2c-adapter/i2c-${i2c_bus}"
    local addr_hex addr_dec device_key device_path

    addr_dec=$((i2c_address))
    addr_hex=$(printf "0x%02x" "$addr_dec")
    device_key=$(printf "%d-%04x" "$i2c_bus" "$addr_dec")
    device_path="/sys/bus/i2c/devices/${device_key}"

    if [ -x "$init_script" ] && [ -f "$service_file" ] && [ -e "$device_path" ] \
        && systemctl is-enabled --quiet auto-battlebot-ds3231.service; then
        echo "DS3231 RTC setup already installed and enabled; skipping."
        return
    fi

    echo "Configuring DS3231 RTC on i2c-${i2c_bus} at address ${addr_hex}..."

    for cmd in hwclock modprobe systemctl i2cdetect; do
        if ! command -v "$cmd" >/dev/null 2>&1; then
            echo "Error: required command '$cmd' was not found."
            echo "Install missing dependencies, then re-run this script."
            exit 1
        fi
    done

    if [ ! -d "$i2c_adapter" ]; then
        echo "Error: I2C adapter '$i2c_adapter' does not exist."
        echo "Check your DS3231 wiring and selected bus number."
        exit 1
    fi

    echo "Ensuring rtc-ds1307 kernel module loads on boot..."
    echo "rtc-ds1307" | sudo tee "$modules_file" >/dev/null
    sudo modprobe rtc-ds1307

    if [ ! -e "$device_path" ]; then
        echo "Binding DS3231 via sysfs (${device_key})..."
        echo "ds3231 ${addr_hex}" | sudo tee "${i2c_adapter}/new_device" >/dev/null
    else
        echo "DS3231 already bound at ${device_key}."
    fi

    if [ ! -e "$device_path" ]; then
        echo "Error: DS3231 device did not appear at ${device_path}."
        echo "Check wiring and verify with: sudo i2cdetect -y ${i2c_bus}"
        exit 1
    fi

    echo "Installing boot-time DS3231 init helper: ${init_script}"
    sudo tee "$init_script" >/dev/null <<EOF
#!/bin/bash
set -e

I2C_BUS="${i2c_bus}"
I2C_ADDRESS="${addr_hex}"
I2C_ADAPTER="/sys/class/i2c-adapter/i2c-\${I2C_BUS}"
DEVICE_KEY=\$(printf "%d-%04x" "\${I2C_BUS}" "\$((I2C_ADDRESS))")
DEVICE_PATH="/sys/bus/i2c/devices/\${DEVICE_KEY}"

modprobe rtc-ds1307

if [ -d "\${I2C_ADAPTER}" ] && [ ! -e "\${DEVICE_PATH}" ]; then
    echo "ds3231 \${I2C_ADDRESS}" > "\${I2C_ADAPTER}/new_device"
fi

hwclock --hctosys --utc || true
EOF
    sudo chmod 755 "$init_script"

    echo "Installing and enabling systemd service: ${service_file}"
    sudo tee "$service_file" >/dev/null <<EOF
[Unit]
Description=Initialize DS3231 RTC and set system time
DefaultDependencies=no
After=systemd-modules-load.service
Before=sysinit.target time-sync.target
Wants=systemd-modules-load.service
ConditionPathExists=${i2c_adapter}

[Service]
Type=oneshot
ExecStart=${init_script}
RemainAfterExit=yes

[Install]
WantedBy=sysinit.target
EOF

    sudo systemctl daemon-reload
    sudo systemctl enable auto-battlebot-ds3231.service
    sudo systemctl restart auto-battlebot-ds3231.service

    echo "Running one-time synchronization..."
    sudo hwclock --hctosys --utc || echo "Warning: hwclock --hctosys failed (clock may be unset)."
    sudo hwclock --systohc --utc || echo "Warning: hwclock --systohc failed."

    echo ""
    echo "DS3231 setup complete."
    echo "Verification checklist:"
    echo "  - sudo i2cdetect -r -y ${i2c_bus}"
    echo "  - ls -l /sys/bus/i2c/devices/${device_key}"
    echo "  - timedatectl"
    echo "  - sudo hwclock -r"
}
