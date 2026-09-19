#!/bin/bash

# Build and install the e-CAM25_CUONX camera driver and device tree overlay on a
# JetPack 7 Jetson (L4T 39.2.x, kernel 6.8). e-con ships no JetPack 7 package, so
# the driver is built from the vendored source in third_party/ecam25/.
#
# Installs:
#   ecam25_ar0234.ko -> /lib/modules/$(uname -r)/updates/drivers/media/i2c/
#   the .dtbo        -> /boot/, registered through jetson-io
#
# The overlay only takes effect after a reboot. This does not reboot.
# Details and the verification ladder: docs/ecam25_camera_setup.md

install_ecam25_camera() {
    local SCRIPT_DIR
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    local PROJECT_ROOT
    PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

    local build_dir="$PROJECT_ROOT/build-ecam25"
    local kernel_release
    kernel_release="$(uname -r)"
    local module_dir="/lib/modules/${kernel_release}/updates/drivers/media/i2c"
    local installed_module="$module_dir/ecam25_ar0234.ko"
    local overlay_name="AR0234 Sensor 4lane"
    local dtbo_name="tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo"
    local config_tool="/opt/nvidia/jetson-io/config-by-hardware.py"
    local extlinux="/boot/extlinux/extlinux.conf"

    # Already complete: the module is installed and built for the running kernel,
    # the overlay is in /boot, and it is registered for the next boot.
    if [ -f "$installed_module" ] \
        && [ "$(modinfo -F vermagic "$installed_module" 2> /dev/null | awk '{print $1}')" = "$kernel_release" ] \
        && [ -f "/boot/$dtbo_name" ] \
        && grep -q "$dtbo_name" "$extlinux" 2> /dev/null; then
        echo "e-CAM25 camera driver already installed for ${kernel_release}; skipping."
        return 0
    fi

    echo "Building the e-CAM25 camera driver for ${kernel_release}..."
    "$PROJECT_ROOT/scripts/build_ecam25_driver.sh" -o "$build_dir"

    local module="$build_dir/ecam25_ar0234.ko"
    local dtbo="$build_dir/$dtbo_name"

    echo "Installing ecam25_ar0234.ko into $module_dir"
    sudo install -D -m 644 "$module" "$installed_module"
    sudo depmod -a

    echo "Installing $dtbo_name into /boot"
    sudo install -m 644 "$dtbo" "/boot/$dtbo_name"
    sudo cp -a "$extlinux" "${extlinux}.bak.$(date +%Y%m%d%H%M%S)"

    if grep -q "$dtbo_name" "$extlinux" 2> /dev/null; then
        echo "Overlay already registered in $extlinux; skipping jetson-io."
    elif [ -x "$config_tool" ]; then
        # The CSI header number differs between JetPack releases: JetPack 6 had a
        # 24pin CSI header, JetPack 7 has a 22pin one. Read it rather than assume.
        #   Header 1 [default]: Jetson 40pin Header
        #     Available hardware modules:
        #     1. Adafruit SPH0645LM4H
        #   Header 2: Jetson 22pin CSI Connector
        #     1. AR0234 Sensor 4lane
        # -l needs root: Board() reads /sys and /proc nodes and otherwise dies
        # with a traceback on stderr and an exit code of 0.
        local header_index
        header_index=$(sudo "$config_tool" -l 2> /dev/null \
            | awk -v name="$overlay_name" '
                /^Header [0-9]+/ { current = $2; gsub(/[^0-9]/, "", current) }
                current && index($0, name) { print current; exit }')
        if [ -n "$header_index" ]; then
            echo "Registering overlay on header $header_index"
            sudo "$config_tool" -n "${header_index}=${overlay_name}"
        else
            echo "Warning: jetson-io did not list '$overlay_name'." >&2
            echo "Run '$config_tool -l' and register it by hand, or add to $extlinux:" >&2
            echo "  FDT /boot/kernel_tegra234-p3768-0000+p3767-0000-nv.dtb" >&2
            echo "  OVERLAYS /boot/$dtbo_name" >&2
        fi
    else
        echo "Warning: '$config_tool' not found; add the overlay to $extlinux by hand." >&2
    fi

    # This is a Canonical-derived kernel, so apt will offer a newer -tegra ABI. A
    # kernel update leaves the module unloadable and the camera silently absent.
    sudo apt-mark hold "linux-image-${kernel_release}" "linux-headers-${kernel_release}" > /dev/null 2>&1 || true

    echo "e-CAM25 camera driver installed. Reboot for the overlay to take effect."
    echo "Verification ladder: docs/ecam25_camera_setup.md"
}
