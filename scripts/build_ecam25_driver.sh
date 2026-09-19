#!/bin/bash
# Build the e-CAM25_CUONX camera driver and its device tree overlay for JetPack 7
# (L4T 39.2.x, kernel 6.8).
#
# Runs natively on the Jetson by default. It needs only the two header packages
# that JetPack already installs:
#   nvidia-l4t-kernel-headers      -> /lib/modules/$(uname -r)/build
#   nvidia-l4t-kernel-oot-headers  -> /usr/src/nvidia/nvidia-public
# No source sync, no full nvidia-oot rebuild.
#
# Usage:
#   scripts/build_ecam25_driver.sh [-o OUTPUT_DIR]
#
# Cross-compiling from the dev box instead, against an unpacked JetPack 7 BSP:
#   KDIR=<...>/linux-headers-6.8.12-1021-tegra-linux_x86_64/3rdparty/canonical/linux-noble \
#   CROSS_COMPILE=aarch64-linux-gnu- \
#   NV_OOT_DIR=<...>/nvidia-public \
#   scripts/build_ecam25_driver.sh
#
# Outputs into build-ecam25/:
#   ecam25_ar0234.ko
#   tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo
#
# install/install_ecam25_camera.sh calls this and installs the result.
# scripts/install_jetson.sh runs that as one of its steps.

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_DIR="$PROJECT_ROOT/third_party/ecam25/src"
PATCH_DIR="$PROJECT_ROOT/third_party/ecam25/patches"
DTS_DIR="$PROJECT_ROOT/third_party/ecam25/dts"
OUTPUT_DIR="$PROJECT_ROOT/build-ecam25"

KDIR="${KDIR:-/lib/modules/$(uname -r)/build}"
CROSS_COMPILE="${CROSS_COMPILE:-}"
NV_OOT_DIR="${NV_OOT_DIR:-/usr/src/nvidia/nvidia-public}"
DTS_NAME="tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234"

while [ $# -gt 0 ]; do
    case "$1" in
        -o | --output)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        -h | --help)
            sed -n '2,25p' "${BASH_SOURCE[0]}"
            exit 0
            ;;
        *)
            echo "Error: unknown argument '$1'." >&2
            exit 1
            ;;
    esac
done

NV_OOT_INC="$NV_OOT_DIR/include"
NV_OOT_SYMVERS="$NV_OOT_DIR/Module.symvers"
BUILD_DIR="$OUTPUT_DIR/work"
CONFTEST_DIR="$OUTPUT_DIR/conftest"

# ---- 1. Preconditions -------------------------------------------------------

for path in "$KDIR" "$NV_OOT_INC" "$NV_OOT_SYMVERS"; do
    if [ ! -e "$path" ]; then
        echo "Error: '$path' is missing." >&2
        echo "On the Jetson: sudo apt install nvidia-l4t-kernel-headers nvidia-l4t-kernel-oot-headers" >&2
        exit 1
    fi
done

for cmd in make cpp dtc patch modinfo "${CROSS_COMPILE}gcc" "${CROSS_COMPILE}nm"; do
    if ! command -v "$cmd" > /dev/null 2>&1; then
        echo "Error: command '$cmd' was not found." >&2
        exit 1
    fi
done

if [ ! -f "$NV_OOT_INC/media/camera_common.h" ]; then
    echo "Error: '$NV_OOT_INC/media/camera_common.h' is missing." >&2
    echo "NV_OOT_DIR does not look like an nvidia-public OOT header tree." >&2
    exit 1
fi

rm -rf "$BUILD_DIR" "$CONFTEST_DIR"
mkdir -p "$BUILD_DIR" "$CONFTEST_DIR/nvidia"

# ---- 2. Stage the vendor source and apply the JetPack 7 patches -------------

cp "$SRC_DIR"/* "$BUILD_DIR/"

shopt -s nullglob
patches=("$PATCH_DIR"/*.patch)
shopt -u nullglob
if [ ${#patches[@]} -eq 0 ]; then
    echo "Error: no patches found in '$PATCH_DIR'." >&2
    exit 1
fi

for patch_file in "${patches[@]}"; do
    echo "Applying $(basename "$patch_file")"
    if ! patch -p1 -d "$BUILD_DIR" --no-backup-if-mismatch -i "$patch_file" > /dev/null; then
        echo "Error: $(basename "$patch_file") did not apply." >&2
        exit 1
    fi
done

# ---- 3. Generate nvidia/conftest.h ------------------------------------------
#
# camera_common.h includes <nvidia/conftest.h> and gates a member of
# struct camera_common_data on NV_TEGRA_PMC_IO_PAD_POWER_ENABLE_PRESENT. The
# shipped tegra-camera.ko was built with that member present, so building
# without the macro shifts every field after it: the driver writes s_data->priv
# and tegra-camera reads a different offset. That compiles, links and loads
# cleanly, then corrupts memory on the first capture. NVIDIA's own conftest is
# not shipped in any header package, so probe the header directly.

probe_dir="$CONFTEST_DIR/probe"
mkdir -p "$probe_dir"
cat > "$probe_dir/probe.c" << 'EOF'
#include <linux/module.h>
#include <soc/tegra/pmc.h>
static void *conftest_ref = (void *)&tegra_pmc_io_pad_power_enable;
void conftest_use(void);
void conftest_use(void) { (void)conftest_ref; }
MODULE_LICENSE("GPL");
EOF
echo 'obj-m += probe.o' > "$probe_dir/Makefile"

echo "Probing for tegra_pmc_io_pad_power_enable"
if make -s -C "$KDIR" M="$probe_dir" ARCH=arm64 CROSS_COMPILE="$CROSS_COMPILE" \
    modules > "$probe_dir/probe.log" 2>&1; then
    echo '#define NV_TEGRA_PMC_IO_PAD_POWER_ENABLE_PRESENT' > "$CONFTEST_DIR/nvidia/conftest.h"
else
    : > "$CONFTEST_DIR/nvidia/conftest.h"
fi

# The gate. A mismatch here is the one failure mode that produces a working
# build and a broken camera, so refuse to go on rather than warn.
if ! grep -q '#define NV_TEGRA_PMC_IO_PAD_POWER_ENABLE_PRESENT' "$CONFTEST_DIR/nvidia/conftest.h"; then
    echo "Error: could not confirm NV_TEGRA_PMC_IO_PAD_POWER_ENABLE_PRESENT." >&2
    echo "struct camera_common_data would not match the shipped tegra-camera.ko." >&2
    echo "Probe log: $probe_dir/probe.log" >&2
    echo "Cross-check with: nm -u ${KDIR%/build}/updates/drivers/media/platform/tegra/camera/tegra-camera.ko | grep pmc" >&2
    exit 1
fi

# ---- 4. Build the module ----------------------------------------------------

echo "Building ecam25_ar0234.ko"
make -C "$KDIR" M="$BUILD_DIR" ARCH=arm64 CROSS_COMPILE="$CROSS_COMPILE" \
    NV_OOT_INC="$NV_OOT_INC" NVIDIA_CONFTEST="$CONFTEST_DIR" \
    KBUILD_EXTRA_SYMBOLS="$NV_OOT_SYMVERS" modules

module="$BUILD_DIR/ecam25_ar0234.ko"
[ -f "$module" ] || {
    echo "Error: '$module' was not produced." >&2
    exit 1
}

# ---- 5. Verify the module before anyone tries to load it --------------------

vermagic=$(modinfo -F vermagic "$module")
echo "vermagic: $vermagic"

unresolved=0
while read -r symbol; do
    [ -n "$symbol" ] || continue
    if ! grep -qP "\t${symbol}\t" "$NV_OOT_SYMVERS" "$KDIR/Module.symvers" 2> /dev/null; then
        echo "Error: unresolved symbol '$symbol'." >&2
        unresolved=1
    fi
done < <("${CROSS_COMPILE}nm" -u "$module" | awk '{print $2}')
[ "$unresolved" -eq 0 ] || {
    echo "Error: the module would fail to load." >&2
    exit 1
}

# ---- 6. Build the device tree overlay ---------------------------------------

echo "Building $DTS_NAME.dtbo"
cpp -nostdinc -I "$KDIR/include" -undef -x assembler-with-cpp \
    "$DTS_DIR/$DTS_NAME.dts" -o "$BUILD_DIR/$DTS_NAME.pre.dts"
dtc -@ -I dts -O dtb -o "$BUILD_DIR/$DTS_NAME.dtbo" "$BUILD_DIR/$DTS_NAME.pre.dts" 2> /dev/null

# ---- 7. Collect ------------------------------------------------------------

cp "$module" "$BUILD_DIR/$DTS_NAME.dtbo" "$OUTPUT_DIR/"

echo
echo "Built:"
echo "  $OUTPUT_DIR/ecam25_ar0234.ko    ($vermagic)"
echo "  $OUTPUT_DIR/$DTS_NAME.dtbo"
echo
echo "Install with: source install/install_ecam25_camera.sh && install_ecam25_camera"
