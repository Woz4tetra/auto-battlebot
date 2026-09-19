# e-CAM25_CUONX camera setup

Bring-up guide for the e-con Systems e-CAM25_CUONX on the Jetson Orin NX, covering the flash,
the driver install, and the checks that prove the camera streams. Written from release package
`R04_RC3` (29-MAR-2025), which lives in `~/Documents/e-CAM25`.

The camera is the RGB replacement for the ZED described in
`docs/experiments/rgb_homography/rgb_camera_migration_2026-09-09.md`.

**We run the JetPack 7 path.** The compute Jetson is on JetPack 7.2.1 and JetPack 6 will not
flash it, so we build the driver ourselves against kernel 6.8. Jump to
[JetPack 7.2.x](#jetpack-72x-l4t-392x-kernel-68). The JetPack 6 sections below describe the
vendor install and are kept because the hardware facts, the mode table, the controls and the
stream pipelines are the same on both.

## JetPack 6.1 version lock

The vendor release package targets one JetPack version and fails on anything else:

| | |
| --- | --- |
| JetPack | 6.1.0 |
| L4T | R36.4.0 |
| Kernel | 5.15.148-tegra |
| MCU firmware | `1125CUONXXXXX011100301f6eXXXXXXX` |
| guvcview | 1.7.2-g81ddbfb |

Three separate things enforce that, so there is no talking your way past it:

- `install_binaries.sh` parses the L4T version out of the package name and compares it against
  `/etc/nv_tegra_release`. A mismatch exits before anything is written.
- `update_modules()` copies into `/lib/modules/5.15.148-tegra/updates`, a hard-coded path.
- The three `.ko` files carry 5.15.148-tegra vermagic, so a different kernel refuses to load them.

Flash JetPack 6.1.0. Not 6.2, not 6.2.3, not JetPack 7. JetPack 6.2.3 is L4T 36.4.3 with
kernel 5.15.199-tegra, so even that close a miss needs a rebuild.

The package replaces only modules and a device tree overlay. `update_kernel()` exists in the
script but is commented out of the main flow, so `/boot/Image` stays NVIDIA's. That is why the
kernel version has to match exactly.

## Package contents

`~/Documents/e-CAM25/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_29-MAR-2025_R04.tar.gz` unpacks to:

| Path | Holds |
| --- | --- |
| `install_binaries.sh` | The on-target installer. Runs on the Jetson, not the host |
| `e-CAM25_CUONX_L4T36.4.0_JP6.1.0_JETSON-ONX-ONANO_R04.tar.gz` | `ar0234.ko`, `tegra-camera.ko`, `capture-ivc.ko`, both `.dtbo` files, the guvcview binary |
| `Kernel/*_oot.patch`, `*_dtb.patch`, `*_module.patch` | Source patches for building it yourself |
| `Firmware/ecam25_cuonx_mcu_fw.bin` | MCU firmware. The installer never touches it. Leave it alone unless e-con support asks |
| `release_integrity.md5` | Checksum the installer verifies. Run the script from this directory |

## Hardware

The camera is two boards: the `e-CAM217_CUMI0234_MOD` module and the `ACC-RB-WTB-ADP` adapter,
joined by a 26-pin Samtec connector. The adapter supplies the module's rails and exposes a 22-pin
FFC connector for the 15 cm FPC cable.

- **Use CAM1.** Only CAM1 on the p3768 carrier supports 4 lanes. CAM0 is 2-lane.
- **Conductive side of the FPC faces the board** at both ends. A reversed cable can destroy the
  camera and the Jetson.
- **Power from the 19V DC jack, not USB-C.** On USB-C power the camera only reaches low
  resolutions.
- A greenish yellow LED on the adapter board lights when the module has power. It is a 1-2 mm
  SMD part, not an obvious indicator: on the adapter board (the one with the 22-pin FFC
  connector, not the lens module), immediately right of that connector, just above the `TP2` and
  `R66` silkscreen. Check in a dim room. Dark means the module has no power, so check the FPC
  seating before anything else. e-con's Getting Started Manual, Figure 19, shows it.

Supported modes, which are what `v4l2-ctl --list-formats-ext` should report:

| Resolution | 2 lane | 4 lane |
| --- | --- | --- |
| 640x480 | 120 fps | n/a |
| 1280x720 | 120 fps | 120 fps |
| 1920x1080 | 65 fps | 70 fps |
| 1920x1200 | 60 fps | 60 fps |

Those rates assume manual exposure. Under auto exposure the frame rate drops with the light level.

## JetPack 6.1 step 1: flash

In SDK Manager, pick JetPack 6.1 from the version list. Launch it as `SSH_AUTH_SOCK= sdkmanager`
or the key-agent limit shows up as a bogus "NVMe not connecting" failure.

To flash by hand instead, from an extracted L4T 36.4.0 `Linux_for_Tegra` with the board in
recovery mode (jumper FC_REC to GND, USB-C to the host, then power on):

```bash
sudo ./tools/kernel_flash/l4t_initrd_flash.sh --external-device nvme0n1p1 \
  -c tools/kernel_flash/flash_l4t_t234_nvme.xml \
  -p "-c bootloader/generic/cfg/flash_t234_qspi.xml" \
  --showlogs --network usb0 jetson-orin-nano-devkit internal
```

`lsusb` on the host shows `0955:7323 NVidia Corp.` when the board is in recovery. The flash erases
the NVMe and takes 10 to 30 minutes.

If it dies with `ERROR: might be timeout in USB write`, move to a different USB port, or disable
autosuspend and power cycle:

```bash
sudo bash -c 'echo -1 > /sys/module/usbcore/parameters/autosuspend'
```

Complete the first-boot OS setup, then confirm the version before going further:

```bash
cat /etc/nv_tegra_release    # expect R36 REVISION: 4.0
uname -r                     # expect 5.15.148-tegra
```

## JetPack 6.1 step 2: install the e-con binaries

Copy the package to the Jetson and run the installer there. It is an on-target install, not a
host-side patch of `Linux_for_Tegra`.

```bash
scp ~/Documents/e-CAM25/e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_29-MAR-2025_R04.tar.gz ben@<jetson>:~
ssh ben@<jetson>
tar -xaf e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_29-MAR-2025_R04.tar.gz
cd e-CAM25_CUONX_JETSON_ONX_ONANO_L4T36.4.0_29-MAR-2025_R04
sudo chmod +x ./install_binaries.sh
sudo -E ./install_binaries.sh
```

It prompts first:

```
Enter Lane Configuration mode to flash
1. 2lane
2. 4lane
```

**Answer `2`.** The camera is on CAM1 and 4 lanes is what buys 1080p at 70 fps.

What it then does, in order:

1. Verifies the md5, the board model, the architecture, and the L4T version.
2. Copies `tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo` to `/boot/`.
3. Reads the overlay name out of the blob with `fdtdump` and registers it:
   `config-by-hardware.py -n 2="AR0234 Sensor 4lane"`, which adds an `OVERLAYS` line to
   `/boot/extlinux/extlinux.conf`.
4. Copies `ar0234.ko`, `tegra-camera.ko` and `capture-ivc.ko` into `/lib/modules/5.15.148-tegra/updates/`.
   Note that it replaces NVIDIA's `tegra-camera.ko` and `capture-ivc.ko`, not just the sensor driver.
5. Installs the guvcview sample app to `/usr/local/ecam_tk1/bin`, pulling in a long list of GTK,
   SDL, v4l and libav apt packages plus `v4l-utils` and `nvidia-l4t-gstreamer`, and appends its
   path to `~/.bashrc`.
6. Runs `depmod -a` and **reboots the board on its own** after a 5 second warning.

The run is logged to `binary_installation_log.txt` in the package directory.

To change lane configuration later without redoing the module install, run `./install_binaries.sh -d`,
which does the overlay step and reboots.

## JetPack 6.1 step 3: verify

```bash
sudo dmesg | grep -i ar0234        # expect: subdev ar0234 10-0042 bound
ls -l /dev/video0
grep -i overlays /boot/extlinux/extlinux.conf
v4l2-ctl -d /dev/video0 --list-formats-ext
```

`10-0042` is i2c bus 10 (CAM1), address 0x42. The overlay also instantiates a PCA6408 GPIO
expander at 0x20 on the same bus.

Only one video node appears in 4-lane mode, because CAM1 is the only 4-lane port.

## JetPack 7.2.x (L4T 39.2.x, kernel 6.8)

e-con ships no JetPack 7 package, so we build the driver from their source. The port is three
kernel API changes and a new Makefile, not a rewrite. What changed and why is in
`third_party/ecam25/README.md`; this section is how to build, install and prove it.

| | |
| --- | --- |
| Board | Jetson Orin NX, `nvidia,p3768-0000+p3767-0000` |
| JetPack | 7.2.1 |
| L4T | R39.2.1 |
| Kernel | 6.8.12-1021-tegra |
| Module | `ecam25_ar0234.ko`, renamed to avoid JetPack 7's own `nv_ar0234.ko` |
| NVIDIA modules | Untouched. `tegra-camera.ko` and `capture-ivc.ko` stay NVIDIA's |

Unlike the JetPack 6 install, nothing here replaces an NVIDIA module. That is what keeps us off
a permanent kernel pin: only our own sensor module has to be rebuilt for a new kernel ABI.

### Build

On the Jetson, with the kernel and OOT headers JetPack already installs:

```bash
sudo apt install v4l-utils          # not installed by default, the checks below need it
scripts/build_ecam25_driver.sh
```

That stages `third_party/ecam25/src`, applies `third_party/ecam25/patches`, generates
`nvidia/conftest.h`, builds `ecam25_ar0234.ko`, checks its vermagic and that every undefined
symbol resolves, then compiles the overlay. Output lands in `build-ecam25/`.

Cross-compiling from the dev box works too, against an unpacked JetPack 7 BSP:

```bash
KDIR=<bsp>/kernel_headers/linux-headers-6.8.12-1021-tegra-linux_x86_64/3rdparty/canonical/linux-noble \
CROSS_COMPILE=aarch64-linux-gnu- \
NV_OOT_DIR=<bsp>/kernel_oot_headers/nvidia-public \
scripts/build_ecam25_driver.sh
```

The build fails outright if it cannot confirm `NV_TEGRA_PMC_IO_PAD_POWER_ENABLE_PRESENT`. That
gate is load-bearing: without the macro, `struct camera_common_data` loses a member that the
shipped `tegra-camera.ko` was built with, and the driver and the VI then disagree about where
`priv` lives. It compiles, links and loads cleanly, then corrupts memory on the first capture.

### Install

`scripts/install_jetson.sh` builds and installs the camera as one of its steps, and skips it
when the module is already installed for the running kernel and the overlay is registered. To
run only that step:

```bash
source install/install_ecam25_camera.sh && install_ecam25_camera
sudo reboot
```

It installs the module to `/lib/modules/6.8.12-1021-tegra/updates/drivers/media/i2c/`, copies
the `.dtbo` to `/boot/`, backs up `extlinux.conf`, registers the overlay through jetson-io, and
holds the kernel packages. It refuses to install a module built for a different kernel, and it
does not reboot on its own.

The overlay's `jetson-header-name` is `"Jetson 22pin CSI Connector"`. JetPack 6 called the same
connector 24pin and JetPack 7 deleted that header entirely, so the vendor string matches nothing
and `config-by-hardware.py` silently finds no overlay to register.

### Verification ladder

Run `sudo nvpmodel -m 0 && sudo jetson_clocks` before any frame rate number.

| Stage | Command | Pass |
| --- | --- | --- |
| L0 overlay | `ls /proc/device-tree/bus@0/cam_i2cmux/i2c@1/` | an `ecam_ar0234_c@42` node |
| L1 i2c | `i2cdetect -l`, then `sudo i2cdetect -y -r <mux child bus>` | 0x20 (PCA6408) and 0x42 (MCU) |
| L2 module | `sudo modprobe ecam25_ar0234 && dmesg \| tail -50` | `MIPI Clock = 1200, MIPI Lanes = 4`, the firmware version, `subdev ecam25_ar0234 <bus>-0042 bound` |
| L3 node | `ls -l /dev/video0`, `media-ctl -p` | one node, sensor to nvcsi to vi links enabled |
| L4 formats | `v4l2-ctl -d /dev/video0 --list-formats-ext` | UYVY 1280x720@120, 1920x1080@70, 1920x1200@60 |
| L5 stride | set UYVY 1920x1080, then `v4l2-ctl --get-fmt-video` | `Bytes per Line: 3840`, `Size Image: 4147200` |
| L6 throughput | `v4l2-ctl --stream-mmap --stream-count=600 --stream-to=/dev/null` | ~70 fps, no DQBUF failures |
| L7 gstreamer | the `nvv4l2camerasrc` pipeline below at `framerate=70/1` | records; this is what exercises the new pad frame-interval ops |
| L8 controls | `v4l2-ctl --list-ctrls-menus`, round-trip exposure and gain | set and get agree |

**L5 is the decision gate.** `1920` and `2073600` instead of `3840` and `4147200` mean the VI is
using the wrong stride, which is the one case that forces rebuilding NVIDIA's `tegra-camera.ko`.

### When a stage fails

| Signature | Cause |
| --- | --- |
| `Unknown symbol camera_common_initialize` | `tegra-camera` not loaded, or the module was built without `KBUILD_EXTRA_SYMBOLS` |
| `disagrees about version of symbol camera_common_*` | Built against a different `Module.symvers` |
| `Trying to Detect Bootloader mode` | The MCU did not answer. **Stop.** The driver's next move is to flash the firmware compiled into it. Fix the i2c or power problem first, and do not power cycle while this is on screen |
| `Unable to toggle RESET GPIO` | The PCA6408 did not probe, or the overlay's gpio phandle did not resolve |
| `Failed to initialize cam.` | A per-mode device tree property is missing. Suspect `dynamic_pixel_bit_depth`, which the vendor overlay omits on mode2 (1920x1200) |
| `PXL_SOF syncpt timeout` | No MIPI data: lane count, `port-index` or `tegra_sinterface` wrong, or the MCU never got its stream config |
| `Timeout: MW_ACK_DONE` | Data arriving, VI mis-programmed. Stride or pixel format, so go back to L5 |
| `-EBUSY` from the second `STREAMON` | The capture-ivc race. The vendor semaphore hunk is the fix |

Turn on the RTCPU trace before re-running a failed L6:

```bash
echo 1 | sudo tee /sys/kernel/debug/tracing/events/tegra_rtcpu/enable
echo 2 | sudo tee /sys/kernel/debug/camrtc/log-level
sudo cat /sys/kernel/debug/tracing/trace_pipe
```

### If L5 or L6 forces the issue

Sync the 39.2.1 nvidia-oot sources, apply only the hunk the failed check points at (they are
listed in `third_party/ecam25/README.md`), rebuild with `./nvbuild.sh -m`, and ship
`tegra-camera.ko` alongside the sensor module. Keep `.orig` copies of anything you replace and
`apt-mark hold nvidia-l4t-kernel-oot-modules`, because an apt upgrade overwrites
`/lib/modules/*/updates`. Record which check forced it.

## Performance mode

Both e-con guides call for this before measuring frame rate. It also matters for our latency
budget:

```bash
sudo nvpmodel -m 0
sudo jetson_clocks
```

## Stream tests

Preview at 1080p, hardware path:

```bash
gst-launch-1.0 nvv4l2camerasrc device=/dev/video0 \
  ! "video/x-raw(memory:NVMM), format=(string)UYVY, width=(int)1920, height=(int)1080" \
  ! nvvidconv ! "video/x-raw(memory:NVMM), format=(string)I420, width=(int)1920, height=(int)1080" \
  ! nv3dsink sync=false
```

Use `nvv4l2camerasrc`, not `nvarguscamerasrc`. The module does its own ISP and emits UYVY, so
nothing goes through Argus.

Record 10 seconds of 1080p H.264:

```bash
gst-launch-1.0 -e nvv4l2camerasrc device=/dev/video0 \
  ! "video/x-raw(memory:NVMM), format=(string)UYVY, width=(int)1920, height=(int)1080, framerate=70/1" \
  ! nvvidconv ! "video/x-raw(memory:NVMM), format=(string)I420, width=(int)1920, height=(int)1080" \
  ! nvv4l2h264enc ! h264parse ! matroskamux ! queue ! filesink location=file.mkv
```

Headless check over the network, which is the useful one when the Jetson has no monitor. On the
Jetson:

```bash
gst-launch-1.0 nvv4l2camerasrc device=/dev/video0 \
  ! "video/x-raw(memory:NVMM), format=(string)UYVY, width=(int)1920, height=(int)1080" \
  ! nvvidconv ! "video/x-raw(memory:NVMM), format=(string)NV12" \
  ! nvv4l2h264enc control-rate=1 ! h264parse ! rtph264pay mtu=1400 \
  ! udpsink clients=<dev_box_ip>:5000 sync=false buffer-size=100000
```

On the dev box:

```bash
gst-launch-1.0 udpsrc port=5000 \
  caps="application/x-rtp, media=(string)video, encoding-name=H264, payload=(int)96" \
  ! rtph264depay ! h264parse ! decodebin ! videoconvert ! autovideosink sync=false
```

Single JPEG:

```bash
gst-launch-1.0 nvv4l2camerasrc device=/dev/video0 num-buffers=1 \
  ! "video/x-raw(memory:NVMM), format=(string)UYVY, width=(int)1920, height=(int)1080" \
  ! nvvidconv ! nvjpegenc ! filesink location=capture.jpg
```

Raw V4L2 throughput, which is the path `V4l2RgbCamera` actually uses and therefore the number
that predicts our loop rate:

```bash
v4l2-ctl -d /dev/video0 \
  --set-fmt-video=width=1920,height=1080,pixelformat=UYVY \
  --stream-mmap --stream-count=300 --stream-to=/dev/null
```

GUI, if a monitor is attached. JetPack 6 only: guvcview comes from e-con's installer, which the
JetPack 7 path does not run.

```bash
/usr/local/ecam_tk1/bin/ecam_tk1_guvcview --device=/dev/video0
```

## Pin the kernel (JetPack 6)

An `apt upgrade` that installs a new `nvidia-l4t-kernel` leaves the e-con modules unloadable and
the camera disappears with no obvious cause:

```bash
sudo apt-mark hold nvidia-l4t-kernel nvidia-l4t-kernel-dtbs nvidia-l4t-kernel-headers
```

## Camera controls

All exposed as plain V4L2 ioctls, so `v4l2-ctl --set-ctrl` and our own config reach them. Defaults
are what the driver applies at open.

| Control | Min | Max | Default | Auto mode |
| --- | --- | --- | --- | --- |
| Brightness | -15 | 15 | 0 | no |
| Contrast | 0 | 30 | 10 | no |
| Saturation | 0 | 60 | 16 | no |
| White balance | 10 | 10000 | 4600 | yes |
| Gamma | 40 | 500 | 220 | no |
| Gain | 1 | 40 | 1 | no |
| Horizontal flip | 0 | 1 | 0 | no |
| Vertical flip | 0 | 1 | 0 | no |
| Sharpness | 0 | 127 | 16 | no |
| Exposure | 1 (100 us) | 10000 (1 s) | 312 (31.2 ms) | yes |
| ROI window size | 8 | 64 | 8 | no |
| External trigger | 0 | 1 | 0 | no |
| Strobe | 0 | 1 | 0 | no |
| Denoise | 0 | 15 | 8 | no |
| Exposure compensation | 8000 | 1000000 | 16000 | no |

Exposure has manual, full-FOV auto, and ROI-based auto modes. For our use, run manual exposure:
auto exposure lowers the frame rate as the light drops, and the default 31.2 ms exposure will
smear a robot moving across the cage.

External trigger takes a 3.3V input. The two trigger application notes in the package cover the
wiring and the strobe output.

## What this means for the repo

- `config/_jetson.toml` sets `device = "/dev/video0"`, which matches. It runs the 1920x1080 70 fps
  mode, which is what `config/cameras/ecam25_h01r1_estimated.toml` was written for and the better
  trade for the latency budget than 1920x1200 at 60 fps.
- The calibration in the repo is fitted to e-con's published fields of view, not measured. Shoot a
  checkerboard through an offcut of the cage panel at the mounted standoff, as the migration doc
  says. Calibrating in free air folds the panel's refraction into the field pose where no
  reprojection residual will reveal it.
- Anything reading `CameraData::depth` is now reading an empty image. That is the rest of the
  migration doc, not this guide.

## Rebuilding from source (JetPack 6)

The JetPack 7 build is `scripts/build_ecam25_driver.sh`, described above. This section is the
vendor's own JetPack 6 flow, kept for reference.

Only needed to change the driver or move to another L4T. The package ships three patches against
the L4T 36.4.0 sources: `_oot.patch` for `nvidia-oot`, `_dtb.patch` for the device tree (it adds
both overlays to `hardware/nvidia/t23x/nv-public/overlay/Makefile`), and `_module.patch` for the
sensor driver itself, which is applied into a fresh `sensor_driver/` directory.

Cross-compile on the host with NVIDIA's GCC 11.3.0 aarch64 toolchain
(`aarch64--glibc--stable-2022.08-1`) and the R36.4.0 `public_sources.tbz2`. Dry-run every patch
first:

```bash
patch -p1 -i $RELEASE_PACK_DIR/Kernel/..._oot.patch --dry-run
```

Then `make -C kernel`, `make modules`, `sudo -E make modules_install`, `make dtbs`, and copy
`kernel-devicetree/generic-dts/dtbs/tegra234-p3767-0000-p3768-0000-a0-4lane-ar0234.dtbo` into the
rootfs `/boot`. The full environment variable block is in the Developer Guide. A custom kernel
config must keep module versioning support or the camera driver will not work.

## Troubleshooting

| Symptom | Cause and fix |
| --- | --- |
| No LED on the adapter board | Module has no power. Reseat the FPC at both ends, conductive side toward the board |
| Installer exits on L4T mismatch | The board is not on L4T 36.4.0. Reflash, do not patch around it |
| No `/dev/video0`, nothing in dmesg | Overlay not registered. Check for the `OVERLAYS` line in `/boot/extlinux/extlinux.conf` and reboot |
| Blue noise at high resolution | Known issue at high gain with low exposure. Lower gain or raise exposure |
| Black preview in guvcview | Known issue, usually self-recovers within a few seconds |
| Frame rate below the table | Auto exposure in low light, or USB-C power instead of the 19V jack, or `nvpmodel`/`jetson_clocks` not set |
| Poor quality over RTP | Software decoder on the receiving end, or bandwidth. Decode in hardware on the client |
