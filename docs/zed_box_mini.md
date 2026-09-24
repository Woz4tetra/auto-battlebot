# ZED Box Mini

State of the ZED Box Mini as inspected on 2026-09-24, what blocks the current code from running
on it, and the upgrade path to JetPack 7.2. Inspection was read-only; nothing on the box was
changed.

Access: `ssh -o IdentitiesOnly=yes user@<box-ip>` (default password `admin`). `sudo` needs the
password.

## Hardware

- **Module:** Jetson Orin NX 16GB (`p3767-0000`) on the Box Mini carrier. 15.3 GiB RAM, 7.6 GiB
  zram swap.
- **Compute:** 8-core Cortex-A78AE, Ampere GPU (sm87).
- **Power mode:** ships in 15W. That mode has 4 of 8 CPU cores online and caps the GPU at
  612 MHz (top step is 918 MHz). MAXN is `nvpmodel` id 0.
- **Storage:** 256 GB NVMe (M.2 2242), 18 GB used after the factory image.
- **Networking:** Intel AX210 WiFi + Bluetooth (M.2 2230), Realtek RTL8111 Gigabit Ethernet,
  `can0`.
- **USB:** one USB3 Type-A port, one micro-USB port for flashing and OTG.
- **Camera inputs:** 2x GMSL2 (FAKRA-Z) for Stereolabs ZED X cameras, plus sync trigger in/out.
  No MIPI CSI ribbon connector.
- **GPIO port:** CAN, UART, 2 GPIOs, 5V, 3.3V, GND. No I2C listed.
- **Other:** HDMI 1.4, 12V fan header.
- **Hostname (factory):** `GTW-ONX1-D27QLL7T`.

## Software as shipped

| Component | On the box | What the repo targets |
|---|---|---|
| OS | Ubuntu 22.04.5, kernel 5.15.148-tegra | Ubuntu 24.04 |
| L4T / JetPack | R36.4.4 / 6.2.1 | R39.2 / 7.2 |
| CUDA | 12.6 | 13.2 |
| TensorRT | 10.3.0 | 10.16 |
| cuDNN / VPI | 9.3.0 / 3.2.4 | |
| Python | 3.10.12 (numpy 2.2.6, pyzed 5.2) | 3.12 |
| OpenCV | NVIDIA 4.8.0 | built from source by `install/install_opencv.sh` |
| GCC | 11.4 | |
| ZED SDK | 5.2.3 | 5.5 has a JetPack 7.2 Orin build |
| Stereolabs camera driver | `stereolabs-zedbox-mini` 1.4.1 (L4T 36.4) | 1.4.3 exists for L4T 39.2.1 |
| Docker | 29.3.1 | |

The Stereolabs driver modules (`sl_zedx`, `sl_max9296`, ...) load and `zed_x_daemon` runs. The
default boot entry is `Stereolabs`, which applies
`/boot/tegra234-p3768-camera-zedbox-mini-sl-overlay.dtbo`.

## What blocks the current code on JetPack 6.2

1. **Fixed 2026-09-24: install scripts hardcoded Python 3.12.** They now follow the system
   python and pick packages per L4T major. See "Staying on JetPack 6.2".
2. **Fixed 2026-09-24: TensorRT Python bindings.** JetPack 6 builds `python3-libnvinfer` for 3.10,
   and the venv is now 3.10 there.
3. **TensorRT 10.3 predates 10.7.** `src/tensorrt_inference/trt_engine.cpp` falls back to the old
   `IStreamReader` API, so the C++ side should compile. Not verified on the box. Engines must be
   built on the box either way.
4. **The e-CAM25 has nowhere to plug in.** Camera options:
   - **ZED X / ZED X Mini over GMSL2:** works with the existing `ZedRgbdCamera` backend and gives
     depth, which the height gate needs. `BUILD_WITH_ZED` defaults ON when the SDK is installed.
   - **USB3 camera:** runs through `V4l2RgbCamera`, but takes the only USB-A port. Anything else on
     USB (radio link, keyboard) then needs a hub.
5. **DS3231 RTC may have no bus.** The GPIO port lists no I2C, so `install/install_ds3231_rtc.sh`
   may have nothing to attach to. Check the hardware manual.

## Upgrade to JetPack 7.2

### 1. Reflash

SDK Manager does not support the ZED Box. Use Stereolabs' script
`zedbox_mini_usb_flash_7.2_gpio.sh` from the
[flash docs](https://docs.stereolabs.com/docs/products/embedded/zed-box-mini/reset-update).

- **Enter recovery:** `sudo reboot --force forced-recovery` on the box, or hold RCV and press RST.
  The power LED stays off in recovery.
- **Connect:** micro-USB cable from the host to the OTG port. `lsusb -d 0955:` on the host should
  show `0955:7323` (Orin NX 16GB). `0955:7020` means it booted normally instead.
- **Host PC:** Stereolabs lists Ubuntu 20.04/22.04 with 80 GB free. The dev box is 24.04 with
  82 GB free. Unverified whether the 7.2 flash tools accept a 24.04 host.

### 2. Box setup after the flash

Use wired Ethernet; WiFi may not come up before JetPack is installed. Do not run `apt upgrade`
before the kernel hold.

```bash
sudo apt update && sudo apt install nvidia-jetpack
sudo dpkg -i stereolabs-zedbox-mini_1.4.3-SL-MAX9296-ZEDBOX-MINI-L4T39.2.1_arm64.deb
sudo ./hold-zedbox-kernel.sh          # script from the flash docs page
chmod +x ZED_SDK_Tegra_L4T39.2_v5.5.*.zstd.run && ./ZED_SDK_Tegra_L4T39.2_v5.5.*.zstd.run
sudo reboot
```

- Driver download: [ZED X drivers page](https://www.stereolabs.com/developers/drivers).
- ZED SDK download: [release page](https://www.stereolabs.com/developers/release).
- `hold-zedbox-kernel.sh` only names L4T 36.4 kernel versions. Check that every `nvidia-l4t-*`
  package shows `[HELD]` afterwards.
- There is no RT-kernel build of the 7.2 driver yet.

### 3. Repo install

```bash
./scripts/install_jetson.sh
```

- Build pycuda from source; there is no JetPack 7 wheel.
- Rebuild every `aarch64_sm87` engine on the box. Filenames don't change on rebuild.

### 4. Full power

```bash
sudo nvpmodel -m 0    # MAXN
```

Then run `install/install_jetson_clocks.sh`. The 15W default costs half the CPU cores and a third
of GPU clock against the 60ms latency budget.

### 5. Camera config

Pick a ZED X / X Mini (GMSL2) or a USB3 camera, then add the camera section to
`config/_zed_box.toml`.

## Staying on JetPack 6.2

The repo supports both JetPack 6.2 and 7.2, so `./scripts/install_jetson.sh` runs on the box as
shipped:

- The venv uses the system python (3.10 here), since `python3-libnvinfer` is built for it only.
- `install/jetson_r36_packages.txt` replaces the noble-only packages.
- PyTorch comes from NVIDIA's JetPack 6.2 wheel (torch 2.6, cp310) with NumPy 1.26.
- The C++ build takes the TensorRT <10.7 `IStreamReader` path in `trt_engine.cpp`.

Not yet verified on the box: the C++ build under GCC 11, and whether TensorRT 10.3 builds the
yolo26 engines. Run the install, build, and a playback replay before trusting it.

## Sources

- [ZED SDK release page](https://www.stereolabs.com/developers/release)
- [ZED X drivers page](https://www.stereolabs.com/developers/drivers)
- [ZED Box Mini flash docs](https://docs.stereolabs.com/docs/products/embedded/zed-box-mini/reset-update)
- [ZED Box Mini product docs](https://docs.stereolabs.com/docs/products/embedded/zed-box-mini)
- [Stereolabs forum: Box Mini JetPack upgrade](https://community.stereolabs.com/t/jetpack-zed-sdk-upgrade-for-zed-box-mini-carrier-board/11542)
