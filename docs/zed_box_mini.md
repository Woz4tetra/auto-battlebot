# ZED Box Mini

State of the ZED Box Mini as inspected on 2026-09-24 and how the repo runs on it. The box stays
on its factory JetPack 6.2.1. The inspection was read-only; nothing on the box was changed.

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

The box stays on the factory JetPack 6.2.1. The repo supports it alongside JetPack 7.2, so the
e-CAM25 Jetson and the Box Mini run the same tree.

| Component | On the box (JetPack 6.2.1) | e-CAM25 Jetson (JetPack 7.2) |
|---|---|---|
| OS | Ubuntu 22.04.5, kernel 5.15.148-tegra | Ubuntu 24.04 |
| L4T | R36.4.4 | R39.2 |
| CUDA | 12.6 | 13.2 |
| TensorRT | 10.3.0 | 10.16 |
| cuDNN / VPI | 9.3.0 / 3.2.4 | |
| Python | 3.10.12 (numpy 2.2.6, pyzed 5.2) | 3.12 |
| OpenCV | NVIDIA 4.8.0 | built from source by `install/install_opencv.sh` |
| GCC | 11.4 | |
| ZED SDK | 5.2.3 | |
| Stereolabs camera driver | `stereolabs-zedbox-mini` 1.4.1 (L4T 36.4) | |
| Docker | 29.3.1 | |

The Stereolabs driver modules (`sl_zedx`, `sl_max9296`, ...) load and `zed_x_daemon` runs. The
default boot entry is `Stereolabs`, which applies
`/boot/tegra234-p3768-camera-zedbox-mini-sl-overlay.dtbo`.

## How the repo handles JetPack 6.2.1

The install scripts branch on the L4T major (`/etc/nv_tegra_release`):

- The venv uses the system python, 3.10 here, because JetPack builds `python3-libnvinfer` for
  the system python only. `install/install_python_environment.sh` reads it from `/usr/bin/python3`.
- `install/jetson_r36_packages.txt` adds the jammy-only packages (`libstdc++-12-dev` for
  clang-tidy); `install/jetson_r39_packages.txt` holds the noble-only ones.
- PyTorch comes from an NVIDIA JetPack 6 wheel (cp310) with `numpy==1.26.1` pinned for its ABI.
  JetPack 7 installs cuSPARSELt and a newer torch instead.
- The C++ build takes the TensorRT <10.7 `IStreamReader` path in
  `src/tensorrt_inference/trt_engine.cpp`.
- Python code keeps a 3.10 floor: `from auto_battlebot.compat import tomllib`, no 3.11+ stdlib
  API. ruff and mypy target 3.10.

## Setup on the box

Use wired Ethernet for the install.

```bash
./scripts/install_jetson.sh
./scripts/build.sh
sudo nvpmodel -m 0                     # MAXN
./install/install_jetson_clocks.sh
```

- The 15W default has half the CPU cores and a third less GPU clock, which the 60ms latency
  budget can't afford.
- Build every `aarch64_sm87` engine on the box against its TensorRT 10.3. Engines from the
  JetPack 7 Jetson (10.16) or a dev machine will not load, and a rebuilt engine keeps the same
  filename.
- Do not run `apt upgrade` without holding the kernel. A newer `nvidia-l4t-*` kernel drops the
  Stereolabs camera driver. `hold-zedbox-kernel.sh` from the
  [flash docs](https://docs.stereolabs.com/docs/products/embedded/zed-box-mini/reset-update)
  names the L4T 36.4 kernel packages; check that every `nvidia-l4t-*` package shows `[HELD]`.
- `config/_zed_box.toml` runs a ZED X One S on a GMSL2 port (see open issue 3).

## Open issues

1. **Not verified on the box yet:** the install, the C++ build under GCC 11, whether TensorRT
   10.3 builds the yolo26 engines, and a playback replay. Run all four before trusting it.
2. **PyTorch wheel selection misses 6.2.1.** `get_jetson_torch_install_url` in
   `install/install_pytorch_jetson.sh` turns `R36 (release), REVISION: 4.4` into JP version 64,
   which matches no case and falls through to the JetPack 6.1 wheel (torch 2.5). The JetPack 6.2
   wheel (torch 2.6) is the `62` case. Set `TORCH_INSTALL` to override, or map R36.4 to `62`.
3. **ZED X One S is untested on hardware.** The e-CAM25 has nowhere to plug in (GMSL2 only, no
   MIPI CSI), so `config/_zed_box.toml` uses a ZED X One S through `ZedOneRgbCamera`
   (`sl::CameraOne`). It is monocular, so the config runs the RGB path the e-CAM25 used:
   `FiducialFieldFilter`, height gate off, static gate on. Things to check on the box:
   - The X One S launched in December 2025, and neither the driver changelog nor the SDK release
     notes name it. It uses the X One GS sensor (AR0234), so SDK 5.2.3 and driver 1.4.1 probably
     open it as `ZED_XONE_GS`. The startup log prints the model the SDK reports.
   - First open writes `config/cameras/zed_x_one_<serial>_<W>x<H>.toml` from the factory
     calibration. Commit it: `VideoPlaybackCamera` needs it to replay recordings from the box.
   - The SDK rectifies each frame, then the camera converts BGRA to BGR on the CPU and records
     H.264 through the same encoder as the e-CAM25. Measure the capture-to-`get()` latency at 1920x1200 60 fps.
   - For depth, a ZED X or ZED X Mini on the same port works with `ZedRgbdCamera` instead.
4. **DS3231 RTC may have no bus.** The GPIO port lists no I2C, so `install/install_ds3231_rtc.sh`
   may have nothing to attach to. Check the hardware manual.

## Why not JetPack 7.2

Stereolabs does ship a 7.2 path (`zedbox_mini_usb_flash_7.2_gpio.sh`, driver 1.4.3 for
L4T 39.2.1, ZED SDK 5.5), but it needs a reflash over micro-USB from an Ubuntu 20.04/22.04 host,
has no RT-kernel driver build, and gains nothing the repo needs now that 6.2.1 is supported. The
flash steps are on the
[flash docs](https://docs.stereolabs.com/docs/products/embedded/zed-box-mini/reset-update) page
if that changes.

## Sources

- [ZED SDK release page](https://www.stereolabs.com/developers/release)
- [ZED X drivers page](https://www.stereolabs.com/developers/drivers)
- [ZED Box Mini flash docs](https://docs.stereolabs.com/docs/products/embedded/zed-box-mini/reset-update)
- [ZED Box Mini product docs](https://docs.stereolabs.com/docs/products/embedded/zed-box-mini)
- [Stereolabs forum: Box Mini JetPack upgrade](https://community.stereolabs.com/t/jetpack-zed-sdk-upgrade-for-zed-box-mini-carrier-board/11542)
