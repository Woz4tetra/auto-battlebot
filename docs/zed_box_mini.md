# ZED Box Mini

State of the ZED Box Mini and how the repo runs on it. First inspected read-only on 2026-09-24,
re-inspected the same night after the repo install, the switch to MAXN, and the first live runs.
The box stays on its factory JetPack 6.2.1.

Access: `ssh -i ~/.ssh/auto-battlebot-compute-2 user@192.168.50.183`. The factory password
`admin` still works; `sudo` needs it.

## Hardware

- **Module:** Jetson Orin NX 16GB (`p3767-0000`) on the Box Mini carrier. 15.3 GiB RAM, 7.6 GiB
  zram swap.
- **Compute:** 8-core Cortex-A78AE, Ampere GPU (sm87).
- **Power mode:** runs MAXN (`nvpmodel` id 0). It ships in 15W (id 2), which has 4 of 8 CPU
  cores online and caps the GPU at 612 MHz. Under MAXN all 8 cores run at 1984 MHz and the GPU
  devfreq is pinned at 918 MHz (min = max). The mode survives a reboot through
  `/var/lib/nvpmodel/status` (`pmode:0000`); `/etc/nvpmodel.conf` still says `DEFAULT=2`, so a
  reflash or a deleted status file drops it back to 15W. Check with `nvpmodel -q`.
- **Power and thermals at 60 Hz:** 13.9 W at `VDD_IN`, junction 68.6 C, fan PWM 255 under the
  `quiet` nvfancontrol profile. The app logged 58.4-61.9 C over the 2026-09-25 01:22 recording.
- **Storage:** 256 GB NVMe (M.2 2242), 18 GB used after the factory image, 30 GB after the repo
  install and build.
- **Networking:** Intel AX210 WiFi + Bluetooth (M.2 2230), Realtek RTL8111 Gigabit Ethernet,
  `can0`.
- **USB:** one USB3 Type-A port, one micro-USB port for flashing and OTG.
- **Camera inputs:** 2x GMSL2 (FAKRA-Z) for Stereolabs ZED X cameras, plus sync trigger in/out.
  No MIPI CSI ribbon connector.
- **GPIO port:** CAN, UART, 2 GPIOs, 5V, 3.3V, GND. No I2C listed.
- **Other:** HDMI 1.4, 12V fan header.
- **Hostname:** `auto-battlebot-compute-2` (factory: `GTW-ONX1-D27QLL7T`).

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
| Python | 3.10.12 (system numpy 2.2.6, pyzed 5.2; venv numpy 1.26.1) | 3.12 |
| PyTorch (venv) | 2.5.0a0 nv24.08, the JetPack 6.1 wheel (open issue 2), CUDA available | |
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
```

State on 2026-09-25 (box clock, UTC): the install and build are done, `auto_battlebot.service`,
`viz_relay.service` and `zed_x_daemon.service` are running, and the service runs the
`mr_stabs_mk2_zed_box` profile. The field and `yolo26s-pose_d50000` `aarch64_sm87` engines were
rebuilt on the box. The deployed tree is a copy with no `.git`.

- The last run logged in `~/install_jetson.log` (00:17) failed in the cuSPARSELt step, because
  PyTorch's `install_cusparselt.sh` has no CUDA 12.6 archive and ran `tar xf .tar.xz`. The
  current `install/install_pytorch_jetson.sh` fetches NVIDIA's linux-aarch64 0.6.3.2 archive
  directly instead, and `libcusparseLt` is now in `ldconfig`.

- The 15W default has half the CPU cores and a third less GPU clock, which the 60ms latency
  budget can't afford.
- Build every `aarch64_sm87` engine on the box against its TensorRT 10.3. Engines from the
  JetPack 7 Jetson (10.16) or a dev machine will not load, and a rebuilt engine keeps the same
  filename.
- Do not run `apt upgrade` without holding the kernel. A newer `nvidia-l4t-*` kernel drops the
  Stereolabs camera driver. `hold-zedbox-kernel.sh` from the
  [flash docs](https://docs.stereolabs.com/docs/products/embedded/zed-box-mini/reset-update)
  names the L4T 36.4 kernel packages; check that every `nvidia-l4t-*` package shows `[HELD]`.
  **Not done yet:** `apt-mark showhold` is empty and 0 of the 49 installed `nvidia-l4t-*`
  packages are held (open issue 5).
- `config/_zed_box.toml` runs a ZED X One S on a GMSL2 port (see open issue 3).

## Open issues

1. **Playback replay is not verified on the box.** The install, the C++ build under GCC 11, and
   the TensorRT 10.3 engine builds (DeepLab field, `yolo26s-pose_d50000`) all work, and the live
   pipeline runs. The `yolo26x-pose` `aarch64_sm87` engine on the box has not been rebuilt for
   10.3 yet.
2. **PyTorch wheel selection misses 6.2.1.** `get_jetson_torch_install_url` in
   `install/install_pytorch_jetson.sh` turns `R36 (release), REVISION: 4.4` into JP version 64,
   which matches no case and falls through to the JetPack 6.1 wheel (torch 2.5). The JetPack 6.2
   wheel (torch 2.6) is the `62` case. Set `TORCH_INSTALL` to override, or map R36.4 to `62`.
   Confirmed on the box: the venv has torch 2.5.0a0 nv24.08.
3. **ZED X One S works, but depth needs another camera.** The e-CAM25 has nowhere to plug in
   (GMSL2 only, no MIPI CSI), so `config/_zed_box.toml` uses a ZED X One S through
   `ZedOneRgbCamera` (`sl::CameraOne`). It is monocular, so the config runs the RGB path the
   e-CAM25 used: `FiducialFieldFilter`, height gate off, static gate on.
   - SDK 5.2.3 and driver 1.4.1 open it as a ZED X One GS. The startup log reads
     `ZED XOne GS serial 301999176 at 1920x1200 60 fps, fx 665.0 fy 716.2`.
   - `FiducialFieldFilter` locks the field (10 frames, 2.24 px reprojection error in the
     01:56 run).
   - Recordings embed the SDK's rectified intrinsics in their MCAP metadata, so they replay
     through `VideoPlaybackCamera` with no `config/cameras/` file.
   - The camera logs `Video recording requested but no encoder started`, so the box records MCAP
     only, with no video.
   - For depth, a ZED X or ZED X Mini on the same port works with `ZedRgbdCamera` instead.
4. **DS3231 RTC may have no bus.** The GPIO port lists no I2C, so `install/install_ds3231_rtc.sh`
   may have nothing to attach to. Check the hardware manual.
5. **Kernel packages are not held.** Run `hold-zedbox-kernel.sh` (or `apt-mark hold` on every
   `nvidia-l4t-*` package) before any `apt upgrade`.

## Latency

`auto_battlebot_mr_stabs_mk2_zed_box_2026-09-25_01-22-23.mcap` (MAXN, `yolo26s-pose_d50000`,
1920x1200 at 60 fps) against the two Jetson runs with the same engine
(`auto_battlebot_mr_stabs_mk2_jetson_2026-09-22_12-37-54` and `12-38-47`: JetPack 7.2, ZED 2i at
1280x720 30 fps). Numbers from `scripts/mcap_latency_report.py`, ticks after field init only,
mean / p95 in ms.

| | ZED Box Mini | Jetson 12-37-54 | Jetson 12-38-47 |
|---|---:|---:|---:|
| End-to-end (`pipeline.latency`) | 33.0 / 33.6 (max 44.0) | 58.9 / 62.8 (max 70.5) | 60.0 / 63.9 (max 272.7) |
| Loop rate | 60.3 Hz | 27.9 Hz | 26.9 Hz |
| `runner.tick` | 15.9 / 16.9 | 37.7 / 66.3 | 40.3 / 67.4 |
| `runner.camera.get` | 6.8 / 7.6 | 29.3 / 58.1 | 23.4 / 50.8 |
| `yolo_keypoint_model.inference` | 6.19 / 6.23 | 5.87 / 6.68 | 5.85 / 6.67 |
| `yolo_keypoint_model.preprocess` | 1.96 / 2.16 | 1.36 / 1.85 | 1.37 / 1.90 |

- The camera accounts for the whole gap. The Jetson loop blocks on 30 fps ZED 2i frames; the Box
  Mini keeps up with 60 fps at a steady 7 ms `get()`.
- Inference is 0.3 ms slower on the Box Mini even at MAXN, on the same engine. TensorRT 10.3
  against 10.16 is the likely cause; not isolated.
- Preprocess costs 0.6 ms more because the input is 1920x1200. `RES_960x600` would recover it.
- The window is short on both sides: 98 s on the Box Mini (field init came 187.6 s in), about
  22 s per Jetson run.

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
