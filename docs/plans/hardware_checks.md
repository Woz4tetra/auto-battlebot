# Hardware checks

Changes to hardware-facing code from the comment purge (commits in `main..benw/nav-v2-phase0`)
that so far are verified only by unit tests, x86 builds, native firmware tests, pty fakes or replay.
Add a row when such a change lands; delete it once the check passes. Built 2026-11-19 from a sweep
of the 233 commits then on the branch. Behaviour changes inside commits titled "Test ..." or
"Type ..." were found by per-path diffs, so a small one could still be missing. The 2026-11-20
lock-down batches changed comments, names and docs only and added no rows.

## Jetson and camera

| Change | Commit | Check (device, steps, pass) |
| --- | --- | --- |
| DeepLab feeds RGB with the training mean/std | `89bd2ada` | Jetson SVO playback passed 2026-11-20 (0.9949 IoU on `rgb, dataset, deployed`, `docs/experiments/perception/2026-11-19_deeplab_preprocessing_fix.md`). Still open: a live ZED session with field inits, scored with `playground/deeplab_preprocess/recorded_mask_agreement.py`; recorded masks match the `rgb, *` arms better than `bgr, *` |
| Camera cancel flag survives `initialize()`; runner retry no longer drains commands before the open | `2a1bd029` | ZED X and ZED X One: hold the open failing, post Reboot during the retry sleep; the open is cut short and the host reboots. A normal first open still succeeds |
| libnvjpeg link guard (`scripts/check_no_nvjpeg_link.sh`, CTest `NoNvjpegLink`, POST_BUILD) | `1a494090` | Jetson build with `HAVE_NVJPG` ON: build prints `ok:` for `auto_battlebot` and `viz_relay`, `ctest -R NoNvjpegLink` passes |
| ZED debug override is `-O2 -DNDEBUG -U_DEBUG`, ZED sources build under `-Werror` | `86c44dab` | Jetson Debug build with `BUILD_WITH_ZED` ON compiles; ZED capture runs |
| TensorRT quiet logging is `thread_local`; engine buffers are size-checked spans; renamed borrowing execute | `86c44dab`, `58c2780c` | Jetson: run YOLO and DeepLab with engine probing; real TensorRT errors still print on other threads, no new size errors |
| MCAP recorder `recordings_dir` (default `data/recordings`, made absolute) and optional size limits | `f342a0f4`, `764e5e8b` | Jetson service: record, toggle, let eviction run; files land in the service's recordings dir and eviction deletes only there |
| Recorder built clear of replay inputs in `main.cpp`; resolved config in MCAP metadata; `--print-config` rewritten | `a71c40c2`, `61a04cb1` | Jetson: a recording carries `resolved_config`; `--print-config` prints with the deployed profile |
| Runner destructor stops the recording worker; `jetson_temperature_c` always published | `fb013c44`, `05025ea1`, `d8032518` | Jetson: SIGINT and `systemctl stop` exit cleanly with the MCAP closed; dashboard temperature is plausible |
| Host reboot and power-off refactor (`system_action_commands`) | `75c48602`, `05025ea1` | Jetson: dashboard Reboot, then Power off, as the service user; no password prompt |
| Health logger `nvpmodel` parsing; `disk_used_warn_percent` | `75c48602` | Jetson health log shows the right power mode |
| ZED X open: `read_camera_info()` optional, frame layout checks that can throw, exposure cap helper; ZED One settings passed by value | `df9df7ab`, `daf406d4`, `599dd447`, `f5174db3` | ZED X and ZED X One: open, change exposure and gain, restore; full frame rate, no layout throw, exposure capped at 1e6/fps |
| SVO recorder refuses to start with the camera closed; `unused_svo_path` | `c07d29bb`, `f3b1ded5` | ZED: toggle SVO recording, trigger a size roll, reopen the camera; files created with `_partNN` on collision |
| JPEG encoder owner-thread wrapper; NVJPG plugin signature | `58c2780c` | Jetson with NVJPG and viz: preview works, no assert in Debug |
| Viz sink state in `SinkThreadState` | `58c2780c`, `be30120f` | Dashboard and viz relay: connect, reconnect, stall behave as before |
| Config reshape: `from` section files, `[plant]` in `config/robots/`, renamed keys rejected, `init_button_channel` unset, `esp32_host` IPv4 only | `61a04cb1`, `52a5faf5`, `91f3b071` | Jetson: start each deployed profile (mr_stabs_mk2 ESP-NOW and OpenTX); no "renamed to" error, init button off, control feels unchanged |
| OpenTX transmitter device lookup and trainer check renamed; optional init button | `fb013c44`, `61a04cb1` | OpenTX radio on USB: found, trainer follows SA, 500 ms stall trips when the cable is pulled |

## Robot firmware and radio link

None of these were flashed. Verified by native PlatformIO tests or compile only.

| Change | Commit | Check (device, steps, pass) |
| --- | --- | --- |
| ESP-NOW: enabling with SA already on ramps from standstill | `b9acf394` | Wheels off the ground: SA on with a nonzero linear command; output ramps at `max_motor_rpm_per_s`, no jump. Repeat SA off and on |
| ESP-NOW `espnow_round_trip_us` extracted; telemetry fields renamed | `58c2780c` | Dongle linked: health log `min_round_trip_us` is a few ms, not wrapped |
| Mk2 `MotorGuard::update` split into six helpers | `e96d4512` | Bench, wheels off: drive, hard reverse, stall one wheel; guard scale, stall flags and thermal trip as before |
| `YawController` capture and `heading_dt_s` extracted | `e96d4512`, `8d7a886a` | Turn and hold heading at throttle and idle; captures and settles as before |
| DShot: `dshot_bit_ns()`, derived `TICK_NS`, `build_packet` in `erpm_decode`, `Esc::deinit` | `c5cd01d5`, `e96d4512` | Arm, spin both ESCs low, read eRPM, disarm; no CRC errors, stop on deinit |
| `i2c_bus::scan` refused while armed; tuning save moved into the store | `c5cd01d5` | Scan disarmed and armed, change tuning armed; no scan or flash write while armed |
| ESP-NOW link counters atomic; `ota_should_abort` extracted | `c5cd01d5` | Pair and run several minutes; counters move, pair and unpair work, OTA aborts when armed or silent |
| Diagnostics form validation `tuning::first_invalid_field` | `c5cd01d5` | Robot AP page: good value applies, bad or unknown field gets 400 with nothing applied |
| JSON, telemetry and `espnow_protocol.h` field renames, OTA enums | `f05de2d1`, `8d625706` | Diagnostics page gauges all fill; Jetson parses `vbat`, `yaw_rate`, `motor_*` |
| Dongle `CommandRepeater`, ack-rate LED at 90%, silence LED rename, ping buffer `AIR_MAX_FRAME` | `f05de2d1`, `e96d4512`, `8d625706` | Dongle and robot: repeats during 100 ms Jetson silence, LED red after 500 ms robot silence, degraded colour under 90% ack |
| Dongle `scripts/upload` defaults to `/dev/espnow_dongle` | `764e5e8b` | Robot and dongle both plugged in: the script flashes the dongle |
| Velocity jig `g_encCount` atomic | `8d625706` | Jig run: encoder count tracks wheel turns |
| Velocity jig `console.py` IMU scale 0.140 dps and 0.000488 g per LSB | `86c44dab` | Jig `console` STREAM: about 0 dps at rest, 1 g vertical, a 90° turn reads 90° |
| Mk2 USB log drain, INA238 id check, CRSF encode helpers | `8d7a886a`, `c5cd01d5` | USB log lines complete, VBAT reads, CRSF frames decode |
| Mk2 WiFi manager renames | `8d7a886a` | Join and save a network from the AP page |

## Host tools and install

| Change | Commit | Check (device, steps, pass) |
| --- | --- | --- |
| ESP32 USB ports opened through `open_esp32_serial()` (DTR and RTS high before open); `usb_dashboard.py` moved from DTR high, RTS low to both high | `08c92764` | Robot on USB: `usb_dashboard.py` connects without resetting the chip; `espnow_diag.py` and `espnow_ota.py` as before |
| `install_udev_rules.sh`, `install_dashboard_network.sh`, `rename_legacy_services.sh` split into functions | `0de5a7dd`, `62705c08`, `c141a688` | Jetson or spare box: symlinks `/dev/mr_stabs`, `/dev/espnow_dongle` appear; nmcli and nftables install; legacy units rename with states kept |
| `deploy_to_jetson.sh`, `download_recordings.sh`, `build.sh` restructured | `0de5a7dd`, `557cc305`, `4bf60e4b` | Deploy, `--sync-only`, download recordings; service restarts with the chosen profile |
| Install script version checks, `check_install_option_leaks.sh` | `0de5a7dd`, `62705c08` | Re-run the Jetson install; completes |
| Venv `auto_battlebot.pth` writer removed (modules moved into `auto_battlebot/`) | `be3a4302` | Re-run the venv install on each box, or delete the `logo` and `training/synthetic` lines from the existing `auto_battlebot.pth`; scripts and tests still run |
| Velocity jig tools renamed fields, `math.isnan` | `a44ad268`, `1ce9d72d`, `4906aa77`, `b58ed3a6` | Jig and OpenTX radio: short `--waveform` with Ctrl-C; trainer commands sent, channels zero on exit, CSV columns intact |
| `blheli32` renames (`switch_from_msp`, `FieldKind`, `ESC_PINS`) | `1ce9d72d`, `a44ad268`, `188e6e32` | Mk2 over USB and WiFi: `blheli32.py read` and `backup` decode both ESCs. Do not `write` first |
| `espnow_diag.py`, `otx_tool.py`, `espnow_ota.py` typed and renamed | `b75e5994`, `e96d4512` | Dongle capture and one OTA to a robot complete |

## Needs a real run, no hardware

| Change | Commit | Check |
| --- | --- | --- |
| Synthgen retyped (`SceneState`, `ScenePoses`, `MountPoses`, `CameraSample`, enums); `hue_jitter_degrees` removed | `e3ee9c45`, `c17b6616`, `d81ad494`, `867875c2`, `0afa0a93` | Same render: images, labels and manifests byte-identical or the differences explained |
| `semantic_train.py` drops `seed_worker` | `764e5e8b` | Two short seeded training runs reproduce |
| GPU queue moved to `auto_battlebot/gpu_queue.py`, `training/gpu_queue.py` is an entry point | `be3a4302` | On megamind, after the next pull: restart the queue worker, `status` and one `submit` work |

## Ledger rows that need a device

Workarounds whose last check needs hardware. Delete a row once the check passes.

| Change | Commit | Check (device, steps, pass) |
| --- | --- | --- |
| NVJPG plugin installs next to `libauto_battlebot_lib.so` (CMakeLists.txt) | `8ad8c7a4` | Jetson, Release build with `HAVE_NVJPG` ON: plugin sits beside `libauto_battlebot_lib.so`, `scripts/check_no_nvjpeg_link.sh` passes, JPEG encoder loads the plugin |
| nlohmann `DOWNLOAD_EXTRACT_TIMESTAMP` gated on CMake >= 3.24 | `8ad8c7a4` | compute-1 (JetPack 6, CMake 3.22): clean `cmake` configure succeeds |
| ZED sources build with `-U_DEBUG` only, no forced `-O2 -DNDEBUG`, in debug builds | `8ad8c7a4` | ZED box, debug build: camera opens and streams at full rate |
| Host reboot and power-off run after Runner and UI teardown (`system_action.hpp`) | existing | Jetson: send REBOOT_HOST from the UI; log shows "Runner returned" then "Running host reboot now", no SEGV, host reboots. Repeat with POWEROFF_HOST |
| FFmpeg on JetPack 7 lacks libx264, so `h264_backend` keeps x264enc | existing | Jetson JP7: `ffmpeg -encoders | grep libx264`. If present, drop the x264enc backend |
| Pending-open cancel in `rgbd_camera_interface` | existing | ZED 2i or ZED X One: cancel during open returns cleanly and the next open succeeds |
| SVO recorder rollover | existing | ZED: record past the rollover size; a new SVO starts and both files play back |
| `ZedDevice` open hard timeout leaks a wedged open | existing | ZED with a wedged SDK open: open returns Leaked at the hard timeout and shutdown does not hang |
| ZED X One `capture_thread_done_` shutdown | existing | ZED X One: make grab() hang, then shut down; shutdown completes |
| ZED X One lens model left as pinhole (`zed_one_rgb_camera.hpp:24`) | existing | ZED X One: once `CameraCalibration` supports the rational plus thin-prism model, record raw frames and compare |
| ZED X One settings read-back keeps the last value when a control reads -1 | existing | ZED X One: dashboard shows the last known value, never -1 |
| Jig drive reads radio channel B negated (`velocity_jig_drive.py` `--no-read-invert-b`) | `8abc1574` | Radio, robot OFF: `velocity_jig_drive.py --check-radio`; channel B reads negated and straight runs stay straight. Root-cause fix: clear servo-reverse in the radio model |
| Jig `confirm_stopped` prompt order and `stop_recording` with the cable out | `8abc1574` | Jig + USB: press Enter before replugging, step does not hang to 300 s; plug USB in mid-recording, last log row is complete |
| Jig `reopen_after_replug` follows the serial number and waits `CDC_ENDPOINT_SETTLE_S` | `8abc1574` | Jig: replug so it enumerates as ttyACM1, reopen follows it; with `CDC_ENDPOINT_SETTLE_S=0` the open still succeeds before the sleep can go |
| Jig prompt stdin drain | `8abc1574` | Real terminal: double-tap Enter at a jig prompt; the second Enter does not answer the next prompt |
| Install scripts split into tested functions (`install_opencv.sh`, `install_python_environment.sh`, `install_pytorch_jetson.sh`, `check_jetson_orin.sh`, `install_jetson_stability.sh`) | `54946b33` | Jetson JP7 and compute-1 (JP6): run `scripts/install_jetson.sh` end to end; it finishes, `import cv2, tensorrt, torch` works in the venv, and `cgroup_disable=memory` is added only on the 5.15 kernel |
| Kiosk unit gets the numeric login uid in `XDG_RUNTIME_DIR` (`build_and_install_service.sh`) | `ed6b8f25` | Jetson with a desktop, after install: `systemctl show dashboard-kiosk -p Environment` shows `XDG_RUNTIME_DIR=/run/user/<login uid>`; gsettings in `enable_kiosk.sh` works |
| `with_display.sh` waits for the X socket (`X_WAIT_SECONDS`) | `ed6b8f25` | Boot the Jetson: `journalctl -u dashboard-kiosk` shows a display found, not "no X server" |
| `download_recordings.sh` first-download fix (`FILENAME == ARGV[1]`) and missing-dir loop removed | `ed6b8f25` | Fresh laptop with an empty local index: `scripts/download_recordings.sh` downloads every remote recording from the Jetson |
| Dongle upload options (`use_1200bps_touch=no`, `wait_for_upload_port=no`, `before_reset=usb_reset`) | existing | QT Py S3 dongle: upload with default options stalls; with the pinned options it flashes and boots |
| Dongle USB TX FIFO reset decision `usb_tx_action` | `c9f36551` | Dongle + host: stop reading the port for 5 s, resume; then replug USB. Telemetry resumes within 1 s both times, no dongle reboot |
| Dongle diag web page Space keydown/keyup and visibilitychange stop | existing | Dongle diag page in a browser: holding Space drives, releasing stops; switching tabs while held stops the robot |
| DShot `rmt_reserved_channels` | existing | Mk2 running DShot and the NeoPixel together: channels differ and eRPM replies decode while the NeoPixel animates |
| Mk2 firmware refactors: `armed_guard`, `ina238_id::begin_sequence`, `erpm_decode::decode_capture`, `wifi_types`, `SendGate` | `c9f36551` | Flash Mk2 and the dongle: WiFi, dongle dump and tuning writes are refused while armed; VBAT reads; eRPM decodes on both ESCs; ESP-NOW link pairs and drives |
| **Mk2 `main.cpp` loop gating moved into `lib/drive_gate`** (loop mode, trip clear, OTA and erase gating, ESP-NOW arm input, SA substitution, flip and tail-first signs, guarded mix) | `32312698` | **Bench, wheels off the ground, before any drive.** Disarmed: no output. Armed, flip MIDDLE: forward stick drives forward, steer turns the right way. Flip UP: inverted drive. Flip DOWN: auto-steer follows the sensed side. Radio off while armed: outputs stop. ESC tuning owns the ESCs: no drive. SA with fresh ESP-NOW: substitution drives |
| Mk2 upload `board_upload.*` options | existing | Mk2 over USB-Serial/JTAG: `pio run -e local -t upload` enters the bootloader with no BOOT press |
| `scripts/coredump` reads offset and size from `partitions.csv` | `32312698` | Mk2 after a crash: `scripts/coredump /dev/mr_stabs` reads 0x310000/0x10000 and prints a backtrace |
| ESC config RX threshold, open-drain TX routing, tuning idle release (`lib/esc_tuning_plan`) | `32312698` | Mk2 + ESCs: tuning reads succeed; with frames flowing for 60 s the ESCs stay held, then release 60 s after the last frame |
| ESP-NOW channel-change peer re-add and 6M rate; `is_dongle` during pair rewrite | existing | Mk2 + dongle: change channel, packets keep flowing at rate; pair and unpair while streaming, no dongle rejected mid-rewrite |
| `disableLoopWDT()` during OTA, `delay(10)` at ESC wake | existing | Mk2: ArduinoOTA upload completes without a watchdog reset; power on, no ESC glitch at startup |
| VBAT `ina238_sampler`, I2C scan watchdog hook (`lib/i2c_scan`) | `32312698` | Mk2: VBAT reads with the INA238 fitted and NaN without it; I2C scan from the dashboard completes without a watchdog reset |
| Host reboot or power-off runs before camera and TensorRT teardown (`src/main.cpp`) | existing | Jetson with ZED: send REBOOT_HOST from the dashboard; the journal shows the reboot before camera close and TensorRT teardown, and the host reboots even if camera close hangs |
| `ScopedSignalQuit` replaces the `main.cpp` signal handler; battery diagnostics kept before `DiagnosticsLogger::initialize` | `748237da` | Jetson: `systemctl stop auto-battlebot` and Ctrl-C both exit cleanly with the MCAP closed; battery SOC appears in the first diagnostics after start |
| **BNO055 samples read as two error-checked 12-byte register reads (`lib/bno055_decode`)** instead of four `getEvent` calls that zero-filled on a failed read | `fbd2dffc` | Mk2 bench: yaw_rate, up/down and heading_rate match the old firmware within noise while spinning the board by hand; with SDA pulled, `lost_count` increments and `last_error` shows the I2C status, then recovers. If it passes, the per-sample CHIP_ID read can go |
| BNO055 gyro unwrap resets on reconnect | existing | Mk2: spin at about 200 dps, drop the sensor power or SDA so it reconnects, hold still; first yaw_rate under 5 dps, no `gyro_disagree` jump |
| `/status` built as one ArduinoJson document (`lib/status_doc`) | `fbd2dffc` | Mk2 diagnostics page: every panel fills, I2C "Devices found" lists the BNO055, rates show one decimal |
| USB bridge task pinned to core 1 | existing | Mk2 over USB-Serial/JTAG: stream 10 kB with the task on core 0 (expect 2 to 5 lost per 10 kB) and core 1 (expect 0) |
| Velocity jig diagnostics idle `delay(2)` (USB is serviced from an IRQ every 1 ms, so the reason looks wrong) | existing | Jig: build with the delay removed; diagnostics exits within 100 ms of a serial byte and the console never stalls over 5 minutes |
