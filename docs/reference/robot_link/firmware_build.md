# Firmware build configuration

PlatformIO reads `platformio.ini` in each firmware project. This page explains the options that are not self-evident. Run the Mk2 native tests with `~/.platformio/penv/bin/pio test -e native` from `firmware/mr_stabs_mk2`.

## `firmware/mr_stabs_mk2/platformio.ini`

| Option | Meaning |
| --- | --- |
| `[common] platform = espressif32@6.13.0` | Pinned to the official espressif32 platform (Arduino core 2.x, IDF 4.4). The pioarduino fork (55.x, IDF 5) drops NeoPixel's `rmt_reserved_channels` and deprecates the legacy RMT driver that `DShot.cpp` uses. |
| `[common] lib_extra_dirs = ../shared` | Pulls in the ESP-NOW wire format, shared with `firmware/espnow_dongle` and the Jetson app. `[env:native]` sets it again because it does not extend `[common]`. |
| `[env:local] board_upload.use_1200bps_touch = no` | The board file assumes TinyUSB: a 1200 bps touch to enter the bootloader, then a wait for the port to come back. |
| `[env:local] board_upload.wait_for_upload_port = no` | Same reason as above. |
| `[env:local] board_upload.before_reset = usb_reset` | With the USB-Serial/JTAG port, esptool's `usb_reset` enters the bootloader directly. No BOOT press is needed, even with the firmware hung. |

## `firmware/espnow_dongle/platformio.ini`

| Option | Meaning |
| --- | --- |
| `board_upload.use_1200bps_touch = no` | `adafruit_qtpy_esp32s3_nopsram.json` sets it true, for TinyUSB: a 1200 bps touch to enter the bootloader. `-DARDUINO_USB_MODE=1` selects the USB-Serial/JTAG port in the firmware only; upload settings come from the board file, so the override is still needed. |
| `board_upload.wait_for_upload_port = no` | The board file sets it true, to wait for the TinyUSB port to come back. Same reason as above. |
| `board_upload.before_reset = usb_reset` | With the USB-Serial/JTAG port, esptool's `usb_reset` enters the bootloader directly, with no BOOT press. |

`scripts/upload` defaults to `/dev/espnow_dongle` (`install/install_udev_rules.sh`). The dongle and the Mk2 both report `303a:1001`, so PlatformIO's auto-detect can pick either one.

## `firmware/velocity_jig/platformio.ini`

| Option | Meaning |
| --- | --- |
| `platform = https://github.com/maxgerhardt/platform-raspberrypi.git` | The Philhower core (`board_build.core = earlephilhower`) is provided through maxgerhardt's platform fork. |
| `board = adafruit_feather` | The Adalogger is pin-identical to the Feather RP2040 for our purposes (8 MB flash). If your PlatformIO install lists a dedicated board id, prefer it: `pio boards adafruit \| grep -i adalogger`. |
| `lib_deps = adafruit/Adafruit LSM6DS` | The ISM330DHCX driver (`Adafruit_ISM330DHCX.h`) ships inside the LSM6DS library, so there is no separate ISM330DHCX dependency. |

## Workarounds

- ESP32 platform pin: `firmware/espnow_dongle/platformio.ini` and `firmware/mr_stabs_mk2/platformio.ini` both pin `espressif32@6.13.0`. The globally installed pioarduino 55.x (IDF 5) otherwise wins over an unpinned `espressif32`. The wire format assumes IDF 4.4's 250-byte ESP-NOW frames. Not verified by building unpinned.
- `AIR_MAX_FRAME` in `firmware/shared/espnow_protocol/src/espnow_protocol.h` is 250, `ESP_NOW_MAX_DATA_LEN` on IDF 4.4. ESP-NOW v2 needs IDF 5.4, which the pin excludes.
- `firmware/velocity_jig/include/config.h` names the status LED pin `PIN_STATUS_LED`, not `PIN_LED`, because the arduino-pico variant defines `PIN_LED` as a macro. Not verified by building.
- `firmware/velocity_jig/src/main.cpp` calls `delay(2)` each pass of the diagnostics idle loop so USB CDC is serviced. Unverified.
- The debug-build wait for the serial monitor in `firmware/velocity_jig/src/main.cpp` loops `delay(10)` up to a timeout, before any hardware init, blinking the LED. `delay()` services USB, so the CDC port enumerates even if a later init call hangs. Not tested against a hanging init.
- `firmware/mr_stabs_mk2/scripts/coredump` reads the coredump partition with esptool, using the offset and size of the coredump row in `partitions.csv`. `esp_coredump` reads flash itself only with a full ESP-IDF install. The offset matches `partitions.csv` (0x310000, 64K). The `esp_coredump` claim is unverified.
- `firmware/mr_stabs_mk2/scripts/ota` passes one key (`-i`, `IdentitiesOnly=yes`) because the ssh agent's full key set exhausts sshd's `MaxAuthTries`. Unverified here.
