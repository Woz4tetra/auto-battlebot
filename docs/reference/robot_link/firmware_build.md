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
