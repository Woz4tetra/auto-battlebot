# ESP-NOW dongle

USB serial from the Jetson in, ESP-NOW to mr_stabs_mk2 out. Design and wire format:
`docs/plans/espnow_link_plan.md` and `firmware/shared/espnow_protocol/`.

## What it does

- Forwards every framed message from the Jetson as soon as it arrives.
- Re-sends the latest drive command every 4 ms (250 Hz), byte for byte, so one lost packet is
  covered quickly. Stops 100 ms after the Jetson goes quiet, so it never keeps a dead process's
  command alive.
- Sends a keepalive (`MSG_PING`) whenever the radio has been idle 10 ms. The robot uses it to
  find the channel, to know where to send telemetry, and to validate a freshly flashed image.
- Broadcasts until it hears the robot's first telemetry or OTA status, then unicasts to that MAC
  (MAC-layer ACK and retries).
- Reports link stats (`MSG_LINK_STATS`) to the Jetson every 100 ms.
- Picks the channel at boot: `espnow_protocol::CHANNEL_OVERRIDE` when nonzero (1 in Phase 1),
  otherwise a promiscuous scan for the quietest of 1, 6, and 11.

It holds no authority and never invents a command.

## Hardware

Development dongle: Adafruit QT Py ESP32-S3, 8 MB flash, no PSRAM (`adafruit_qtpy_esp32s3_nopsram`),
MAC `d8:3b:da:8e:ed:2c`. The match dongle will be an ESP32-S3 with a u.FL antenna. Adding one
takes:

1. An env with its `board =` line in `platformio.ini`.
2. Its MAC in `DONGLE_MACS[]` in `firmware/mr_stabs_mk2/include/espnow_link.h`, then reflash the
   robot.
3. Its MAC (the USB serial number) in the dongle rule in `install/install_udev_rules.sh`.

The robot uses the same chip, so label both boards. The dongle prints its role and MAC once at
boot.

## Build and flash

```bash
cd firmware/espnow_dongle
~/.platformio/penv/bin/platformio run -t upload --upload-port /dev/ttyACM0
```

No button press needed: esptool resets the chip into its ROM bootloader over the USB-Serial/JTAG
port (`board_upload.before_reset = usb_reset`). Use the port's `/dev/serial/by-id/` name if more
than one ESP32-S3 is plugged in, since the robot shows up with the same VID/PID.

If the board was put into the bootloader by hand (BOOT held through reset), esptool's reset after
the upload can leave it in the bootloader. Press RESET, or:

```bash
~/.platformio/penv/bin/python ~/.platformio/packages/tool-esptoolpy/esptool.py \
    --port /dev/ttyACM0 --before no_reset --after watchdog_reset read_mac
```

## Checking it

With nothing else on the port, the dongle streams a `MSG_LINK_STATS` frame every 100 ms.
`sent` climbs by ~100 a second from keepalives alone, and `robot_known` turns 1 once a robot
running the ESP-NOW firmware answers.
