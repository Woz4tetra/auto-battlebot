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
2. Pair it on the robot's diagnostics page: ALLOW next to its MAC in the ESP-NOW section. No
   reflash; the robot keeps paired dongles in NVS and accepts no others.
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

## LED

The QT Py's NeoPixel shows the link. Color is the link state, brightness is the Jetson: bright
while it is commanding (a command inside the last 100 ms), dim otherwise.

| LED | Meaning |
| --- | --- |
| Solid blue, dim | Booting, or scanning for a channel (~2 s when the scan runs) |
| Blinking blue | Broadcasting: the robot has never answered |
| Red | Robot heard before, nothing from it for 500 ms |
| Amber | Robot answering, unicast ACK rate under 90% over the last 100 ms |
| Green | Robot answering, ACKs healthy |

Boards without `PIN_NEOPIXEL` in their variant build without the LED.

## Diagnostics page

`scripts/espnow_diag.py` reads the dongle's serial stream and serves a page styled after the main
dashboard: a checklist that stops at the first failure, dongle link stats, robot telemetry
delivery and timing, every telemetry field, per-type frame counts, an event log, and 60 s plots.
It flags frames from a second dongle on the channel. Whether the robot has paired this dongle is
only visible on the robot's page; here an unpaired dongle shows as the robot never answering.

```bash
# The dongle allows one reader: this stops auto-battlebot and restarts it on exit.
venv/bin/python firmware/espnow_dongle/scripts/espnow_diag.py --manage-service
# open http://<host>:8090
```

- **SEND COMMANDS** streams commands at `--command-hz` (50), starting neutral, and measures round
  trip from the echo in telemetry. Two sliders set channel a (ch0, linear) and channel b (ch1,
  angular) in [-1, 1], raw: no drive processor, deadzone, or channel reversal. **Space** sets both
  to neutral wherever focus is. The page refreshes the values every 100 ms; after 0.3 s without
  one (tab closed or hidden, laptop asleep) the tool sends neutral on its own. The robot only
  drives from them while armed with SA on. The panel shows the drive command the robot used next
  to the one it should report for the sliders, and turns red on a mismatch while ESP-NOW drives.
- **RESET DONGLE** pulses the USB-Serial/JTAG reset so the dongle reprints its banner. Untested on
  hardware so far.
- **SAVE LAST 2 MIN** writes the in-memory ring (every frame, text line, and event) to JSONL;
  **CAPTURE** writes the live stream. Both land in `~/espnow_diag/`. **SNAPSHOT** downloads the
  current state as JSON.

`scripts/fake_dongle.py` simulates the dongle and robot on a pty, so the page runs with no
hardware. Flags inject faults: `--no-robot`, `--ack`, `--drop`, `--robot-channel`,
`--foreign-every`, `--crc-every`.

```bash
venv/bin/python firmware/espnow_dongle/scripts/fake_dongle.py /tmp/fake_dongle --autonomy &
venv/bin/python firmware/espnow_dongle/scripts/espnow_diag.py --port /tmp/fake_dongle
```
