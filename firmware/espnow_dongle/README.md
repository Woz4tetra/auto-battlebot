# ESP-NOW dongle

USB serial from the Jetson in, ESP-NOW to mr_stabs_mk2 out. Design and wire format:
`firmware/shared/espnow_protocol/`.

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

## Notes moved from code comments

### General

- `espnow_dongle/platformio.ini`: Hardware USB-Serial/JTAG CDC: enumerates as 303a:1001 with the MAC as its USB serial number, which the udev rule keys on, and needs no TinyUSB stack.
- `espnow_dongle/scripts/fake_dongle.py`: Simulated ESP-NOW dongle on a pty, for running espnow_diag.py without hardware. Speaks the dongle's serial side: the boot banner, MSG_LINK_STATS every 100 ms, and robot telemetry at 100 Hz, echoing the seq and jetson_tx_us of the last MSG_COMMAND it reads. Flags inject the faults the diagnostics page should catch. venv/bin/python firmware/espnow_dongle/scripts/fake_dongle.py /tmp/fake_dongle & venv/bin/python firmware/espnow_dongle/scripts/espnow_diag.py --port /tmp/fake_dongle
- `shared/espnow_protocol/src/espnow_protocol.h`: MD5 or image check failed in Update.end host abort, arming, or silence

### espnow_diag section

- `espnow_dongle/scripts/espnow_diag.py`: ESP-NOW link diagnostics: reads the dongle's serial stream and serves a live page. Decodes every serial frame the dongle forwards (link stats, robot telemetry, OTA status, and any frame some other host put on the channel), tracks ACK rate, telemetry drops, arrival gaps, and command round trip, and runs a checklist that names the likely fault. Every frame lands in a 2-minute ring that can be saved after the fact, and CAPTURE writes the whole stream to JSONL. The dongle allows one reader. Stop the app first, or pass --manage-service: venv/bin/python firmware/espnow_dongle/scripts/espnow_diag.py --manage-service # then open http://<host>:8090 SEND COMMANDS streams commands at --command-hz so the command path, sequence numbers, and round trip can be measured. They start neutral (channel_a = channel_b = 0); the page's sliders set them, and the space bar returns both to neutral. The page refreshes the values every 100 ms; after COMMAND_DEADMAN_S without a refresh (tab closed, laptop asleep, network gone) they fall back to neutral. The robot only drives from them while armed with SA on, so the handset keeps authority.

### Diagnostics page

- `espnow_dongle/scripts/espnow_diag.py`: The dongle forwards every air frame carrying the session magic, whoever sent it. Only these two come from the robot; anything else read back from the air is another dongle or host on this channel.
- `espnow_dongle/scripts/espnow_diag.py`: Same command echoed again: its second sample also holds a 10 ms slot wait.
- `espnow_dongle/scripts/espnow_diag_web/app.js`: Checks after the first error are not shown by the server; list them greyed so the shape of the checklist stays put.

### resetting the dongle

- `espnow_dongle/scripts/espnow_diag.py`: One chip reset through the USB-Serial/JTAG lines: DTR low with RTS high holds EN low, then RTS low releases it with GPIO0 free, so the dongle boots its firmware and prints the banner. One pulse only: two resets inside ~500 ms drop an Adafruit board into TinyUF2.

### command repeat

- `espnow_dongle/src/main.cpp`: Re-send the latest command at 250 Hz so a lost packet is covered quickly.

### status LED

- `espnow_dongle/src/main.cpp`: NeoPixel levels out of 255. Full brightness is blinding at arm's length.

### command slot

- `espnow_dongle/src/main.cpp`: Frames waiting for the radio. Commands do not queue here: only the newest command matters, so it lives in its own slot and jumps the queue.

### channel selection

- `espnow_dongle/src/main.cpp`: Scores every channel by bytes heard and picks the quietest of 1, 6, and 11, counting energy from overlapping neighbors. Bytes heard is a proxy for airtime: it misses non-WiFi interferers (other teams' control links, video TX), which nothing in a WiFi scan can see.

### USB serial

- `espnow_dongle/src/main.cpp`: Never block on the host. With no reader the frame is dropped, which is what we want.

### OTA path

- `espnow_dongle/src/main.cpp`: Everything else goes out once. OTA retries are the host's job; a repeated chunk would only be dropped by the robot as out of order.

### link stats

- `espnow_dongle/src/main.cpp`: Broadcast until the robot is heard from. Broadcast gets no MAC-layer ACK or retry, so the link stats only mean something once robot_known is set.

### robot discovery

- `espnow_dongle/src/main.cpp`: Only the robot sends these types, so the first one names the robot.

### OTA serial buffer

- `espnow_dongle/src/main.cpp`: The default 256-byte RX buffer overflows on the OTA host's 16-chunk (~4 KB) window and the HWCDC ISR drops the excess, so every chunk after the first fails its CRC. Two windows.

### PHY rate

- `espnow_dongle/src/main.cpp`: After esp_wifi_start(), which WiFi.mode() ran. The 1 Mbps default costs ~1.2 ms a frame.

### wire format

- `shared/espnow_protocol/src/espnow_protocol.h`: Wire format for the ESP-NOW link between the Jetson, the USB dongle, and mr_stabs_mk2. Plain C++17 with no Arduino dependency: the robot firmware, the dongle firmware, the native unit tests, and the Jetson app all compile this one header, so the three ends cannot disagree about a field. Two hops: Jetson <--serial frame--> dongle <--air frame--> robot Serial frame: magic:u16 (0xB07B, little endian), type:u8, length:u8, payload, crc16:u16 over type, length, and payload. Air frame: session magic:u8[4], type:u8, payload. Length is the ESP-NOW frame length.
- `shared/espnow_protocol/src/espnow_protocol.h`: Jetson to robot, CommandPayload robot to Jetson, TelemetryPayload Jetson to robot, TunableSetPayload dongle to Jetson, LinkStatsPayload dongle to robot, no payload: keepalive, see PING_PERIOD_MS host to robot, OtaBeginPayload host to robot, OtaChunkHeader then data host to robot, no payload host to robot, no payload robot to host, OtaStatusPayload

### threading

- `src/main.cpp`: The receive callback runs in WiFi task context. It copies the frame into the queue and returns. Everything else happens in `loop()`.

### LED states

- `src/main.cpp`: Color is the link, brightness is the Jetson. Blue blinking: broadcasting, the robot has never answered. Red: robot heard before, nothing from it for `ROBOT_SILENCE_LED_RED_MS`. Amber: robot answering, unicast ACK rate under 90% over the last 100 ms. Green: robot answering, ACKs healthy. Bright while the Jetson is commanding (a command inside `COMMAND_SILENCE_MS`), dim otherwise.
