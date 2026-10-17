# Plan: ESP-NOW command and telemetry link for mr_stabs_mk2

## Goal

Add a direct ESP-NOW link between the Jetson and the mr_stabs_mk2 firmware that:

1. Delivers autonomous velocity commands with roughly 5 ms less transport delay.
2. Streams the firmware diagnostics struct back to the Jetson at loop rate.
3. Lets the on-robot web dashboard be deleted, freeing RAM and flash for the camera port.
4. Carries firmware updates, so the SoftAP and `ArduinoOTA` can be deleted too. The robot radio
   then runs ESP-NOW only, with no access point and no combat/tuning radio mode switch.

The Crossfire link keeps full authority. ESP-NOW is a transport shortcut layered on top of the
existing path, never a replacement for it.

### Status (2026-10-14)

Phase 1 code is built and unit tested, and ran on the robot and the development dongle on the
bench: ESP-NOW OTA worked end to end (887 KB in 18.3 s, validated). The robot's radio is ~19 dB
short of the dongle's on receive, so **ESP-NOW is switched off until the new PCBA** (see Robot
radio) and the arena measurements wait for it.

| Piece | Where | State |
|---|---|---|
| Wire format, sequence rules, OTA ordering | `firmware/shared/espnow_protocol/` | Built. 17 host tests in `firmware/mr_stabs_mk2/test/test_espnow_protocol/` |
| Dongle firmware | `firmware/espnow_dongle/` | Flashed to the development QT Py (`d8:3b:da:8e:ed:2c`). Talks to the robot on the bench |
| Robot firmware | `firmware/mr_stabs_mk2/src/espnow_link.cpp`, `main.cpp` | Flashed. Shadow mode (`ESPNOW_COMMAND_SUBSTITUTION = false`), ESP-NOW OTA, sweep. USB rescue needs no BOOT press |
| `EspNowTransmitter`, `CompositeTransmitter` | `src/transmitter/` | Built. Tests in `tests/test_espnow_transmitter.cpp` |
| Flasher | `firmware/mr_stabs_mk2/scripts/espnow_ota.py`, `scripts/ota` | Tested against a simulated robot dropping 5% of chunks, then against the robot |
| ZED Box profiles | `config/mr_stabs_mk2_*_zed_box.toml` | Back on OpenTX alone until the new PCBA; the composite sections are in `d8c74c0f` |
| udev rule | `install/install_udev_rules.sh` | Written and checked with `udevadm verify`. Not installed on the box |

Remaining: next steps 1 to 3 and 6 (CRSF baud, NHRL rules, ZED Box setup, the Phase 1 OTA tests and
measurements), then Phases 2 and 3.

2026-10-17: the firmware slew limiter ("New: linear slew limiter on the arbitrated output") is
deleted, and the ZED Box profiles set `max_motor_rpm_per_sec = 0`. Nothing rate-limits the drive
command on either transport, so the plant is characterized and the RL policy trained without
one. The sections below that describe the limiter are history.

2026-10-17: the `COMMAND_TIMEOUT` check (5 s of identical CRSF frames) is deleted. With the handset
sticks idle it stopped the motors 5 s into any ESP-NOW drive. `isLinkUp()` is the only link
failsafe now, which needs the Crossfire receiver's failsafe set to cut.

### Robot radio (2026-10-13)

The robot's QT Py is the weak end of the link. Measured with both boards ~30 cm apart, chassis
open, both at 20 dBm TX power, by a promiscuous sniffer on each board:

| Receiver | Hears | RSSI |
|---|---|---|
| Robot | Dongle | -85 dBm |
| Dongle | Robot | -66 dBm |
| Laptop Wi-Fi | Dongle softAP | 95-100% |
| Laptop Wi-Fi | Robot softAP | 57-74% |

The dongle transmits strongly (the laptop agrees), yet the robot hears it 19 dB worse than the
dongle hears the robot. The robot's RF path is bad, worse on receive than transmit. Crossfire
(868/915 MHz) is not the cause: low RSSI is missing signal, not added noise. The fix is the new
PCBA, not a new dongle. Re-run the sniffer on it before any arena measurement; ~-40 dBm at 30 cm
is the target.

The robot sends ESP-NOW from its softAP MAC (`f6:12:fa:59:5c:b4`), not its STA MAC (`f4:...`).

**ESP-NOW is off until the new PCBA (2026-10-14).** At the robot's usual distance from the ZED Box
the 6 Mbps link delivered nothing: 0% ACK and no telemetry, both directions. 1 Mbps
(`WIFI_PHY_RATE_1M_L`, ~5 dB more receive sensitivity) ran at 96 telemetry frames/s and 100% ACK
with the robot near the dongle, then fell to 0% within minutes at normal range, so it was reverted
to 6 Mbps. Instead:

- Robot: `ESPNOW_ENABLED = false` in `firmware/mr_stabs_mk2/src/main.cpp` skips ESP-NOW init,
  telemetry, OTA, and command substitution. The SoftAP dashboard and `scripts/ota_wifi` are the
  ways in.
- ZED Box: the three `mr_stabs_mk2_*_zed_box` profiles are back on the OpenTX transmitter alone
  (`Esp32DiagnosticsOpenTxTransmitter` for sysid). No `CompositeTransmitter`, no dongle.

To turn it back on with the new board: re-run the RSSI test (~-40 dBm at 30 cm), set
`ESPNOW_ENABLED = true`, and restore the profiles' composite sections from `d8c74c0f`.

### Standing decision: latency wins every tradeoff

Where a choice in this plan trades latency against something else, take the lower-latency option.
Applied throughout, that resolves to:

| Choice | Decision | Cost accepted |
|---|---|---|
| ESP-NOW PHY rate | 6 Mbps OFDM via `esp_wifi_config_espnow_rate()` | No `WIFI_PROTOCOL_LR`, so ~10 dB less link budget |
| Channel | Dongle scans and picks at boot, robot sweeps to find it | Each match runs on an untested channel |
| Dongle repeat rate | 250 Hz, forwarding on arrival | ~6% channel duty cycle |
| `FRESH_STREAK_REQUIRED` | 3, not 5 | Slightly more source-switch flapping at range |
| CRSF serial baud | 921600, not the 400000 default | None, it is free |
| SoftAP | Deleted, OTA moves to ESP-NOW | No laptop Wi-Fi to the robot; every flash goes through a dongle |

Each is justified in the section that raises it. The range cost of dropping LR is the one that
carries real risk, and it is mitigated by the fact that losing ESP-NOW costs latency, not control.

## Tradeoffs

### The radio configuration is already optimal

Every setting that could have been a cheap win has been checked and is already at its best value:

| Parameter | Status | Evidence |
|---|---|---|
| Crossfire over-air rate | 150 Hz, Crossfire's maximum | Confirmed by operator |
| EdgeTX mixer scheduler | Enabled, module-synchronized | `radio/src/targets/taranis/hal.h:2961` (x9dp2019), `horus/hal.h:1212` (tx16s) |
| Mixer period source | Driven by the Crossfire module's reported rate | `radio/src/pulses/crossfire.cpp:191-199` |
| CRSF serial baud | 400000 minimum, 4 ms frame period | `radio/src/telemetry/crossfire.h:126-146` |
| 115200 baud (16 ms period) | Unreachable on this hardware | `crossfire.h:136`, `CROSSFIRE_MAX_EXTERNAL_BAUDRATE` on STM32F4 excludes it |

There is no free latency left in the radio config. Any further reduction requires a different
link, which is what this plan proposes.

### Latency: fixed pipeline delay

Two stages are common to both paths and cancel in the difference: the 33.3 ms command generation
interval (`camera_fps = 30` in `config/_common.toml:13`, with `send()` called once per tick at
`src/runner.cpp:473`) and the firmware `loop()` quantization.

**Current path, Jetson to firmware:**

| Stage | Best | Typical | Worst |
|---|---|---|---|
| Jetson to handset, USB CDC | 0.3 | 1.0 | 2.0 |
| Mixer quantization, synchronized to the 150 Hz module | 0 | 3.3 | 6.7 |
| Handset to module, CRSF 26 B at 420 kbaud | 0.62 | 0.62 | 0.62 |
| Module processing plus air time | 1.0 | 1.5 | 2.5 |
| RX processing plus UART out | 0.9 | 1.2 | 1.6 |
| **Total (ms)** | **2.8** | **7.6** | **13.4** |
| One lost frame costs a full period | | | **+6.7 = 20.1** |

**ESP-NOW path, and it depends heavily on the PHY rate:**

| Stage | 6 Mbps OFDM | 1 Mbps DSSS (default) | LR mode |
|---|---|---|---|
| Jetson to dongle, USB CDC | 1.0 | 1.0 | 1.0 |
| Dongle parse plus `esp_now_send` queue | 0.6 | 0.6 | 0.6 |
| DIFS, backoff, data, SIFS, ACK | 0.25 | 1.4 | ~3.0 |
| Recv callback to shared struct | 0.2 | 0.2 | 0.2 |
| **Typical total (ms)** | **2.1** | **3.2** | **4.8** |

**Resulting saving:**

| Case | Saving |
|---|---|
| Best case, both clean | ~2 ms |
| Typical, ESP-NOW at 6 Mbps OFDM | ~5.5 ms |
| Typical, ESP-NOW at the 1 Mbps default | ~4.4 ms |
| Typical, ESP-NOW with `WIFI_PROTOCOL_LR` enabled | ~2.8 ms |
| Crossfire loses a frame, ESP-NOW clean | ~15 ms |
| ESP-NOW hits a retry chain, Crossfire clean | **ESP-NOW is 5-15 ms worse** |

These are computed from frame sizes, baud rates, and 802.11 timing constants. Nothing here is
measured. Phase 1 measures it, and the `jetson_tx_us` round-trip echo gives it directly.

### Range and latency pull in opposite directions

`WIFI_PROTOCOL_LR` buys roughly 10 dB of link budget by lowering the PHY rate, which roughly
doubles air time and eats about half the latency saving. You can have the range mitigation or the
fastest link, not both.

**Decision: 6 Mbps OFDM, no LR.** That is the ~5.5 ms row rather than the ~2.8 ms row. Two things
make the range cost tolerable: the trainer path still carries every command, so a weak ESP-NOW
link degrades to today's latency rather than to no control; and the failure is measurable in
Phase 1 as ACK rate, so it will not be a surprise.

If arena testing shows the link failing on range rather than congestion, the fallback is not LR
but a better antenna and dongle placement, which costs no latency.

### Channel selection is a latency lever

CSMA backoff sits directly in the ESP-NOW budget, and backoff grows with channel occupancy. A
congested channel does not just raise loss, it raises median latency through deferral and retry.

**Decision: the dongle picks the channel at boot and the robot sweeps to find it.** No pit scan,
no per-event constant edit, nothing to do on competition day.

The obvious version of this breaks. If both ends scan independently and each picks the quietest
channel, they will disagree: different antennas, different locations (dongle at the Jetson, robot
in the box), different instants, different tie-breaks. A channel mismatch produces silent send
failures, not an error, so the link would be dead with nothing in the logs explaining why.

Make it asymmetric instead.

**The dongle picks.** One scan at boot, then fixed for the rest of the run. The dongle is the right
authority because it is not the end that takes hits and brown-out reboots mid-match. Score all 11
channels by occupancy but only ever select from {1, 6, 11}, so the pick absorbs adjacent-channel
energy from its neighbors instead of landing in the skirts between the clusters.

**The robot discovers by sweeping.** The dongle already transmits every 4 ms (the 250 Hz repeat
below), which gives the robot a continuous signal to search for:

```
for channel in [last_known_good, 1, 6, 11]:
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE)
    wait 25 ms                     // ~6 dongle repeats plus switch settling
    if a frame carrying the session magic arrived:
        lock, persist the channel, done
```

Three candidates at 25 ms is a 75 ms worst-case sweep. The common case is one dwell on the
last-known-good channel, so 25 ms. The robot runs on the trainer path for the duration, which is
the same degradation as any other ESP-NOW dropout.

The sweep also covers re-acquisition. After `REACQUIRE_SILENCE_MS` with no valid frame, the robot
resumes sweeping. That self-heals a robot reboot mid-match, which a flashed constant does not, and
it self-heals the dongle restarting on a different channel.

#### What auto selection costs

- **The scan measures beacons, not airtime.** The interferers that actually drive CSMA backoff at
  an event are other teams' 2.4 GHz control links, video TX, Bluetooth, and the ZED's own USB3
  noise. None of them emit beacons, so an AP-count scan can rank a channel quiet while it is
  saturated. Sampling occupancy in promiscuous mode is the better metric and costs 100 ms or more
  per channel, so budget 1-3s of dongle boot time for a scan worth trusting.
- **Boot time and boot location are the wrong sample.** The dongle scans where the Jetson powers
  on, not in the cage, and before the match rather than during it. The pit scan has the same flaw,
  so this is not a regression, but auto selection does not fix it either.
- **Reproducibility is gone.** Every match runs on a channel that was never bench tested. Mitigate
  by reporting the selected channel in `diag_data_t` and the `0x04` link stats frame so it lands in
  the MCAP, and by pinning `ESPNOW_CHANNEL_OVERRIDE` for bench runs.
- **Sweep flap.** A weak but alive link can cross the staleness threshold and trigger a sweep that
  loses the packets which would have recovered it. `REACQUIRE_SILENCE_MS` sits well above
  `COMMAND_STALE_MS` for exactly this reason.
- **No rescan mid-run.** A dongle that periodically rescans to stay adaptive drops the link on
  every rescan while the robot re-acquires. Scan once at boot. There is no other scan trigger: with
  the SoftAP gone there is no radio mode transition to rescan on.

What makes this an acceptable trade is that ESP-NOW holds no authority. A bad pick costs ~5 ms of
latency, not control, because the trainer path carries every command. If ESP-NOW were the control
link, a flashed constant and a pit scan would be the right answer.

### The 30 Hz command rate dominates everything, and is solved elsewhere

At 30 fps a new command exists only every 33.3 ms, so the average command is already 16.7 ms stale
before any transport touches it. That is more than three times what this entire plan saves.

**Perception cannot be sped up.** The camera runs at 30 fps (`config/_common.toml:13`) and the
TensorRT inference behind it sets the rest. Raising the frame rate is not the fix.

**This is solved by [`control_loop_thread_plan.md`](control_loop_thread_plan.md)**, which moves the
filter, target selection, navigation, and transmit onto their own thread at a configurable rate
(default 250 Hz). Perception becomes a slow measurement correcting a fast prediction rather than
the clock gating the pipeline. That plan claims ~15 ms from the command interval alone, and up to
~66 ms once the filter predicts forward to compensate for perception latency.

Two consequences for this plan:

1. **The Jetson will send at the control rate, not 30 Hz.** The dongle repeat below becomes a
   safety net covering real packet loss rather than a correctness requirement covering a slow
   producer.
2. **`CompositeTransmitter` needs a per-child transmit rate cap.** The trainer path cannot absorb
   250 Hz; ESP-NOW can. Both plans need this, and it belongs in the composite.

Sequencing: the control loop thread is worth roughly 13x what this plan's command path is worth and
needs no new hardware. Land it first.

### The trainer path is worth speeding up too

Both paths carry every command, and ESP-NOW falls back to the trainer path on any dropout, so the
fallback's latency is part of the real distribution rather than a rare edge case.

`CROSSFIRE_FRAME_PERIODS` in `radio/src/telemetry/crossfire.h:146` ties the frame period to the
CRSF serial baud: 400000 gives 4 ms, 921600 and above give 2 ms. Raising it also cuts the 26-byte
frame transmission from 0.62 ms to 0.28 ms.

**Decision: raise the model's CRSF baud to 921600.** The gain is roughly 1 ms and partly absorbed
when `getAdjustedRefreshRate()` has a valid module sync, but it costs nothing and it lowers the
fallback path. Set it in Model Setup on the mr_stabs_mk2 model; it is a per-model field
(`g_model.moduleData[EXTERNAL_MODULE].crsf.telemetryBaudrate`).

### Summary

| | Crossfire (current) | ESP-NOW |
|---|---|---|
| Typical transport delay | ~7.6 ms | ~2.1 to 4.8 ms |
| Frequency agility | FHSS across 868/915 MHz | None, one fixed 20 MHz channel |
| Band congestion at an event | Low | Severe |
| Behavior on a lost packet | Next frame in 6.7 ms, bounded | Exponential backoff, unbounded |
| Redundancy | Re-sends at 150 Hz regardless of new data | Only sends when the Jetson does, see below |
| Link quality metric | LQ and RSSI in CRSF telemetry | Must be derived from ACK counts |
| Telemetry bandwidth | A few hundred bytes/s | 72-byte struct at loop rate |
| Arm authority | Yes | Never |

ESP-NOW wins on median latency and telemetry bandwidth. Crossfire wins on every tail-behavior
property, which is why the design gives ESP-NOW no authority and always mirrors to the trainer
path.

### The dongle must re-send, or the tail gets worse

Crossfire transmits at 150 Hz continuously, re-sending the last trainer value whether or not the
Jetson updated it. A lost Crossfire frame is covered 6.7 ms later.

ESP-NOW as designed here sends only when `send()` is called, which is 30 Hz. One lost packet
leaves a 33.3 ms gap, and two consecutive losses exceed `COMMAND_STALE_MS` and drop back to the
trainer path.

Fix: the **dongle** repeats the last received command on its own 250 Hz timer and forwards
immediately on arrival rather than waiting for the next slot. That restores the redundancy
Crossfire gets for free and keeps the 60 ms staleness window meaningful.

Once `control_loop_thread_plan.md` lands, the Jetson sends at the control rate rather than 30 Hz,
so the repeat covers only real packet loss instead of also covering a slow producer. Build the
repeat anyway: it is the difference between a bounded and an unbounded dropout tail, and it keeps
this plan independent of the control loop work landing first.

### What this does not fix

ESP-NOW runs on the same 2.4 GHz radio as WiFi, on one fixed 20 MHz channel with no frequency
hopping. It is more exposed to arena congestion than Crossfire at 868/915 MHz, which is exactly
why it gets no authority.

## Design principle: authority and transport are separate

`radio_data_t` (`firmware/mr_stabs_mk2/include/crsf_bridge.h:20`) already splits the two concerns:

```c
typedef struct radio_data {
    float a_percent, b_percent;                 // transport: the command
    bool armed, connected, button_state;        // authority: arm, link state, mode
    three_state_switch_t flip_switch_state;
} radio_data_t;
```

ESP-NOW may substitute `a_percent` and `b_percent`. It may never touch `armed`, `connected`,
`button_state`, `flip_switch_state`, or the new `autonomy_enabled` below. Those stay on Crossfire so a switch throw on the handset
always stops the robot, including when the ESP-NOW link is the thing misbehaving.

The gate that decides whether ESP-NOW may substitute at all is a new authority field,
`autonomy_enabled`, read from ch5 (SA, `AutoEn` in `firmware/config/CHANNEL_CONVENTION.md`). SA is
the switch that already mixes the trainer inputs into ch0/ch1 on the handset, and the Jetson reads
the same channel back as `trainer_enable_channel = 5`. The firmware ignores ch5 today, so
`CrsfBridge::update()` gains one line:

```c
radio_data->autonomy_enabled = channels->ch5 > MID_CYCLE;   // SA down: autonomy on
```

and the link-down branch sets it false alongside `armed`. Do not gate on `flip_switch_state`: ch7
(SB) is the flip direction override, and its UP position forces the robot upside-down.

### Both paths carry every command

The Jetson sends each command over the trainer link *and* over ESP-NOW. This gives the safest
degradation:

- ESP-NOW fresh: the firmware uses it, and the command arrives roughly 5 ms sooner.
- ESP-NOW stale or dead: the firmware falls back to the trainer channels, which carry the same
  autonomous command. Behavior degrades to exactly what `OpenTxTransmitter` does today.
- SA up (autonomy off): the handset stops mixing in the trainer inputs, so ch0/ch1 carry only the
  pilot's sticks, and the firmware's `autonomy_enabled` gate blocks ESP-NOW substitution. One
  switch takes both paths away from the Jetson.

### Bumpless transfer belongs in the firmware, not the Jetson

The two paths cannot be guaranteed bit-identical from the Jetson side. `limit_linear_acceleration`
(`src/transmitter/opentx_transmitter.cpp:128`) is stateful and reads
`get_channel_value(config_.trainer_enable_channel)` at line 134, so its behavior depends on handset
channel data that arrives only over the OpenTX serial link. A standalone `EspNowTransmitter` has no
channel input and cannot reproduce it. The limiter also sits behind the
`if (!serial_.is_open() || !enabled_) return;` guard at `opentx_transmitter.cpp:112`, so its state
stops advancing whenever that serial port drops, diverging from any parallel copy.

Rather than couple the two transmitters to keep their limiter state in sync, move the rate limit to
where the arbitration happens. The firmware slew-limits its arbitrated output, so whichever source
wins, the command ramps instead of stepping.

Applying the same rate limit twice in series is idempotent: when the Jetson already limited the
signal, the firmware limiter is a no-op and costs nothing. When the sources disagree, it catches
the step. This also makes the Jetson-side transmitters fully independent, which is what lets them
be separate implementations rather than one wrapping the other.

## Current state

### Firmware gates (all preserved)

`loop()` in `firmware/mr_stabs_mk2/src/main.cpp:234` runs three gates before driving motors:

| Line | Gate | Action on fail |
|---|---|---|
| `main.cpp:245` | `crsf->update()` returns `isLinkUp()` | `stop_escs()` |
| `main.cpp:49` | `COMMAND_TIMEOUT`, 5000 ms of identical frames | `stop_escs()` |
| `main.cpp:323` | `radio_data->armed` (CH4/SF) | `stop_escs()` |
| `main.cpp:377` | `is_upside_down` inverts `a_percent` | n/a |
| `main.cpp:387` | `mix_motor_outputs()` drives ESCs | n/a |

The ESP-NOW substitution goes after line 323. The new firmware slew limiter goes after line 377, so
it limits the post-inversion signal. Limiting before the flip would let a genuine inversion produce
a full-scale reversal in one tick.

Note on `COMMAND_TIMEOUT`: 5000 ms of identical frames is a backstop, not a watchdog.
`crsf->isLinkUp()` (`firmware/mr_stabs_mk2/src/crsf_bridge.cpp:15`) does the real failsafe work.
Do not copy the 5000 ms figure for the ESP-NOW staleness window.

### WiFi surface to be replaced

- `main.cpp:224` `WiFi.softAP(WIFI_SSID, WIFI_PASSWORD)`, defaults to channel 1
- `main.cpp:225` `setup_ota()`, ArduinoOTA over TCP
- `main.cpp:229` `diag_server.begin(tunables)`, ESPAsyncWebServer plus AsyncTCP

`diag_data_t` (`firmware/mr_stabs_mk2/include/diagnostics_server.h:4`) packs to 72 bytes with
padding, well inside the 250-byte ESP-NOW payload.

Current `firmware.bin` is 879 KB (899,808 bytes, `.pio/build/ota`, 2026-10-10). The board's
`tinyuf2-partitions-4MB.csv` has two 1408 KB app slots (`ota_0`, `ota_1`), so an image can grow by
about 60% before OTA of any kind stops fitting.

### Jetson transmitter

`OpenTxTransmitter` (`src/transmitter/opentx_transmitter.cpp`) does two jobs:

1. **Send** (`opentx_transmitter.cpp:110`): slew-limits linear, runs `DifferentialDriveProcessor`,
   writes trainer channels.
2. **Read** (`opentx_transmitter.cpp:65`): parses CRSF telemetry and handset channels, which is
   where `did_init_button_press()` (`opentx_transmitter.cpp:186`), the `trainer_enable_channel`
   gate, and `CommandFeedback` all come from.

Job 2 has no ESP-NOW equivalent. `EspNowTransmitter` therefore implements only job 1 and reports
nothing on the read side, and a new `CompositeTransmitter` drives both.

`OpenTxTransmitter` needs no changes at all under this design.

## Architecture

Three pieces, each a plain `TransmitterInterface` implementation registered in the existing
factory (`src/transmitter/config.cpp:36`):

```
CompositeTransmitter                     fans send() out to every child
  ├── OpenTxTransmitter    (primary)     trainer channels + all read-side duties
  └── EspNowTransmitter                  framed serial to the USB dongle
```

`EspNowTransmitter` is standalone and usable on its own. That matters for bench work: you can run
it without a handset connected to drive the robot straight from the Jetson, which is not possible
with the current transmitter at all.

### Composite policy

The first child in config order is **primary**. Everything read-side comes from it.

| Method | Policy |
|---|---|
| `initialize()` | Call all. Return primary's result. Non-primary failures log a warning and are ignored. |
| `send()` | Fan out the raw command to every child. No preprocessing in the composite. |
| `update()` | Call every child so their receive paths drain. Merge feedback maps, primary applied last so it wins on key collision. |
| `did_init_button_press()` | Call every child (no short-circuit, so latches clear), return the OR. |
| `is_connected()` | Primary only. A dead dongle costs latency, not control. |
| `enable()` / `disable()` | Fan out to all. |

The composite does no command preprocessing. It is a pure fan-out, which keeps it reusable for any
future transmitter pairing.

## Wire protocol

Two hops with different framing. The Jetson never speaks ESP-NOW directly.

```
Jetson <--framed serial--> ESP32 dongle <--ESP-NOW unicast--> mr_stabs_mk2
```

### Dongle hardware

Development uses an Adafruit QT Py ESP32-S3, the same chip as the robot. The development unit is
the 8 MB no-PSRAM variant (`adafruit_qtpy_esp32s3_nopsram`); the robot is the N4R2. It plugs into the ZED Box Mini's one USB-A port through a USB 2.0 hub shared with the
handset, on a short extension away from the box. The dongle firmware is a new PlatformIO project,
`firmware/espnow_dongle/`, pinned to `espressif32@6.13.0` like the robot. Sharing the board and
platform lets the dongle and robot share the framing, session magic, and peer setup code.

Identical boards bring two hazards:

- **They look alike on USB.** The dongle builds with the hardware USB-Serial/JTAG CDC
  (`ARDUINO_USB_MODE=1`), which enumerates as Espressif `303a:1001` with the chip MAC as its USB
  serial number. The robot shows the same VID/PID in its bootloader, so the udev rule below matches
  on the serial number.
- **They are easy to swap by hand.** Label both, and have the dongle firmware print its role and
  MAC address at boot. A dongle running the robot firmware tries to drive ESCs that are not there,
  and a robot running the dongle firmware does nothing.

The QT Py has only a PCB antenna. The match dongle should be an ESP32-S3 with an external antenna.
Pick a board where the antenna is fixed in hardware, such as an ESP32-S3-WROOM-1U module, which has
only a u.FL connector. Boards with both a PCB antenna and a u.FL connector can need a resistor moved
to switch over. Swapping boards changes three things: the `board =` line in
`firmware/espnow_dongle/platformio.ini`, the MAC in the robot's `DONGLE_MACS[]`, and the serial
number in the udev rule. The protocol and code do not change.

An ESP8266 cannot stand in, even for development. It has no `esp_wifi_config_espnow_rate()`, so it
is likely stuck at 1 Mbps, and it reaches USB through a CP2104 UART bridge. Both distort the Phase 1
latency numbers this plan is decided on.

### Serial framing (Jetson to dongle)

```
uint16_t magic   = 0xB07B
uint8_t  type
uint8_t  length            // payload bytes
uint8_t  payload[length]
uint16_t crc16             // CCITT over type, length, payload
```

Streaming-safe: the parser accumulates bytes and scans for magic, matching how `CrsfParser`
already handles partial reads from `SerialPort::read_available()`.

### Message types

| Type | Direction | Payload |
|---|---|---|
| `0x01` command | Jetson to robot | `seq:u32, jetson_tx_us:u32, channel_a:f32, channel_b:f32` |
| `0x02` telemetry | robot to Jetson | `TelemetryPayload`: the `diag_data_t` fields plus `autonomy_enabled` and the link fields, `seq` and `jetson_tx_us` echoed |
| `0x03` tunable set | Jetson to robot | `id:u8, value:f32` |
| `0x04` link stats | dongle to Jetson | `sent:u32, acked:u32, failed:u32, last_rtt_us:u32, channel:u8, robot_known:u8` |
| `0x05` ping | dongle to robot | none |
| `0x10` OTA begin | host to robot | `size:u32, md5:u8[16]` |
| `0x11` OTA chunk | host to robot | `offset:u32, data:u8[n]`, `n <= 240` |
| `0x12` OTA end | host to robot | none |
| `0x13` OTA abort | host to robot | none |
| `0x14` OTA status | robot to host | `state:u8, next_offset:u32, error:u8, running_md5:u8[16]` |

The binary layouts live in one header, `firmware/shared/espnow_protocol/src/espnow_protocol.h`,
which the dongle, the robot, and the Jetson app all compile. `channel_a` and `channel_b` are the
two values the Jetson's drive processor would write to the trainer channels. The firmware puts
them where ch0 and ch1 land, so the mixing matches the trainer path whichever processor is
configured (`drive_processor`, `DifferentialDriveProcessor` by default).

`0x04` originates on the dongle from `esp_now_register_send_cb`, not from the robot. It is the only
link-quality signal available, since ESP-NOW has no equivalent to Crossfire's LQ and RSSI.

`0x05` is the dongle's keepalive, sent whenever its radio has been idle 10 ms. Without it the
dongle is silent unless the Jetson is commanding, and the robot then cannot sweep onto its
channel, has nowhere to send telemetry, and cannot validate a freshly flashed image.

Echoing `jetson_tx_us` in telemetry gives round-trip time. Each sample also holds the robot's
wait for its next telemetry slot (100 Hz, below) and the age of a repeated command, so the
per-second minimum is the transport estimate. One-way is approximately half of that.

### Over-the-air frame (dongle to robot)

The dongle prepends a fixed 4-byte session magic to every ESP-NOW frame and drops any received
frame without it. Two teams running ESP-NOW example code on channel 1 is a realistic accidental
cross-talk failure, and 4 bytes rules it out. The Jetson does not need to know about this field.

Peer setup, with the gotchas that cause silent failures:

```c
esp_now_peer_info_t peer = {};
memcpy(peer.peer_addr, PEER_MAC, 6);
peer.ifidx   = ESPNOW_IF;        // must name an interface that is up, see below
peer.channel = resolved_channel; // the sweep result, see channel selection
peer.encrypt = false;
esp_now_add_peer(&peer);
```

`ESPNOW_IF` is `WIFI_IF_AP` on the robot while the SoftAP still exists (Phase 1); `WIFI_IF_STA`
fails there with `ESP_ERR_ESPNOW_IF`. Phase 2 deletes the SoftAP and puts the robot in
`WIFI_MODE_STA`, and from then on it is `WIFI_IF_STA`. The dongle is always STA.

Unicast gets MAC-layer ACK and retries. Broadcast gets neither. The robot knows its dongles from
the `DONGLE_MACS[]` allowlist and replies to whichever one last sent a valid frame. The dongle
broadcasts until the robot's first telemetry or OTA status names it, then unicasts to that MAC.

Set the PHY rate explicitly. The 1 Mbps default costs ~1.2 ms per packet against 6 Mbps OFDM.
The pinned `espressif32@6.13.0` is IDF 4.4, which has no per-peer `esp_now_set_peer_rate_config()`
(IDF 5.x). Its rate setting is per interface, which is equivalent here since each end has one peer
interface:

```c
esp_wifi_config_espnow_rate(ESPNOW_IF, WIFI_PHY_RATE_6M);
```

Call it after `esp_wifi_start()` and again after every channel or mode change.

Do not enable `WIFI_PROTOCOL_LR` alongside this. The two are mutually exclusive and LR costs about
half the latency saving. See the standing decision in Goal.

### The dongle repeats the last command

Two distinct behaviors, and conflating them adds latency:

1. **Forward on arrival.** The moment a frame arrives from the Jetson, send it. Never wait for the
   next repeat slot, which would add up to one repeat period of pure delay.
2. **Repeat on timer at 250 Hz.** Independently, re-send the most recent command every 4 ms so a
   lost packet is covered quickly.

Without the repeat, a 30 Hz command stream means one lost packet leaves a 33.3 ms gap and two
consecutive losses trip `COMMAND_STALE_MS`. 250 Hz is chosen over 150 Hz to shorten post-loss
recovery; at ~80 bytes and 6 Mbps that is roughly 6% channel duty, which is affordable.

Repeats are byte-identical copies of the last command, `seq` included. The dongle never assigns,
increments, or rewrites `seq`, and never invents or extrapolates a command. The robot tells a repeat
from a new command by `seq` alone (rules under `espnow_link` below).

Two more repeat rules, both needed once OTA shares the link:

- **Repeat `0x01` commands only.** Every other type is forwarded once on arrival. Retrying OTA
  chunks is the host's job, and a repeated chunk would only be dropped by the robot as out of order.
- **Stop repeating after 100 ms of serial silence.** When the app exits, the dongle would otherwise
  re-send its last command indefinitely, holding the channel during a flash and handing the robot a
  command no live process stands behind.

## OTA over ESP-NOW

Firmware updates ride the same dongle and the same link as commands. `Update.h` does not care where
the bytes come from, and the bootloader verifies the image the same way it does for `ArduinoOTA`
today. It does not roll back (see Rollback). Moving OTA here is what lets Phase 2 delete the SoftAP, `ArduinoOTA`,
and the dashboard together, so the robot radio never has an access point to tear down and the
combat/tuning radio mode switch never has to be built.

### Flow

```
dev box: pio run -e local -> scp firmware.bin -> ssh: espnow_ota.py firmware.bin
ZED Box: stop auto-battlebot -> open /dev/espnow_dongle -> transfer -> wait for reboot
         -> confirm running_md5 matches -> start auto-battlebot
```

The app owns `/dev/espnow_dongle` while it runs, so the flasher stops the `auto-battlebot` service
for the duration and restarts it afterward, including on failure. The robot is disarmed for any
flash (below), so losing the app and the trainer link for ~20 s costs nothing. The same flasher
runs on the dev box against a bench dongle with `--port`, for flashing without the ZED Box.

### Transfer protocol

Go-back-N over the `0x10`-`0x14` messages:

1. Host sends `0x10` begin with the image size and MD5. The robot refuses unless it is disarmed,
   calls `Update.begin(size)` and `Update.setMD5()`, and answers `0x14` with `state = RECEIVING`,
   `next_offset = 0`.
2. Host streams `0x11` chunks of 240 bytes, keeping at most 16 chunks (3,840 bytes) past the last
   acknowledged `next_offset`. 240 data bytes plus the 4-byte offset, 1-byte type, and 4-byte
   session magic is 249, inside the 250-byte ESP-NOW limit.
3. The robot accepts only the chunk at exactly `next_offset` and drops everything else. It sends
   `0x14` after every 8 accepted chunks, and when it drops one (at most every 20 ms, since a lost
   chunk drops the rest of the window), so a loss costs one round trip and not a timeout.
4. Host resends from the robot's `next_offset` if 500 ms pass without progress. That sits above a
   typical 64 KB block erase (below), which stalls acknowledgements while it runs. It gives up
   after 10 consecutive timeouts and sends `0x13` abort.
5. At `next_offset == size` the host sends `0x12` end. The robot calls `Update.end(true)`, which
   checks the MD5 and the ESP-IDF image header and SHA-256, sets the boot slot, answers `0x14`
   with `state = DONE`, and reboots 200 ms later.
6. After reboot, the robot sends `0x14` with `state = VALIDATED` and `running_md5 =
   ESP.getSketchMD5()` once it has run 10 s and heard the dongle. The host waits up
   to 30 s for it and fails loudly on a mismatch or silence.

The dongle forwards these frames like any other. Its own send path waits on
`esp_now_register_send_cb` before the next `esp_now_send`, so a 16-chunk burst paces itself at air
rate rather than overflowing the ESP-NOW queue.

Expected time for the 899,808-byte image is 3,750 chunks. At 6 Mbps a 249-byte frame plus preamble
and MAC ACK is roughly 0.6 ms, so about 2.3 s of air time. `Update::_writeBuffer()` erases a whole
64 KB block when it reaches an aligned one (`Updater.cpp:213`), so the image costs 14 block erases,
roughly 2 s at a typical 150 ms each, plus about 2.5 s of page programming. These are datasheet
typicals that partly overlap with the air time. Budget 10 s and measure it in Phase 1.

### Robot side

- **The recv callback only copies.** It runs in the WiFi task on core 0, the same rule as commands.
  OTA frames go into a ring buffer sized for the 16-chunk window plus slack (8 KB). `loop()` on
  core 1 drains it into `Update.write()`. A block erase blocks for ~150 ms, and it must never
  block the WiFi task.
- **Disarmed only.** `0x10` is refused with `error = ARMED` while `radio_data->armed` is set. With
  the Crossfire link down `armed` is already false, so bench flashing without the handset works.
- **Arming aborts.** If `armed` goes true mid-transfer, the robot calls `Update.abort()` and goes
  back to normal control. A flash never holds the pilot off the robot. Crossfire keeps authority
  here as everywhere else.
- **Silence aborts.** 5 s without a valid OTA frame aborts the transfer. The running image is
  untouched until `Update.end()` switches the boot slot, so every abort leaves the old firmware
  running.
- **ESCs stop for the duration.** The same `stop_escs()` and early return in `loop()` that
  `ArduinoOTA`'s `onStart` uses today (`main.cpp:126`), entered on an accepted `0x10`.
- **Accept OTA only from known dongles.** The robot keeps a `DONGLE_MACS[]` allowlist (the ZED Box
  dongle and a bench dongle), drops frames from other MACs, and replies to the MAC that sent the
  `0x10`.

### Rollback

None. The plan assumed the Arduino core's prebuilt bootloader had
`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=1`. It does not: on 2026-10-13 a new image's `otadata` entry
stayed in state 0 (new) across several boots, where a rollback bootloader moves it to pending
verify, then aborted. The `verifyRollbackLater()` override did nothing and is deleted. Building and
flashing a rollback bootloader was rejected: the bootloader cannot be updated over OTA, and a bad
one costs a USB rescue on every board.

USB is the recovery path instead, and needs no button. The firmware builds with
`ARDUINO_USB_MODE=1`, so `Serial` is the chip's USB-Serial/JTAG port. That port is hardware: it
stays up with the firmware hung, and esptool's `usb_reset` puts the chip in its ROM bootloader
through it (`scripts/upload`). The robot's BOOT and RESET buttons are glued, so this matters.

The flasher still waits for proof the new image runs: every boot sends `OTA_VALIDATED` with its MD5
once it has run 10 s and heard a dongle, and the flasher checks the MD5.

### Host side

New `firmware/mr_stabs_mk2/scripts/espnow_ota.py`, next to the existing helpers. It has one caller,
so it stays a standalone script and implements the serial framing itself with `pyserial`. `--port`
defaults to `/dev/espnow_dongle`, and `--manage-service` does the service stop and start.

`firmware/mr_stabs_mk2/scripts/ota` becomes: build with `pio run -e local`, `scp` the image to the
ZED Box, and run `espnow_ota.py --manage-service` there over `ssh`, using the `JETSON_HOST`,
`JETSON_USER`, and `JETSON_KEY` conventions from `scripts/deploy_to_jetson.sh`. Phase 2 deletes
`[env:ota]` from `platformio.ini`.

## Firmware changes

### New: `firmware/mr_stabs_mk2/include/espnow_link.h` and `src/espnow_link.cpp`

`EspNowLink` is the hardware glue. The rules that can be tested off the board live in
`firmware/shared/espnow_protocol/src/command_tracker.{h,cpp}`: `CommandTracker` applies the
sequence rules and the freshness streak, and `OtaSequencer` is the receive side of the go-back-N
transfer. Their constants are `COMMAND_STALE_MS = 60`, `FRESH_STREAK_REQUIRED = 3`, and
`REACQUIRE_SILENCE_MS = 500`.

`EspNowLink::update()` runs once per `loop()`, after `crsf->update()` so `armed` is current. It
drains received frames, runs the OTA, sweep, and boot validation state machines, and aborts a transfer
on arming or 5 s of silence. `send_telemetry()` sends at most every 10 ms (100 Hz). The control loop
runs at ~1 kHz, and telemetry at that rate would hold the channel ~35% of the time at 6 Mbps.

Rules the implementation must follow:

- **The recv callback runs in WiFi task context on core 0.** It filters on the session magic and
  the `DONGLE_MACS[]` allowlist, copies into a FreeRTOS queue (40 deep, enough for the 16-chunk
  OTA window during a block erase), and returns. Never touch ESCs or flash there.
- **Sequence numbers: accept equal, drop older.** Compare each frame's `seq` to the last accepted
  one:
  - `seq > last`: a new command. Replace the values and refresh the arrival time.
  - `seq == last`: a dongle repeat. Refresh the arrival time only. The values are identical, so
    nothing else changes.
  - `seq < last`: a late frame from before the current command. Drop it.

  Requiring `seq` strictly greater, as an earlier draft did, rejects every repeat and makes the
  250 Hz repeat useless for freshness. Accepting equal is safe because the dongle only repeats a
  command the Jetson sent within the last 100 ms (the serial-silence rule above). A held command is
  therefore never older than 100 ms plus `COMMAND_STALE_MS`.
- **Re-baseline after silence.** When the app restarts, `seq` starts again from 0 and every frame
  would fall under `seq < last`. After `REACQUIRE_SILENCE_MS` (500 ms) with no accepted frame, the
  next frame's `seq` becomes the new baseline whatever its value. The app takes seconds to restart
  and the dongle stops repeating 100 ms after the old process goes quiet, so no frame from the old
  run can arrive after the reset.
- **Asymmetric hysteresis.** Leave the ESP-NOW source immediately on staleness. Require
  `FRESH_STREAK_REQUIRED` consecutive accepted frames, new or repeated, to re-enter it. Symmetric
  thresholds flap at the edge of range and the robot oscillates between sources.

### New: linear slew limiter on the arbitrated output

The rate comes straight from the existing Jetson config defaults
(`include/transmitter/config.hpp:62,68`): `max_motor_rpm_per_sec / max_motor_rpm`, so
`3247.0 / 1500.0 = 2.165` per second in normalized units, or **216.5 percent per second**.

```c
// Slew-limit the arbitrated command so a source switch, an ESP-NOW dropout, or an orientation
// flip cannot step the wheels past the slip threshold. Applying the same rate the Jetson already
// applies is idempotent: when the trainer path is driving, this is a no-op.
const float MAX_A_PERCENT_PER_SEC = 216.5f;

float limit_a_percent(float requested, float dt) {
    static float prev = 0.0f;
    const float max_delta = MAX_A_PERCENT_PER_SEC * dt;
    prev = constrain(requested, prev - max_delta, prev + max_delta);
    return prev;
}
```

Reset `prev` to zero on every `stop_escs()` path so re-arming always ramps from standstill.

### Edit: `firmware/mr_stabs_mk2/src/main.cpp`

The substitution goes after the armed gate. `ESPNOW_COMMAND_SUBSTITUTION` is `false` for Phase 1
(shadow mode) and Phase 3 sets it `true`; it gates both the substitution and the slew limiter:

```c
// ESP-NOW command substitution. Runs only after every CRSF gate has passed, so ESP-NOW can
// never arm the robot or extend its armed window. SA (ch5) down is the pilot's "accept
// autonomous commands" position; SA up forces the stick values on ch0/ch1.
bool espnow_active = false;
if (ESPNOW_COMMAND_SUBSTITUTION && radio_data->autonomy_enabled && espnow.fresh(now_ms)) {
    const espnow_protocol::CommandPayload &cmd = espnow.latest();
    // Same place the trainer path lands: the Jetson's two channel values go into ch0/ch1 at
    // full scale, and crsf_bridge negates ch0 into a_percent.
    radio_data->a_percent = -100.0f * cmd.channel_a;
    radio_data->b_percent = 100.0f * cmd.channel_b;
    espnow_active = true;
}
```

The `-100` and `100` scaling assumes the handset mixes the trainer inputs into ch0/ch1 at 100%.
Phase 1 measurement 5 (path agreement) is what confirms it.

An ESP-NOW OTA transfer owns the loop the same way `ArduinoOTA` does: right after
`espnow.update()`, `if (espnow.ota_in_progress()) { stop_escs(); return; }`.

Then apply the limiter after the inversion at line 377, before `mix_motor_outputs` at line 387.
Gate it on `autonomy_enabled` so the pilot keeps unlimited stick authority in manual mode,
matching how the Jetson-side limiter gates on the same channel (`trainer_enable_channel`) today:

```c
     if (is_upside_down) radio_data->a_percent *= -1;
+    if (ESPNOW_COMMAND_SUBSTITUTION && radio_data->autonomy_enabled) {
+        radio_data->a_percent = limit_a_percent(radio_data->a_percent, loop_us / 1000000.0f);
+    }
```

Radio setup happens once, at boot. There is no combat/tuning radio mode switch, because once OTA
rides ESP-NOW nothing needs an access point:

```
Phase 1:   WiFi.softAP(ssid, pass, ESPNOW_CHANNEL_OVERRIDE) -> setup_ota() -> diag_server.begin()
           -> espnow.begin(ESPNOW_CHANNEL_OVERRIDE, ...) -> esp_wifi_config_espnow_rate(6 Mbps OFDM)

Phase 2+:  WiFi.mode(WIFI_STA) -> sweep_for_dongle()   // sets the channel, see channel selection
           -> espnow.begin(resolved_channel, ...) -> esp_wifi_config_espnow_rate(6 Mbps OFDM)
```

Phase 1 keeps today's SoftAP, `ArduinoOTA`, and dashboard as the fallback while ESP-NOW OTA proves
itself, and runs on a pinned channel. Sweeping would drag the SoftAP across channels and drop any
laptop on it. Phase 2 deletes all three and turns the sweep on. From then on no AP management
frames or dashboard TX bursts compete with the command path in any mode.

No `WIFI_PROTOCOL_LR`, per the standing decision.

The sweep and re-acquisition still change channels at runtime. Peers need re-registering and the
rate re-applying after every channel change, so treat that as an explicit sequence rather than
assuming the peer survives.

Replace the default channel 1 from `WiFi.softAP()` at `main.cpp:257` with a resolved channel. The
firmware does not choose it. It is whatever the sweep locked onto, or the override:

```c
// 0 selects the sweep. Set a real channel to pin both ends, which is what bench runs should do so
// results are reproducible against a known channel.
const uint8_t ESPNOW_CHANNEL_OVERRIDE = 0;

// Sweep candidates in order. Last-known-good is tried ahead of these at runtime.
const uint8_t CHANNEL_CANDIDATES[] = {1, 6, 11};
const uint32_t CHANNEL_DWELL_MS = 25;   // ~6 dongle repeats at 250 Hz, plus switch settling

// Well above COMMAND_STALE_MS (60). A marginal link should fall back to the trainer path and stay
// there, not drop into a sweep that loses the packets which would have recovered it.
const uint32_t REACQUIRE_SILENCE_MS = 500;
```

From Phase 2 there is no AP, so the sweep is the only re-acquisition path, in the pit and during a
match. It has to work without one.

### Telemetry fields: `TelemetryPayload`, not `diag_data_t`

The link fields travel in the ESP-NOW telemetry message, `TelemetryPayload` in
`espnow_protocol.h`, and the Jetson records it on `/robot/espnow_telemetry`
(`docs/foxglove_recording_format.md`):

```c
uint8_t  autonomy_enabled; // ch5 (SA) as the robot saw it; gates espnow_active
uint8_t  espnow_active;    // ESP-NOW supplied this tick's a/b percent
uint8_t  espnow_channel;   // channel the robot listens on, 0 while sweeping
uint32_t espnow_age_ms;    // since the last accepted ESP-NOW command frame
uint32_t espnow_seq;       // last accepted sequence number
uint32_t jetson_tx_us;     // echoed for round-trip measurement
uint8_t  espnow_lq;        // telemetry send ACK success percent, last 32 frames
uint32_t telemetry_seq;    // +1 per telemetry frame sent, for drop counting
```

`espnow_channel` is what makes a bad auto pick diagnosable after the fact. Without it in the MCAP,
a match with poor LQ has no attributable cause.

`diag_data_t` keeps `wifi_clients` through Phase 1. It feeds the WiFi dashboard, the
`/robot/esp32_diagnostics` recording, its JSON schema, and the Python sysid loaders, and all of
them stay correct while the SoftAP exists. Phase 2 removes it along with the dashboard.

### Delete (Phase 2): `DiagnosticsServer`, the SoftAP, and `ArduinoOTA`

Remove `diagnostics_server.cpp/.h`, and drop `me-no-dev/ESPAsyncWebServer` and `me-no-dev/AsyncTCP`
from `firmware/mr_stabs_mk2/platformio.ini`. AsyncTCP runs its own task and has a long history of
stack-overflow crashes under load. A firmware crash mid-match costs the match.

Also delete `WiFi.softAP()`, `setup_ota()`, `ArduinoOTA.handle()`, `WIFI_SSID`/`WIFI_PASSWORD`,
`scripts/ota_wifi`, and `[env:ota]` in `platformio.ini`; switch `espnow.begin()` to `WIFI_IF_STA`
and set `espnow_protocol::CHANNEL_OVERRIDE = 0` to turn on the scan and the sweep. The
`/robot/esp32_diagnostics` stream ends with the dashboard, so `Esp32DiagnosticsOpenTxTransmitter`,
`Esp32WifiDiagnostics`, `wifi_clients`, and the sysid loaders move to `/robot/espnow_telemetry`.
Then rewrite the OTA and Wi-Fi sections of
`firmware/mr_stabs_mk2/README.md` around `scripts/ota`. Gated on ESP-NOW OTA passing every Phase 1
OTA test, because after this the only other way onto the robot is USB.

## Jetson changes

`OpenTxTransmitter` and its header are untouched. The interface has moved on since the first draft
(`is_connected()` became `get_status()`, and `behavior_mode()` was added), and the composite
delegates both to its primary.

### New: `EspNowTransmitter` (`include/transmitter/espnow_transmitter.hpp`, `src/transmitter/espnow_transmitter.cpp`)

- `send()`: runs the configured drive processor (`drive_processor`, default
  `DifferentialDriveProcessor`), increments `seq`, stamps `jetson_tx_us` from the monotonic clock,
  and writes a `0x01` frame. Nothing goes out while disabled, so the dongle stops repeating 100 ms
  after autonomy turns off. No slew limit: the firmware limits the arbitrated output.
- `update()`: reopens the dongle at most once a second, feeds the shared `SerialFrameParser`, and
  records each `0x02` frame on `/robot/espnow_telemetry`. Health goes to
  `/diagnostics/espnow_transmitter` once a second: dongle open, channel, `robot_known`, ACK percent
  over the period, telemetry rate, minimum round trip, CRC errors.
- Read side: no feedback, no init button, default status. Those come from the OpenTX primary.

### New: `CompositeTransmitter` (`include/transmitter/composite_transmitter.hpp`, `src/transmitter/composite_transmitter.cpp`)

Implements the policy table above, with `get_status()` and `behavior_mode()` from the primary.

### Edit: `include/transmitter/config.hpp`, `src/transmitter/config.cpp`

`EspNowTransmitterConfiguration` carries `dongle_serial_path`, `drive_processor`, and the five
processor fields. `CompositeTransmitterConfiguration` parses `[[transmitter.children]]` through
`parse_transmitter_config`, refuses nesting and an empty list, and refuses an ESP-NOW child whose
`drive_processor` or processor fields differ from the OpenTX child's. That is the construction-time
equality check the duplicated-config risk below asked for: a mismatch would leave the transports
disagreeing by a fixed offset that the firmware slew limiter cannot correct. `make_transmitter`
builds the children recursively and passes them the same clock, viz sink, and recorder.

The wire format comes from the shared header: `CMakeLists.txt` adds
`firmware/shared/espnow_protocol/src` to the include path and compiles `espnow_protocol.cpp` into
`auto_battlebot_lib`.

### Config

The three Mr Stabs ZED Box profiles (`mr_stabs_mk2_aer_zed_box`, `mr_stabs_mk2_fid_aer_zed_box`,
`mr_stabs_mk2_sysid_zed_box`) use the composite. The type differs from `_common`, and the merge
replaces arrays outright, so each file restates every child in full rather than inheriting:

```toml
[transmitter]
type = "CompositeTransmitter"

[[transmitter.children]]
type = "OpenTxTransmitter"     # Esp32DiagnosticsOpenTxTransmitter in the sysid profile
drive_processor = "DifferentialDriveProcessor"
linear_channel = 0
angular_channel = 1
trainer_enable_channel = 5      # radio CH6, SA
# ... the rest of the radio fields

[[transmitter.children]]
type = "EspNowTransmitter"
dongle_serial_path = "/dev/espnow_dongle"
drive_processor = "DifferentialDriveProcessor"
# Must match the OpenTX child; the config loader refuses a mismatch.
lifted_deadzone_percent = 0
zero_deadzone_percent = 2
reverse_angular_channel = true
velocity_saturation_limit = 1.0
```

### udev rule

The handset (VID 0x0483, PID 0x5740) and the dongle both enumerate as `ttyACM`, and enumeration
order is not stable across boots. `find_opentx_device()` already handles the handset by VID/PID.
The dongle gets a stable symlink from `install/install_udev_rules.sh`, which
`scripts/install_zed_box.sh` and `scripts/install_jetson.sh` already run:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", \
    ATTRS{serial}=="D8:3B:DA:8E:ED:2C", \
    SYMLINK+="espnow_dongle", ENV{ID_MM_DEVICE_IGNORE}="1", \
    GROUP="dialout", MODE="0660"
```

The serial number is the dongle's MAC. Add one rule per dongle. Read it with:

```bash
udevadm info -a /dev/ttyACM0 | grep -m1 'ATTRS{serial}'
```

## Phases

### Phase 1: shadow mode, measure before committing

No control change. The firmware receives ESP-NOW commands, logs them, and keeps driving from the
trainer channels. `espnow_active` is recorded but never acted on.

Build only:

- `espnow_link` receive path plus telemetry send, on a pinned channel
- `EspNowTransmitter` and `CompositeTransmitter`, with the firmware substitution disabled
  (`ESPNOW_COMMAND_SUBSTITUTION = false`)
- The link fields in `TelemetryPayload`
- ESP-NOW OTA end to end: robot handler, boot validation report, `espnow_ota.py`, the new `scripts/ota`.
  The SoftAP and `ArduinoOTA` stay up as the fallback (`scripts/ota_wifi`)

All built; see Status.

Measurements to collect, in the arena and not on the bench:

1. **Round trip.** `jetson_tx_us` echoed in telemetry. One-way is approximately half.
2. **Trainer path delay.** Cross-correlate the Jetson's commanded linear against `a_percent` as
   reported in telemetry. The lag at peak correlation is the trainer path delay. The difference
   between this and (1) is the actual latency saving, which either justifies the work or does not.
3. **Loss rate.** Sequence gaps at the firmware plus ACK failures from `0x04` link stats.
4. **Dropout distribution.** Gap durations, not just a mean. The p90 and max matter far more, the
   same way they did for the perception dropout baseline.
5. **Path agreement.** Log `a_percent` from both sources on the same tick. Any steady-state offset
   means the two `DifferentialDriveProcessor` configs have drifted apart.
6. **ACK rate at 6 Mbps.** The standing decision gives up ~10 dB by refusing LR, so this is the
   number that says whether that was affordable in the arena. Record it per match position, not
   pooled: range failures are positional.
7. **Source switch rate.** How often arbitration moves between ESP-NOW and the trainer path, given
   `FRESH_STREAK_REQUIRED = 3`. Confirms the faster re-entry is not causing churn.

The Tradeoffs section predicts ~5.5 ms at 6 Mbps. Measurement 2 either confirms that or points at
the 33.3 ms command interval as the better target.

OTA tests, on the bench. Each must leave the robot running a known image, checked by `running_md5`:

1. **Repeat.** 20 consecutive flashes alternating two builds, all succeed. Record the transfer time
   against the 10 s budget.
2. **Rollback.** Dropped: the bootloader has no rollback (see Rollback). USB rescue replaces it, and
   was tested 2026-10-13.
3. **Bad image.** Flash a truncated or corrupted `firmware.bin`. `Update.end()` must reject it and
   the old image keep running without a reboot.
4. **Dongle pulled mid-transfer.** The robot aborts after 5 s of silence and keeps the old image.
5. **Armed mid-transfer.** Arm on the handset partway through. The transfer aborts, and control
   comes back without a reboot.
6. **Armed at start.** `0x10` with the robot armed is refused with `error = ARMED`.
7. **Service restore.** After a success and after a forced failure, `auto-battlebot` is running
   again on the ZED Box.

### Phase 2: delete the SoftAP, `ArduinoOTA`, and the dashboard

Move telemetry rendering to the Jetson web tool from `004342f`. Delete `DiagnosticsServer`, the
SoftAP, and `ArduinoOTA` per the delete section above, put the robot in `WIFI_MODE_STA`, and turn
the channel sweep on. Record the new `firmware.bin` size and free heap against the 879 KB baseline.
That number sets the headroom available for the camera port.

Re-run OTA tests 1 and 2 on the STA-only build, since `ESPNOW_IF` changes, then test the sweep per
the auto channel selection risk.

### Phase 3: promote to command source

Enable the substitution and the firmware slew limiter. Verify in this order:

1. On the bench, ESP-NOW commanding full forward, release SH (ch4 arm). Motors must stop. This
   single test validates the entire authority split.
2. Flip SA up mid-command. Output must drop to the pilot's sticks and ramp rather than step, and
   `espnow_active` must go false on the same tick.
3. Move SB through all three positions with ESP-NOW commanding. `espnow_active` must not change:
   the flip override is not an autonomy gate.
4. Pull dongle power mid-command. Output must fall back within `COMMAND_STALE_MS`.
5. Confirm the firmware limiter is a no-op when the trainer path is driving, by comparing logged
   `a_percent` against the Jetson's commanded value.
6. Only then, in the box.

Running this after Phase 2 means the command path is measured and promoted without SoftAP
traffic on the channel.

## Risks

**The ~5 ms saving is computed, not measured.** Every checkable radio setting already came back
optimal, so there is no cheaper alternative left to try, but the figure still rests on frame sizes
and 802.11 timing constants. Phase 1 confirms it. If it measures materially lower, the fix is to
attack the 33.3 ms command interval via `control_loop_thread_plan.md` instead, not to abandon the
transport work.

**Dropping LR costs range.** The standing decision takes 6 Mbps OFDM for ~2.7 ms, giving up roughly
10 dB of link budget. In a steel-framed arena with the Jetson outside, that is the one accepted
cost in this plan with real failure potential. Phase 1 measures it as ACK rate, and the mitigation
is antenna and placement work rather than reverting to LR.

**NHRL rules.** Moving primary command onto a custom link is a stricter rules question than
telemetry. Confirm with the event before building Phase 3. This can invalidate the plan, so ask
first. Ask about firmware updates over the link too, though the robot only accepts them disarmed.

**Command repeat rate.** Without the dongle-side 250 Hz repeat, ESP-NOW's dropout tail is worse
than Crossfire's despite its better median. This is a correctness requirement, not an
optimization.

**Faster source switching.** `FRESH_STREAK_REQUIRED = 3` re-enters the fast path 8 ms sooner than 5
would, at the cost of more switching at the edge of range. The firmware slew limiter absorbs the
resulting steps, but Phase 1 measurement 7 should confirm the switch rate stays sane before
Phase 3.

**Duplicated drive processor config.** The two children each hold their own copy of
`drive_processor` and the five processor fields. If they drift, the transports disagree by a fixed
offset, and the firmware slew limiter cannot correct an offset (it limits rate, not value). The
config loader refuses a mismatch at parse time. Phase 1 measurement 5 still catches what the check
cannot: a handset mix that is not 100%, which breaks the firmware's `-100 * channel_a` scaling.

**No frequency hopping.** ESP-NOW stays on one channel for the run. A channel that goes bad
mid-match takes the link out with nothing to hop to, and the sweep will not help because the dongle
is still transmitting there. Mitigation is that both paths carry every command: losing ESP-NOW
costs latency, not control.

**Auto channel selection.** The scan can rank a channel quiet that is loud with traffic it cannot
see, and the sweep is a new failure surface on a path that used to be a flashed constant. Worst
case is a robot that never locks and runs the whole match on the trainer path, which is exactly
today's behavior. Test the sweep with the dongle deliberately parked on each of 1, 6, and 11, and
with the dongle off entirely, and confirm the robot settles into trainer-only rather than sweeping
forever and stalling `loop()`. Full reasoning in the channel selection section.

**Channel changes.** Peers need re-registering and the ESP-NOW rate re-applying after every
channel change, and the sweep changes channels at runtime. Deleting the SoftAP removes the mode
transition that used to be the other path through this code. Test re-acquisition repeatedly by
restarting the dongle on a different channel.

**No Wi-Fi fallback after Phase 2.** A dead dongle, a USB problem on the ZED Box, or an ESP-NOW bug
leaves USB as the only way onto the robot. Keep a flashed bench dongle
in the pit kit, and a USB-C cable that reaches the QT Py without opening more of the robot than a
battery swap does. There is no rollback, so every bad image, including one that crashes or goes
deaf, is a USB rescue (`scripts/upload`, no BOOT press).

**Flashing stops the app.** `espnow_ota.py` stops `auto-battlebot` to take the dongle, so a flash
also drops the trainer link and the recording. The robot is disarmed by then, so this costs only
the ~20 s it takes. A crash in the flasher between stop and start must still restart the service;
OTA test 7 checks it.

**OTA authentication.** The 4-byte session magic and the dongle MAC allowlist stop accidental
cross-talk, not someone deliberately spoofing a dongle MAC. The disarmed gate means a spoofed flash
can only land between matches. If that is not enough, sign the image with an HMAC-SHA256 key kept
out of the repo, verified before `Update.end()`.

**USB3 noise.** The ZED 2i emits broadband noise near 2.4 GHz. Keep the dongle on a short extension
away from the camera cable and ferrite the ZED cable. This looks like arena interference and is
easy to misattribute.

**ESP-NOW v2 is not available.** The 1490-byte payload needs IDF 5.4, and the pinned
`espressif32@6.13.0` is IDF 4.4 (`ESP_NOW_MAX_DATA_LEN 250`). The OTA design assumes 250 bytes. v2
would cut packet count sixfold but not erase and program time, so it would save at most ~2 s per
flash.

## Next steps

Done, 2026-10-13:

- [`control_loop_thread_plan.md`](control_loop_thread_plan.md) landed first, as planned
  (`ThreadedControlLoop` at 250 Hz). Its per-child transmit cap is `transmit_rate_hz` in each
  `[[transmitter.children]]` entry; the Mr Stabs profiles cap the trainer child at 150 Hz.
- Dongle firmware, built and flashed to the development QT Py (MAC `d8:3b:da:8e:ed:2c`, in
  `DONGLE_MACS[]` and the udev rule). The channel is pinned with
  `espnow_protocol::CHANNEL_OVERRIDE = 1`; the boot scan and the robot sweep are written but stay
  off until Phase 2.
- `CompositeTransmitter`, `EspNowTransmitter`, the config check, and the three ZED Box profiles.
- `autonomy_enabled` from ch5 in `CrsfBridge`, reported in `TelemetryPayload`.
- ESP-NOW OTA: robot handler, boot validation report, `scripts/espnow_ota.py`, `scripts/ota`.

Remaining, in order:

1. Raise the model's CRSF baud to 921600 in Model Setup. Roughly 1 ms off the trainer path, which
   is also the ESP-NOW fallback path, for the cost of one menu change. Free latency, no code.
2. Confirm the NHRL rules question. Only gates the command path (Phase 3).
3. On the ZED Box: deploy, run `install/install_udev_rules.sh` (or `scripts/install_zed_box.sh`),
   plug the dongle in through a USB 2.0 hub with the handset, and check `/dev/espnow_dongle`.
4. Done 2026-10-13: the robot is flashed and the dongle's link stats show `robot_known = 1` on the
   bench. Still to check on the ZED Box: `/robot/espnow_telemetry` records at ~100 Hz.
5. Done 2026-10-13: there is no rollback (see Rollback). USB rescue without a button press is
   proven instead.
6. Run the Phase 1 OTA tests and, once the new PCBA is in, the latency, loss, ACK-rate, and
   path-agreement measurements. Get all of them before writing any of Phase 2.
