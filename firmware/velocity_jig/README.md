# Velocity Jig Firmware

Records a robot's linear or angular motion to microSD for velocity
characterization. Runs on the Feather RP2040 Adalogger. Wiring and pin map:
[`velocity_jig_wiring.md`](velocity_jig_wiring.md).

## Build and flash

Uses PlatformIO with the arduino-pico core (Earle Philhower).

```bash
cd firmware/velocity_jig
pio run                 # build
pio run -t upload       # flash over USB
pio device monitor      # serial log at 115200
```

First flash: hold BOOTSEL while plugging in USB so the board enumerates as a
UF2 drive, then `pio run -t upload`. Subsequent uploads reset automatically.

If your PlatformIO install lists a dedicated Adalogger board id, prefer it over
`adafruit_feather` in `platformio.ini`:

```bash
pio boards adafruit | grep -i adalogger
```

The two boards are pin-identical for this firmware, so `adafruit_feather`
(RP2040, 8MB) works either way.

## Usage

| Button | Action |
|---|---|
| A | start recording |
| B | stop recording, return to menu |
| C | live encoder + IMU readout (diagnostics), B or C returns to the menu |

One recording stream logs the encoder and IMU together, one row per IMU sample.
For an **angular-only** run, physically unplug the encoder: its inputs are pulled
up, so the count column just stays constant.

The screen shows the menu, then a static "RECORDING" screen while logging. It is
not redrawn during a recording, so the display adds no load to the capture path.
On stop it shows the last filename, sample count, and any dropped-sample count.

## Diagnostics screen

Press **C** from the menu for a live readout. Nothing is recorded and no file is
opened, so this is safe to leave running while checking wiring:

```
 DIAGNOSTICS
 ENC 1042            AB 10
 G      -1     123      -4
 A    0.02   -0.15    0.99
 xyz dps/g       B/C exit
```

- `ENC` is the free-running quadrature count, `AB` the raw A/B pin levels. A
  disconnected encoder shows `AB 11` (both pulled up) and a frozen count.
- `G` is gyro in dps, `A` is accel in g, one column per axis (x, y, z). At rest
  one accel axis reads about 1.00 and the gyro rows sit near 0.

The screen refreshes at 10 Hz (`DIAG_REFRESH_MS`). B or C returns to the menu;
any USB console command also exits and is answered normally.

## Files

New file each run, `LOG-N.TXT`, counting up from `-0` until an unused name is
found. One row per IMU sample at the configured ODR (default 1.66 kHz):

```
# columns: t_us,count,gx,gy,gz,ax,ay,az  (raw int16 imu)
<microseconds>,<quadrature_count>,<gyro xyz>,<accel xyz>
```

IMU data is written as raw int16 counts to keep the write path light and
lossless. The header line records the scale factors. Convert offline:

```
gyro_dps  = raw * gyro_dps_per_lsb     # 0.070 dps/LSB at +/-2000 dps
accel_g   = raw * accel_g_per_lsb      # 0.000244 g/LSB at +/-8 g
```

`t_us` is `time_us_64()`, microseconds since boot (monotonic, 64-bit). `count` is
the 4x quadrature position, reset to 0 at the start of each file.

## Clock probe (`TIME`)

`t_us` counts from RP2040 boot, so it has no relation to the clock on whatever
host issues drive commands. `TIME` over the USB console recovers that relation:

```
> TIME
TIME 12345678 12345702
```

Both fields are microseconds from the same `time_us_64()` base the log rows use.
The first is when the request line completed, the second is sampled just before
the reply goes out, so the pair brackets jig-side processing (tens of
microseconds).

Host procedure, one probe:

1. Record `t_send` (`CLOCK_MONOTONIC`), write `TIME\n`, read the reply, record
   `t_recv`.
2. Pair the midpoint of `(t_send, t_recv)` with the midpoint of `(rx_us, tx_us)`.
3. Keep only the lowest-decile round trips. USB Full Speed polls at 1 ms frames,
   so most of the spread is host scheduling, and the fastest probes are the ones
   with the most symmetric transport.

Take ~200 probes before a recording and ~200 after, then fit a line through the
surviving pairs for offset and skew. The RP2040 crystal drifts on the order of
30 ppm, which is ~0.9 ms over a 30 s run, so skew is small but worth removing.

Unlike every other console command, `TIME` is answered during recording instead
of `BUSY`. It touches neither the SD card nor the capture path, and a mid-run
probe measures skew across the run window rather than extrapolating into it.

This matters because command-to-motion latency is ~60 ms: a constant clock
offset and the transport delay are indistinguishable to a cross-correlation of
commands against measured acceleration, so the offset has to be measured
separately. See
[`docs/experiments/control/kalman_filter/kalman_filter_plan.md`](../../docs/experiments/control/kalman_filter/kalman_filter_plan.md).

## How gaps are avoided

1. **Capture runs in interrupt context.** The IMU INT1 data-ready line (GPIO12)
   triggers each row, which reads the IMU and latches the current encoder count.
   An SD write in the main loop cannot delay a capture, because the ISR preempts
   it. The encoder count itself is maintained continuously by edge interrupts.
2. **RAM ring buffer** absorbs SD latency spikes. Sized at `RING_SLOTS` (96 KB),
   it covers a ~100 ms stall many times over. A `g_overflow` counter reports any
   loss; it should stay 0.
3. **Preallocated contiguous files** avoid FAT-growth latency mid-run.
4. **Separate SPI buses:** the SD card is on SPI0, the IMU on SPI1, so logging
   and IMU reads never contend.

If `DROPPED` is ever nonzero on the summary screen, lower the ODR
(`IMU_ODR` in `include/config.h`), raise `SD_SCK_MHZ_VAL`, or use a faster card.

## Configuration

All tunables are in [`include/config.h`](include/config.h): pin map, IMU ODR and
ranges, SD clock, preallocation size, and buffer sizes.

## Encoder note

The decoder assumes a quadrature encoder on A0/A1 (GPIO26/27), with internal
pull-ups enabled so an unplugged encoder holds a stable count. For a
single-channel encoder, replace the two `attachInterrupt(...encISR...)` calls
with one edge interrupt on a single pin that increments `g_encCount`.

The IMU is the sampling heartbeat, so it must stay connected. To record angular
motion only, unplug the encoder and ignore the (constant) count column.

## Notes moved from code comments

### General

- `velocity_jig/include/config.h`: Logging sample rate. A hardware timer polls the IMU + encoder at this rate (no dependency on the IMU INT1 wire). The IMU runs faster internally, so each poll returns a fresh, BDU-coherent sample. 1 kHz is plenty for velocity.
- `velocity_jig/include/config.h`: SD SPI clock. This bounds the SD-read side of downloads. 24 MHz is a good balance; drop to 16 if a card is flaky. (USB Full Speed ~1 MB/s is the real download ceiling, not the serial baud, which USB CDC ignores.)
- `velocity_jig/include/config.h`: Contiguous preallocation per file. Preallocating avoids FAT-growth latency spikes mid-run (a key part of gap-free logging). Truncated to actual size on close. Rows are ~60 B at ~1.66 kHz (~100 KB/s), so 32 MB covers ~5 min. Writes past the reservation still work but may stall; raise for longer runs.
- `velocity_jig/include/config.h`: SD write staging buffer. Formatted ASCII accumulates here and flushes in large blocks for efficient sector writes.
- `velocity_jig/include/config.h`: Max records drained to SD per loop pass (keeps the STOP button responsive).
- `velocity_jig/include/config.h`: Diagnostics screen refresh period. A full 128x64 frame is 1 KB over I2C at 400 kHz (~25 ms), so 100 ms keeps the buttons and USB responsive. Idle mode only: the screen is never redrawn while recording.
- `velocity_jig/platformio.ini`: Add -DVJIG_DEBUG=1 for serial bring-up trace (waits for the monitor at boot). Leave it off for normal use so the console TUI has a quiet serial line.
- `velocity_jig/scripts/install_udev_rules`: Install PlatformIO's udev rules so picotool (./scripts/upload) can access the RP2040 over USB without root, and reload udev. Run once: ./scripts/install_udev_rules Needs sudo (it will prompt for your password). After it runs, unplug and replug the board so the new rules apply.
- `velocity_jig/src/main.cpp`: --------------------------------------------------------------------------- IMU raw access (fast burst read, bypasses the library for per-sample reads) ---------------------------------------------------------------------------
- `velocity_jig/src/main.cpp`: --------------------------------------------------------------------------- SD write staging (integer-only formatting -> light CPU load) ---------------------------------------------------------------------------
- `velocity_jig/src/main.cpp`: Live encoder + IMU readout, converted to physical units. Idle mode only, nothing is recorded. Raw counts are scaled here (floats are fine off the capture path) so a wiggle test reads directly in dps and g.
- `velocity_jig/src/main.cpp`: Encoder: count plus the raw A/B levels, so an unplugged or miswired encoder is visible even when the count is not moving.
- `velocity_jig/src/main.cpp`: re-emit so a late-attached monitor sees it
- `velocity_jig/src/main.cpp`: Dedicated SPI: the card owns SPI0, so SdFat keeps CS optimizations.
- `velocity_jig/src/main.cpp`: 921600 is nominal only: this is native USB CDC, so the baud is ignored and transfers run at USB Full Speed regardless. Kept in sync with the host.

### log format, scale factors

- `velocity_jig/include/config.h`: LSB scale factors for the ranges above (documented in the file header). String forms are used in the file header to avoid a float-printf dependency. These are the nominal datasheet sensitivities, so full int16 swing covers more than the range name: 0.000488 * 32768 = 15.99 g, 0.140 * 32768 = 4587 dps. read_jig_log derives the header's range that way, which is why a +/-2000 dps log reports 2293.76. 0.488 mg/LSB at +/-16g 140 mdps/LSB at +/-4000dps

### splash animation

- `velocity_jig/scripts/make_splash.py`: Bake the BWBots splash GIF into a 1-bit OLED animation header. Uses the pre-cropped 128x64 GIF as-is (no crop, scale, or filtering). The source has 3 gray levels: 0 maps to black; 204 (the gray outline) and 255 map to white. Writes include/splash_anim.h. Add --preview <png> for a montage. Regenerate with: scripts/make_splash.py

### flashing

- `velocity_jig/scripts/upload`: Button-free flash. Reboots the jig into the UF2 bootloader over USB (1200-baud touch), then copies the firmware image onto the RPI-RP2 drive. No need to reach the physical BOOTSEL button buried under the OLED. Usage: upload [SERIAL_PORT] SERIAL_PORT defaults to the first of /dev/ttyACM0..2 that exists.

### design

- `velocity_jig/src/main.cpp`: auto-battlebot velocity jig firmware Feather RP2040 Adalogger. Logs encoder count and IMU (accel + gyro) together to the microSD card, one row per IMU sample. For an angular-only run, just unplug the encoder: its inputs are pulled up, so the count column stays constant instead of drifting. OLED menu: A = start recording, B = stop, C = live diagnostics readout. Gap-free design: - Every sample is captured in the IMU INT1 data-ready ISR, so SD write latency never delays a capture. Captures land in a RAM ring buffer. - The main loop drains the ring to SD in large blocks. A long SD stall just lets the ring fill; the ISR keeps sampling. An overflow counter flags any loss (it should stay 0). - Files are preallocated contiguous to avoid FAT-growth latency, and the card is on its own SPI peripheral (SPI0), separate from the IMU (SPI1). - The screen is drawn once at record start and left untouched until stop. Wiring and pin map: velocity_jig_wiring.md and include/config.h
- `velocity_jig/src/main.cpp`: SPSC ring buffer. Producer = IMU ISR, consumer = loop(). Single core, so the ISR atomically preempts the loop; volatile 32-bit indices are sufficient.

### console protocol

- `velocity_jig/src/main.cpp`: --------------------------------------------------------------------------- Diagnostics mode (idle only) Refreshes the live readout until B or C is pressed. Serial input also breaks out, without being consumed, so the next loop() pass answers the command normally instead of leaving the host console hanging. ---------------------------------------------------------------------------
- `velocity_jig/src/main.cpp`: --------------------------------------------------------------------------- USB serial console (idle only): LIST / GET / DEL / STREAM / TIME A host-side TUI (scripts/console) drives this. It runs only from the idle branch of loop(), so it never touches the SD card or the capture path while recording. During recording, commands are answered with "BUSY", except TIME, which is answered normally because it touches neither. ---------------------------------------------------------------------------

### clock sync

- `velocity_jig/src/main.cpp`: Capture clock at the instant the terminating newline was consumed. TIME reports this rather than sampling inside the handler, so dispatch overhead does not land inside the host's round-trip measurement.
- `velocity_jig/src/main.cpp`: Clock probe for aligning the jig's log timestamps with a host clock. Replies "TIME <rx_us> <tx_us>", both from the same time_us_64() base the log rows use. rx_us is when the request line completed, tx_us is sampled just before the reply is pushed out, so the pair brackets the jig-side processing span. The host times the round trip, keeps the lowest-RTT probes, and pairs the midpoint of (t_send, t_recv) with the midpoint of (rx_us, tx_us). Probing before and after a recording gives offset and skew; the RP2040 crystal drifts ~30 ppm, which is ~0.9 ms over a 30 s run. Runs during recording too. It touches neither the SD card nor the capture path, and a mid-run probe measures skew over the run window instead of extrapolating into it.

### bring-up

- `velocity_jig/src/main.cpp`: OLED on I2C1 (GP2/GP3). First a non-blocking bus-idle check: with internal pull-ups both lines should read high when idle. A line stuck low means it is held down (short, bad connection, or a device holding the bus), which is exactly what hangs the first I2C transaction. Report instead of hanging.
