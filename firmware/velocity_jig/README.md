# Velocity Jig Firmware

Records a robot's linear or angular motion to microSD for velocity
characterization. Runs on the Feather RP2040 Adalogger. Wiring and pin map:
[`docs/velocity_jig_wiring.md`](../../docs/velocity_jig_wiring.md).

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
[`docs/experiments/kalman_filter/kalman_filter_plan.md`](../../docs/experiments/kalman_filter/kalman_filter_plan.md).

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
