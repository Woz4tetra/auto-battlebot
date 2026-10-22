# Mrs Buff Mk4 control electronics: ELRS and RP2350

## Goal

Mrs Buff Mk4 is built in two stages:

1. **Manual only.** An off-the-shelf ExpressLRS receiver drives the ESCs directly. There is no
   MCU and no autonomy. This gets a driveable robot without waiting on a board.
2. **Control PCBA.** A custom RP2350 board takes the same receiver over CRSF through a port and
   drives the ESCs over DShot. Autonomy reaches the robot the way it does on Mrs Buff Mk3: the
   Jetson writes trainer inputs to the pilot's handset over USB, and the handset mixes them into the
   one ELRS link.

The robot has one receiver in both stages. A second, built-in receiver for a dedicated Jetson link
is the fallback, and the board reserves room for it without populating it.

Stage 1 hardware carries into stage 2 without rework: the same receiver, the same ESCs, and the
same harness, with the receiver moving from the ESCs to the PCBA port.

## Why RP2350, not ESP32

Mr Stabs Mk2 uses an ESP32-S3 mainly for WiFi: the SoftAP dashboard, live tunables, and OTA. WiFi is
optional for Mk4, and USB upload is fine, which removes the ESP32's main advantage.

- **PIO DShot.** Bidirectional DShot (ESC RPM read back on the signal wire) is easy with PIO and
  awkward on the ESP32's RMT. RPM feedback helps the plant fit and closed-loop speed control.
- **No 2.4 GHz radio on the board except ELRS.** The ESP32's SoftAP would transmit centimeters from
  the receiver. Without it, there is no on-board desense to manage.
- **Deterministic timing.** No WiFi interrupts on either core.
- **UARTs.** 2 hardware UARTs, plus PIO UARTs if a third serial device turns up. The receiver uses
  one, and the fallback receiver gets the other.
- **USB upload without a button.** Firmware can reboot into BOOTSEL on a host command (picotool or
  a 1200 bps touch), so a hung board is the only case that needs the BOOTSEL button.

What I give up: the wireless dashboard and OTA. Diagnostics move to USB serial, or to ELRS
telemetry.

## Link design: one receiver, Jetson through the handset

```
Jetson ──USB serial (trainer <ch> <value>)──► pilot handset (EdgeTX, SA gates trainer mix)
                                                   │
                                                   ▼ 2.4 GHz ELRS
                                              ELRS receiver ──► robot (ESCs or PCBA)
```

This is the path `src/transmitter/opentx_transmitter.cpp` drives on Mk3 today
(`[transmitter]` in `config/_common.toml`, channel map in `firmware/config/CHANNEL_CONVENTION.md`).

### Why not two receivers

I considered a second receiver on the robot, bound to a Jetson-driven 900 MHz TX module, with the
robot arbitrating between the two. Compared with it, one receiver:

| Risk with two receivers | With one receiver |
| --- | --- |
| Telemetry bursts from one receiver desense the other, centimeters away | Gone: one receiver, one antenna |
| The manual port gets a receiver bound to the Jetson's TX, handing the Jetson the kill path | Gone on the robot: arm and kill stay on handset switches |
| Two receivers on one 5 V rail, twice the brownout exposure | One receiver's draw, same as stage 1 |
| A 900 MHz antenna (~8 cm dipole) to fit without chassis shadowing | Gone |
| Two failsafe demos at inspection | The standard single-link demo |

Firmware is in the kill path once the PCBA exists, whichever link design is used. That risk is
covered under stage 2.

### Risks this design adds

1. **Stale trainer values.** If the Jetson process dies or the USB cable drops while SA is down, the
   handset may keep mixing in the last trainer value, and the robot drives on a dead command. I
   have not checked whether EdgeTX times out trainer values set over the CLI or holds them. If it
   holds them, the fix lives on the handset: a Lua script or special function that zeroes the
   trainer inputs when the CLI goes quiet.
2. **Latency.** Commands go USB serial, then the EdgeTX mixer cycle, then the ELRS uplink. I have
   no measured number for this path.
3. **Tether.** A USB cable runs from the Jetson to the radio the pilot is holding. A snag drops
   autonomy, and under risk 1 might also hold the last command.
4. **Handset dependency.** The gating lives in the EdgeTX model's SA mixes. A firmware update or a
   model edit can break it without any visible sign. The model stays in the repo under
   `firmware/config/opentx/`, and failsafe gets re-tested after any radio change.
5. **Shared telemetry.** Robot state for the Jetson comes down the pilot link's telemetry slots and
   then over USB. When the pilot link has trouble, the Jetson loses its data at the same time.

### When to switch to two receivers

Move to the fallback design (below) if any of these happen:

- The stale-trainer test fails and has no clean fix on the handset.
- The tether causes a real problem at an event.
- The Jetson needs to come off the pilot's radio for some other reason.

## Stage 1: manual receiver wired to the ESCs

```
Pilot handset (2.4 GHz ELRS) ──► ELRS PWM receiver ──PWM──► drive ESCs, weapon ESC
```

- **Receiver.** Use an ELRS receiver with configurable outputs (RadioMaster ER-series style), where
  each pin can be PWM or serial. Stage 1 uses PWM pins. In stage 2 the same receiver is
  reconfigured to output CRSF on one UART pin pair and plugs into the PCBA.
- **Mixing on the radio.** Tank mixing and the SA trainer mix live in the EdgeTX model, as on
  Mrs Buff Mk3 (`firmware/config/opentx/mrs_buff_mk3.etx`). With trainer mixing in place, stage 1
  can run autonomy too, since nothing on the robot changes.
- **Failsafe.** Set the receiver to "no pulses". Confirm each ESC stops on loss of signal, not holds
  the last throttle. Test this by unplugging the receiver's power with the robot on a stand.
- **ESCs.** Pick ESCs that take both PWM and DShot (AM32, Bluejay, or BLHeli_32 all auto-detect),
  and that support bidirectional DShot. Then stage 2 changes only the signal source.
- **Channel map.** Follow `firmware/config/CHANNEL_CONVENTION.md`. Add a Mrs Buff Mk4 column.
- **Harness.** Leave room in the chassis for the PCBA footprint, and route the ESC signal wires so
  they reach the PCBA location without splicing.

Exit criteria:

- Drives and spins the weapon at a test session.
- The failsafe test passes on every ESC.
- The stale-trainer and latency tests below pass, if stage 1 runs autonomy.

## Stage 2: control PCBA

```
Pilot handset ──2.4 GHz ELRS──► receiver ──CRSF──► [port] ──UART0──► RP2350 ──DShot──► ESCs
                                                   [DNP fallback RX] ──UART1──┘
```

### What the board does

- Decodes CRSF from the receiver and drives the ESCs over PIO DShot with RPM read back.
- Does mixing in firmware, so the radio model can send raw sticks. This needs a separate EdgeTX
  model from stage 1, with the SA trainer mix kept.
- Enforces arm and failsafe: the weapon and drive outputs go to zero when the receiver stream is
  stale (no frame for ~100 ms) or the arm channel is low.
- Sends robot state up as ELRS telemetry: armed state, ESC RPM, battery voltage, reset reason, and
  receiver link drops. The Jetson reads it back through the handset.

Autonomy gating stays on the handset (SA). The board has no autonomy logic in the default design.

### Manual receiver port

- 4-pin JST-GH: 5 V, GND, TX, RX. That is the standard pinout for ELRS receiver serial, so the
  stage 1 receiver plugs straight in after a config change from PWM to CRSF.
- The 5 V rail must supply the receiver's peak draw at full telemetry power, with headroom for the
  fallback receiver. Check the receiver datasheet.
- If the port is empty or the receiver is dead, the robot stays disarmed.

### Fallback receiver footprint (do not populate)

Reserve a footprint for a small 900 MHz ELRS receiver module on UART1, with its antenna connector
location and 5 V feed. Leave it unpopulated. If the design has to switch to two receivers, it
becomes a populate-and-flash change, not a respin.

Constraints if it is ever populated:

- 900 MHz only, so it can never bind to the pilot's 2.4 GHz handset. That makes it physically
  impossible to put the Jetson in the kill path by binding the wrong receiver. Never use a
  dual-band LR1121 receiver in the manual port for the same reason.
- Separate binding phrases and ELRS Model Match on both links.
- Stock ELRS firmware, no fork.
- The board then arbitrates. The manual receiver keeps arm and kill, and the Jetson link only
  supplies drive values while AutoEn is high and the link is fresh. It falls back to the sticks
  when the Jetson link goes stale. Write this as a pure function in `lib/` with native GoogleTest
  coverage, the way `firmware/mr_stabs_mk2` tests run on the host (`[env:native]`).
- It needs a desense test before use: log LQ and RSSI on each receiver with the other at max
  telemetry power, inside a cage or behind polycarbonate.

### Board contents

- RP2350A, or RP2354A if it is stocked at JLCPCB (it has 2 MB of flash stacked in the package, so
  no external flash chip). Otherwise RP2350A with a W25Q flash.
- USB-C for upload and serial diagnostics, BOOTSEL button.
- Manual receiver port, plus the unpopulated fallback receiver footprint.
- DShot outputs for each ESC on PIO-capable pins, with series resistors.
- Battery voltage divider to an ADC pin. Pack current sense if it fits: Mr Stabs Mk2's INA238 is
  what made `docs/plans/motor_stall_protection_plan.md` possible.
- Status LED.
- IMU if Mk4 needs orientation (flip detection, heading hold). Mr Stabs Mk2 uses a BNO055 over I2C.
- 5 V regulator sized for both receivers at peak telemetry draw plus the RP2350.

Lay it out with the pcba-design skill, the same flow as `hardware/mr_stabs_mk2_pcb/`.

### Firmware

- arduino-pico core under PlatformIO (`maxgerhardt/platform-raspberrypi`), so the CRSF parsing,
  `pid`, and `yaw_control` libraries from `firmware/mr_stabs_mk2/lib/` port with little change.
  The Pico SDK directly is the fallback if arduino-pico's PIO or DShot support gets in the way.
- New PIO DShot driver with bidirectional RPM decode. The existing `lib/DShot` is RMT-specific
  and does not port.
- Hardware watchdog on. A hung main loop must stop DShot frames, which makes the ESCs stop.
- Pin the platform version in `platformio.ini`, unlike the unpinned ESP32 env that pioarduino
  shadowed (see the Mr Stabs Mk2 `platformio.ini` comment).
- New directory `firmware/mrs_buff_mk4/`.

### Stage 2 exit criteria

- The failsafe test passes through the board: receiver power pulled, every ESC stops.
- Forced-hang test: stall the main loop with the weapon spinning on a stand, and every ESC stops.
- A brownout during weapon spin-up is visible in telemetry as a reset reason or link drop.

## Tests on the handset path

Mk3 runs this path today, so these tests are useful before Mk4 exists.

1. **Stale trainer values.** Hold SA down with the app sending a nonzero drive command. Kill the app,
   then separately unplug the USB cable. Watch ch0/ch1 on the EdgeTX channel monitor. Pass: both
   return to stick-only values within ~100 ms. Fail: the trainer value holds.
2. **Latency.** Timestamp the Jetson's trainer write, and timestamp the matching change in the
   receiver's CRSF output (logic analyzer or a CRSF sniffer on the receiver UART). Record the
   median and p99.

## Open questions

- Mk4 drivetrain and weapon: number of drive motors, weapon ESC type, and whether the weapon ESC
  takes DShot. These set the PIO pin count.
- Does Mk4 need an IMU? It is only needed for flip handling or heading hold.
- Does EdgeTX hold or time out trainer values set over the CLI? Test 1 answers this.
- Is the RP2354A stocked at JLCPCB?
- For the fallback: do the NHRL rules allow a second, non-safety 900 MHz link, and which receiver
  module fits the footprint?

## Unverified

I wrote these ELRS details from memory and have not checked them against current docs:

- 900 MHz packet rates (I expect 200 Hz max on LoRa)
- Telemetry payload per slot
- Pass-through of extended CRSF frames from the receiver UART
- ER-series per-pin PWM/serial configuration

## Next steps

1. Run the stale-trainer test and the latency test on the Mk3 radio.
2. Answer the drivetrain and weapon questions, then choose ESCs that do PWM plus bidirectional
   DShot.
3. Buy the stage 1 receiver with configurable PWM/serial outputs, and build the Mk4 EdgeTX model
   with tank mixing and the SA trainer mix.
4. Wire stage 1 and run the failsafe test on every ESC.
5. In parallel, bench test a PIO DShot driver with RPM read back on a Pico 2 and one of the chosen
   ESCs before committing the board pinout.
6. Start the PCBA layout, with the fallback receiver footprint reserved.
