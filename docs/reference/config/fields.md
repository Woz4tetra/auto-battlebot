# Config fields

What each config field means, per section. Values and their source per profile come from
`--print-config` (`docs/reference/config/profiles.md`); this page holds what a value cannot say.

## `[transmitter]`

### Drive processor fields

`OpenTxTransmitter`, `EspNowTransmitter` and `PlaybackTransmitter` run the command through a drive
processor; `SimTransmitter` applies the same saturation itself.

| Field | Meaning |
| --- | --- |
| `drive_processor` | `TankDriveProcessor` sends left and right motor commands; `DifferentialDriveProcessor` sends linear and angular for a radio-side mixer. On ESP-NOW the firmware puts the two values where ch0/ch1 land, so it must match how the handset model mixes |
| `velocity_saturation_limit` | Combined budget `\|linear\| + \|angular\| <= limit`. Angular takes priority and linear fills the headroom. 0 clamps each channel to [-1, 1] on its own. The sim has no drive processor, so without it the simulated robot keeps forward authority in a turn the real one has lost in the mixer |
| `zero_deadzone_percent` | Input magnitude below which the output is forced to zero |
| `lifted_deadzone_percent` | Smallest non-zero output magnitude once the zero deadzone is exceeded |
| `max_motor_rpm`, `max_motor_rpm_per_s` | Linear slew limit: the normalized linear command changes by at most `max_motor_rpm_per_s / max_motor_rpm` per second. Unset means no limit |

Under `CompositeTransmitter`, an ESP-NOW child's drive processor fields (`drive_processor`,
`velocity_saturation_limit`, both deadzones, both `reverse_*_channel`) must equal the OpenTX
child's, or the two transports disagree by a fixed offset. The parser rejects a mismatch.

### `OpenTxTransmitter`

| Field | Meaning |
| --- | --- |
| `linear_channel` | RC channel for the first mixer output: left wheel under tank drive, linear under differential |
| `angular_channel` | RC channel for the second mixer output: right wheel under tank drive, angular under differential |
| `trainer_enable_channel` | RC channel of the trainer enable switch (SG). Autonomous motion reaches the wheels only while it is engaged, when the channel reads negative, and the linear slew limiter holds at standstill until then. SG must be mapped to this output (radio CH6) in the OpenTX model |
| `init_button_channel`, `init_button_threshold` | Channel whose value above the threshold counts as a field re-init press. Unset means no button |
| `invert_angular_channel_readback` | The radio model inverts the angular output after mixing in the trainer input (the Mrs Buff CRSFR model does this to its right motor, `firmware/config/CHANNEL_CONVENTION.md`). Sending is unaffected, since the trainer value goes through the same inversion as the sticks, but the read-back channels are mixer outputs. Left uninverted, the wheels read back with opposite signs and forward stick reports as a spin |

### `Esp32DiagnosticsOpenTxTransmitter`

`OpenTxTransmitter` plus the Mr Stabs Mk2 firmware's telemetry ring, polled over WiFi and recorded
on `/robot/esp32_diagnostics`.

| Field | Meaning |
| --- | --- |
| `esp32_host`, `esp32_port` | The robot's address, an IP literal. 192.168.4.1 is its access point; in client mode use the address the router reserves |
| `reconnect_period_s` | Wait after a failed poll. The ring holds two minutes of records, so a poll back within that loses nothing |

### `EspNowTransmitter`

Autonomous drive commands to Mr Stabs Mk2 over the ESP-NOW USB dongle, plus its telemetry back. It
carries no arm authority (`firmware/mr_stabs_mk2/README.md`).

| Field | Meaning |
| --- | --- |
| `dongle_serial_path` | A udev symlink (`install/install_udev_rules.sh`), not a raw ttyACM path: the handset also enumerates as ttyACM and the order is not stable |

### `CompositeTransmitter`

Fans one command stream out to several transmitters. The first child is primary and owns everything
read-side. `[[transmitter.children]]` is an array of tables so the order is guaranteed. Each entry
may set `transmit_rate_hz`, the most sends per second that child gets; 0 passes every send.

### `SimTransmitter`

| Field | Meaning |
| --- | --- |
| `command_delay_ms` | Artificial command delay for lag experiments. Wall-clock based, so it does not survive accelerated runs |
