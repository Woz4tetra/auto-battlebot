# CRSF channel convention

One channel mapping shared by every robot, radio model, and the Jetson. The
reference implementation is the "Mrs Buff CRSFR" model in
`firmware/config/opentx/mrs_buff_mk3.etx`.

Channel numbers below are 0-based CRSF indices, matching the firmware structs
(`channels->ch4`) and the Jetson config (`config/_common.toml`). The EdgeTX UI
shows the same channel as CH(n+1).

## Channel map

| CRSF ch | EdgeTX CH | Name   | Source                  | mrs buff mk3                        | mr stabs mk2                                  |
| ------- | --------- | ------ | ----------------------- | ----------------------------------- | --------------------------------------------- |
| 0       | CH1       | Motor A | Ele (+ mixes, see below) | Left drive motor (tank mix on radio) | Linear velocity (firmware negates into `a_percent`) |
| 1       | CH2       | Motor B | Ail (+ mixes, see below) | Right drive motor (tank mix, output inverted) | Angular velocity (`b_percent`)        |
| 2       | CH3       | Weapon | Thr                     | Weapon ESC speed                    | Unused                                        |
| 3       | CH4       | Rud    | Rud                     | Unused                              | Unused                                        |
| 4       | CH5       | Arm    | SH                      | Ignored (weapon arming is the SH override on ch2) | Drive arm: `armed = ch4 > mid`  |
| 5       | CH6       | AutoEn | SA                      | Autonomy enabled (robot side unused; Jetson reads it back) | Same                   |
| 6       | CH7       | AutoGo | SF                      | Autonomy behavior: attack vs run away (Jetson reads it back) | Same                 |
| 7       | CH8       | Flip   | SB                      | Ignored                             | Flip direction override (three-state, see below) |
| 8       | CH9       | free   |                         |                                     |                                               |
| 9       | CH10      | Button | none yet                | Ignored                             | Momentary button, diagnostics only today      |
| 10-15   | CH11-16   | free   |                         |                                     |                                               |

## Transmitter controls (expected mixer)

Sticks:

| Control | Stick                 | Function            | Output |
| ------- | --------------------- | ------------------- | ------ |
| Ele     | right stick vertical  | drive forward/back  | ch0 (and ch1 via tank mix on mrs buff) |
| Ail     | right stick horizontal | rotate             | ch1 (and ch0 via tank mix on mrs buff) |
| Thr     | left stick vertical   | weapon speed        | ch2    |
| Rud     | left stick horizontal | spare passthrough   | ch3    |

Switches:

| Switch | Function              | Detail |
| ------ | --------------------- | ------ |
| SA     | enable autonomy       | SA down: trainer inputs `tr(0)`/`tr(1)` mix into ch0/ch1 and ch5 goes high. SA up: sticks only, ch5 low. |
| SF     | set autonomy behavior | Drives ch6. Selects attack vs run away on the Jetson. |
| SB     | flip direction        | Drives ch7. SB off (default power-on position) means auto: IMU decides orientation and heading-hold auto-steer is enabled. Middle forces upright. Full toward pilot forces upside-down. |
| SH     | arm weapon            | Drives ch4. Also gates the weapon output directly: a special function overrides ch2 to -100 while SH is released. |

Mrs Buff CRSFR mixer, as built:

- ch0 `Left` = Ele 100% + Ail 60% + `tr(0)` 100% when SA down
- ch1 `Right` = Ele 100% - Ail 60% + `tr(1)` 100% when SA down, output inverted
- ch2 `Weapon` = Thr, with special function `SH released -> OVERRIDE_CHANNEL ch2 = -100`
- ch3 `Rud`, ch4 `WeapEn` = SH, ch5 `AutoEn` = SA, ch6 `AutoGo` = SF

Mr Stabs mk2 differences: the radio sends raw Ele on ch0 and Ail on ch1; the
ESP32 firmware (`firmware/mr_stabs_mk2`) does the differential mix, heading
hold, and flip handling from ch7. Firmware note: `get_switch_state()` returns
DOWN for a low CRSF value, which is the switch physically up. The "auto" flip
case in `main.cpp` is the DOWN enum value.

## Jetson side

From `config/_common.toml` (`[transmitter]`):

- `linear_channel = 0`, `angular_channel = 1`: autonomy drive commands, written
  as EdgeTX trainer inputs over the CLI (`trainer <n> <value>`). They only reach
  ch0/ch1 while SA down enables the trainer mixes.
- `trainer_enable_channel = 5`: reads SA back; below 0 means autonomy off.
- `behavior_mode_channel = 6`: reads SF back, attack vs run away.
- `init_button_channel = 31`: disabled; no transmitter maps a button. Reinit is
  a touchscreen action. ch9 stays reserved for this.

## Known gaps

- The "mr stabs mk2" model in `mrs_buff_mk3.etx` predates this convention: arm
  is SF on ch4, trainer forwarding uses special functions on SA, and there is no
  SB flip mix on ch7. It needs a rebuild to match this table before flying the
  new firmware (which reads flip on ch7 and button on ch9).
- No model maps the ch9 button yet.
