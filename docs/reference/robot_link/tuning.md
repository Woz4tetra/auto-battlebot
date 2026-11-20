# Tuning settings

`firmware/mr_stabs_mk2/lib/tuning` holds the runtime-tunable settings. `main.cpp` copies the deadzones onto the ESC stop thresholds, and `espnow_link.cpp` sets them from tunable frames.

## `Settings`

| Field | Meaning |
| --- | --- |
| `left_deadzone` | Percent of command below which the left ESC gets DShot stop. Default 1.0. |
| `right_deadzone` | Same, right ESC. Default 1.0. |

## Workarounds

- `tuning_store` loads saved values over the defaults. A saved value outside its field's range keeps the default.
