# ESC drive

How `firmware/mr_stabs_mk2` maps commands to ESC duty, decodes ESC replies, and re-enters ESC tuning mode.

## Command to duty

- A command inside `stop_threshold` runs the ESC at duty 0. Any other command runs `esc_map`'s floor-offset duty, so a 1% command runs about 7%.
- The ESCs run every step from 1 to 65 at the same speed, the speed of step 65. They respond linearly from step 66 up.
- Commands start at step 66, so the smallest command past `stop_threshold` already moves the motor.

## eRPM reply clock

- The ESC's reply clock is its own. BLHeli_32 on Mr Stabs sends bits at 95 ticks against the nominal 107, which leaves a 3-bit run at 2.66 bits, close to rounding down.
- `erpm_decode` measures the bit from the reply's single-bit runs (within 35% of nominal) and decodes with that.

## Tuning mode reentry

- Entry waits until COMBAT has lasted `REENTRY_MS` (`esc_config.h`).
- Exiting restarts the ESCs. Entering again right away left them out of the bootloader.

## Config request scheduling

A pending ESC config request is checked after the frame, so a write always finishes before the pins change hands. `EscConfig::update()` runs from `loop()` every pass and at most one queued frame, which blocks for up to about 3 s (a write). Armed and driving refuse entry, and a radio button press exits. The HTTP side is safe to call from the network task.
