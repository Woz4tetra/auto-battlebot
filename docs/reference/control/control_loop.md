# Control loop

`ControlLoop` runs one filter, target, navigation, transmit pass per cycle. `SteppedControlLoop` and `ThreadedControlLoop` drive it. The cycle order is in `plant_backed_control.md`.

## ControlOutput

- `command_feedback`: what the transmitter read back this cycle, in normalized stick units. On OpenTX it is the decoded position of the drive channels, so it reflects the mixed stick plus trainer output rather than what navigation asked for.
- `manual_target`: `target` is the dashboard's held point, not the selector's pick.

## Target override

`set_target_override` takes a field point that navigation steers to in place of the target selector's pick. `nullopt` hands targeting back to the selector.

## Watchdog stamp

`last_cycle_wall_time` is the wall-clock stamp of the last cycle's start. `ThreadedControlLoop::is_healthy` compares it against `watchdog_timeout_ms`.
