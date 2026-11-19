# Control loop

`ControlLoop` runs one filter, target, navigation, transmit pass per cycle. `SteppedControlLoop` and `ThreadedControlLoop` drive it. The cycle order is in `plant_backed_control.md`.

## Drivers

- `SteppedControlLoop` runs cycles synchronously from the perception tick against the logical clock. It has no thread, so the same recording produces the same cycle sequence regardless of wall-clock speed or machine load.
- `SteppedControlLoop` with `rate_hz = 0` runs exactly one cycle per `advance_to()` call: one filter, target, navigation, transmit pass per perception frame. `rate_hz = 30` is not equivalent: `floor(dt / period)` runs zero cycles on a frame that arrives slightly early, so it skips commands under frame jitter.
- `ThreadedControlLoop` runs cycles on its own thread, paced against the wall clock at `rate_hz`. The command tracks the filter's evolving estimate between perception frames instead of freezing.
- The control thread owns the transmitter and reads input itself. `ThreadedControlLoop::pump_input` is a no-op, and the Runner reaches the transmitter only through `ControlLoop`'s request setters.

## ControlOutput

- `command_feedback`: what the transmitter read back this cycle, in normalized stick units. On OpenTX it is the decoded position of the drive channels, so it reflects the mixed stick plus trainer output rather than what navigation asked for.
- `command_feedback.stick_commands` holds `StickCommand`, in normalized [-1, 1] stick space: the space the command was sent in, read back after mixing, deadzones, and saturation. Physical units never appear there. Every consumer converts to m/s and rad/s with the fitted `[plant]` gains at the point of use (`plant_stick_to_body_velocity`, or `JigPlantModel` for the EKF), so drivetrain scaling is defined once and a refit moves the estimator and the controller together. `VelocityCommand` is the physical-unit type.
- `manual_target`: `target` is the dashboard's held point, not the selector's pick.

## TransmitterStatus

The transmitter reports its health each cycle. The control loop samples it once per iteration and passes it along whole, so a new signal from the transmitter means one more field on `TransmitterStatus` rather than another accessor threaded through the control loop, the runner, and the UI.

- `connected` is transport-level: the serial port or socket is open.
- `receiving_channels` is the stream on top of it. The two differ on OpenTX, where the port stays open while the radio sends nothing. Trainer mode failing to engage looks exactly like that.
- A transmitter that cannot tell the two apart reports the same value for both.

## Target override

`set_target_override` takes a field point that navigation steers to in place of the target selector's pick. `nullopt` hands targeting back to the selector.

## Watchdog stamp

`last_cycle_wall_time` is the wall-clock stamp of the last cycle's start. `ThreadedControlLoop::is_healthy` compares it against `watchdog_timeout_ms`.
