# Control loop

`ControlLoop` runs one filter, target, navigation, transmit pass per cycle. `SteppedControlLoop` and `ThreadedControlLoop` drive it. The cycle order is in `plant_backed_control.md`.

## Drivers

- `SteppedControlLoop` runs cycles synchronously from the perception tick against the logical clock. It has no thread, so the same recording produces the same cycle sequence regardless of wall-clock speed or machine load.
- `SteppedControlLoop` with `rate_hz = 0` runs exactly one cycle per `advance_to()` call: one filter, target, navigation, transmit pass per perception frame. `rate_hz = 30` is not equivalent: `floor(dt / period)` runs zero cycles on a frame that arrives slightly early, so it skips commands under frame jitter.
- `ThreadedControlLoop` runs cycles on its own thread, paced against the wall clock at `rate_hz`. The command tracks the filter's evolving estimate between perception frames instead of freezing.
- The control thread owns the transmitter and reads input itself. `ThreadedControlLoop::pump_input` is a no-op, and the Runner reaches the transmitter only through `ControlLoop`'s request setters.

## ControlOutput

- `command_feedback`: what the transmitter read back this cycle, in normalized stick units. On OpenTX it is the decoded position of the drive channels, so it reflects the mixed stick plus trainer output rather than what navigation asked for.
- `command_feedback.stick_commands` holds `StickCommand`, in normalized [-1, 1] stick space: the space the command was sent in, read back after mixing, deadzones, and saturation. Physical units never appear there. The EKF reads them through `JigPlantModel`, which applies the fitted `[plant]` gains. Navigation outputs `VelocityCommand` in m/s and rad/s, and `PlantCommandShaper` turns it into the stick that is sent, with the same `[plant]` table, so a refit moves the estimator and the controller together.
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

## Robot set each cycle

Target selection, hazards and navigation all read the filter's state from the same cycle. A robot is present exactly while the estimator keeps its track, and absent once the track expires (`max_position_sigma_m`). Nothing substitutes an earlier set when a robot is missing; each navigator runs its own no-robot path.

## Control loop surface

The control loop interface forwards the rest of the loop's calls so callers deal with one object and not with the split between the two halves. Each forwarded call is safe from another thread while a driver is cycling.

## Host actions

The UI can ask for a host action (`SystemAction`). Every action stops the runner loop. `main.cpp` runs the host command once the loop and the UI are down.

## Runner signals

- `Runner` is `Quittable` so SIGINT and SIGTERM can stop the run.

## OpenTX transmitter behaviour

- The init button fires once per rising edge on the configured init button channel. The channel value must cross `init_button_threshold` to count as a press.
- `channels_fresh()` is true while complete 32-channel updates keep arriving. It is seeded on port open, so a fresh (re)connect gets one timeout window of grace before it reports a stall. It assumes the port is open, so callers pair it with `serial_.is_open()`.
- `log_switch_states()` logs decoded switch flags with the raw channel value next to each, once on the first channel frame and then on every change. Recordings then show what the transmitter believed even when the driver never touched a switch.
- `limit_linear_acceleration()` slew-limits the normalized linear command so wheel-surface acceleration stays under the configured RPM/s cap. It uses the logical clock for dt and passes the command through unchanged when the limiter is disabled or on the first tick after (re)enabling. The limit applies only while the trainer enable channel (`trainer_enable_channel`) reads negative. Until then the baseline is held at standstill, so motion ramps from zero once the trainer switch engages.
