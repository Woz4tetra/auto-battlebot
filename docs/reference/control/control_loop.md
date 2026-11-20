# Control loop

`ControlLoop` runs one filter, target, navigation, transmit pass per cycle. `SteppedControlLoop` and `ThreadedControlLoop` drive it. The cycle order is in `plant_backed_control.md`.

## Drivers

- `SteppedControlLoop` runs cycles synchronously from the perception tick against the logical clock. It has no thread, so the same recording produces the same cycle sequence regardless of wall-clock speed or machine load.
- `SteppedControlLoop` with `rate_hz = 0` runs exactly one cycle per `advance_to()` call: one filter, target, navigation, transmit pass per perception frame. `rate_hz = 30` is not equivalent: `floor(dt / period)` runs zero cycles on a frame that arrives slightly early, so it skips commands under frame jitter.
- `ThreadedControlLoop` runs cycles on its own thread, paced against the wall clock at `rate_hz`. The command tracks the filter's evolving estimate between perception frames instead of freezing.
- The control thread owns the transmitter and reads input itself. `ThreadedControlLoop::pump_input` is a no-op, and the Runner reaches the transmitter only through `ControlLoop`'s request setters.

## Test strategy

`ControlLoopTest.SteppedZeroRateMatchesInlineSequence` proves the one-cycle-per-frame property of `SteppedControlLoop` with `rate_hz = 0` against an inline reference sequence written independently of the `ControlLoop` body. SVO replay cannot prove it while playback frame selection is nondeterministic.

- The test navigation derives its command from the robots it is handed, so a filter difference also shows in the command stream.
- `ControlLoopTest.ZeroRateRunsExactlyOneCyclePerAdvance` advances to deliberately uneven times (1.5x the frame stamp). At `rate_hz = 0` the cycle count must not depend on them, which is why 0 exists instead of setting `rate_hz` to the camera rate.

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

## Robot descriptions cache

`RobotDescriptionsCache` keeps the most recent navigation-critical robot set (our robot and an opposing robot) and substitutes it when the current frame lacks either side. `Runner` owns it so target selection and navigation read the same resolved set within a tick. Without it, target selection would flap to the previous selection while navigation operated on a stale cache.

- The substitute expires `robot_cache_max_age_s` after its own stamp (`[control_loop]`). Past that the current set goes through as is, so a navigator sees our robot is gone and runs its no-robot path.
- Replaying the frozen snapshot forever kept `RlPolicyNavigation` feeding the policy one unchanging observation, and it held a full-rate spin that kept the robot invisible to the camera.
- `resolve()` returns a `Resolved` whose `robots` reference points at either the `current` argument or the cache's snapshot. It is valid until the next `resolve()` or `reset()`, and for the `current` branch only as long as the caller's argument.

## Control loop surface

The control loop interface forwards the rest of the loop's calls so callers deal with one object and not with the split between the two halves. Each forwarded call is safe from another thread while a driver is cycling.

## Host actions

The UI can ask for a host action (`SystemAction`). Every action stops the runner loop. `main.cpp` runs the host command once the loop and the UI are down.

## Runner signals and recording ticks

- `Runner` is `Quittable` so SIGINT and SIGTERM can stop the run. The UI manager used to be the only quittable. With `ui.enable = false` nothing was registered, both signals were no-ops, and only SIGKILL could end the process.
- `ticks_started_` is incremented at the top of `tick()`. The recording worker uses it to tell when frames taken before a recording toggle have been published.

## OpenTX transmitter behaviour

- The init button fires once per rising edge on the configured init button channel. The channel value must cross `init_button_threshold` to count as a press.
- `channels_fresh()` is true while complete 32-channel updates keep arriving. It is seeded on port open, so a fresh (re)connect gets one timeout window of grace before it reports a stall. It assumes the port is open, so callers pair it with `serial_.is_open()`.
- `log_switch_states()` logs decoded switch flags with the raw channel value next to each, once on the first channel frame and then on every change. Recordings then show what the transmitter believed even when the driver never touched a switch.
- `limit_linear_acceleration()` slew-limits the normalized linear command so wheel-surface acceleration stays under the configured RPM/s cap. It uses the logical clock for dt and passes the command through unchanged when the limiter is disabled or on the first tick after (re)enabling. The limit applies only while the trainer enable channel (`trainer_enable_channel`) reads negative. Until then the baseline is held at standstill, so motion ramps from zero once the trainer switch engages.

## Workarounds

- `RlPolicyNavigation` computes our-robot unmeasured time as `cached_snapshot_age_s` (control clock now minus the robots header stamp) plus `measurement_age_s`. A snapshot that `RobotDescriptionsCache` substitutes carries the age frozen when it was cached, so `measurement_age_s` alone understates how long our robot has gone unmeasured. The code path past `max_our_unmeasured_s` is unverified end to end.
- `RlPolicyNavigation::update` moves `robots` into `last_visualization_` after its last use of `our_robot` and `opponent`, which point into `robots`. Moving earlier would dangle them. A restructure of `update` and `log_step` would remove the ordering rule.
- `Runner::quit_from_signal_handler` calls `camera_->cancel_initialize()`, because a camera re-open can take 30 s. The call is safe from a signal handler only while the override stores an atomic. `QuitCancelsACameraReopen` covers the behaviour, not signal safety. `main.cpp` has the same contract for `Runner::quit_from_signal_handler`, which only stores atomics.
- `main.cpp` runs the host command (`run_host_command_before_teardown`) inside `run_application`, before the camera and model destructors. A ZED close or TensorRT teardown that blocks or faults would swallow the reboot otherwise. The loop is stopped and both recordings are closed by then, so nothing is lost if systemd kills the process mid-destructor. Unverified: a 2026-09-05 SEGV fix had deferred the command past teardown and broke reboot.
- `tests/test_control_loop.cpp` asserts the control cycle rate is tracked and not an exact count. 240 Hz over a 33.3 ms frame is 7.99 periods, so `floor()` gives 7 or 8 depending on accumulated rounding. Integer tick arithmetic would make the count exact.
