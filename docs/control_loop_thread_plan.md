# Plan: run filter and control on their own thread

## Goal

Move the filter prediction, target selection, navigation, and transmit onto a thread with a
configurable rate, decoupled from the 30 Hz perception tick. Perception becomes a slow measurement
source that corrects a fast prediction, rather than the clock that gates the whole pipeline.

Perception cannot be sped up. The camera runs at 30 fps (`config/_common.toml:13`) and the
TensorRT inference behind it sets the rest. This plan accepts that as fixed and attacks the two
costs it imposes instead: the command update interval, and the age of the state estimate the
controller acts on.

## Latency impact

### Where the time goes today

Everything runs serially inside `Runner::tick()` (`src/runner.cpp:331`), gated on
`camera_->get()` at `src/runner.cpp:366`:

```
camera get -> field filter -> perception batch -> robot filter -> target select -> nav -> transmit
```

The command is sent immediately after perception completes, so there is no queueing delay inside a
tick. The cost is what happens between ticks: the command is **held constant for 33.3 ms while the
world keeps moving**, and it was computed from a state estimate valid at image capture time, which
is already roughly 55 ms in the past by then (README footnote: ~20 ms acquisition, ~35 ms
sense-and-control).

### Two separate wins, and they are not the same size

| Term | Today | Threaded, prediction only | Threaded + forward prediction |
|---|---|---|---|
| Capture to perception result | ~55 ms | ~55 ms | ~55 ms |
| Perception latency compensated by prediction | 0 | 0 | up to ~55 ms |
| Command update interval, mean age | 16.7 ms | 2.0 ms | 2.0 ms |
| Command update interval, worst case | 33.3 ms | 4.0 ms | 4.0 ms |
| Transport, Crossfire | 7.6 ms | 7.6 ms | 7.6 ms |
| **Effective age of the state the wheels act on** | **~79 ms** | **~65 ms** | **~13 ms** |

**Win 1, the thread alone: ~15 ms.** Running nav at 250 Hz means the command tracks the filter's
evolving estimate instead of freezing between frames. This needs no model quality at all, only the
thread.

**Win 2, forward prediction: up to ~55 ms more.** Today navigation acts on an estimate valid 55 ms
ago. A prediction step lets the filter propagate that measurement forward to the moment the command
is sent, so navigation acts on an estimate of *now*. This is the larger win by a factor of four,
and it is entirely dependent on the prediction model being good. That is what the Kalman filter is
for.

The ~13 ms residual assumes the Kalman plan's C1 criterion holds (RMSE < 15 mm at a 100 ms
horizon). At 5.6 m/s, 15 mm of position error is equivalent to 2.7 ms of staleness.

### For comparison

| Change | Latency saved | Cost |
|---|---|---|
| This plan, thread only | ~15 ms | One thread, one interface split |
| This plan, with forward prediction | ~66 ms | Depends on the Kalman filter landing |
| ESP-NOW command link (`espnow_link_plan.md`) | ~5 ms | New radio, new firmware, new dongle |
| Raising CRSF baud to 921600 | ~1 ms | One menu change |

The control thread is worth more than everything else under consideration, combined.

### What this does not fix

**It does not make the estimate more informed.** Running navigation at 250 Hz on a predicted state
creates no new information about where the opponent is. Between corrections the filter is
extrapolating, and extrapolation quality is bounded by the model.

**Prediction is not free accuracy.** Propagating a measurement forward 55 ms with a bad model is
worse than acting on a 55 ms old measurement, because the error compounds instead of staying
bounded. Win 2 is contingent on the Kalman work, and Phase 3 below gates on measuring it rather
than assuming it.

**Opponents are harder than our robot.** We know our own commanded velocity, so our prediction has
a control input. Opponents get constant-velocity extrapolation with no input, and the existing
dropout baseline says the opponent track is live under 50% of frames with p90 gaps of 340 ms. The
prediction horizon has to be capped, which is what `max_prediction_horizon_ms` is for.

## Current state

### The interface conflates predict and correct

`RobotFilterInterface::update()` (`include/robot_filter/robot_filter_interface.hpp:10`) takes
perception outputs and returns state in one call:

```cpp
virtual RobotDescriptionsStamped update(KeypointsStamped keypoints, FieldDescription field,
                                        CameraInfo camera_info,
                                        KeypointsStamped robot_blob_keypoints,
                                        CommandFeedback command_feedback) = 0;
```

There is no way to advance time without supplying a measurement, which is exactly what a fast
control loop needs to do.

`RobotTemporalMotionFilter::update_with_prediction`
(`src/robot_filter/robot_temporal_motion_filter.cpp:42`) already does dead-reckoning propagation for
tracks that went unmeasured this frame, so the concept exists. It just cannot be invoked
independently, and it only runs for missing tracks rather than to advance the whole state to the
present.

### The clock is driven by camera frames

`ClockInterface` (`include/time/clock_interface.hpp`) documents the constraint directly: the Runner
sets logical time from the camera frame stamp each tick so "sim/playback runs become deterministic
and reproducible."

A free-running control thread breaks that. Playback with SVO recordings is the project's primary
regression-test mechanism, so preserving determinism is a hard requirement, not a nicety.

### Threading already exists here

`ParallelModelBatch` (`include/perception_batch/parallel_model_batch.hpp`, selected by
`runner_config_.parallel_models`) already runs the keypoint and blob models on worker threads. The
codebase has the pattern; this plan adds a second, coarser split.

## Design

### Split the filter interface

```cpp
class RobotFilterInterface {
   public:
    virtual bool initialize(int opponent_count) = 0;

    /**
     * Advance every track to `now` using the last commanded velocity as the control input.
     * Called at control rate. Default is a no-op so filters that only move on measurements
     * (NoopRobotFilter, GroundTruthRobotFilter) need no change.
     */
    virtual void predict([[maybe_unused]] double now,
                         [[maybe_unused]] CommandFeedback command_feedback) {}

    /**
     * Fold in a perception measurement. `keypoints.header.stamp` is the capture time and will be
     * older than the last predict() time, typically by 55 ms. Implementations that predict must
     * roll back to that stamp, apply the correction, and replay forward.
     */
    virtual void correct(KeypointsStamped keypoints, FieldDescription field, CameraInfo camera_info,
                         KeypointsStamped robot_blob_keypoints) = 0;

    /** Best estimate as of the most recent predict(). */
    virtual RobotDescriptionsStamped state() const = 0;

    virtual bool last_our_blob_present_no_keypoint() const { return false; }
};
```

`command_feedback` moves from the measurement path to the prediction path, because it is the
control input `u` in the Kalman plan's notation, not an observation.

The existing `update()` becomes `correct()` followed by `state()`. For the three filters that do
not predict, that is a mechanical rename.

### Out-of-sequence measurements are the hard part

Perception output arrives ~55 ms after the image it describes. By then the control thread has
already predicted past that timestamp, so the correction cannot simply be applied at "now."

**Use rollback and replay.** Keep a ring buffer of `(timestamp, state, covariance, command_input)`
covering at least `max_prediction_horizon_ms`. On correction:

1. Find the buffered state at or just before `keypoints.header.stamp`.
2. Apply the measurement update there.
3. Replay prediction steps forward to the current control time using the buffered command inputs.

At 250 Hz a 55 ms replay is 14 steps on a small state, which is cheap. This is exact, unlike
extrapolating the measurement forward, and it is the standard fix for delayed measurements in a
Kalman filter.

Use the **capture** stamp, not the arrival stamp. Note the known SVO offset: image stamps sit about
half a frame before pipeline stamps, so join on frame interval rather than nearest timestamp when
reconciling recorded data.

### Thread topology, and who owns the filter

```
Perception thread (30 Hz, camera-bound)        Control thread (configurable, default 250 Hz)
──────────────────────────────────────         ─────────────────────────────────────────────
camera_->get()                                 transmitter_->update()  -> CommandFeedback
field_filter_->track_field()                   drain measurement queue
perception_batch_->update()                      └─ robot_filter_->correct(...)   [rollback+replay]
push measurement ──────────────────────────►   robot_filter_->predict(now, feedback)
publisher_, mcap_recorder_, ui_state_          robots = robot_filter_->state()
                                               target_selector_->get_target(robots, field)
                                               navigation_->update(robots, field, target)
                                               transmitter_->send(command)
```

**Only the control thread mutates the filter.** Perception pushes an immutable measurement struct
into a single-producer single-consumer queue and never touches filter state. That removes the need
for a lock on the hot path entirely, which matters at a 4 ms budget.

`FieldDescription` rides along in the measurement payload; the control thread holds the most recent
one.

Publishing, MCAP recording, and UI updates stay on the perception thread. Navigation visualization
is produced on the control thread, so hand it over as a double-buffered snapshot and publish it at
the perception rate. Nobody needs 250 Hz debug images.

### Determinism: two drivers behind one interface

```cpp
class ControlLoopInterface {
   public:
    virtual ~ControlLoopInterface() = default;
    virtual void start() = 0;
    virtual void stop() = 0;
    /** Stepped driver only. Runs whole control cycles until logical time reaches `until`. */
    virtual void advance_to([[maybe_unused]] double until) {}
    virtual bool is_healthy() const = 0;
};
```

- **`ThreadedControlLoop`** (live): spawns a thread paced against `steady_clock` at `rate_hz`.
- **`SteppedControlLoop`** (playback, sim): no thread. The perception loop calls `advance_to(stamp)`
  after each frame, which runs `floor(dt / period)` cycles against the ManualClock.

Same code path for filter, nav, and transmit in both. The only difference is what drives the
cycles. Playback stays single-threaded and byte-for-byte reproducible, so SVO regression tests keep
working.

Selected by config through the existing factory pattern, same as every other component.

### Transmitter output rate

Sending trainer channels at 250 Hz will not fit the wire. `write_trainer_channels`
(`src/transmitter/opentx_transmitter.cpp:166`) writes two ASCII lines totaling ~34 bytes, and
`SerialPort::open` defaults to 115200 baud. That is 2.95 ms per send, so 250 Hz would need 74% of
the link and 150 Hz needs 44%.

**Decimate per transmitter child.** The trainer path caps at 150 Hz, which matches Crossfire's
over-air rate so no information is lost. ESP-NOW, when it lands, runs at the full control rate.
`CompositeTransmitter` from `espnow_link_plan.md` is the natural place for a per-child rate cap.

Measure first: if the handset's USB CDC ignores the baud setting and runs at USB speed, which is
likely for an STM32 virtual COM port, the constraint disappears. If it does not, fall back to
100 Hz on the trainer path.

### Watchdog

A stalled control thread leaves the robot executing its last command. `is_healthy()` reports a
missed deadline beyond `watchdog_timeout_ms`, and the Runner zeroes the command and disables
autonomy when it trips. The firmware failsafes are the backstop, but the Jetson should not need
them for its own scheduling bug.

## Config

```toml
[control_loop]
type = "ThreadedControlLoop"     # SteppedControlLoop for playback and sim
rate_hz = 250.0
watchdog_timeout_ms = 20.0
max_prediction_horizon_ms = 400.0
transmit_rate_hz = 150.0         # per-child cap; 0 = every cycle
```

`rate_hz = 250` comes from the plant, not from taste. The Kalman plan measured an accel time
constant of 58 ms and an actuation lag of 59 ms; sampling at 10-20x the dominant constant puts the
useful band at 170-340 Hz. Making it configurable lets Phase 2 sweep it rather than argue about it.

`max_prediction_horizon_ms = 400` matches the existing coast timeout derived from the perception
dropout baseline, where opponent gaps have a p90 of 340 ms. Past that the prediction is not worth
acting on and the track should be marked stale.

## Relationship to the Kalman filter plan

`docs/experiments/kalman_filter/kalman_filter_plan.md` and this plan are complements, and neither
delivers its full value alone.

The Kalman plan's baseline critique lists five defects in the current dead reckoning. **Defect 5 is
a scheduling problem this plan fixes directly:** "At 30 Hz and a yaw rate near the calibrated
61.5 rad/s, one tick is 2 rad of rotation. Straight-line Euler stepping is meaningless there." At
250 Hz the same rotation is 0.25 rad per step. A better integrator helps, but a shorter step is
what makes the integration tractable at all.

The other four defects (transport delay, actuator dynamics, deadzone and saturation, no
uncertainty) are model problems that this plan does not touch. It provides the place to run them.

Division of labor:

| Concern | Owned by |
|---|---|
| When prediction runs, and how often | This plan |
| Handling delayed measurements | This plan (rollback and replay) |
| Playback determinism | This plan |
| What the prediction step computes | Kalman plan |
| Covariance, and when to stop trusting a coast | Kalman plan |
| Plant model, transport delay, actuator dynamics | Kalman plan |

Land this first with the existing dead-reckoning prediction. It gets Win 1 immediately and gives
the Kalman work a fast prediction step to drop into, instead of requiring both changes to land
together.

## Relationship to the ESP-NOW plan

`docs/espnow_link_plan.md` identified the 33.3 ms command interval as the dominant latency term and
larger than everything in that plan. This is the fix for it.

Two knock-on effects on that plan:

1. **The dongle repeat becomes a safety net rather than a requirement.** That plan needs a 250 Hz
   dongle-side repeat because the Jetson only sends at 30 Hz. With the control thread, the Jetson
   sends at the control rate and the repeat only covers actual packet loss.
2. **Per-child transmit rate capping** is needed by both plans, and `CompositeTransmitter` should
   carry it.

## Phases

### Phase 1: split the interface, stay single-threaded

No thread yet. Split `update()` into `predict()`, `correct()`, and `state()`, and have the Runner
call them in sequence exactly where `update()` was. Migrate the four existing filters.

This should be a behavioral no-op. Verify with an SVO replay producing identical output to the
current build. Cheap to review, and it de-risks the interface before any concurrency lands.

### Phase 2: add the control loop, no forward prediction

Add `ControlLoopInterface` with both drivers. Run the filter, target selection, navigation, and
transmit at `rate_hz`. `predict()` advances the state, but corrections still apply at the current
time without rollback, so the estimate remains 55 ms old.

This is Win 1. Measure:

1. Command update interval, mean and p99, against the 33.3 ms baseline.
2. Control loop deadline misses at 100, 250, and 500 Hz. Find where the Jetson stops keeping up.
3. Trainer serial duty cycle at 150 Hz, confirming or killing the decimation concern.
4. SVO replay determinism: byte-identical output across repeated runs under `SteppedControlLoop`.
5. Nav sim sweep hit rate and time-to-stop, against the Stage 3 baseline. This should improve, and
   if it does not, the control rate is not the binding constraint and Phase 3 needs rethinking.

### Phase 3: rollback and replay, forward prediction

Add the ring buffer and out-of-sequence correction. Navigation now acts on an estimate of the
present rather than of 55 ms ago.

This is Win 2, and it is the phase that can make things worse if the prediction model is poor. Gate
it on:

1. Position error of the forward-predicted estimate against the next arriving measurement, binned
   by prediction horizon. This is a direct, continuous measure of whether prediction helps.
2. Nav sim sweep, again. Prediction that degrades the sweep is prediction that should be off.
3. A config kill switch that caps the forward-prediction horizon at zero, so Phase 3 can be
   disabled at the field without a rebuild.

### Phase 4: Kalman filter

Hand off to `kalman_filter_plan.md`. The prediction step it specifies replaces the dead-reckoning
one, and its covariance replaces the fixed `max_prediction_horizon_ms` cutoff with something that
knows when it stopped being confident.

## Risks

**Forward prediction can make things worse.** Extrapolating 55 ms with a bad model compounds error
rather than removing staleness. This is why Phase 3 is separate from Phase 2 and gated on a
measurement, and why the kill switch caps the horizon at zero.

**Playback determinism.** The single most likely thing to break, and it breaks the project's main
regression-test mechanism. `SteppedControlLoop` exists specifically to prevent it, and Phase 2
measurement 4 checks it explicitly. Do not let live and playback share a threaded driver.

**Target selection at 250 Hz may flip.** Running selection eight times more often on a
predicted-only state could oscillate between opponents during a coast.
`previous_selected_target_` (`include/runner.hpp:98`) already provides hysteresis; confirm it holds
at the higher rate, and if not, keep target selection on the perception thread.

**Jetson CPU headroom.** The control thread competes with TensorRT inference for cores. Phase 2
measurement 2 finds the ceiling. If 250 Hz does not fit, the rate is configurable for exactly this
reason, and the plant analysis says 170 Hz is still in the useful band.

**Filter implementations that cannot predict.** `predict()` defaults to a no-op, so
`NoopRobotFilter` and `GroundTruthRobotFilter` keep working, but they get none of Win 1. Any config
using them sees no benefit, which is correct and should not be papered over.

## Next steps

1. Land Phase 1. It is a mechanical interface split with an SVO replay proving it changed nothing.
2. Confirm whether the handset's USB CDC honors the 115200 baud setting. That decides whether
   trainer decimation is required or optional, and it is a 10 minute measurement.
3. Build `ControlLoopInterface` with `SteppedControlLoop` first. Playback determinism is the
   constraint most likely to force a redesign, so prove it before writing the threaded driver.
4. Run Phase 2 and find the rate ceiling on real Jetson hardware, not on the dev machine.
5. Only then decide whether Phase 3 is worth it, using measured prediction error rather than the
   ~55 ms figure this document assumes.
