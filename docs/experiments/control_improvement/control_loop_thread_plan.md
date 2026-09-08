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

### Rolling back to single-threaded

Phase 2 must keep a configuration that reproduces pre-Phase-2 behavior exactly. Two reasons, and
the second is the one that matters at an event:

1. It is the regression baseline every Phase 2 measurement is compared against.
2. It is the fallback if the threaded loop misbehaves in the field. Switching config and rebooting
   gives up the latency win and returns to known behavior without a rebuild.

**No new config field is needed.** `SteppedControlLoop` with `rate_hz = 0` means one control cycle
per perception frame, which is exactly the current pipeline: camera, perception, filter, target
selection, navigation, transmit, once per frame, on one thread.

```toml
[control_loop]
type = "SteppedControlLoop"
rate_hz = 0.0    # one cycle per perception frame = pre-Phase-2 behavior
```

`rate_hz = 0` rather than `rate_hz = 30`: at 30 Hz, `floor(dt / period)` runs zero cycles on a
frame that arrives slightly early and one otherwise, so frame jitter would silently skip commands.
Zero means one cycle per `advance_to` regardless of timing, which is what the current tick does.
This matches the existing convention where `max_loop_rate <= 0` means free-run
(`src/runner.cpp:298`).

Three requirements fall out of this, and each is a way the rollback could be subtly wrong:

- The watchdog must be inert under the stepped driver. There is no deadline to miss when the
  perception loop drives the cycles.
- The measurement queue must drain synchronously inside `advance_to`, so no correction is deferred
  by a frame relative to today.
- `transmit_rate_hz` must not decimate below one send per cycle in this mode, or the robot would
  get fewer commands than it does today.

Verify it the way Phase 1 was verified: replay an SVO under the legacy config and diff against the
pre-Phase-2 build. Once the synchronous playback path lands (Determinism item 1) that diff is exact
rather than statistical, so this becomes a real gate rather than a plausibility check.

### Transmitter output rate

**The 115200 baud in `SerialPort::open` does not throttle anything.** Confirmed by reading the
EdgeTX source rather than measuring. `CDC_SET_LINE_CODING`
(`radio/src/targets/common/arm/stm32/usbd_cdc.cpp:191-201`) stores the host's requested bitrate in
`g_lc` and invokes `baudRateCb` if one is registered. `baudRateCb` is initialized to `nullptr`
(`:140`), and nothing in the tree ever registers one: `setBaudrateCb` exists only as a
`serial_driver.h` vtable slot, wired to `usbSerialSetBaudRateCb` in the CDC driver and set to
`nullptr` everywhere else, with no call site anywhere. The stored value is only echoed back on
`CDC_GET_LINE_CODING`. Bytes land in `cliRxBuffer` and are consumed by `cliTask`
(`radio/src/cli.cpp:2000`), which blocks on a stream buffer with no rate limiting.

So an earlier concern in this plan was wrong: 250 Hz does not need 74% of a 115200 link, because
there is no 115200 link. Traffic moves at USB full speed, where 34 bytes at 250 Hz is 8.5 KB/s
against roughly 1 MB/s of practical bulk throughput.

### Measured on the X9D+ 2019

Throughput, writing `trainer 0 0` continuously at three nominal baud settings:

| Nominal baud | Throughput | Would cap at |
|---|---|---|
| 9600 | 584.6 KB/s | 0.96 KB/s |
| 115200 | 585.6 KB/s | 11.5 KB/s |
| 921600 | 584.5 KB/s | 92.2 KB/s |

Flat to within 1.00x across a 96x range of settings, and 51x the 115200 cap. The setting is
ignored, as the source says.

**But throughput is not the useful number.** `cliDefaultRx` (`radio/src/cli.cpp:107`) pushes into a
256-byte stream buffer with `xStreamBufferSendFromISR` and never checks the return, so anything
that does not fit is **dropped silently**. `CLI_RX_BUFFER_SIZE` is 256 (`cli.cpp:62`) and
`cliTask` drains it one byte at a time (`cli.cpp:2015`, with a standing TODO to make it a block
read). The 585 KB/s figure measures the USB path, not commands the CLI actually executed.

A dropped byte garbles a line, and the CLI answers a garbled line with `Invalid command` or
`Invalid argument` (`cli.cpp:~1958`), so loss is directly detectable. Sending the real two-line,
26-byte update at increasing rates for 2 s each:

| Rate | CLI errors |
|---|---|
| 50 to 3000 Hz | 0 |
| 5000 Hz | 4 |

**Clean to 3000 Hz, lossy at 5000 Hz.** A 250 Hz control loop has 12x margin. This is a
conservative floor: the test ran with CLI echo on, since it connected without enabling streaming,
whereas `OpenTxTransmitter::initialize` turns channel and telemetry streaming on, which makes
`cliEchoEnabled()` (`cli.cpp:182`) false and removes the device's echo work entirely.

**Cap per transmitter child anyway, for two reasons.** Sending faster than Crossfire's 150 Hz
over-air rate carries no information. And the failure mode above the ceiling is silent byte-drop
producing corrupted commands, not backpressure, so there is no safety margin in finding out
empirically during a match. `CompositeTransmitter` from `espnow_link_plan.md` is the natural place
for the cap.

### Watchdog

A stalled control thread leaves the robot executing its last command. `is_healthy()` reports a
missed deadline beyond `watchdog_timeout_ms`, and the Runner zeroes the command and disables
autonomy when it trips. The firmware failsafes are the backstop, but the Jetson should not need
them for its own scheduling bug.

## Config

```toml
[control_loop]
type = "ThreadedControlLoop"     # SteppedControlLoop for playback, sim, and rollback
rate_hz = 250.0                  # 0 = one cycle per perception frame (pre-Phase-2 behavior)
watchdog_timeout_ms = 20.0       # inert under SteppedControlLoop
max_prediction_horizon_ms = 400.0
transmit_rate_hz = 150.0         # per-child cap; 0 = every cycle
```

Three configurations matter, and all three are the same two fields:

| Purpose | `type` | `rate_hz` |
|---|---|---|
| Live, the point of this plan | `ThreadedControlLoop` | 250 |
| Playback and sim, deterministic | `SteppedControlLoop` | 250 |
| Rollback to pre-Phase-2 behavior | `SteppedControlLoop` | 0 |

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

### Phase 1: split the interface, stay single-threaded (done)

No thread yet. `update()` is split into `predict()`, `correct()`, and `state()`, called in sequence
where `update()` was (`src/runner.cpp:436`). `command_feedback` moved to `predict()` as the control
input. `NoopRobotFilter` and `GroundTruthRobotFilter` take the default no-op `predict()`;
`RobotFrontBackSimpleFilter` overrides it to record the control input its `correct()` consumes.

**Playback is not reproducible today, so "identical output" was not verifiable.** The plan assumed
it was. Measured on `2026-05-02T11-45-08.svo2` from frame 13000, comparing `/robot_markers` poses
joined on `svo_frame_index`:

| Comparison | Median | p95 | Max |
|---|---|---|---|
| Same binary, 7 run pairs | 2.3 to 39.5 mm | 11.6 to 110.4 mm | up to 1056 mm |
| Baseline vs Phase 1, 2 run pairs | 32.9 to 47.2 mm | 104.5 to 130.3 mm | up to 2462 mm |

Two runs of the *same* binary differ as much as baseline versus the change, so the harness cannot
resolve a difference at this scale. Two independent causes:

1. `ZedRgbdCamera` grabs on an async `capture_thread_` and the loop consumes `latest_data_`, so
   which frames reach the pipeline depends on thread scheduling. Runs processed 1922 to 2028 frames
   of the same file, sharing only ~75%. The filter is history-dependent, so a different frame
   subset gives a different trajectory.
2. Message header stamps are rebased to wall clock, so payload bytes never match across runs. Only
   `/camera/frame_meta.image_stamp_ns` and `svo_frame_index` are stable.

Parity was instead proven with a differential test. `RobotFrontBackSimpleFilter` from the commit
before the split was vendored under a second name and driven alongside the new one through a
deterministic 400-frame scenario covering our-robot dropouts (dead reckoning), opponent dropouts
(hold last pose), blob-only frames, and varying command feedback. Every output field matched
bit-exactly, including pose, rotation, size, velocity, staleness, and
`last_our_blob_present_no_keypoint`.

The test is sensitive to the failure mode it was written for: mutating `predict()` to drop the
control input makes it fail at frame 35, the first our-robot dropout. The vendored copy is not in
the tree, since it duplicates 671 lines and stops being meaningful the moment Phase 2 changes
prediction deliberately. Reproduce with
`git show <pre-split-commit>:src/robot_filter/robot_front_back_simple_filter.cpp`.

The 257 pre-existing unit tests also pass unchanged. Only their call mechanism was rewritten; every
numeric assertion is untouched, so they encode the old behavior's expected values.

**This moves determinism from a Phase 2 checkbox to a prerequisite.** `SteppedControlLoop` cannot
make playback reproducible while the camera still delivers a scheduling-dependent frame subset. The
camera needs a synchronous playback path that hands every frame to the loop in order before Phase 2
measurement 4 means anything.

### Phase 2: add the control loop, no forward prediction (done)

`ControlLoopInterface` with both drivers is in. `ControlLoop` owns the filter, target selection,
navigation, and transmitter; the Runner keeps camera and perception and hands measurements over
through `submit_measurement`. Corrections still apply at the current time without rollback, so the
estimate remains 55 ms old.

**Parity at `rate_hz = 0` is proven exactly**, by unit test rather than by replay.
`tests/test_control_loop.cpp` drives the same components two ways, through the control loop and
through an inline reference sequence written independently of the `ControlLoop` body, over 200
frames covering our-robot dropouts and opponent dropouts. Robot poses, staleness, frame ids, and
the full command stream match bit-exactly. Three further tests pin the properties the rollback
depends on: `rate_hz = 0` runs exactly one cycle per `advance_to` regardless of advance timing, a
non-zero rate runs the expected multiple, and a measurement is corrected exactly once.

Replay agrees to the resolution the harness allows. Detections are 100% identical, and pose deltas
between pre-Phase-2 and Phase 2 stepped (20.1 and 38.7 mm median) sit inside the same-binary
control range (23.6 and 21.7 mm median). Exact replay comparison still waits on Determinism item 1.

**Threaded at 250 Hz, measured on the dev machine over two ~40 s replays:**

| Metric | Result |
|---|---|
| Achieved rate | 250.0 Hz mean, 249.7 to 250.9 Hz range |
| Deadline misses | 1 per run, out of ~10,000 cycles |
| Cycle time, mean | 97.6 us against a 4000 us budget (2.4% duty) |
| Cycle time, worst | 2805 us, still inside the period |
| Command update interval | 20.1 ms stepped, **4.0 ms threaded** |
| Perception tick rate | 49.7 Hz stepped, 49.4 Hz threaded (no regression) |

The command interval result is Win 1, measured. Replay runs perception faster than the 30 fps the
camera delivers, so the real-hardware improvement is 33.3 ms to 4.0 ms rather than 20.1 to 4.0.

At 97.6 us mean the loop has roughly 40x headroom at 250 Hz, so the rate ceiling is far above what
the plant needs. That ceiling still has to be confirmed on the Jetson, where the loop competes with
TensorRT for cores.

**Two bugs this surfaced, both fixed:**

- `DiagnosticsModuleLogger` had no locking. A module logger now has two writers, the control thread
  logging and the Runner thread clearing during `DiagnosticsLogger::publish()`, and the unguarded
  `std::map` members segfaulted within seconds. Guarded with a mutex.
- The watchdog fired during startup because `run_cycle()` stamped its heartbeat at the end, after
  the early return taken while no field is defined yet. A cycle that legitimately does nothing is
  still a cycle, so the stamp moved to cycle entry.

**The existing prediction model cannot simply be called faster.** `RobotTemporalMotionFilter::
update_with_prediction` (`src/robot_filter/robot_temporal_motion_filter.cpp:29`) has four
properties that make a naive 250 Hz call a no-op or worse:

| Line | Property | Effect at control rate |
|---|---|---|
| `:54,101` | `dt` is measured against the last *measurement* stamp, refreshed for every track every call | Called at 250 Hz with a 30 Hz stamp, `dt` is 0 on 7 of 8 calls |
| `:55` | `if (dt <= 0.0 \|\| dt > 1.0) continue;` | Those 7 calls skip prediction entirely, so nothing advances |
| `:43` | `if (measured_frame_ids.count(frame_id) != 0) continue;` | Measured tracks are never propagated, which is exactly what Phase 3 needs |
| `:45-49` | No command feedback for a frame id means hold last pose | Opponents are frozen between measurements, never extrapolated |

Three changes are needed before the rate increase buys anything:

1. **Take control time, not the measurement stamp.** `predict(now, ...)` already carries it; the
   temporal filter needs to use it and keep a separate per-track propagation clock.
2. **Propagate measured tracks too**, not just unmeasured ones.
3. **Give opponents a motion model.** Holding the last pose means navigation at 250 Hz chases a
   30 Hz staircase for the opponent, so Win 1 lands almost entirely on our-robot dead reckoning
   until this exists. The opponent track is already live under 50% of frames, so this matters more
   than it looks.

Item 3 is the constant-velocity opponent EKF in the Kalman plan. Items 1 and 2 are prerequisites
for it and belong to this plan.

This is Win 1. Measure:

1. Command update interval, mean and p99, against the 33.3 ms baseline.
2. Control loop deadline misses at 100, 250, and 500 Hz. Find where the Jetson stops keeping up.
3. Trainer command loss at the configured rate, using the `Invalid command` detector from
   Transmitter output rate. Measured ceiling is 3000 Hz, so this should be clean, but confirm it
   from the Jetson rather than the dev machine.
4. SVO replay determinism: byte-identical output across repeated runs under `SteppedControlLoop`.
5. **Rollback fidelity:** `SteppedControlLoop` with `rate_hz = 0` replayed against the
   pre-Phase-2 build, byte-identical. This gates the field fallback, so it is not optional.
6. Nav sim sweep hit rate and time-to-stop, against the Stage 3 baseline. This should improve, and
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

## Determinism: what it would take

Playback and a live camera need different definitions, and both are reachable.

**Playback: same file, identical output, every run.** Fully achievable.

**Live: not reproducible, but a pure function of the frames consumed.** A camera sees a different
world each run, so "same output every run" is meaningless. The useful property is that replaying
the exact frame sequence a live run consumed reproduces that run's output. That is what makes a
recorded match debuggable offline, and it is achievable.

### What is already deterministic

Measured on `2026-05-02T11-45-08.svo2` from frame 13000, two runs of the same binary.

**TensorRT is bitwise reproducible.** All three engines the desktop config loads, tested at the raw
TensorRT level with no ultralytics preprocessing or NMS, over 8 fixed inputs with 10 repeats each,
run twice in separate processes:

| Engine | Within process | Across processes |
|---|---|---|
| `yolo26n-pose_our_robots_2026-05-01` | 8/8 identical | identical |
| `yolo26n_nhrl_robots_bbox_2class_mixed_2026-07-31` | 8/8 identical | identical |
| `field_deeplabv3p_r50_2026-07-29` | 8/8 identical | identical |

**`svo_frame_index` is correct.** Strictly monotonic, no duplicates, and the index gap agrees with
elapsed `image_stamp_ns` in 2055 of 2056 steps. `capture_frame` also follows both documented ZED
pitfalls correctly: `grab()` is called exactly once per iteration
(`src/rgbd_camera/zed_rgbd_camera.cpp:355`) and the position is read only after the success check
at `:363`.

**Perception is deterministic given the same frame.** Joining the two runs on frame index via
`camera_info`'s stamp, detections match on **1522 of 1522 shared frames (100%)** on both
`/keypoint_detections` and `/blob_detections`, once the buffer race below is fixed. Before that fix
it was 99.87% and 99.80%.

### The one real cause

**Frame selection, not frame processing.** `ZedRgbdCamera` grabs on an async `capture_thread_` and
the loop consumes whatever `latest_data_` holds. The two runs processed 2057 and 2034 frames of the
same file and shared only 1529 of them. The filter is history-dependent, so a different frame
subset produces a different trajectory. That alone accounts for the divergence measured in Phase 1.

### The pixel buffer race (fixed)

`ZedRgbdCamera::get` does `data = latest_data_;`, and `cv::Mat` assignment is a shallow copy, so the
consumer held a pointer into the buffer `capture_frame` wrote the next frame into via
`cv::cvtColor`. `cv::Mat::create()` reuses an existing allocation whenever size and type match
without consulting the reference count, so perception could read pixels being overwritten one frame
behind. `data_mutex_` protected the struct copy, not the pixels afterward.

Fixed by converting into a per-frame `cv::Mat` and assigning it, so the consumer's reference keeps
its allocation alive. Costs one allocation per frame and no extra copy, since the conversion writes
the full image either way. Measured effect: perception determinism went from 99.87% and 99.80% to
100% on both channels, with no change in tick time (mean `tick_ms_max` 33.5/32.3 before,
32.9/33.3 after).

The UI path had already worked around this locally (`src/runner.cpp:155` notes UIState clones "to
detach from the camera SDK's reusable buffer"), which was the same bug surfacing once before.

### What it would take

1. **Synchronous frame delivery in playback.** No capture thread: `get()` grabs and returns that
   frame, every frame in order, none dropped. This is the whole fix for playback.
2. ~~**No shared mutable pixel buffer at the handoff.**~~ Done: per-frame conversion buffer.
3. **Record which frames the pipeline consumed.** `/camera/frame_meta` already carries a
   trustworthy identity, so the record already exists.
4. **A replay mode that consumes exactly a recorded frame list**, rather than whatever is latest.
   This is what turns a live recording into a reproducible one, and it is the only sense in which a
   live camera can be deterministic.

Item 1 is the one remaining prerequisite for Phase 2. Items 3 and 4 stand on their own merits and
are independent of the control loop work. Nothing here is blocked on model or SDK behavior.

## Risks

**Forward prediction can make things worse.** Extrapolating 55 ms with a bad model compounds error
rather than removing staleness. This is why Phase 3 is separate from Phase 2 and gated on a
measurement, and why the kill switch caps the horizon at zero.

**Playback is not reproducible today, for one reason below the control loop.** See
[Determinism](#determinism-what-it-would-take) for the measurements and the cause.
`SteppedControlLoop` stops the control loop from adding a second source, but it cannot fix the
camera. This is a prerequisite for Phase 2 measurement 4, not a byproduct of it.

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

1. ~~Land Phase 1.~~ Done. The interface split is in; see the Phase 1 notes for what its
   verification could and could not establish.
2. ~~Land Phase 2.~~ Done. Both drivers are in, parity at `rate_hz = 0` is proven by unit test,
   and 250 Hz measured 250.0 Hz achieved at 2.4% duty. See the Phase 2 notes.
3. ~~Fix the shared pixel buffer.~~ Done. Perception is now bitwise reproducible per frame, so
   frame selection is the only remaining source of nondeterminism.
4. **Give the camera a synchronous playback path** that grabs and returns each frame in order.
   This is the whole determinism fix for playback, and it is what turns the Phase 2 parity check
   from a unit test into an exact end-to-end replay diff.
5. Add a replay mode that consumes a recorded frame list, making live runs reproducible offline.
   Independent of the control loop work. See Determinism for the full list.
6. ~~Confirm whether the handset's USB CDC honors the 115200 baud setting.~~ Measured on the
   X9D+ 2019: it does not, and the CLI sustains 3000 Hz cleanly against a 250 Hz need. Decimation
   is optional, not required. See Transmitter output rate.
7. Find the rate ceiling on real Jetson hardware, not on the dev machine. The dev machine shows
   2.4% duty at 250 Hz; the Jetson competes with TensorRT for cores.
8. Only then decide whether Phase 3 is worth it, using measured prediction error rather than the
   ~55 ms figure this document assumes.
