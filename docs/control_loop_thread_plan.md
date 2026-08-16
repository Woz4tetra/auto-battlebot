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

### Phase 2: add the control loop, no forward prediction

Add `ControlLoopInterface` with both drivers. Run the filter, target selection, navigation, and
transmit at `rate_hz`. `predict()` advances the state, but corrections still apply at the current
time without rollback, so the estimate remains 55 ms old.

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
`camera_info`'s stamp, detections match on **1527 of 1529 shared frames (99.87%)**.

### The one real cause

**Frame selection, not frame processing.** `ZedRgbdCamera` grabs on an async `capture_thread_` and
the loop consumes whatever `latest_data_` holds. The two runs processed 2057 and 2034 frames of the
same file and shared only 1529 of them. The filter is history-dependent, so a different frame
subset produces a different trajectory. That alone accounts for the divergence measured in Phase 1.

### One latent race worth fixing regardless

`ZedRgbdCamera::get` (`zed_rgbd_camera.cpp:561`) does `data = latest_data_;`. `cv::Mat` assignment
is a shallow copy, so the consumer holds a pointer into the buffer `capture_frame` writes the next
frame into via `cv::cvtColor`, which reuses the destination allocation when size and type match.
`data_mutex_` protects the struct copy, not the pixels afterward. The UI path already works around
this locally (`src/runner.cpp:155` notes UIState clones "to detach from the camera SDK's reusable
buffer").

The 99.87% figure bounds how often this actually bites at current frame rates, so it is a
correctness fix rather than the determinism blocker.

### What it would take

1. **Synchronous frame delivery in playback.** No capture thread: `get()` grabs and returns that
   frame, every frame in order, none dropped. This is the whole fix for playback.
2. **No shared mutable pixel buffer at the handoff.** Deep copy costs roughly 2.7 MB per frame; a
   rotating buffer pool avoids the copy if that measures.
3. **Record which frames the pipeline consumed.** `/camera/frame_meta` already carries a
   trustworthy identity, so the record already exists.
4. **A replay mode that consumes exactly a recorded frame list**, rather than whatever is latest.
   This is what turns a live recording into a reproducible one, and it is the only sense in which a
   live camera can be deterministic.

Items 1 and 2 are prerequisites for Phase 2. Items 3 and 4 stand on their own merits and are
independent of the control loop work. Nothing here is blocked on model or SDK behavior.

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
2. **Fix the shared pixel buffer at `zed_rgbd_camera.cpp:561`.** A latent data race that can hand
   perception a half-overwritten frame, worth fixing whether or not determinism is pursued.
3. **Give the camera a synchronous playback path** that grabs and returns each frame in order.
   This is the whole determinism fix for playback and the blocking item for Phase 2.
4. Add a replay mode that consumes a recorded frame list, making live runs reproducible offline.
   Independent of the control loop work. See Determinism for the full list.
5. Confirm whether the handset's USB CDC honors the 115200 baud setting. That decides whether
   trainer decimation is required or optional, and it is a 10 minute measurement. Independent of
   everything else.
6. Build `ControlLoopInterface` with `SteppedControlLoop` first, on top of step 3.
7. Run Phase 2 and find the rate ceiling on real Jetson hardware, not on the dev machine.
8. Only then decide whether Phase 3 is worth it, using measured prediction error rather than the
   ~55 ms figure this document assumes.
