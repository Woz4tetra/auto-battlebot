# Robot filter pipeline

The robot filter turns per-frame perception output into stable, identified robot tracks for
navigation. The active implementation is `RobotFrontBackFilter`
(`src/robot_filter/robot_front_back_filter.cpp`), selected by `[robot_filter] type` in the
TOML config.

It runs on two clocks. `correct()` runs once per perception frame with new detections.
`predict()` runs every control-loop tick, at 250 Hz, and advances tracks between frames.
Both are called from `src/control_loop/control_loop.cpp`.

How `predict()` propagates is a swappable `MotionEstimatorInterface`, chosen by
`[robot_filter.motion_estimator] type`. `DeadReckoningMotionEstimator` coasts along the
last commanded velocity and only moves our robot, which is the only one with command
feedback. `KalmanMotionEstimator` is what `config/_common.toml` selects; it coasts
opponents too, from their estimated velocity.

![Robot filter pipeline](../../diagrams/robot_filter.svg)

Regenerate the SVG after editing `diagrams/robot_filter.dot`:

```bash
./docs/generate_diagrams.sh   # needs graphviz (dot)
```

## Inputs

- `keypoints`: front/back keypoints per detected robot, from the keypoint model.
- `robot_blob_keypoints`: segmentation blob detections, a fallback when keypoints are missing.
- `field + camera_info`: the field transform and intrinsics used to project pixels onto the field plane.
- `command_feedback`: the velocity command currently driving OUR_ROBOT_1, read back from the transmitter
  (live RC channels on hardware). Only our robot has feedback, so only our robot gets predicted.

## Sim keypoint model

`SimKeypointModel` feeds the filter from the lockstep sim instead of a detector. It builds front and back keypoints from the poses the sim's sensing model delivered (`simulation/kinematic_sim_server.py`), so the real `RobotFrontBackFilter` and `KalmanMotionEstimator` run on simulated perception: its latency, dropouts, noise and heading flips. A flipped heading arrives as swapped front and back keypoints, the way a real flip does.

- Keypoints are projected at floor height (z = 0). The sim's depth image is bare floor, so the height gate abstains on box-less detections and stamps each keypoint with the height it measured there, which is zero. Projecting at z = 0 keeps the converter's round trip exact.
- The result carries the frame's capture stamp. The tick that delivers it comes later by the sensing model's latency, which is what the filter sees on the robot.

## Stages

1. **`convert_keypoints_to_measurements`** pairs front/back keypoints per label
   (`FrontBackKeypointConverter`), projects them onto the field plane, drops out-of-field detections,
   then assigns each measurement to a `FrameId` with `FrameIdAssigner::assign`. Assignment is greedy
   nearest-neighbor against each track's last position, with a `max_jump_distance` gate that rejects
   implausible jumps (and force-accepts after `max_consecutive_jump_rejects` to recover from a bad lock).

2. **`merge_blob_detections`** adds blob detections, suppressing any that fall near an existing keypoint
   measurement so the same robot is not counted twice, and assigns leftover frame_ids.

3. **`RobotTemporalMotionFilter::update_with_prediction`** stores the measured robots, then for any
   tracked robot that was not measured this frame and has command feedback (OUR_ROBOT_1), dead-reckons
   its pose forward along the commanded velocity, clips it to the field, and flags it `is_stale`. It
   emits one `RobotDescription` per tracked robot, measured or predicted.

## Persistent state

Three per-`FrameId` maps survive across frames:

- `FrameIdAssigner::last_position_per_frame_id_` (plus jump-reject counters): drives data association.
- `last_description_per_frame_id_`: the last full pose of each tracked robot.
- `last_timestamp_per_frame_id_`: the last time each robot appeared in the output, used only to compute
  the `dt` for dead-reckoning.

## Why prediction is load-bearing

The output `velocity` field is unused downstream (target selection and navigation read pose only). The
load-bearing path is the position prediction (indigo loop in the diagram): during a detection dropout it
both gives navigation an extrapolated pose instead of a frozen one, and advances the association
`last_position` (`set_last_position`) so the reappearing detection still matches inside the jump gate.
This helps the common short dropouts and drifts on rare long ones. The playback A/B that
measured it lived in `playground/control_stage0/`, retired in `9246cc64`; see
`docs/experiments/control/control_stage0_retired.md`.

## Control loop ownership and threading

- `ControlLoop` is the filter, target, navigation and transmit half of the pipeline, decoupled from perception. It owns the transmitter outright. Every call into the transmitter happens on whichever thread drives the cycle, so the Runner reaches it only through thread-safe request setters.
- Requests (target override, recording and mode toggles) are applied at the top of the next cycle, so the components stay single-threaded.
- `SteppedControlLoop` and `ThreadedControlLoop` decide when `run_cycle()` fires; the cycle body is identical either way. A stepped driver reads the transmitter inline, so a single-threaded run reads the transmitter before the camera grab.
- Once the threaded driver runs, the Runner reaches the transmitter only through the control loop, since the driver owns it on another thread.
- The keep-out discs published each cycle are assembled by the control loop, already inflated. The Runner cannot rebuild them because the hazard assembler holds the stale-track state.

## Parallel model batch

- `ParallelModelBatch` runs the keypoint model and the robot blob model on one worker thread each. `update()` dispatches the image and blocks until both finish.
- The wait is bounded at 1 s so a hung model cannot stall the main loop. On timeout the missing model's result fields stay default-constructed (empty keypoints). A late worker drops its result when the request id has moved on.
- A worker copies the image and camera info together under `mutex_`, so the pair always belongs to the same request. The robot blob model takes only the image. All request, result and done-id fields are guarded by `mutex_`.
- The image is a shallow `cv::Mat` copy. Each worker takes its own, so the pixel buffer stays alive when `update()` times out and the caller frees its image.

## Frame and size conventions

- `RobotDescription.size` is the footprint. For labels listed in `[robot_filter.robot_size_meters_per_label]` it is the known dimensions, not the detector's box. The measured box is logged to the `detected_size` diagnostics channel instead.
- `pose` is the point control tracks, our robot's axle. Hazard and clearance checks measure from the footprint centre instead, through `footprint_center()`. The offset is in the body frame (x forward, y left), in metres from `pose`, set per label from `[robot_filter.footprint_center_offset_per_label]`, and zero where the pose is the centre.
- Fiducial board layout frame: the board origin (the grid's minimum-x, minimum-y printed corner) is composed from the configured corner, offsets and orientation. The layout frame is centred on the field with the printed board's x and y and z into the floor. `compute_field` turns it into the published field frame.

## Field filters

### Calibrated filter

The match-day path. Fitting the mat outline at match time depends on segmenting a floor that has a robot on it and on the near corners being in frame, which at a cage mount they may not be. A cage measured beforehand has neither problem.

The line fit still runs at every startup, against the same frame, purely as a seating check. Nothing else can notice that the fixture did not seat or that the cage got bumped: a clamped camera has no visual odometry to disagree with. Above `max_seating_error_px` it warns and publishes the stored calibration anyway, which is the answer we trust.

### Fiducial filter

- Mode for a venue nobody has surveyed. It depends on nothing we have to segment and on no assumed mat size, so a clipped, cluttered or badly segmented outline costs it nothing.
- Put the board at a near corner. A 65 mm marker needs about 18 px of edge to decode; at the far mat corner foreshortening leaves it 8 px, and scaling the markers does not rescue that. The near corners are also the ones a cage-mounted camera cannot resolve as mat corners, so the board covers that weak spot. Numbers: `docs/experiments/perception/rgb_homography/rgb_camera_migration_2026-09-09.md`.
- Marker ids are reversed within each row (`floor_board_ids`). The physical board numbers markers right to left within a row; `cv::aruco::GridBoard` fills them left to right. Pairing them the wrong way round gives a reflection rather than a rotation, and a reflection still decodes, so everything upstream looks like it is working. It is the same class of failure as the mirrored robot tag mesh. Measured cost: `docs/experiments/perception/2026-11-15_fiducial_detector_parameters.md`.
- It replays: detection runs off `camera_data.rgb`, so a recording whose first frames contain the board carries everything needed to re-derive the field transform later. Record ten seconds with the board down before pulling it, every time.
- Gamma stretch: arena lighting is often dim enough that a raw frame decodes zero tags because marker contrast is crushed into shadow. The stretch is monotonic and per channel, so corner geometry and the recovered pose are unchanged.

### Free-roam mask model

Selects field pixels with classical heuristics instead of a segmentation model, so `PointCloudFieldFilter` can fit whatever flat surface the camera points at. Two selectors are ANDed:

- Center ROI: a centered rectangle covering `roi_fraction` of the frame (1.0 is the full frame).
- Seeded color match: the median Lab color of a small center patch is the reference and the whole frame is thresholded per channel against it. Lighting gradients and shadows mostly move L, so `tolerance_l` is set very loose while `tolerance_ab` stays tighter. Downstream RANSAC rejects same-colored non-plane pixels, so the mask only needs to be roughly right.

## Static detection gate

Suppresses robot-blob detections that hold the same field position for seconds on end. Arena floor graphics are fixed in the field frame; robots are not. Detections are tracked in field coordinates, so camera motion does not make scenery look mobile, and the ones that never move are dropped. Measurements: `docs/experiments/perception/2026-08-29_static_detection_gate.md`.

- Known cost: an immobilised opponent that stops for `min_dwell_seconds` looks like scenery and is suppressed. That is what `min_dwell_seconds` trades.
- A cluster loses its static verdict the moment an observation lands outside `static_radius_meters` of its running mean, so a robot that stops and then moves again is picked straight back up.
- Robot blobs only. Our own robot's keypoints come from a separate model and are not gated.
- The cluster statistic is the running mean distance from the cluster mean, not the maximum. Both statistics stop averaging past `responsiveness_window` and start tracking, so a robot parking on a logo can still move the verdict after thousands of frames.

## Keypoint heights

Keypoint heights correct the radial bias of flat-plane projection. Heights are per robot label with a shared default for labels without an entry. A height of 0.0 disables the correction.

`Keypoint::height_above_plane_m` is the height `KeypointHeightGate` measured from depth. It is empty when nothing measured it, and consumers then fall back to the configured per-label height. Projecting with the measured value removes the assumption that every robot stands exactly the configured height tall.

## Motion estimator seam

- `MotionEstimatorInterface` is the propagation seam inside `RobotFrontBackFilter`. It owns per-track motion state between the association front-end (keypoint conversion, blob merging, FrameId assignment) and the emitted `RobotDescriptions`. Implementations swap the state estimation strategy (dead reckoning vs Kalman) without touching association.
- Call pattern mirrors `RobotFilterInterface`: `predict()` at control rate, `update()` at perception rate, and the two are not in lockstep.
- Render lead: `coast()` renders `render_lead_s()` past `now`. Consumers steer by a state that has to describe the field when the command they are computing reaches the wheels, not when the shutter closed. 0 means render at `now`.

### Kalman arm

- Non-OURS tracks run a 6-state linear Kalman filter [px, py, vx, vy, theta, omega]: a constant-velocity translation block and an independent constant-yaw-rate heading block, each driven by its own continuous white-noise acceleration.
- Keypoint measurements correct position and heading, blob centroids position only. Position is chi-square gated, with reinitialization after repeated rejects.
- A heading more than pi/2 off the track is a front/back flip and corrects nothing; a run of flips re-initializes the heading block.
- Unmeasured tracks predict forward with growing covariance up to `max_coast_s` of track age, then hold position and heading. The opponent horizon is `opponent_max_coast_s` (`docs/experiments/control/2026-10-15_opponent_max_coast.md`).
- Measurements older than the track state (perception latency) rewind to a stored snapshot, correct at the measurement's own stamp, and re-propagate.
- The Python mirror in `auto_battlebot/perception/opponent_filter.py` must change with this arm. `tests/test_kalman_motion_estimator.cpp` holds the two to a golden fixture.
- Our-robot track has two modes. `DEAD_RECKONING` applies the last commanded velocity directly, matching `DeadReckoningMotionEstimator`. `EKF` runs a 5-state EKF [x, y, theta, v, w] propagated through `JigPlantModel` with the issued-command history (normalized stick space, from `CommandFeedback::stick_commands`) read at t - delay, corrected by keypoint poses (position and heading) or blob centroids (position only). A heading innovation past pi/2 falls back to the position rows, because the front/back keypoint converter can flip and folding a flipped heading in would spin the whole state.
- `coast()` renders every track advanced to the control-loop clock plus `render_lead_s()`, so the emitted state moves between perception frames instead of freezing at the last `correct()`.
- Rewind history: 64 snapshots. At the 250 Hz control rate that is roughly 240 ms, because `coast()` commits an opponent snapshot every tick and a 30 Hz frame burns about nine slots. It covers the measured 63 ms perception latency with room to spare; `num_rewind_missed` reports when it does not.
- Estimator config: `opponent_max_coast_s` and `max_coast_s` defaults. `max_coast_s` sits at the measured p90 to p95 of perception dropout gaps; predicting multi-second gaps with a short-horizon model is fiction.
- Plant fit status: the plant fit has not passed acceptance (heading misses by 4x at the 400 ms coast horizon), so production configs stay on `DEAD_RECKONING`. Sim and playback experiments opt in through the extends chain.

## Time

`ClockInterface` is logical (control) time in seconds: derivative terms, prediction horizons and message stamps. Wall-clock time for profiling, loop pacing and hardware timeouts must keep using `std::chrono` directly. The Runner sets the clock from the camera frame timestamp each tick (see `ManualClock`), so one source of time drives the whole pipeline and sim and playback runs are deterministic.

## Fiducial accumulation window

The Runner keeps the mask model running for 60 frames (one second at 60 Hz) on a field-filter request. That is long enough for a fiducial board to accumulate and short enough that a request with no board in frame stops running the mask model.

## Height gate

The gate measures relief (the median height of the box's inscribed circle over the median of a surrounding ring), plus a ceiling on absolute height above the fitted plane.

- Relief, not absolute height, decides flatness: if the fitted plane has drifted or the whole area sits above it, a flat graphic there is still flat and goes.
- The ceiling uses absolute height because relief alone admits walls and people. A metre off the floor is not a robot.
- Blob keypoints sit at the ends of the box midline, which land on the floor beside a robot. Sampling the inscribed circle reads the robot where sampling at the keypoints reads the floor.
- A missing depth frame abstains instead of dropping detections, so a dropped frame never blinds the pipeline. A model that produces no boxes cannot be gated, only annotated.
- A group survives whole. One keypoint over a hole in the depth map still measures from its partner, and both keep the group height, because a half-dropped front/back pair is useless downstream.
- `ModelResultStamped::boxes` is indexed by `Keypoint::detection_index`. It is empty for models that produce no boxes. It has an entry for every detection, including ones that yielded no keypoints, so the index stays aligned with what the detector emitted and `box_for` returns the right detection's box.
- Test fixture: the field transform puts the plane 3 m down the optical axis with the normal parallel to it, so keypoint height is exactly `3.0 - depth` at every pixel. Fixture boxes are the keypoint bounding rectangle inflated by a pad so the inscribed circle covers the midline like a real detection box.

## Opponent filter test scenarios

- The opponent Kalman arm is pinned explicitly in tests because the config default is HOLD, which pins every opponent at its last measured pose.
- The control loop runs ahead of the camera: a track coasts to last_t + 50 ms, then a measurement stamped at last_t + 33 ms arrives. This is the rewind case.
- After a 1 s unseen stretch the heading prior is wider than the flip test, so a measurement 120 degrees off is a turn the track missed, and the first one is taken.
- Heading tests use straight-line translation with an independent constant yaw rate, which is exactly the filter's model, so tolerances can be tight.
- The our-robot arc test command (v about 1.3 m/s, w about 0.95 rad/s under the Stage A gains) is a gentle arc.

## Notes moved from `src/` comments (2026-11-15)

Each entry names the file and the code it described.

### `src/keypoint_filter/static_gate.cpp`

Beside `const double weight =`:

Cap the averaging weight so both statistics keep tracking instead of going numb. Below the window this is an exact running mean; above it, an exponential one with that time constant.

### `src/keypoint_filter/static_gate.cpp`

Beside `Eigen::Vector3d sum = Eigen::Vector3d::Zero();`:

The group's field position is the centroid of its model_results projected onto the plane. Using the measured height where the height gate supplied one keeps a tall robot's footprint from being pushed outward by the flat-plane assumption.

### `src/main.cpp`

Beside `auto hazard_assembler = make_hazard_assembler(*class_config.field_filter, clock);`:

The control loop owns the filter/target/navigation/transmit half. A threaded driver runs it on its own thread, so nothing else may touch those components after Runner::initialize().

### `src/robot_filter/ground_truth_robot_filter.cpp`

Beside `const double now = result.header.stamp;`:

Field-frame velocity from the pose delta since the last frame. Consumers that close a loop on speed (MotionProfileNavigation) read velocity rather than differencing poses themselves, because only the filter knows whether a pose is a fresh measurement or a propagated one.

### `src/robot_filter/kalman_motion_estimator.cpp`

Beside `constexpr double kFlipTestMaxVariance = (M_PI / 4.0) * (M_PI / 4.0);`:

The flip test only means something while the heading's own 2-sigma is inside its pi/2: past that, a measurement more than pi/2 away is as likely a turn the track did not see as a flip.

### `src/robot_filter/kalman_motion_estimator.cpp`

Beside `ekf::propagate_covariance<5>(track.covariance, jacobian,`:

Process noise covers the whole step, held or not: time passing without measurements is uncertainty regardless of whether the pose is allowed to move.

### `src/robot_filter/kalman_motion_estimator.cpp`

Beside `track.covariance(4, 4) = config_.plant->k_ang * config_.plant->k_ang;`:

Full-stick yaw rate as the initial yaw-rate sigma: the fitted gain is the scale of what the drivetrain can be doing while unobserved.

### `src/robot_filter/kalman_motion_estimator.cpp`

Beside `if (!input.keypoints.empty()) {`:

Heading is the input the motion profile brakes on: a rejected keypoint yaw leaves theta running on the plant model alone, so record the state and the innovation that was thrown away. Without both sides here a diverged theta is indistinguishable from a bad measurement.

### `src/robot_filter/kalman_motion_estimator.cpp`

Beside `double speed_measurement = 0.0;`:

Forward speed differenced against an earlier accepted pose. The state's v row is otherwise unobserved, so it tracks the plant model rather than the robot, and the motion profile brakes on that number. Only attempted alongside an accepted heading: the difference has to be projected onto a direction, and a rejected yaw is not one.

### `src/robot_filter/kalman_motion_estimator.cpp`

Beside `const double floor_var =`:

Independent position noise at both ends, propagated through the divide, held above a floor for the part that is common-mode.

### `src/robot_filter/kalman_motion_estimator.cpp`

Beside `if (has_speed || !track.has_speed_ref) {`:

Only move the reference once its baseline has been spent, or when there is none to spend. Advancing every frame would pin the baseline at one frame and put the speed row back in the noise it was widened to escape.

### `src/robot_filter/kalman_motion_estimator.cpp`

Beside `const double target = now + render_lead_s();`:

Two lags separate the shutter from the wheels, and the render has to cover both. `now` is already past the frame stamp by the perception latency, and a command computed from this render bites another plant.delay_s later. Rendering at `now + delay_s` puts every track where it will be when the command lands, which is the only state target selection and navigation can steer by without aiming at the past. The extra span is not extrapolation into unknown commands: JigPlantModel reads each substep's command at t - delay_s, so integrating to now + delay_s consumes exactly the commands already issued up to `now`, the ones in flight.

### `src/robot_filter/kalman_motion_estimator.cpp`

Beside `double worst_span = std::numeric_limits<double>::infinity();`:

The smallest span any track is propagated over. Both failure modes below emit an estimate frozen at the shutter while looking exactly like a healthy filter that is not moving, so this is worth naming rather than inferring: a span past kMaxIntegrationStepS short-circuits to "hold position", and a non-positive span propagates nothing at all. Both mean the control clock and the frame stamps are not on the same timeline, which is what SVO playback does under SystemClock. The minimum is the right reducer for that question, and it goes out in update()'s diagnostics so a recording answers "did anything propagate, and by how much" without needing a rebuild. It is not the render lead: opponent tracks commit every control tick and so report one tick, while the our-robot track only commits in update() and so reports a whole frame period plus the lead.

### `src/robot_filter/kalman_motion_estimator.cpp`

Beside `for (const auto &[frame_id, track] : our_ekf_tracks_) {`:

Rendered on a copy, like the dead-reckoning arm: the committed state only advances in update(), so measurements always fold in at or after the track stamp and the our-robot arm needs no snapshot rewind. propagate_unwrapped skips the finite-difference Jacobian (10 extra plant integrations) that a covariance-free render never uses.
