# Config profiles

What each file under `config/` is for and where its non-obvious values come from. For the value a
profile loads and which file set it, run:

```bash
./build/auto_battlebot -c config/<profile> --print-config
```

Every field the parsers read is listed, defaults included, each tagged with `file.toml:line` or
`default`. The same text is in each MCAP's `auto_battlebot` metadata record under
`resolved_config`, and the dashboard's System tab shows it live. Merge rules (`extends`, `from`,
type swaps) are in `docs/reference/config/config_extends.md`.

## Bases

| File | Role |
| --- | --- |
| `_common.toml` | Root of every chain. Perception, filter, target, nav, and transmitter defaults for Mrs Buff Mk3 |
| `_desktop.toml` | x86 dev machine with a ZED: x86 engines |
| `_zed_box.toml` | ZED Box Mini with a ZED X One S (`docs/setup/zed_box_mini.md`) |
| `playback/_playback.toml` | SVO replay on the desktop |
| `playback/_video_playback.toml` | Replay of an MCAP that carries `/camera/video` |
| `simulation/_simulation.toml` | Lockstep sim: `SimRgbdCamera` over a socket, sim transmitter, `SteppedControlLoop` |

Model sections list engine candidates rather than one path. `EngineSelector` loads the first one
that deserializes on the machine's GPU, so a box with a different compute capability needs a new
engine build, not a config edit. Candidates only ever list builds of the same trained model:
falling back to a different model would change detection behavior between machines. `_desktop`'s
field model has no sm86 build yet, so it is the one model that fails on an RTX A6000.

Engine candidate tables look like this, and parse as a nested table so any model section can embed one:

```toml
[keypoint_model.engine]
candidates = [
    "data/models/pose_x86_64_sm89.engine",
    "data/models/pose_aarch64_sm87.engine",
]
```

Order is a preference. `EngineSelector` asks TensorRT which candidate deserializes on this GPU and never decodes the `sm` tag in the filename, so listing engines for architectures this machine lacks is expected and cheap.

`EngineSelector::select` logs its outcome once: a success line, or one error block listing every rejected candidate and why. spdlog fans out to stdout and the MCAP `/log` topic (`setup_logging` in `src/logging/logging.cpp`), so one call covers both. `describe_cuda_device` returns a diagnostic string instead of throwing when CUDA is unavailable, because it only feeds log lines.

## Section files

Loaded with `from` (`config_extends.md`). Each holds one fact used by more than one profile.

| File | Loaded by |
| --- | --- |
| `robots/mrs_buff_mk3.toml`, `robots/mr_stabs_mk2.toml`, `robots/mr_stabs_mk2_geometry.toml` | `[plant]`, see `robot_plant_files.md` |
| `transmitters/mrs_buff_mk3_opentx.toml` | `_common` |
| `transmitters/mr_stabs_mk2_opentx.toml` | `mr_stabs_mk2_desktop` |
| `transmitters/mr_stabs_mk2_espnow.toml` | `mr_stabs_mk2_aer_zed_box`, `mr_stabs_mk2_fid_zed_box`, `mr_stabs_mk2_sysid_zed_box` |
| `transmitters/playback.toml` | `playback/_playback`, `playback/_video_playback` |
| `navigation/mr_stabs_mk2_rl.toml` | `mr_stabs_mk2_aer_zed_box`, `mr_stabs_mk2_fid_zed_box` |
| `navigation/mrs_buff_mk3_rl.toml` | `mrs_buff_mk3_zed_box`, `mrs_buff_mk3_aer_zed_box` |
| `navigation/motion_profile.toml` | `_common`, `auto_battlebot/rl/lockstep_eval.py` |
| `field_filters/fiducial_1p5m.toml` | `mr_stabs_mk2_fid_zed_box`, `mr_stabs_mk2_sysid_zed_box` |
| `keypoint_models/mr_stabs_mk2_apriltag.toml` | `mr_stabs_mk2_sysid_zed_box`, `playback/mr_stabs_mk2_sysid_playback` |

`playback/svo_start_frames.toml` is a data table, not a section file: the first match frame of each
SVO, read by `training/model_eval/make_eval_dataset.py`. The NHRL 2026-05-02 frames land the
playback transmitter's field init on match state.

## Shared values in `_common.toml`

### Keypoint model

- `reference_offset_m_per_label`: the robot's reference point ahead of its labelled front/back
  keypoint midpoint, along the heading, from the CAD keypoints in `training/synthetic/config.toml`.
  Mr Stabs: the axle, the body origin the MuJoCo plant and RL policy use; the sysid YOLO fills
  measured 0.046-0.048 m. Mrs Buff Mk3: the footprint centre, where the sim and the RL environment
  centre her box.

### Keypoint filter gates

The static gate runs first and sees every detection; the height gate runs after and annotates what
survives.

- `[keypoint_filter.height]` measures relief against the floor around each detection: the 90th
  percentile of depth inside the box's inscribed circle minus the 25th percentile of a ring around
  it. The 0.035 m threshold and the sweep behind it are in
  `docs/experiments/perception/2026-08-29_height_gate_local_relief.md`. Survivors carry their
  measured height, which replaces `robot_filter.keypoint_height_meters` when projecting them onto
  the field.
- `[keypoint_filter.static]` is off on ZED 2i profiles: on the 2026-08-29 MassD recording it is a
  worse trade than the height gate at every setting and stacks badly on top of it
  (`docs/experiments/perception/2026-08-29_static_detection_gate.md`). `static_radius_meters` is
  compared against a cluster's mean deviation, not its worst frame. The gate cannot tell an
  immobilised robot from a painted one; a robot driving into a suppressed cluster raises its spread
  and switches suppression off there until it leaves.

### Robot filter

- `our_robot_hold_window_s`: drop an unmeasured our-robot track after this long without a keypoint
  confirmation instead of predicting it forward.
- `[robot_filter.motion_estimator]`: `KalmanMotionEstimator`, opponents on the 6-state filter. The
  heading block (`opponent_yaw_accel_psd`, `opponent_heading_sigma_rad`), `opponent_max_coast_s` and
  `opponent_position_sigma_m` are grid-searched on the 2026-10-15 Mr Stabs Mk2 sysid sessions
  against AprilTag truth: `docs/experiments/control/kalman_filter/opponent_filter_evaluation.md`.
- `[robot_filter.robot_size_meters_per_label]`: footprints off the CAD models in
  `training/synthetic/render_sprites.py`'s upright pose; x/y match
  `simulation/assets/sprites/sprites.json`. Without them, size comes from the detector box, and a
  loose box on the house bot inflates the keep-out built from it. The detector's measurement is
  logged to the `detected_size` diagnostics channel.
- `[robot_filter.footprint_center_offset_per_label]`: Mr Stabs's MuJoCo collision hull runs
  0.025 m behind the axle to 0.1105 m ahead, so its footprint centre is 0.043 m ahead of the pose.
  The hull was measured once, resting nose-down on the skid the way the sprite was rendered.

### Target selector

- `retarget_improvement_m = 0.1`: the safest-point solver's plateau is flat, and with no hysteresis
  the goal moved 10-13 times a second on the 2026-08-23 recordings. A new candidate has to beat the
  held one by this much.

### Control loop and runner

- `ThreadedControlLoop` on hardware. `SteppedControlLoop` is for reproducible playback, sim, and
  rollback; `watchdog_timeout_ms` is inert under it. `rate_hz = 0` runs one cycle per perception
  frame.
- `runner.max_loop_rate_hz` is double the camera FPS.
- `ui.label_colors` match the class colors in the training dataset YAMLs; unlisted labels get a
  generated color.

## Transmitters

- OpenTX channel map: config index = radio channel label - 1, so `linear_channel = 0` is CH1,
  `angular_channel = 1` is CH2, `trainer_enable_channel = 5` is CH6. No transmitter maps an init
  button; field re-init is a touchscreen action.
- `TankDriveProcessor` sends left/right motor commands; `DifferentialDriveProcessor` sends
  linear/angular for a radio model that mixes them itself. Both Mr Stabs radio models mix, so Mr
  Stabs sends body axes.
- The Mrs Buff CRSFR model inverts CH2 (right motor) on output, so its readback is inverted back
  (`invert_angular_channel_readback = true`). The Mr Stabs model sends raw Ele/Ail.
- `velocity_saturation_limit = 1.0` keeps `|linear| + |angular| <= 1`.
- ESP-NOW (`mr_stabs_mk2_espnow`): commands go out over the dongle alone. The handset talks only to
  the robot: it arms, holds the link failsafe, and SA hands the wheels to these commands. Feedback
  and the SA state come back in the robot's telemetry. `max_motor_rpm_per_s = 2000` against
  `max_motor_rpm = 1500` is a normalized slew of 1.33/s. Without `max_motor_rpm_per_s` nothing
  rate-limits the command on the way to the wheels.

## ZED Box (`_zed_box.toml`)

The ZED X One S is monocular: no depth, no visual odometry. Everything that read depth changes with
it:

- the field filter solves off the fiducial board instead of fitting the point cloud
- the height gate abstains on the empty depth image (`reject_enable = false` says so). The static
  gate is enabled, but the runner tick does not call it
- `/field_points` stops publishing

The camera runs the full sensor; `RES_960x600` bins it for up to 120 fps. The SVO bitrate is the ZED
SDK's own H.264 rate at this mode, 31 Mbps measured on the box (about 13.5 GB per hour), until a
live-vs-replay tag comparison shows the 15 Mbps default loses nothing.

## Robot profiles

- The switcher offers `*_zed_box` profiles only. `config/default_profile` is `mrs_buff_mk3_zed_box`.
- `mr_stabs_mk2_desktop`: the x86 bench with a ZED 2i, on `_common`'s `MotionProfileNavigation`.
- No profile drives the ZED 2i handheld (`handheld-autonomy`). A deploy there selects a ZED Box
  profile, and `ZedOneRgbCamera` cannot open a ZED 2i. Bringing it back needs a `_handheld` base
  with aarch64 engines; the stereo path (`ZedRgbdCamera`, `PointCloudFieldFilter`, `HeightGate`)
  is still built and reached by `mr_stabs_mk2_desktop`.
- `mr_stabs_mk2_fid_zed_box`, `mr_stabs_mk2_aer_zed_box`: ESP-NOW transmitter, fiducial and AER
  fields, `RlPolicyNavigation` from `navigation/mr_stabs_mk2_rl.toml` on bundle `rl_nav_p4`,
  retrained from p3 with sensing dropouts off. It trained in the sim's 2.4 m arena; this field is 1.5 m. It has no
  HazardAvoidance layer: walls and hazards reach it only through its observation.
- `mr_stabs_mk2_sysid_zed_box`: system identification sessions (`docs/guides/sysid_guide.md`). The
  driver drives by hand in the 1.52 m test box under a tripod-mounted ZED X One S; the app records
  the AprilTag poses and ESP-NOW telemetry. The dongle pings the robot every 10 ms on its own, which
  is enough for 100 Hz telemetry on `/robot/espnow_telemetry`, so the handset needs no USB link.
  Autonomy starts off, so flipping SA cannot put zeros on the wheels.
- `keypoint_models/mr_stabs_mk2_apriltag.toml`: `tag_size_m` is the edge of the black square in
  CAD; measure the print before the first session. Front and back keypoints straddle the axle
  midpoint, so the tracked pose is the axle. `axle_height_m` is the wheel radius from
  `simulation/assets/robots/mr_stabs_mk2/mass_properties.toml`; `rest_pitch_rad` is the robot
  resting nose down on the wedge tip (`.../collision/collision.toml`). The tag mountings come from
  `mass_properties.toml` (`[tags.N]`): `rotation` is `R_body_tag` row-major, and with
  `translation_m` it maps the OpenCV marker frame (x right, y up in the printed image, z out of the
  face) into the body frame (FLU at the axle midpoint). 76 is on top and 41 underneath.
- `mrs_buff_mk3_zed_box`, `mrs_buff_mk3_aer_zed_box`: `RlPolicyNavigation` from
  `navigation/mrs_buff_mk3_rl.toml`, commands through
  the OpenTX transmitter (the handset relays them over ELRS through the trainer port), plant from
  `robots/mrs_buff_mk3`. Bundle `rl_nav_mk3_c_near` (queue job 58 on megamind, 2026-10-25), exported
  from `runs/rl_nav/rl_nav_mk3_c_near_s15_2026-10-25_18-11-33`. It trained with the weapon off,
  against a MuJoCo plant fitted to the jig plant rather than to Mk3 recordings, and has no
  HazardAvoidance layer.
- `field_filters/fiducial_1p5m.toml`: `accumulate_frames = 1` because the AprilTag corner fit lands
  at 0.55 px from one frame and its error is bias, so stacking frames buys nothing. One frame stalls
  the loop about 530 ms on the ZED Box, once per lock.

## Playback profiles

- `playback/_playback`: points `svo_file_path` at one SVO. Recordings it has replayed:
  `data/svo/1729981242.4295187.svo2`, `data/svo/2026-03-27T21-48-29.svo2`,
  `data/svo/mini_bot_2024-10-26T13-29-41.svo2` (needs `position_tracking = false`), the
  `data/svo/tests/2026-04-*` and `2026-05-01T*` bench clips (`11-22-34` field finding, `11-27-37`
  autonomous driving, `14-17-26` 1080p foil safety cage), NHRL 2026-05-02
  (`data/svo/NHRL_2026-05-02/`), Open Sauce 2026-07-18 (`data/svo/2026-07-18/`), MassD 2026-08-29,
  and `data/svo/tests/2026-08-28/`.
- Playback frame stamps are not wall-clock, so `SystemClock` cannot say how old the newest frame is:
  unrebased they sit on the recording's clock, and rebased they carry an offset captured during the
  startup transient. `CameraFollowingClock` anchors to each frame and advances with the wall clock
  between them, so `now - frame_stamp` is the real processing age.
- MCAP recording is off in replays. `McapRecorder` refuses to record over the file the camera
  reserved, but there is no reason to write a second file during replay iteration.
- `playback/_video_playback`: an empty `calibration_id` rectifies with whatever calibration the
  recording names. Set it to re-rectify old footage against a revised calibration.
- `playback/mrs_buff_mk3_playback`: hazard files for the venues are
  `config/hazards/massd_arena_north_south.toml` and `config/hazards/aer_arena_east_west.toml`.
- `playback/mr_stabs_mk2_playback`: `MotionProfileNavigation` and the EKF from `_common`, on the
  distilled Mr Stabs plant.
- `playback/brettzone_cage_high`: NHRL's fixed `Cage-N-Overhead-High` footage, re-wrapped by
  `scripts/nhrl_to_recordings.py`, which writes the file on the set's first clip; `MANIFEST.md` in
  the set's `data/saved_recordings/` directory lists the rest. One fixed 1080p
  view, no depth, no visual odometry, so the field filter fits the outline (no fiducial board in
  broadcast footage). The intrinsics are backed out of NHRL's stated field of view
  (`camera_calibrations.md`) and the homography inherits that error linearly, so ranges are
  indicative. The detectors trained on ZED frames at a different mount, so detections report
  transfer rather than accuracy.
- `playback/mr_stabs_mk2_sysid_playback`: replays a Mr Stabs Mk2 ZED Box MCAP through the sysid
  AprilTag model, for drives recorded on a match profile that never ran it. The output MCAP carries
  `/apriltag/robot_tags` with the original `image_stamp_ns`, so the tags join the source recording's
  stick channels. `scripts/calibration/replay_sysid_tags.py` drives it one recording at a time.
  Decoding is not real time, the field is `mr_stabs_mk2_aer_zed_box`'s (which the 2026-10-10 drives
  ran), and MCAP recording is on with no holding-dir pruning, since the replays land in
  `data/recordings` next to the originals.
- `experiments/mrs_buff_mk3_label_playback`: label-export replay on the Mrs Buff profile so the
  filter has a their-robot label mapping. The bare playback chain never sets one, so every OPPONENT
  detection would stay uncategorized and no their-robot track would be published.

## Simulation profiles

All pair with a sim-side config under `simulation/` and run through `./scripts/run_simulation.sh`.

- `_simulation`: `SteppedControlLoop`, because `SimConnection`, `ManualClock` and the ground-truth
  poses are not thread safe, and stepping makes a run reproducible. 250 Hz keeps the threaded loop's
  cycle rate, in sim time.
- `kinematic_sim`: headless control tuning with `simulation/kinematic_sim_server.py`. Perception is
  bypassed and ground-truth poses come from the sim, so `MotionProfileNavigation` runs unchanged.
  `GroundTruthRobotFilter` maps poses by order (GT[0] is OUR_ROBOT_1, the rest are theirs). Actuation
  latency is modelled in sim ticks by the server, so `SimTransmitter.command_delay_ms` stays 0 (it
  is wall-clock based and would not survive accelerated runs). `max_loop_rate_hz = 0` free-runs, paced
  by lockstep with the server, and `ManualClock` takes time from the sim frame stamp, so the
  controller's dt is right at any loop speed.
- `mr_stabs_mk2_kinematic_sim`: the sim's second `[[opponents]]` entry is the house bot, reported as
  NEUTRAL_ROBOT_1 so the hazard assembler makes it a keep-out instead of a target.
- `mr_stabs_mk2_kinematic_sim_sensing`, `mrs_buff_mk3_kinematic_sim_sensing`: the sim's sensing
  model delivers keypoints with
  latency, dropouts, noise and heading flips, and the real `RobotFrontBackFilter` and
  `KalmanMotionEstimator` run on them. They extend `_simulation`, not `kinematic_sim`, to keep
  `_common`'s robot filter, so they restate the sim parts of `kinematic_sim` and the robot parts of
  the robot's kinematic sim profile. One label per sim ground-truth slot: our robot, then
  `[[opponents]]` in order. The Mk2 file sets the sim opponent's footprint, which a detector box
  would report; without it the filter sizes the opponent from keypoint spacing (about 0.1 m).
  `auto_battlebot/rl/lockstep_eval.py` extends these per scenario.
- `mr_stabs_mk2_kinematic_sim`: Mr Stabs on `MotionProfileNavigation` against the MuJoCo
  plant, to check a `[plant]` table distilled from the MuJoCo fit
  (`auto_battlebot/calibration/distill_mujoco_plant.py`). `auto_battlebot/calibration/sim_stop_test.py`
  drives it headless, one run per goal.
- `mrs_buff_mk3_kinematic_sim`: `MotionProfileNavigation`, which Mrs Buff ships with. It runs the hazard
  tangent and goal-legality layers.
- `mr_stabs_mk2_rl_sim`, `mrs_buff_mk3_rl_sim`: `RlPolicyNavigation`. Swap the bundle for any export
  under `data/models/nav_policies/` (`auto_battlebot/rl/export_checkpoint.py`). The Mk3 bundle is queue
  job 64's 07:17 checkpoint (`rl_nav_mk3_c_margin2`, weapon off). With the hazard barrier on and its
  own `[plant]`, it scored 0.706 hits per hit taken over the 128 Mk3 lockstep scenarios against
  `MotionProfileNavigation`'s 0.011, with no holes (`data/rl/mk3`). That `[plant]` is the
  jig plant from before `cd5d2ba7` zeroed the lags, deadzones and coupling. The barrier's stopping
  horizon is `tau_lin_d + delay_s`: 0.252 s there, 0.052 s on `robots/mrs_buff_mk3`. Delete the
  table to run on the robot file's.

## Invariants kept by hand

- `simulation/*` profiles with a fixed field: `field_filter.size_x` / `size_y` match the sim
  config's `[arena]`.
- `playback/brettzone_cage_high.toml` is generated; change `scripts/nhrl_to_recordings.py` along with it.

## Home path expansion

`expand_home` in `include/config/profile_selection.hpp` expands a leading `$HOME` or `~` against the current user's home directory and leaves other paths as they are. It throws when a home-relative path cannot be resolved. Returning the unexpanded literal would write it as a relative path under the working directory. Under a systemd service with a stripped environment that is inside the repo, and the next deploy wipes it.
