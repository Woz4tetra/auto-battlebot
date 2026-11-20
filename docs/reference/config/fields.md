# Config fields

What each config field means, per section. Values and their source per profile come from
`--print-config` (`docs/reference/config/profiles.md`); this page holds what a value cannot say.

## `[transmitter]`

### Drive processor fields

`OpenTxTransmitter`, `EspNowTransmitter` and `PlaybackTransmitter` run the command through a drive
processor; `SimTransmitter` applies the same saturation itself.

| Field | Meaning |
| --- | --- |
| `drive_processor` | `TankDriveProcessor` sends left and right motor commands; `DifferentialDriveProcessor` sends linear and angular for a radio-side mixer. On ESP-NOW the firmware puts the two values where ch0/ch1 land, so it must match how the handset model mixes |
| `velocity_saturation_limit` | Combined budget `\|linear\| + \|angular\| <= limit`. Angular takes priority and linear fills the headroom. 0 clamps each channel to [-1, 1] on its own. The sim has no drive processor, so without it the simulated robot keeps forward authority in a turn the real one has lost in the mixer |
| `zero_deadzone_percent` | Input magnitude below which the output is forced to zero |
| `lifted_deadzone_percent` | Smallest non-zero output magnitude once the zero deadzone is exceeded |
| `max_motor_rpm`, `max_motor_rpm_per_s` | Linear slew limit: the normalized linear command changes by at most `max_motor_rpm_per_s / max_motor_rpm` per second. The limit applies only while the autonomy switch (SA) is on; with SA off the command passes through, so the robot never lags the sticks. The ramp baseline is the last tick SA was off. Unset means no limit |

Under `CompositeTransmitter`, an ESP-NOW child's drive processor fields (`drive_processor`,
`velocity_saturation_limit`, both deadzones, both `reverse_*_channel`) must equal the OpenTX
child's, or the two transports disagree by a fixed offset. The parser rejects a mismatch.

### `OpenTxTransmitter`

| Field | Meaning |
| --- | --- |
| `linear_channel` | RC channel for the first mixer output: left wheel under tank drive, linear under differential |
| `angular_channel` | RC channel for the second mixer output: right wheel under tank drive, angular under differential |
| `trainer_enable_channel` | RC channel of the trainer enable switch (SG). Autonomous motion reaches the wheels only while it is engaged, when the channel reads negative, and the linear slew limiter holds at standstill until then. SG must be mapped to this output (radio CH6) in the OpenTX model |
| `init_button_channel`, `init_button_threshold` | Channel whose value above the threshold counts as a field re-init press. Unset means no button |
| `invert_angular_channel_readback` | The radio model inverts the angular output after mixing in the trainer input (the Mrs Buff CRSFR model does this to its right motor, `firmware/config/CHANNEL_CONVENTION.md`). Sending is unaffected, since the trainer value goes through the same inversion as the sticks, but the read-back channels are mixer outputs. Left uninverted, the wheels read back with opposite signs and forward stick reports as a spin |

### `Esp32DiagnosticsOpenTxTransmitter`

`OpenTxTransmitter` plus the Mr Stabs Mk2 firmware's telemetry ring, polled over WiFi and recorded
on `/robot/esp32_diagnostics`.

| Field | Meaning |
| --- | --- |
| `esp32_host`, `esp32_port` | The robot's address, an IP literal. 192.168.4.1 is its access point; in client mode use the address the router reserves |
| `reconnect_period_s` | Wait after a failed poll. The ring holds two minutes of records, so a poll back within that loses nothing |

### `EspNowTransmitter`

Autonomous drive commands to Mr Stabs Mk2 over the ESP-NOW USB dongle, plus its telemetry back. It
carries no arm authority (`firmware/mr_stabs_mk2/README.md`).

| Field | Meaning |
| --- | --- |
| `dongle_serial_path` | A udev symlink (`install/install_udev_rules.sh`), not a raw ttyACM path: the handset also enumerates as ttyACM and the order is not stable |

### `CompositeTransmitter`

Fans one command stream out to several transmitters. The first child is primary and owns everything
read-side: status, behavior mode, and command feedback on a key collision. `[[transmitter.children]]`
is an array of tables so the order is guaranteed. Each entry may set `transmit_rate_hz`, the most
sends per second that child gets; 0 passes every send.

- `send()`, `enable()` and `disable()` fan out to every child. `update()` polls every child so their receive paths drain.
- A non-primary child is an accelerator. Its `initialize()` failure is logged and ignored: a dead ESP-NOW dongle costs latency, not control.
- The composite does no command preprocessing. Each child does its own scaling. The rate limit that makes two transports agree lives in the firmware, after the arbitration that picks between them.
- The control loop sends at 250 Hz. The OpenTX trainer path feeds a 150 Hz Crossfire link through serial CLI writes and gains nothing past that, while ESP-NOW takes every cycle.
- A capped child gets the newest command once its period has passed. A command it skips is held and delivered when the period runs out, from `update()` if no newer `send()` comes, and before `disable()`, so the final stop command always lands.
- Pacing is a hardware rate, so it runs on the steady clock. Only tests inject another time source.

### `SimTransmitter`

| Field | Meaning |
| --- | --- |
| `command_delay_ms` | Artificial command delay for lag experiments. Wall-clock based, so it does not survive accelerated runs |

## `[navigation]`

`type` selects the implementation. `PursuitNavigation`, `MotionProfileNavigation` and `RlPolicyNavigation` share the hazard fields, listed once under the first type that has them.

### `PursuitNavigation`

| Field | Meaning |
| --- | --- |
| `velocity_ramp_far_distance` | Distance (m) at and beyond which speed is `velocity_ramp_min_scale` times max |
| `velocity_ramp_near_distance` | Distance (m) at and below which speed is max |
| `velocity_ramp_min_scale` | Speed scale at distances >= far distance, 0..1 as a fraction of max |
| `brake_distance` | Coast-aware brake. Within this distance (m) of the target the commanded speed ramps linearly to zero, so the robot decelerates into the goal instead of coasting past it. Should cover the stopping lead (latency travel + coast distance). 0 disables (ram behavior: full speed into contact) |
| `wall_reverse_distance_m` | Distance from any field wall at which `linear_x` is reversed to back away. Unset turns the reverse off |
| `wall_reverse_min_speed` | Minimum speed (m/s) when wall reverse engages and no forward drive was commanded. Gives a real reversal when the robot was stationary or turning in place |
| `wall_heading_threshold_rad` | Heading must be within this angle of the nearest wall normal to trigger reversal |
| `angular_kp` | Proportional gain for angular control |
| `angular_kd` | Derivative gain for angular control. 0 is P-only |
| `angle_threshold` | Angle (rad) below which the robot drives forward |
| `lookahead_time` | Lookahead for target position prediction, seconds |
| `boundary_margin` | Minimum distance to keep from field boundaries, meters |
| `enable_hysteresis` | Turn-direction hysteresis, prevents dithering when the target is behind |
| `max_linear_command` | Cap on the magnitude of the normalized forward (`linear_x`) command, 0..1 as a fraction of full drive. The plant or transmitter scales it by max linear speed. Unset is no limit |
| `max_angular_command` | Cap on the magnitude of the normalized turn (`angular_z`) command, 0..1 as a fraction of full turn. Unset is no limit |
| `hazard_tangent_enable` | Steer at a tangent point on a blocking hazard's rim instead of at the true goal. Both controllers only chase a `Pose2D`, so this costs nothing in navigation |
| `hazard_tangent_max_iterations` | Times to re-test the substituted waypoint against the remaining hazards. A greedy one-hazard-at-a-time step, not a planner. When it does not converge, the speed cap and the reverse backstop cover it |
| `hazard_waypoint_clearance_m` | Clearance the tangent waypoint leaves beyond the keep-out radius. Aiming exactly at the rim arrives with zero clearance and grinds along the edge; the extra makes the run from the waypoint to the goal unblocked, so the true goal returns instead of the robot orbiting |
| `hazard_side_release_m` | Release the latched pass side once the direct run to the goal clears the hazard by this much. Without the latch the robot re-picks a side every tick and cuts back across the hazard |
| `hazard_prediction_horizon_s` | Advance a tracked hazard along its filtered velocity by this long before testing it. For a house bot moving across our path: reacting to where it is aims at where it was |
| `hazard_heading_threshold` | Heading must point within this angle (rad) of the hazard for the reverse to trigger |
| `hazard_reverse_min_speed` | Reverse speed floor while the hazard backstop is active, normalized command |

### `FixedVelocityNavigation`

| Field | Meaning |
| --- | --- |
| `linear_x` | Fixed linear X command, m/s |
| `linear_y` | Fixed linear Y command, m/s |
| `angular_z` | Fixed angular Z command, rad/s |

### `MotionProfileNavigation`

Requires the top-level `[plant]` table. See `docs/reference/control/plant_backed_control.md`.

| Field | Meaning |
| --- | --- |
| `deadzone` | Residual command deadzone the plant still shows, 0..1. Exact inverse-deadzone is applied to the final command when > 0. Distinct from the fit's `dz_lin_fwd` and `dz_lin_rev`. Defaults to 0 because the transmitter's `lifted_deadzone_percent` handles the physical deadzone upstream on the real robot, and the kinematic sim's residual deadzone is 0 |
| `accel_limit` | Rate limit (m/s^2) on how fast the reference speed may rise, for a clean launch and a bounded feedforward derivative. Braking is never rate-limited |
| `attack_terminal_speed_fraction` | Terminal speed while the driver has ATTACK selected, as a fraction of the plant's `k_fwd` |
| `run_away_terminal_speed_fraction` | Terminal speed while the driver has RUN_AWAY selected, as a fraction of the plant's `k_fwd` |
| `stop_distance_m` | Distance at which a zero-velocity mission is complete and the command is cut. Used only when the active terminal velocity is 0, since a drive-through mission never stops |
| `speed_kp` | Gain from speed error (`v_ref - v_actual`, m/s) to a normalized command |
| `speed_ki` | Integral gain on speed error. 0 is P-only |
| `angular_kp` | Proportional gain on heading error, rad/s per rad |
| `angular_kd` | Derivative gain on heading error, rad/s per rad/s. 0 is P-only |
| `angle_threshold` | Heading error (rad) below which the robot drives forward. Above it, the robot turns in place |
| `max_yaw_rate` | Cap on the yaw-rate reference (rad/s). 0 is no limit. Bounds rotation per control tick: at 30 Hz, 7.9 rad/s is 15 degrees per tick |
| `max_angular_command` | Cap on the magnitude of the normalized turn command, 0..1. Unset is no limit. Output-side safety clamp; `max_yaw_rate` is the one that shapes behavior |
| `enable_hysteresis` | Turn-direction hysteresis, prevents dithering when the target is behind |
| `boundary_margin` | Minimum distance kept from field boundaries, m |
| `max_linear_command` | Cap on the magnitude of the normalized forward command, 0..1. Unset is no limit |
| `wall_reverse_distance_m` | Distance to any wall at which `linear_x` is reversed to back away. Unset turns it off |
| `wall_reverse_min_speed` | Minimum reverse speed (normalized) when wall reverse engages |
| `hazard_tangent_enable`, `hazard_tangent_max_iterations`, `hazard_waypoint_clearance_m`, `hazard_side_release_m`, `hazard_prediction_horizon_s` | Same as `PursuitNavigation` |
| `hazard_barrier_enable` | Velocity barrier. On unless a baseline sweep is measuring the failure mode. The bound is derived entirely from the plant fit (gap to the hard radius over `tau_lin_d + delay_s`), so nothing else is tunable |

### `RlPolicyNavigation`

Requires a `[navigation.policy]` table naming the model bundle. Refuses to start unless `[robot_filter.motion_estimator]` `opponent_mode` is `KALMAN`, because the policy is trained against the 6-state opponent filter.

| Field | Meaning |
| --- | --- |
| `max_our_unmeasured_s` | Seconds without a measurement of our robot before the policy commands zero. Above the filter's 0.1 s stale age, so one or two dropped frames do not interrupt the policy |
| `hazard_prediction_horizon_s` | Seconds ahead to advance a tracked hazard along its velocity, as in `MotionProfileNavigation` |

#### `[navigation.policy]`

| Field | Meaning |
| --- | --- |
| `bundle` | Bundle directory, required. A relative path resolves against the project root, like `hazards_file`. The bundle's manifest decides the policy kind, runtime, decoder, observation spec and execution, so a bundle cannot be paired with the wrong decoder or observation layout. Example: `bundle = "data/models/nav_policies/rl_nav_<run>"` |

## `[field_filter]`

`type` selects the implementation. Every type takes the hazard fields; the rest are per type.

| Field | Meaning |
| --- | --- |
| `hazards_file` | Shared arena geometry file: a plain path, absolute or relative to the project root (for example `config/hazards/one_hole.toml`), with no implied directory and no implied extension. The C++ config and the sim each resolve it against the project root. The kinematic sim reads the same file, so the simulated floor and the keep-out discs the controller steers on cannot drift apart. Empty means no static hazards |
| `hazard_static_margin_m` | Clearance added to a static hazard on top of our robot's half-diagonal |
| `hazard_tracked_margin_m` | Clearance added to a hazard derived from a live neutral track. Smaller than the static margin on purpose, so a house bot parked mid-field does not make large parts of the field unreachable |
| `hazard_tracked_hold_s` | How long a stale neutral track keeps producing a hazard |
| `hazard_hard_margin_m` | Slack on the hard (loss-boundary) radius beyond the geometry plus our half-diagonal. Small on purpose: the hard radius is where a wheel meets the hole lip |

### `FixedFieldFilter`

| Field | Meaning |
| --- | --- |
| `size_x`, `size_y` | Arena extent (m) along field x and field y |

### `HomographyFieldFilter` and `CalibratedFieldFilter`

Both fit the mask outline, so they share these fields. `CalibratedFieldFilter` reads its field size from the calibration file, not from `field_size_x` and `field_size_y`.

| Field | Meaning |
| --- | --- |
| `field_size_x`, `field_size_y` | The floor mat (m), measured per venue with the depth path, not read off a rulebook. The size enters the pose linearly. Measurements: `docs/experiments/perception/2026-11-15_homography_field_filter.md` |
| `max_quad_coverage` | Mask area over quad area. Above this the outline is not a quadrilateral, so at least one side is not a field edge. Structural, because the reprojection residual cannot see it |
| `border_margin_px` | Contour points this close to the frame edge are dropped before the edge fit |
| `corner_skip_fraction` | Fraction of each side dropped at both ends, as belonging to the mat's rounded corners |

### `CalibratedFieldFilter`

| Field | Meaning |
| --- | --- |
| `calibration_file` | Per-cage calibration, e.g. `config/cages/<venue>_<cage>.toml`. Same rule as `hazards_file`: a plain path, absolute or relative to the project root, with no implied directory and no implied extension, so the string in a config reads as the file it names |
| `max_seating_error_px` | Above this corner error the start-up seating check warns and keeps publishing the stored calibration. Behavior: `docs/reference/perception/robot_filter_pipeline.md` |

### `FiducialFieldFilter`

Board geometry defaults match the manufactured board.

| Field | Meaning |
| --- | --- |
| `marker_size` | Printed marker edge, metres |
| `marker_separation` | Gap between markers, metres |
| `corner` | Which field corner the board marks |
| `board_offset_x`, `board_offset_y` | Board origin measured from that corner, metres, because the board cannot physically sit in the corner itself. Both run along the printed board's x and y axes, not the published field frame, which is turned afterwards so +x runs left to right in the camera view |
| `board_offset_z` | Metres. Non-zero for a wall-mounted board |
| `board_pitch_deg` | 90 for a board hung flat on a wall |
| `field_size_x`, `field_size_y` | Field extent (m) measured from that corner |
| `accumulate_frames` | Correspondences are stacked across this many frames before the pose latches, so detection noise averages out. `reset` clears the stack |
| `max_reprojection_error_px` | A real guard here, unlike the four-corner homography: 15 markers give 60 correspondences against 6 unknowns, so the residual measures something. It catches a mis-measured `marker_size`, a board printed at "fit to page", a mirrored id mapping, and a board that was not flat |

## `[robot_filter]`

### `GroundTruthRobotFilter`

| Field | Meaning |
| --- | --- |
| `neutral_count` | How many of the sim's ground-truth opponent slots are neutral robots rather than opponents. They come last. With `runner.default_opponent_count = 1` and `neutral_count = 1` the sim supplies two `[[opponents]]`: the first arrives as `THEIR_ROBOT_1`, the second as `NEUTRAL_ROBOT_1`. That is the house-bot path: target selection ignores a `NEUTRAL` track and the hazard assembler turns it into a moving keep-out disc |

### `RobotFrontBackFilter`

| Field | Meaning |
| --- | --- |
| `max_jump_distance` | Measurements that land further than this (m) from the track's last known position are rejected |
| `max_consecutive_jump_rejects` | After this many consecutive rejected frames the measurement is accepted anyway, which resets the track |
| `blob_overwrite_min_distance_meters` | Minimum radius (m) around a keypoint detection inside which blobs are suppressed |
| `blob_overwrite_size_scale` | Scale on the blob and keypoint size estimates that sets the suppression radius |
| `field_bounds_margin_meters` | Margin (m) added to the field half extents before a measurement is rejected as out of bounds |
| `our_robot_hold_window_s` | How long an unmeasured our-robot track is held before it is dropped from the output. Without it a stale "ours" detection is predicted forward indefinitely. 0 or below disables the decay and holds forever |
| `our_keypoint_dropout_blob_radius_meters` | Radius around our robot's last held pose inside which blob detections are discarded while the keypoint model is not detecting our robot. The blob model cannot identify our robot, so without this an undetected own robot is emitted as an opponent at our own position. 0 or below disables the suppression |
| `our_keypoint_dropout_blob_window_s` | How long after our robot's last keypoint-confirmed frame the suppression radius stays active. It outlives `our_robot_hold_window_s` on purpose: the track decays sooner than a typical keypoint dropout ends. 0 or below disables the suppression |
| `[robot_filter.motion_estimator]` | Propagation strategy behind the association front-end. Absent selects dead reckoning |
| `robot_size_meters_per_label` | Known footprint per label as `[x, y, z]` in metres, replacing the detector's box. The box measures pixels, not the robot, and a loose one on the house bot inflates every keep-out built from it. Labels absent here keep the measured box, which is logged to the `detected_size` channel so a detector regression stays visible |
| `footprint_center_offset_per_label` | `RobotDescription::footprint_center_offset` per label as `[x, y]` in metres in the body frame. Labels absent here have their footprint centred on the pose |
| `keypoint_height_meters_per_label` | Per-label keypoint height; labels absent here fall back to `keypoint_height_meters` |

## `[robot_filter.motion_estimator]`

`type` selects implementation. Both types take their plant gains from the top-level `[plant]` table, not from this section. `apply_plant` stamps them in and throws when the table is missing.

### `DeadReckoningMotionEstimator`

No fields. The estimator scales the normalized stick command to body velocity with `k_fwd`, `k_rev`, `k_ang` from `[plant]` and has no other source for what full stick means. Behavior lives in the outer `[robot_filter]` section.

### `KalmanMotionEstimator`

| Field | Meaning |
| --- | --- |
| `our_robot_mode` | `DEAD_RECKONING` applies the last commanded velocity directly, matching `DeadReckoningMotionEstimator` for that track. `EKF` runs the 5-state EKF with a `JigPlantModel` built from `[plant]` |
| `opponent_mode` | `KALMAN` runs the 6-state filter (constant velocity plus an independent constant-yaw-rate heading) with innovation gating and coasts through dropout gaps. `HOLD` pins each opponent at its last measured pose with no prediction, matching `DeadReckoningMotionEstimator`'s opponent behavior |
| `opponent_accel_psd` | Continuous white-noise acceleration PSD (m^2/s^3) for the opponent constant-velocity model |
| `opponent_yaw_accel_psd` | Continuous white-noise yaw acceleration PSD (rad^2/s^3) for the opponent heading block, which runs a constant-yaw-rate model independent of the translation block |
| `initial_yaw_rate_sigma` | Sigma (rad/s) on the opponent yaw rate when the heading block initializes |
| `opponent_heading_sigma_rad` | Heading measurement sigma for opponent keypoint poses. Separate from `keypoint_heading_sigma_rad` because that one also tunes the our-robot EKF |
| `opponent_position_sigma_m` | Position measurement sigma for opponent keypoint poses, before the per-meter range term. Separate from `keypoint_position_sigma_m`, which also tunes the our-robot EKF. Blob centroids keep `blob_position_sigma_m` |
| `heading_flip_reinit_count` | Consecutive opponent heading measurements more than pi/2 from the track (flips) after which the heading block re-initializes at the measured heading. A run of consistent flips means the track had the front wrong, not the keypoint model |
| `heading_blind_labels` | Labels whose heading is never measured: full-body spinners and robots the keypoint model cannot orient. Their heading sigma renders as infinite, which tells navigation the front is unknown |
| `keypoint_position_sigma_m` | Position measurement sigma for keypoint-derived poses |
| `blob_position_sigma_m` | Position measurement sigma for blob centroids |
| `keypoint_heading_sigma_rad` | Heading measurement sigma for keypoint-derived poses, our-robot EKF only |
| `position_sigma_per_meter` | Extra position sigma per meter of camera range, added on top of the base sigma. 0 disables range dependence |
| `gate_nis` | Innovation gate on normalized innovation squared, chi-square with 2 DOF. 11.83 is the 99.7% quantile. Values <= 0 disable gating |
| `reinit_after_rejects` | Consecutive gated-out measurements on one track before the track resets to the next accepted measurement |
| `initial_velocity_sigma` | Sigma (m/s) on each velocity axis when a track initializes from its first measurement |
| `covariance_floor` | Floor on the diagonal of P so the covariance cannot collapse and lock out measurements |
| `max_coast_s` | Our-robot EKF pose integration stops once the unmeasured age crosses this. Covariance keeps growing over the whole step either way |
| `opponent_max_coast_s` | Same cutoff for opponent tracks |
| `coast_stale_age_s` | `coast()` flags a track `is_stale` once it has gone unmeasured this long, even though `correct()` never ran to say so. 0.1 is three 30 Hz frame periods, so between-frame coasting stays fresh and a real perception dropout reads stale |
| `clock_gap_warn_period_s` | Throttle on the clock-gap warning, which would otherwise fire every tick once tripped |
| `speed_measurement_max_dt_s` | Baseline age past which the held reference pose is dropped rather than differenced. A gap this long means dropped detections, and the robot's path between the two poses is no longer a straight line worth differentiating |
| `speed_gate_nis` | Chi-square gate for the forward-speed update alone, deliberately looser than `gate_nis`. That gate protects a pose the track cannot recover without. This one guards a state whose fallback is the plant model's own guess, which the innovations say is worse than a noisy measurement. Sized to pass the innovations the current process noise makes look impossible |

`plant` and `plant_noise` are not TOML keys. `plant` is the jig-fitted table from the top-level `[plant]` section, so sim, playback, and tests build `JigPlantModel` from the same config extends chain instead of a separate runtime file. `plant_noise` holds the process noise PSDs from `[plant.process_noise]`. Its fields are optional and default to the `fit_process_noise.py` output baked into `JigPlantNoiseParams`.

## `[keypoint_filter.height]`

| Field | Meaning |
| --- | --- |
| `max_height_meters` | Ceiling on absolute height above the field plane. Anything higher is not a robot on the floor: walls, the ceiling, people leaning over the arena |
| `floor_percentile` | Percentile of the surrounding ring taken as the local floor. Low because the ring can still catch part of a robot, and the floor is the low part of that |
| `ring_inner_scale`, `ring_outer_scale` | Inner and outer radius of the floor reference ring, as multiples of the inscribed circle's radius. The gap exists so a robot spilling past its inscribed circle does not contaminate its own floor reference |
| `sample_radius_px` | Half-width of the square depth sample window centred on each keypoint. Feeds the per-keypoint height used for projection, not the gating decision |
| `min_valid_samples` | Minimum valid depth samples, in the circle and in the ring separately, before a reading is trusted. Below this the gate abstains for that detection |
| `max_circle_samples` | Cap on depth samples per detection. Larger boxes stride to stay under it, which bounds the per-frame cost regardless of how close a robot is |

## `[keypoint_filter.static]`

| Field | Meaning |
| --- | --- |
| `min_dwell_seconds` | How long a cluster must hold still before its detections are suppressed |
| `min_observations` | Observations required over that dwell, so a stale track cannot qualify on two hits |
| `responsiveness_window` | Observation count at which the running mean and deviation stop averaging and start tracking. Without it the statistics go numb after a few thousand frames and a robot parking on a logo could never move the verdict. 60 is about 2 s at 30 Hz |
| `forget_seconds` | Clusters unseen for this long are forgotten |

## `[keypoint_model]`

`type` selects the implementation; the rest are per type.

### `YoloKeypointModel`

| Field | Meaning |
| --- | --- |
| `reference_offset_m_per_label` | `Keypoint::reference_offset_m` per detected label: how far the robot's reference point sits ahead of where its front and back keypoints were labelled. Labels absent here get 0, the keypoint midpoint |

### `SimKeypointModel`

The lockstep sim's keypoint model. Design: `docs/reference/perception/robot_filter_pipeline.md`.

| Field | Meaning |
| --- | --- |
| `labels` | Label of each ground-truth slot in sim order: our robot, then the sim's `[[opponents]]`. Each label needs `<LABEL>_FRONT` and `<LABEL>_BACK` keypoint labels |
| `keypoint_offset_m` | Front and back keypoint distance from the robot centre along its heading. The filter only reads the pose the pair implies, so this sets their image spacing, nothing else |

### `AprilTagKeypointModel`

| Field | Meaning |
| --- | --- |
| `tag_size_m` | Edge of the black square, the object points `solvePnP` sees |
| `robot_tag_ids` | The robot's tags. Which one faces up comes from its mount rotation, not its position |
| `roi_margin_px` | Search window around the last detection, grown by this on every side |
| `max_reprojection_error_px` | A tag pose worse than this emits no keypoints. It is still recorded |
| `front_keypoint_m`, `back_keypoint_m` | Body-frame points projected as the front and back keypoints |
| `axle_height_m` | Axle above the floor, the wheel radius. Keypoint heights are measured from here |
| `rest_pitch_rad` | Nose-down pitch the robot rests at on the floor, positive about body y |
| `[[keypoint_model.tags]]` | One robot tag's mounting. `rotation` (row-major `R_body_tag`) and `translation_m` map points in the OpenCV marker frame (origin at the tag centre, x right and y up in the printed image, z out of the printed face) into the robot body frame (FLU at the axle midpoint). The parser rejects a rotation that is not orthonormal or is a reflection |

## `[health]`

| Field | Meaning |
| --- | --- |
| `disk_used_warn_percent` | Log a disk warning at or above this used percentage. MCAP and SVO recording both stop opening new files once the disk is full, so the default 90 warns well before that |

## `[mcap]`

| Field | Meaning |
| --- | --- |
| `max_size_gb` | Roll to a new file past this size. 0 disables the limit |
| `holding_dir_max_size_gb` | Oldest recordings in the output directory are evicted to stay under this. 0 disables the limit |

## `[battery]`

| Field | Meaning |
| --- | --- |
| `source` | What the monitor reads: `NONE` or `WAVESHARE_UPS`. `NONE` runs no monitor and never publishes `/status/battery` |
| `ocv_table_file_path` | Open-circuit-voltage table. Empty uses `data/battery_ocv_table.toml` |
| `state_file_path` | Persisted state of charge. Empty uses `$HOME/.local/state/auto_battlebot/battery_soc_state.txt`. It lives under `$HOME`, not `data/`, because a deploy overwrites the repo tree |

## `[control_loop]`

An absent `[control_loop]` section selects `SteppedControlLoop` with `rate_hz = 0`.

| Field | Meaning |
| --- | --- |
| `rate_hz` | Cycles per second. 0 on `SteppedControlLoop` (its default) means one cycle per perception frame |
| `max_catchup_cycles` | `SteppedControlLoop` only: most cycles one `advance_to` runs after a stall. Every cycle in the backlog sees the same measurement, so replaying more only burns time |
| `watchdog_timeout_ms` | `ThreadedControlLoop` only: a cycle older than this marks the loop unhealthy |
| `robot_cache_max_age_s` | Longest gap `RobotDescriptionsCache` fills with the last robot set that had both our robot and an opponent. Past it navigation sees the robots as they are and runs its no-robot path |

## `[plant]`

The jig-fitted drivetrain, one table for the EKF and `MotionProfileNavigation`
(`docs/reference/control/plant_backed_control.md`). Absent is not a parse error. A consumer that
needs it throws. Provenance of the values is in `docs/reference/config/robot_plant_files.md`.

Every field in the table is required. A partly defaulted plant would predict with a mix of fit and
zero that no experiment produced. A ladder rung disables a term by writing it as zero, as the fit
output files do.

| Field | Meaning |
| --- | --- |
| `k_fwd`, `k_rev` | Forward and reverse speed in m/s at full effective command |
| `k_ang` | Yaw rate in rad/s at full effective command |
| `c_sb` | Steer-brake: fraction of forward authority lost per unit \|u_ang_eff\| |
| `c_ad` | Angular droop: fraction of yaw authority lost per unit \|u_lin_eff\| |
| `c_drift` | Yaw drift in rad/s per unit effective linear command |
| `c_drift_bias` | Yaw drift in rad/s while moving, signed by the linear command |

### `[plant.process_noise]`

Only the EKF reads these. Every field is optional and defaults to the `fit_process_noise.py`
output (`docs/experiments/sysid/2026-08-23_process_noise_fit.md`).

| Field | Meaning |
| --- | --- |
| `q_along` | White-noise acceleration on v, (m/s^2)^2/Hz |
| `q_cross` | Cross-track white-noise acceleration, (m/s^2)^2/Hz. There is no state to feed |
| `q_heading` | White-noise acceleration on w, (rad/s^2)^2/Hz |
| `scale_factor` | Fractional speed error, sigma = \|v\| h eps |
| `heading_scale_factor` | Fractional yaw-rate error, sigma = \|w\| h eps |
| `heading_random_walk` | Heading random walk, rad^2/s |
| `delay_jitter_s` | Command delay jitter in s. Along-track sigma = \|v\| * jitter, flat in h |

## `[rgbd_camera]`

One table per `type`.

### `ZedSvoPlaybackCamera`

Replay of an SVO file. It is a separate type from `ZedRgbdCamera` so recording, reconnection, and the capture thread are not reachable here, and so `svo_recording` cannot be combined with an `svo_file_path`.

`get()` grabs on the caller's thread, so every frame reaches the pipeline in order and none are skipped. That makes replay reproducible, which regression testing rests on. A live camera cannot work this way: it must not block the pipeline on camera I/O, and it drops frames to keep up. Recording while replaying is meaningless, and the file ending is a normal shutdown, not a disconnect to recover from, so there is no recorder, reconnection, or grab-health monitor. `ZedSvoPlaybackCameraTest.DeliversEveryFrameInOrderWhenThePipelineIsSlow` pins the in-order delivery.

| Field | Meaning |
| --- | --- |
| `rebase_stamps` | Shift frame stamps to the current wall clock. Turn off to compare recordings between builds, where the offset otherwise makes every payload differ. Off leaves stamps on the recording's clock, which does not match the control clock, so latency compensation and stamp-difference diagnostics are invalid |
| `svo_real_time_mode` | `false` decodes frames as fast as the pipeline allows, so the wall-clock gap between shutter and control tick belongs to the machine, not the robot. Field timing, including latency compensation, is not reproduced |
| `svo_start_frame` | Must lie inside the recording. A config that overrides `svo_file_path` must set it too, or it inherits the value from the config it extends |

### `V4l2RgbCamera`

e-CAM25_CUONX over raw V4L2. There is no pixel-format or codec knob: the sensor emits UYVY and we encode H.264, so both are hard-coded. There is one correct behaviour and one consumer; a second capture format becomes an enum when one turns up. `device` and `calibration_file` are strings because they are paths. `camera_resolution` is an enum because it has a fixed set of valid values, so a misspelling fails at parse with the alternatives listed.

| Field | Meaning |
| --- | --- |
| `buffer_count` | V4L2 buffers queued. Four, not the driver default: more buffers means the pipeline reads staler frames when it falls behind, and four costs at most one frame of queueing |
| `calibration_file` | `config/cameras/<serial>.toml`. Required: the lens is not a pinhole and everything downstream assumes one |
| `video_bitrate_kbps` | H.264 bitrate. 15000 is about 6.8 GB per hour and 340 MB for a three-minute match |
| `exposure_us`, `gain`, `white_balance_temperature` | 0 keeps the driver default. Pinning them matters because auto-exposure hunting between a lit arena and a dark robot is a plausible source of frame-to-frame detector instability |

### `ZedOneRgbCamera`

ZED X One S (or any ZED X One) over GMSL2 through `sl::CameraOne`, as on the ZED Box Mini. There is no `calibration_file`: the lens model is the factory's 12-coefficient rational one, which our plumb-bob `CameraCalibration` cannot hold, so the SDK rectifies and we publish its rectified intrinsics. Recording and replay follow the V4L2 camera: H.264 on `/camera/video`.

| Field | Meaning |
| --- | --- |
| `camera_resolution` | `RES_1920x1200` is the full sensor, `RES_1920x1080` crops it, `RES_960x600` bins it (up to 120 fps). Anything else fails at open |
| `video_bitrate_kbps` | Same figures as `V4l2RgbCamera` |

### `VideoPlaybackCamera`

Replay of an MCAP recording carrying `/camera/video`. It is separate from the live camera for the same reason SVO playback is: no recorder, no reconnection, no capture thread, so every frame reaches the pipeline in order and replay stays reproducible.

| Field | Meaning |
| --- | --- |
| `rebase_stamps` | Same as `ZedSvoPlaybackCamera` |
| `start_frame` | Video frame ordinal to start at, seeking from the last IDR at or before it. A value at or past the frame count throws `ConfigValidationError`: a scratch overlay that sets `video_file_path` but inherits `start_frame` from the config it extends otherwise seeks past the end and exits before the first heartbeat, which reads as a corrupt recording instead of a config merge |
| `calibration_file` | Empty rectifies with whatever the recording itself names. Set it to re-rectify old footage against a revised calibration |

### Replay input reservation

`reserve_camera_input_paths` tells the recorder which files a camera will be replaying, so it cannot pick one of them as its own output. A replay reads file A and records file B, and pointing the output at the input would corrupt the source. Call it before constructing the recorder.

## `[publisher]`

`FoxglovePublisher` fields. `NoopPublisher` has none.

| Field | Meaning |
| --- | --- |
| `socket_path` | Unix socket `viz_relay` listens on |
| `preview_width` | Width of `/camera/preview` and `/camera/preview_video`. Height keeps the aspect |
| `preview_rate_hz` | `/camera/preview` JPEG rate |

## `[runner]`

| Field | Meaning |
| --- | --- |
| `max_loop_rate_hz` | Loop pacing. 0 free-runs |
| `default_behavior_mode` | Behavior mode at startup. The dashboard changes it at runtime; headless sim and playback runs pin `RUN_AWAY` here |
| `auto_tracking_by_default` | Start with the target selector picking the target. false starts with the target held at the field center until the dashboard places it |
| `parallel_models` | Run the keypoint and robot blob models on parallel worker threads (`ParallelModelBatch`). false calls them sequentially in the tick |

## `[target_selector]`

### `SafestPointTarget`

| Field | Meaning |
| --- | --- |
| `retarget_improvement_m` | Radius (m) a new candidate must beat the held target by before retargeting. Suppresses solver jitter as opponents move |

## `[ui]`

Settings the web dashboard reads from `/status/app`.

| Field | Meaning |
| --- | --- |
| `rate_fail_threshold` | Fraction of `max_loop_rate_hz` below which the loop rate counts as not met (e.g. 0.99) |
| `rate_fail_duration_sec` | How long the rate must stay below the threshold before the dashboard shows an error. Prevents strobing |
| `label_colors` (`[ui.label_colors]`) | Lowercase model label to `"#rrggbb"`: the colors training draws each class in. The dashboard uses them for boxes and tracks. Labels not listed get a generated color |

## `[profiles]`

Read from `config/profiles.toml` by the UI profile switcher.

| Field | Meaning |
| --- | --- |
| `pattern` | Regex matched against each config file's path under `config/`, without `.toml`, `/`-separated. Matching profiles are the ones the UI offers. Empty offers none |
| `selection_file` | Where the current choice is persisted. It lives outside the repo so uploads and installs do not clobber it |
