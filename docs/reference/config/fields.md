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
| `max_motor_rpm`, `max_motor_rpm_per_s` | Linear slew limit: the normalized linear command changes by at most `max_motor_rpm_per_s / max_motor_rpm` per second. Unset means no limit |

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
read-side. `[[transmitter.children]]` is an array of tables so the order is guaranteed. Each entry
may set `transmit_rate_hz`, the most sends per second that child gets; 0 passes every send.

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
