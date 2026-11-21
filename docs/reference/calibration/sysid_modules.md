# Sysid modules

How the sysid code under `auto_battlebot/calibration/` and `playground/calibration/` works. The procedure that runs it is `docs/guides/sysid_guide.md`.

Nothing under `calibration/sysid/` sends a command: the robot is driven by a person, and the package only watches. `test_sysid_package_never_sends_commands` fails if it imports a serial, socket, jig, MuJoCo or RL module.

## `calibration/jig/drive_protocol.py`

Trainer link to the OpenTX/EdgeTX radio: writes commands, reads back what was sent. Mirrors the trainer protocol in `src/transmitter/opentx_transmitter.cpp`. The radio is a USB CDC device (VID 0x0483, PID 0x5740), primed once with `telemetry on` + `channels on`, then driven with `trainer <channel> <value>` at 50 Hz, value in [-500, 500].

The same serial port that accepts `trainer` writes also streams the radio's mixer output back once primed. Trainer mode adds to the human driver's sticks, so the mixer output is the command the robot received, not the one this process asked for. A hand-driven run therefore has a real command log, and a scripted run gets a contamination check: if the driver's stick is not centered, measured diverges from commanded and the run is flagged.

Commands are sent raw. The deployed transmitter applies `DifferentialDriveProcessor`'s lifted and zero deadzones; this does not, because the physical deadzone is what is being measured.

Safety:

- The robot moves fast. Run in a clear, bounded space with guard plates on.
- Keep the human driver's sticks centered.
- `Armed()` is a context manager that zeroes the channels and disarms on every exit path: normal return, Ctrl-C, exception, and a hard wall-clock timeout.

Pieces:

- `_SYNC` framing: sync, phase byte (0 = channels 1..16, 1 = 17..32), length byte, 16 little-endian int16, then a checksum equal to phase ^ length ^ every data byte.
- `Measured`: one tick of the named channels as the radio reported them.
- `stream_measured`: ticks from the radio alone, nothing commanded. This is how a hand-driven run gets a command log. `linear` and `angular` stay zero and the measured channels are the whole record, so the weapon pair rides along with the drive pair.
- `PlayResult.SETTLE_TICKS = 4`: ticks ignored after the command changes, because the radio reports on its own schedule and `measured` lags `commanded` by a tick or two.

### `MixConfig`

How a body command maps onto the two trainer channels, and how the readback is un-mixed.

| Field | Meaning |
| --- | --- |
| `channel_scale` | Full scale of the readback stream |
| `read_linear`, `read_angular`, `read_weapon`, `read_arm` | Slots into the radio's 0-based channel stream, so slot 0 is what the radio's screen calls CH1. On this model CH1 and CH2 are drive, CH3 is weapon throttle and CH5 is weapon arm. CH4 is unused and not logged |
| `read_invert_a`, `read_invert_b` | Per-channel sign on the read path only, for a radio with servo-reverse set on one drive channel. Find the signs with `velocity_jig_drive.py --check-radio` |

Defaults are `read_invert_a = False` and `read_invert_b = True`: `--check-radio` showed channel B coming back negated on this radio.

## `calibration/jig/excitation.py`

Excitation waveforms for velocity jig runs, declared in TOML. A waveform is a TOML table:

```
[waveform.lin_step_full]
kind = "step"
channel = "linear"
role = "fit"
amplitudes = [0.25, 0.5, 0.75, 1.0]
hold_s = 2.0
```

`kind`, `channel` and `role` are also what the fit routes on. Commands are normalized to [-1, 1]. Mapping to the radio's integer range is the transmitter link's job, and deadzone compensation is not applied anywhere.

- `Program`: a playable excitation. `segments` is present for piecewise programs and None for continuous ones (chirp, sine, triangle, PRBS, manual). `at(t)` is the authority; segments exist so the fit can label samples by phase.
- `MIN_COAST_S = 1.5`: shortest coast between cells and the default `coast_s`, so each step decays from a standstill.
- `_build_coast`: accelerate, hold, then drop to zero instantly and let the robot roll to a stop. `reverse_cap` lowers the entry speed of the reverse repetitions only.
- `_build_grid`: cross linear against angular to separate the coupling terms. The angular list must span both signs.
- `_build_chirp`: swept sine, logarithmic by default, for a frequency-domain cross-check on a step-fitted time constant.
- `_build_triangle`: constant slew between peaks.
- `_build_prbs`: maximal-length 16-bit LFSR, taps 16/14/13/11. A 60 ms bit period covers roughly 0.2 to 8 Hz. The combined linear/angular program draws a decorrelated second sequence for the angular channel.
- `_build_trim`: low-amplitude straight drive used to measure the arc before a session and seed the drift parameter. Driven open loop here; the closed-loop correction lives in the CLI.

## `calibration/jig/jig_fit.py`

Fit machinery for jig sessions. Stages and per-parameter fits are described in `plant_fit_modules.md`.

- Per-sample motion floors below which the measurement is noise:

  | Constant | Value | Meaning |
  | --- | --- | --- |
  | `MOTION_EPS_LIN` | 0.02 m/s | Linear speed floor |
  | `MOTION_EPS_ANG` | 0.05 rad/s | Angular rate floor |
  | `ONSET_SIGMA` | 5.0 | "Moving" means this many sigma above the stationary noise |
  | `MIN_HOLD_S` | 0.4 s | Shortest command hold worth fitting a steady state to |

  `stage_a` passes `step_floor = 0.01` as `min_level` to `held_segments`, which drops rest cells and nothing that was driven.
- `COAST_KINDS`: kinds that return the channel to zero mid-run, which is where a decay constant is read.
- `fit_drift` fits both drift columns at once, so neither absorbs the other.
- `build_windows` cuts windows out of every run and stacks them into one batch. Runs with a detached encoder carry no linear truth and are excluded from the position objective. `require_encoder=False` builds the angular-only set from their measured heading. A window needs a known command for every sample it spans, which excludes still holds at the ends of every run and operator-driven runs whose sticks were not logged.
- `residual_autocorrelation`: lag-1 autocorrelation of the longest-horizon position error across window starts. Strong autocorrelation means the model is missing a structural term.

## `calibration/jig/jig_link.py`

USB console client for the velocity jig (`firmware/velocity_jig`). The jig is a Feather RP2040 Adalogger that rides on the robot and logs a wheel encoder plus an ISM330DHCX IMU to microSD at 1 kHz. Line protocol over native USB CDC:

```
LIST            -> "F <name> <size>" per file, then "END"
GET <name>      -> "SIZE <n>", exactly n raw bytes, then "\nEND\n"
DEL <name>      -> "OK" or "ERR"
TIME            -> "TIME <rx_us> <tx_us>"
STREAM          -> "STREAM", then "S t_us,count,gx,gy,gz,ax,ay,az" at 10 Hz until any
                   byte arrives, then "END"
```

Three firmware behaviors shape the client:

1. Every command except `TIME` answers `BUSY` while recording. `TIME` touches neither the SD card nor the capture path, so a mid-run probe measures skew across the run window.
2. Recording starts and stops on physical buttons. Button A emits an unsolicited `recording <name>` and button B a `stopped, n=... dropped=...`. The client latches those lines as they arrive.
3. `STREAM` stops on any byte and can exit into a recording, so `END` is not necessarily terminal.

Baud is nominal; transfers run at USB Full Speed.

- `find_jig_port` locates the jig by exact USB id (`JIG_USB_IDS`), falling back to any ACM port that is not the radio (`TRANSMITTER_USB_IDS`). `--list-ports` shows the real ids.
- `find_port_by_serial` re-finds a port after a replug by the RP2040's serial number, since the device path can change.
- `wait_stopped` blocks until button B is pressed and returns (samples, dropped). Call it with the cable plugged in: the stop summary is unbuffered.
- `probe_clock` fits the host-to-jig clock offset from a burst of TIME round trips. The jig counts microseconds since its own boot. Only the fastest decile of round trips is kept, since they are the most nearly symmetric.
- `skew_ppm` is crystal drift across the run in parts per million. The RP2040 crystal runs around 30 ppm, about 0.9 ms over a 30 s run. It returns 0.0 when the jig span is not positive (a board that restarted between probes).

## `calibration/jig/velocity_jig.py`

Jig log reading, calibration, and dead-reckoned ground truth. The jig is an RP2040 datalogger that rides on the robot and records a wheel encoder plus an ISM330DHCX IMU at 1 kHz (`firmware/velocity_jig`). It writes plain text with a comment header:

```
# auto-battlebot velocity jig
# columns: t_us,count,gx,gy,gz,ax,ay,az  (raw int16 imu)
# encoder: quadrature x4 (constant column if unplugged)
# sample_rate_hz=1000 gyro_dps_per_lsb=0.070 accel_g_per_lsb=0.000244
```

This module turns those raw counts into the ground truth the plant fit consumes: yaw rate about the measured gravity axis, forward speed from encoder arc length with the encoder wheel's lever arm removed, and a dead-reckoned pose. It also carries the host clock mapping (`ClockFit`) and the per-run sidecar (`load_session_dir`, reading the sidecar TOML that `velocity_jig_drive.py` writes next to every downloaded log). See `docs/experiments/control/kalman_filter/kalman_filter_plan.md` part 1.2 and the companion runbook.

Three things are flagged rather than absorbed:

- Saturated gyro channels (`|raw| > 32000`, `RAW_SATURATION`), per axis, per run. This disqualifies a run: a railed yaw axis makes the heading integral wrong from that sample on.
- Clipped accelerometer channels, past `ACCEL_SATURATION_G`. Reported, never gated.
- Gyro bias drift between the pre-run and post-run still holds.

### Log reading

- `ROLES`: whether a run trains the fit or validates it, declared per waveform.
- `saturation`: per-axis saturation report, gyro first then accel. `keep` masks samples that describe the plant, so operator pauses are excluded.
- `read_jig_log`: a decreasing timestamp means a corrupt row, not a rollover. The reader keeps the monotone prefix and counts the rest as malformed. `JigLog` fields: `t_boot_s`, `count_x4` (int64), `gyro_raw_int16` and `accel_raw_int16` as `(N, 3)` int16.
- `_bridge_blips`: fills False runs no longer than `max_run`, so a single over-threshold sample does not split a held segment.
- `find_still_segments`: stationary stretches at least `min_duration` long, tested three ways: gyro for rotation, accelerometer magnitude near 1 g for acceleration, encoder for constant-speed cruising. The thresholds are loose; a hold that is not still shows up later as pre/post bias disagreement.
- `still_stats`: gyro bias and the gravity ("up") unit vector over one still hold. The yaw axis comes from gravity, which makes positive yaw counter-clockwise viewed from above and survives a tilted remount. `StillStats` fields: `bias_radps` (3,), `up_unit` (3,), `gyro_noise_radps` (3,).

### `JigCalibration`

Bench calibration from runbook block 0, one set per hardware configuration. `meters_per_count` (E4) and `gyro_scale` (E3) set the units of every fitted parameter. `from_toml` rejects a non-positive `meters_per_count`. Fit it with `playground/calibration/fit_encoder_scale.py`.

| Field | Meaning |
| --- | --- |
| `r_enc_perp` | Encoder wheel offset perpendicular to the forward axis, m. Removed from every speed measurement |
| `r_imu` | IMU offset from the yaw axis, m. Kept for the report and as a tape-measure check |
| `encoder_rate_limit` | Rate above which the encoder wheel scrubs, rad/s (E5) |

### Run records

- Command coverage: `load_run` counts a grid sample as commanded when it falls between the first command and 2.0 s after the last. Still holds outside the command log, and operator-driven runs with no logged sticks, are not commanded. `problems` rejects a run when the larger of the two hold gaps exceeds `max_hold_gap_s` (default 5.0 s).
- `ClockProbe`: one `TIME` probe burst as `velocity_jig_drive.py` records it. It keeps the fastest decile by round trip and reports the median host-minus-jig offset (`offset_ms`) with the RMS about that median as `residual_ms`.
- `PauseWindow`: one operator stop, in host seconds. The robot is handled inside it, so every gate and duration that describes the run skips it.
- `WaveformSpec`: what a run played, in the terms the fit routes on (kind, channel, role). `params` is the catalog entry verbatim.
- `RunRecord`: the two drive channels exactly as the radio reported them. `cmd_lin` and `cmd_ang` are these read through a mix, so a wrong mix can be corrected without re-recording. `hold_pre_host_s`, `hold_post_host_s` and `cmd_t_host_s` are host seconds. `encoder` is an `EncoderState` (`attached`, `detached`, `either`). `gate_note` is why the capture-time gates rejected the run, `None` when they passed. `command_source` is a `CommandSource`: `measured` is the radio's mixer output read back, `commanded` is what the CLI asked for. For a hand-driven run only `measured` exists.
- `_pause_windows`: pause windows from the sidecar, or reconstructed from command gaps for older sidecars.
- `load_run_record`: reads one sidecar TOML and its command CSV. `commands="measured"` is the default; `"requested"` is the fallback for a run whose readback was mis-decoded.
- `read_command_log`: reads a `LOG-N.cmd.csv` written by the drive CLI. Columns are named in a `# columns:` comment. `t_host_s` shares its origin with the clock probes' `at_host_ms` (both `time.monotonic()` on the host), which `ClockFit` rests on.

### Ground truth

`GroundTruth` is dead-reckoned truth for one run on the jig's 1 kHz grid. Heading drift dominates position drift, so it is trustworthy over the 500 ms windows the plant fit uses and not over a whole 30 s run. `t_s` is host seconds when a `ClockFit` was supplied, else jig seconds. `v_body_mps` is the body forward speed, `v_onset_mps` the same with light smoothing for onset timing, `yaw_rate_radps` the yaw rate, `theta_rad` the unwrapped heading.

`solve_ground_truth` is unicycle dead reckoning from encoder arc length and gyro yaw:

```
w(t)     = (gyro(t) . up - bias . up) * gyro_scale
theta(t) = theta_0 + integral w dt              (trapezoidal, at the log rate)
ds(t)    = (count(t) - count(t-1)) * meters_per_count
v(t)     = ds/dt - w(t) * r_enc_perp            (encoder lever arm removed)
p(t)     = p_0 + integral [v cos(theta), v sin(theta)] dt
```

Bias comes from the pre-run still hold and, when a post-run hold is given, ramps linearly to it across the run. Encoder speed is smoothed over `smooth_s` before use.

- `RunQuality`: set per run in the sidecar. The post hold brackets gyro bias drift. Measured on this board, rate random walk is 1.862e-4 deg/s/sqrt(s) (E2, LOG-97), so 60 s of run leaves roughly 0.04 deg of heading error. The worst drift across 33 two-hold runs was 0.0155 deg/s, about 0.5 deg over 60 s.
- `problems`: gyro only, zero tolerance, since a railed yaw axis is unrecoverable. The accelerometer does not gate.
- `notices`: things worth looking at that do not disqualify the run, kept separate from `problems`.
- `Run`: one run resampled onto a uniform grid with commands aligned to it. Everything downstream reads these arrays. `t` is the uniform grid in host seconds, `v_onset` is lightly smoothed for onset timing, `command_known` masks samples where the command is known, `phase_label` is the per-sample phase from the protocol segments.
- `_hold_uncovered_s`: seconds of a commanded still hold that the log does not show as still, by overlap rather than edge alignment (the jig starts recording at the A press, before the CLI's hold countdown).
- `load_run`: reads a run's log, solves ground truth, and resamples it with its commands onto one grid at `fit_hz` (200 Hz). Every operator pause is dropped before anything is measured.

## `calibration/sysid/coverage.py`

The live coverage session: telemetry and tags in, labeled motions and plant coverage out.

`CoverageSession` is pure and deterministic. A source feeds it decoded messages in log order (`feed_telemetry`, `feed_tags`, `feed_field`), `advance` processes everything older than `LAG_S`, and `snapshot` returns the page's JSON. It does no IO; the refit runs through a `RefitEngine` (`worker.py`), inline for tests and replay or in a process when live.

Pipeline per 10 ms grid sample, once the tags around it have arrived:

1. Telemetry is put on the host clock (`telemetry.LiveClock`). Commands are held to the grid; the gyro is interpolated. The clock map rests on the fastest deliveries, so the minimum radio latency stays in it and every telemetry stamp reads that much late. The fit's delay absorbs it for the commands. The Jetson's ring poller stamps every row of one poll with that poll's arrival and marks all but a caught-up poll's newest row `backfill`. Only each poll's newest row reaches the clock.
2. `maneuvers.ManeuverSegmenter` labels the sticks.
3. Every `model.WINDOW_S` the window closes: gates, three sub-windows from the tag poses, Fisher information at the current estimate, and a status.
4. A refit runs every `REFIT_S` of driving when new windows closed.

Gates, first match wins:

- jetson: `espnow_active` or `autonomy_enabled` anywhere in the window. The sysid profile never commands, so this is a banner as well as a gate.
- link: telemetry missing for over `GAP_S`, counted from `telemetry_seq` (frames lost) or `timestamp_ms` (frames late).
- tags: accepted tag fixes cover under `TAG_COVERAGE_MIN` of the camera frames.
- flip: the robot is upside down, or the running median of `PITCH_MEDIAN_N` tag poses is pitched more than `PITCH_LIMIT_DEG` off the rest pose.
- wall: the axle center within `ROBOT_LENGTH_M` of a rail (`sysid_windows.near_rail`), or, once the fit has settled, a tag residual over `CONTACT_SIGMA` (marked by the refit).
- disarmed: `armed` false.
- idle: no motor command over `IDLE_CMD` and no tag speed.

Every fifth usable window is held out: it scores the closed-loop prediction and never enters the fit.

## `calibration/sysid/maneuvers.py`

Labels what the driver did with the sticks: holds, edges, creeps, arcs, spins. Two consumers:

- `ManeuverSegmenter` labels the stick stream as it arrives for the live coverage monitor. It works on the uniform 10 ms grid the monitor builds, sticks as fractions, throttle forward positive and steer left positive.
- `menu_from_grid` is the offline session summary that `playground/calibration/check_sysid_recordings.py` prints against its `MENU_TARGETS`.

Definitions:

- Plateau: a maximal run where a channel stays within `PLATEAU_TOL` of every other sample in it.
- Hold: a plateau of `HOLD_S` or longer away from zero. A throttle hold with the steer mean under `ARC_STEER` is straight, with more it is an arc. A steer hold with the throttle mean under `ARC_STEER` is a spin.
- Edge: the move between two long plateaus (rest counts at any length) that differ by at least `EDGE_MIN`. Clean when it took `SNAP_S` or less. Only clean edges count toward delay and lag; a slower one is a slew.
- Creep: a monotonic climb out of zero past `CREEP_PAST` that took `CREEP_MIN_S` or longer.

`BINS` are the coverage map's stick-fraction bins, finer near zero where the deadzones are. `EDGE_ROWS` includes yaw-loop rows: steer edges with the flip switch down, where the firmware yaw loop answers instead of the open-loop drivetrain.

## `calibration/sysid/model.py`

The fit behind the sysid monitor: window residuals, Fisher information, refits, suggestions.

Each 1.5 s fit window is cut into three 0.5 s sub-windows. A sub-window starts from the tag state estimate at its start (`pose_stream.state_at` plus the gyro) and runs the plant open loop on the recorded motor commands for 0.5 s, the horizon the Kalman plan scores. Its residuals are:

- tag position, along and across the camera view, at each tag fix, over the per-fix sigma floored at `POS_FLOOR_M`,
- tag heading at each fix, floored at `YAW_FLOOR_RAD`,
- gyro yaw rate every 20 ms over `SIGMA_GYRO_RADPS`.

The fit input is the motor command (`firmware_mixer.motor_to_plant` of `left_cmd`/`right_cmd`), so the same 16 drivetrain parameters come out with the yaw loop on or off and the loop's feedback stays out of the Fisher matrix. Delay is one of the 16: commands are shifted by a fractional, linearly interpolated delay.

Predictions for the page (holdout, ghosts, suggestions) run the sticks through `firmware_mixer.closed_loop_rollout`, so they score the plant the Jetson would command.

Uncertainty is the Cramer-Rao bound `s^2 (J^T J + prior)^-1`, with `s^2` the fit's residual variance per row when that exceeds 1.

- `ParamSpec`: each fit parameter has a prior (the `PlantParams` defaults), a prior sigma, a ring target and a finite-difference step. Ring targets: 4 ms for delay, 6 ms for each lag, 3% of the prior for each gain, 0.008 for each deadzone, 0.03 for each coupling term, 0.12 rad/s for each drift term.
- `NUISANCE_SD`: prior sigmas on each sub-window's start state (x, y, theta, v) from the local tag estimate. The start state is a nuisance: each sub-window refines it, and the information it could fake is projected out of the Jacobian.
- `jacobian`: returns (r, J) with J shaped (S, R + NUISANCE, NP); columns outside `free` are zero. Forward differences at the refined start states, then projected so the start states absorb what they can. `J^T J` of the result is the Schur complement.
- `expected_subwindows`: simulates a motion from rest and cuts it into the sub-windows a session would record. Measurements equal the prediction, so only the Jacobian matters. Also returns how far the robot travels.

## `calibration/sysid/pose_stream.py`

Causal robot pose from the AprilTag stream, for the live sysid monitor. The offline path (`tag_pose_smoother.smooth_session`) runs a two-pass RTS smoother over the whole session. The live monitor needs two things:

- One body pose per detection, for the fit residual and the gates. Each frame goes through the offline `build_candidates` and `select_ippe`; `select_ippe` looks ahead up to its `neighbour_window_s` on close calls, so a detection is final once it is that much older than the newest frame.
- The state each fit window starts from. `state_at` fits a weighted quadratic in time to the poses within `STATE_HALF_SPAN_S` either side of the start. Velocity is its slope projected on the heading; yaw rate comes from the gyro the caller passes in.

Frames follow `tag_pose_smoother`: the output frame is the recorded field frame turned so z points up, the body frame is FLU at the axle midpoint, and yaw is counter-clockwise.

## `calibration/sysid/server.py`

The sysid monitor's page: one static HTML file and a websocket of JSON snapshots, served by the Python process on the laptop. `GET /` returns `static/index.html`; `/ws` sends a `hello` with the parameter list, then every snapshot `publish` hands it.

## `calibration/sysid/telemetry.py`

Robot telemetry for the sysid monitor: the frame schema, the plant inputs, the robot clock.

`/robot/espnow_telemetry` is the source. The column list is read from the JSON schema the recorder writes (`mcap_write.ESPNOW_TELEMETRY_JSON_SCHEMA`). Integers load as nullable `Int64` (`telemetry_seq` is `<NA>` in older recordings), numbers as float with NaN for null, booleans as bool.

Without ESP-NOW telemetry the source is `/robot/esp32_diagnostics`, which only recordings before 2026-11-20 carry: the robot's telemetry ring polled over WiFi (older recordings: its event stream). `from_diagnostics` renames its `seq` to `telemetry_seq`; the columns it lacks (`espnow_active`, `autonomy_enabled`) read as False. Its `backfill` rows waited in the ring, so the live clock skips them.

Plant inputs:

- Fit input: `left_cmd`/`right_cmd`, what the ESCs got after the deadband and the yaw loop, through `firmware_mixer.motor_to_plant`.
- Prediction input and stick display: `a_percent`/`b_percent`, the arbitrated sticks before the mixer. `throttle = -a / 100` (forward positive) and `steer = -b / 100` (left positive).

## `calibration/sysid/worker.py`

Refits for the sysid monitor, off the session's thread. `RefitState` keeps every usable window's sub-windows and turns a `RefitJob` into a `RefitReply`: a warm-started fit, the Fisher matrix over every fit window, the contact gate, the holdout score, every third time the model ladder, and the ranked suggestions. `InlineEngine` calls it on `submit` (tests and replays). `ProcessEngine` runs it in a spawned process so a live session keeps labeling during a refit.

`MAX_FIT_WINDOWS = 300`: the fit thins to this many windows. The Fisher matrix still uses all of them.

## `control/firmware_mixer.py`

`mix_motor_outputs` (`firmware/mr_stabs_mk2/src/main.cpp`) turns the arbitrated sticks into motor commands. With the flip switch down it runs `yaw_control::YawController` on every BNO055 sample and its output replaces the steer stick; otherwise the steer stick passes through a 1% deadband. `mujoco_sim/firmware.py` mirrors both as scalar Python stepping one robot. This module is the same logic over a batch dimension so the sysid fit can roll out hundreds of windows at once, plus `closed_loop_rollout`, which runs `plant.integrate_step` behind the mixer with the heading and gyro read back from the simulated state.

Conventions follow the firmware: percents in [-100, 100], `a_percent` positive drives tail-first (stick back), a positive steer output speeds up the left wheel and turns the robot clockwise, headings in degrees growing clockwise, rates in degrees per second clockwise-positive. The plant is counter-clockwise-positive in radians; `heading_from_theta` and `gyro_from_w` convert. Upside-down driving is not mirrored: the sysid gates drop inverted windows first.

- `motor_to_plant`: the mixer writes left = -a + steer and right = -a - steer. Forward is positive linear, and a counter-clockwise turn (right wheel faster) is positive angular, as in `plant.py`.
- `closed_loop_rollout`: `a_percent`, `b_percent` and `auto_steer` are (B, N) on a `dt` grid starting at the window start. `history` is the plant command (linear, angular) on the same grid before the start, (B, M), most recent last, so the transport delay reads real commands for its first `delay_s`. Returns per-sample x, y, theta, v, w and the motor commands. The command in effect `delay_s` before each step is sampled linearly between grid samples and held for the step, as in `plant.delayed_commands` and `plant.simulate`. It is zero before the history starts.
- `from_telemetry`: seeds the loop from the telemetry sample at a window start. The setpoint is carried as an offset from the recorded heading. The integral is what is left of the recorded output once the feedforward and proportional terms come off.
- `BatchedMixer`: `mix_motor_outputs` plus `YawController`, stepped once per BNO055 sample. The firmware runs the yaw loop only when a fresh IMU sample lands (every 10 ms) and holds the steer output until the next one.

## `control/plant.py` window helpers

- `delayed_commands`: shifts a uniformly sampled command timeline back by `delay_s`, fractional and linearly interpolated between samples, which keeps the delay profile smooth enough to search on a 2 ms grid.
- `subsample`: thins the window set to at most `limit` windows, evenly across the run. Windows overlap heavily at a 50 ms stride, so thinning costs little information and keeps a 60-point delay profile search to minutes.

## `mujoco_sim/actuator.py`

`PlantParams` is everything the fit can move. Mass properties are fixed from CAD. Units are not in the field names because fitted `fit.json` files store them by name.

| Field | Meaning |
| --- | --- |
| `delay_s` | Firmware command to motion. The radio part is measured separately |
| `deadzone_left`, `deadzone_right` | Fraction of full command |
| `lr_gain_ratio` | Left gain over right gain |
| `joint_frictionloss` | N m at the wheel |
| `joint_damping` | N m s / rad at the wheel |

## `mujoco_sim/closed_loop.py`

`ClosedLoopSim` drives Mr Stabs Mk2 on CPU MuJoCo from body-axis stick commands in an arena. The command chain is the robot's:

- Linear and angular stick fractions become the CRSF percents the firmware reads. Channel 0 carries linear and the firmware negates it (`a = -linear`). The transmitter reverses the angular channel (`b = -angular`).
- The firmware mixer and its heading hold turn those into per-motor percents, and the drivetrain delay holds them.
- Command-tape shaping (deadzone, throttle curve, left/right gain, pack voltage) turns them into wheel volts.
- Motor polarity is +1: a positive per-motor command drives forward.

`FlipSwitch` decides when the firmware treats the robot as inverted and negates the throttle before the mixer. In the power-on DOWN position it follows the IMU's upside-down reading (`firmware_is_upside_down`, the firmware's `is_upside_down`) with its hysteresis, so a flipped robot keeps driving wedge-first and the heading hold restarts on the flip.

Arena walls and wall blocks are static geoms, and moving hazards (the house bot) are mocap cylinders, so wall contact is rigid-body contact.

`MovingBlock` is a robot ours can hit, placed every tick by `ClosedLoopSim.set_moving_blocks`. It is a cylinder of `radius`, or with `box` set, a box of full size `BoxSize(length_m, width_m, height_m)` with its length along the robot's heading. That is the shape the RL environment gives the opponent (`warp_nav.OPP_SIZE_M`).

## `mujoco_sim/firmware.py`

Mirrors `mix_motor_outputs` (`firmware/mr_stabs_mk2/src/main.cpp`) and `yaw_control::YawController` (`firmware/mr_stabs_mk2/lib/yaw_control/`). `test_firmware_yaw_mirror_matches_native_golden_fixture` replays a fixture generated from the native `yaw_control.cpp`. The fit never uses this module: it replays the logged per-motor commands, which already come out of the mixer.

Two firmware generations are mirrored, because `validate.py` replays recordings made before the yaw-rate loop:

- `legacy=True`: heading hold with `PidV1` every control loop. The integral accumulates the raw error and multiplies by dt only on output, and the tolerance check returns before the derivative term updates its previous error. The stick passes through while turning and for a 0.25 s coast after.
- `legacy=False` (default): the current firmware. In auto steer, `YawController` runs once per BNO055 sample (100 Hz): the stick commands a yaw rate, heading hold commands the rate back to the held heading, and an inner loop on the gyro sets the differential. Under 5% throttle with the stick centered it idles. It runs upside down too. Outside auto steer the stick passes through with a 1% deadband.

Sign and threshold conventions in `mujoco_sim/firmware.py`:

- The BNO055 heading (`orientation_x`) is degrees in [0, 360) and grows clockwise seen from above. `heading_from_yaw` converts the sim's counterclockwise yaw into it, and rates are clockwise-positive to match.
- `UPDOWN_THRESHOLD_UP_Z` mirrors `RIGHT_SIDE_UP_THRESHOLD` and `UPSIDE_DOWN_THRESHOLD` in `updown_sensor.h`: the sensed gravity z must pass -1.0 m/s^2 to read right side up and +1.0 m/s^2 to read upside down, and holds in between. As a chassis up-axis z component that is a +-0.102 band.
- `FirmwareMixer.resume` seeds the integral with the logged `pid_output` minus its feedforward and proportional terms. Seeding it with the whole output double-counts the feedforward in a turn. Measurement: `docs/experiments/sysid/2026-10-15_mujoco_open_loop_heading_hold.md`.

## `mujoco_sim/fit.py`

`WindowErrors` holds position and heading errors shaped (N, H) for N windows and H horizons, NaN where no measurement landed near a horizon. `pitch_rms_rad` is (N,) over the whole window and NaN for flat windows. `position_sigma_m` and `heading_sigma_rad` are (N, H): the sigma each error was scored against, the measurement's own or the window's.

A `Simulator` takes a window batch and one parameter set per world (`len == len(batch)`) and returns the recorded chassis qpos (N, S, 7) plus the recording interval. `run_stage` scores the starting point first, so a stage never hands back parameters worse than the ones it got.

## `mujoco_sim/greybox.py`

Fits the grey-box plant (`control/plant.py`) on the same windows as MuJoCo, so the MuJoCo model can be compared against it on held-out sessions. Both models are scored by the same `fit.window_errors` against the same raw measurements. The grey-box plant takes linear and angular commands from the per-motor tape through `motor_to_plant`, the inverse of the mixer (`u_lin = (l + r) / 2`, `u_ang = (r - l) / 2`). `plant.simulate`'s `traj[k]` is the state after step k, so `predict` prepends the start and sample k sits at `k * PLANT_DT`.

## `mujoco_sim/mass_properties.py`

Loads `simulation/assets/robots/mr_stabs_mk2/mass_properties.toml` into frozen dataclasses.

| Class | Field | Meaning |
| --- | --- | --- |
| `Body` | `com` | (3,) center of mass, m |
| `Body` | `fullinertia` | (6,) Ixx Iyy Izz Ixy Ixz Iyz about the COM, kg m^2 |
| `TagMount` | `upside_down` | The tag faces the floor when the robot is upright |
| `TagMount` | `translation` | (3,) tag center in the body frame |
| `TagMount` | `rotation` | (3, 3) `R_body_tag` |
| `Weapon` | `center` | (3,) point on the spin axis at the disk's mid-plane, body frame |
| `Weapon` | `radius` | Largest disk radius about the axis |
| `Weapon` | `half_width` | Half the disk thickness along the axis |
| `MassProperties` | `axle_x` | Axle of the driven wheels, body x |
| `MassProperties` | `follower_axle_x` | Axle of the belt-coupled wheels, or `None`. `total_mass` counts each wheel twice when it is set |

## `mujoco_sim/mjcf.py`

`GEAR_SOLREF` is the gear-coupling stiffness: a time constant of two timesteps, the minimum MuJoCo accepts, at the 1 ms `FIT_TIMESTEP_S`. Without a collision hull the skid sits at the bottom-front of the chassis COM box (`SKID_FALLBACK_X_M`, `CHASSIS_BOX_BOTTOM_Z_M`).

## `mujoco_sim/onshape_export.py`

- URDF `rpy` is extrinsic roll-pitch-yaw about fixed x, y, z, which is scipy's lowercase `"xyz"`.
- `LinkInertial.com_link` is (3,) and `inertia_about_com_link` is (3, 3), both in the link frame.
- `onshape_inertia_to_tensor` reads Onshape's "Mass moments of inertia" panel (g mm^2). The panel shows the tensor elements themselves, products already carrying the minus sign.
- `TagFrame`: a tag's marker frame in the body frame. `rotation` and `translation` form `T_body_tag`, mapping points in the OpenCV marker frame (origin at the tag center, x right and y up in the printed image, z out of the face) into the body frame. `black_square_side_m` is OpenCV's marker length. `tilt_from_body_z_deg` is the angle between the tag normal and the body's +z (or -z when underneath). `decodes_mirrored` is set when the CAD face only decodes mirrored. The CAD face is rasterized with the lightest faces painted first so the black pattern lands on top.

## `mujoco_sim/rollout.py`

- `CollisionSet`, loaded from `collision.toml`:

  | Field | Meaning |
  | --- | --- |
  | `pieces` | Convex OBJ pieces in the body frame |
  | `skid_point` | Lowest nose point, body frame; `None` on four wheels |
  | `rest_pitch_rad` | Nose-down pitch with both wheels and the skid on the floor |

- `rotor_diaginertia`: radial inertias are 0.6 of the axial. The radial value reaches chassis yaw, so the fitted plants and the yaw-loop gains assume 0.6. At 0.5 the reverse-spin yaw-loop test overshoots (88 deg against a 60 deg bound).
- `NCONMAX` and `nconmax_for`: contacts and constraint rows per world. The hull pieces, skid and both treads rarely give more than 8 contacts; the 24-contact floor covers a robot on its side. The broadphase pairs the floor plane with every robot geom, so `nconmax_for` returns at least `2 * ngeom` (Mrs Buff Mk3 has 30 pieces and four treads). MuJoCo Warp drops contacts on overflow. `njmax_for` allows `ROWS_PER_CONTACT` = 6 rows per contact (condim 6 on the treads), plus equalities and joint friction.
- `ParamArrays`, with W worlds: `geom_friction` (W, ngeom, 3), `geom_solref` (W, ngeom, 2), `geom_solimp` (W, ngeom, 5), `dof_damping` and `dof_frictionloss` (W, nv), `body_inertia` (W, nbody, 3), `actuator_gainprm` and `actuator_biasprm` (W, nu, 10), `actuator_forcerange` (W, nu, 2). `coast` is (W,), 1.0 where a zero command freewheels. `__post_init__` rejects any other rank or trailing size.
- `InitialState`: per-world planar pose and velocity in the field frame. `pitch_nose_down_rad` is positive nose down. `initial_qpos_qvel` composes the orientation as q = q_yaw * q_pitch (intrinsic z then y).

The free-joint angular velocity is in the body frame, so for a pitched body a pure field yaw rate has components on body x and z.

`WarpRollout` takes tapes shaped (W, T, 2) volts plus a (W, T, 2) zero-command mask. `run` resets every world to its initial state, replays the captured step graph T times and returns (W, S, 7) chassis qpos sampled every `record_every` steps (S = ceil(T / record_every)).

## `mujoco_sim/session.py`

Loads `smooth_tag_poses.py` session bundles into rollout windows for the MuJoCo fit. A bundle is the directory `playground/calibration/smooth_tag_poses.py` writes per recording:

- `smoothed.csv`: uniform-grid smoothed pose
- `measurements.csv`: raw tag detections
- `commands.csv`: ESP32 events on the app clock
- `windows.csv`: gated windows with initial states
- `session.json`: noise floor, clock fit, and the rest

The fit's loss reads the raw measurements, never the smoothed poses. The smoothed state only supplies each window's initial conditions. `TAPE_PAD_S` is the tape history kept before each window start so the delay shift never runs off the front; it must exceed the `delay_s` upper bound in `fit.PARAM_SPECS`.

`WindowBatch`:

- `session_index` indexes `sessions`. `kind` is one of `WINDOW_KINDS` ("flat", "nose_lift").
- `u_raw` is (N, pad + steps, 2), the per-motor command in [-1, 1], forward positive. `vbat` is (N, pad + steps).
- `firmware` is the logged signals on the same grid, in `FIRMWARE_COLUMNS` order (a_percent, b_percent, pid_output, left_cmd, right_cmd in percent, orientation_x in degrees), for the closed-loop check.
- `polarity` is +1 per window if a positive logged motor command drives forward, -1 otherwise (see `motor_polarity`).
- `meas_sigma_xy` and `meas_sigma_yaw` hold one sigma per measurement per window. Tag fixes carry the session noise floor; a YOLO fix carries its own, larger one. An empty list means every measurement uses the window's `noise_*`.
- `tapes` returns `Tapes(volts, zero)`, each (N, steps, 2), for one parameter set per window or one shared. It applies delay, deadzone, throttle curve, left/right ratio and vbat.
- `build_batch` recomputes each window's initial wheel speeds with `rolling_wheel_speeds`, the no-slip relation in the MJCF's convention (positive rolls forward).

## `mujoco_sim/step_response.py`

`StickTape.linear_frac_fwd` and `angular_frac_ccw` are body-axis stick fractions on a uniform `STICK_TICK_S` grid from the step's start: forward positive, counterclockwise positive (`ClosedLoopSim.step`). `fit_armature` clamps both measured and simulated lifts at `FLIP_LIFT_DEG` before differencing.

## `mujoco_sim/validate.py`

`compare_models` returns `ModelComparison(errors, qpos, record_dt)`. `errors` is the JSON-safe report; `qpos` and `record_dt` feed the nose-lift check. `YAW_LOOP_SINCE_NS` marks when the yaw-rate loop replaced heading hold: 2026-10-05 00:00 UTC. `firmware_closed_loop` replays recordings that predate it with `legacy=True`.

## `perception/tag_pose_smoother.py`

Smoother internals and tuning are in `docs/experiments/sysid/2026-10-25_sysid_tag_smoother.md`. This section is the data contracts.

### Data layout

- `Candidates` holds N detections, two IPPE solutions each. `t_up_body` is (N, 2, 4, 4); slot 1 repeats slot 0 where `n_solutions` is 1. `tilt_from_rest_rad` is the attitude error from rest.
- `IppeChoice.solution_index` is 0 or 1 per detection. `close_call_decided_by_neighbours` marks detections whose two tilts were within the margin and were decided by neighbouring frames. `RawPoses.yaw_wrapped` is in [-pi, pi).
- `TagMount.tag_faces_floor_when_upright` is the tag that faces the floor when the robot is upright. The `[tags.N]` table key stays `upside_down` and defaults to the sign of the tag normal's body z. A table gives either `translation_m` and `rotation` (`R_body_tag`) or a 4x4 `t_body_tag`.
- `up_from_field` returns `UpFrame(t_up_field, flipped)`. `flipped` is true when the camera sits at negative field z, so field z points into the floor.
- `Knots` is shared by every axis of one smoother. `kind[k, a]` is a `KnotKind` (int8) saying what axis `a` measures at knot `k`: nothing, its position, or its rate. `variance` is the measurement variance. `t_since_t0_s` is seconds from `t0_ns` and must be non-decreasing.
- `SmootherRun` arrays lead with the batch B: `m_pred` is (B, K, A, 3), `p_pred` is (B, K, A, 3, 3), `outcome` is (B, K) holding `Outcome` codes. `nis` is the joint normalised innovation squared, NaN without a measurement. `innovation` is (B, K, A), normalised per axis, NaN if unused. `Outcome.ACCEPTED_AFTER_LOST_TRACK` is a fix that failed the gate while the filter had lost track, taken with inflated covariance.
- `CrossValidation` keys by group name: `axes_by_group` lists the axes sharing one q, `q_candidates_by_group` the q values tried, `heldout_score_by_group` the mean clipped squared normalised held-out error.
- `ImuYawRate.app_clock_stamp_ns` is a uniform grid on the app clock. `ccw_rate_rad_s` is counter-clockwise positive under the assumed heading sign.
- `SessionSmoothing.detections_df` has one row per detection (the `measurements.csv` columns plus extras). `smoothed_grid_df` has the `smoothed.csv` columns plus pitch and roll rates. `noise_calibration_segments` are detection index ranges the noise model came from. `yolo_fixes` has the YOLO position fixes on frames the tag missed (stamp, x, y, yaw, sigmas, rejected), or None.
- `_planar_knots` returns `KnotBuild(knots, source_index)`. `source_index` is the detection index per knot, or `KnotSource.IMU` or `KnotSource.EXTRA`. `_AxisFit.gate_exempt_after_recheck` marks knots the smoothed re-check let through the gate.

### Gating and branches

- An angle measurement is unwrapped onto the branch nearest a reference: the prediction normally, but after a gap longer than `branch_gap_s` since the axis's last fix, the last fixed angle carried forward at its fixed rate.
- Rate knots (the IMU) skip the gate. A wrapped angle fix cannot tell a rate from that rate plus a whole turn per frame, so gating the IMU against that state would reject the one measurement that can pull it back.
- `smooth_with_recheck` gives gated position fixes a second look against the smoothed track. A fix within the same chi-square gate of the smoothed state (smoothed covariance plus measurement noise) is let through on the next pass. An outlier stays rejected. It returns the final run and the exempted knots.
- `interpolate_smoothed` predicts from the filtered state at the knot before, then applies the RTS gain against the smoothed and predicted states at the knot after. Times outside the knot span clamp to the end knots.
- `_fuse_yolo` sorts the fixes by stamp, so the outcome at each `KnotSource.EXTRA` knot lines up with the same row of the fixes table.

### Tilt

Tilt is fed unwrapped and never wraps its innovations (`_TILT_OPTIONS.angular` is all false). YOLO-derived tilt is moved onto the branch of the unwrapped tag fix before each stamp with `unwrap_onto`. The reported `pitch` and `roll` are wrapped on output.

### Options

`FilterOptions`:

| Field | Meaning |
| --- | --- |
| `angular` | Per axis: wrap the innovation to [-pi, pi) |
| `prior_std` | Per axis: (position, rate, acceleration) std |
| `reacquire_s` | Accept a gated position fix after this long without one (default 0.3 s) |
| `branch_gap_s` | Gap above which the innovation reference is the state carried forward at its rate (default 0.1 s). `_with_rate` sets it to infinity so yaw unwraps onto the prediction's branch always |

`SmootherOptions`:

| Field | Meaning |
| --- | --- |
| `still_yaw_rate` | rad/s below which the tag counts as "not turning" for the IMU rate noise (default 0.15) |
| `spin_rate` | rad/s above which the tag counts as "a spin" for the IMU agreement check (default 3.0) |

### Gated clean fixes

The smoother keeps a fix out when its innovation exceeds the chi-square gate, and a few clean fixes are rejected anyway. The fused IMU's sigma is its spin residual (about 0.26 rad/s in the synthetic scenario), so a few clean fixes at a blackout edge and mid-spin sit right at the innovation gate. `test_tag_pose_smoother.py` allows `MAX_CLEAN_FIXES_AT_GATE` (3) of them.

## `perception/sysid_windows.py`

Fit windows for the drivetrain sysid: gating and initial state. Input is the smoothed grid from `tag_pose_smoother.smooth_session`. Three per-sample gates run first and cut the session into spans:

- near_rail: the axle center within `robot_length_m` of a box rail, where wall contact is possible.
- esp32_gap: inside a gap in the ESP32 stream longer than `esp32_max_gap_s`, or outside the stream altogether, since the fit has no command tape there.
- flip: the upright/inverted state changes, which the fit cannot start or end across.

Each span shorter than `window_s` is dropped; the rest are cut into `floor(L / window_s)` equal windows, so every window is between `window_s` and `2 * window_s` long (1 to 2 s by default). A window is then dropped if accepted tag detections cover less than `coverage_min` of its camera frames. A kept window is `nose_lift` when its pitch leaves the rest pitch by more than `pitch_limit_deg` anywhere, otherwise `flat`.

The initial state per window is the smoothed state at its first sample. Wheel speeds come from the no-slip relation in the body frame, `(v -/+ w * track_half_width) / wheel_radius` for left and right, with v the forward speed along body x and w the yaw rate about body z. A wheel spinning to roll the robot forward is positive, which is positive rotation about body +y. On an inverted robot body z points down, so the body-frame yaw rate is minus the field one.

`WindowOptions.box_half_extent_m` is half the 1.52 m test box side (default 0.76 m).

## `recording/esp32_clock.py`

Puts ESP32 events on the app clock and measures the delays around them. The app stamps every ESP32 event with its receive time, which carries WiFi jitter. The robot's `timestamp_ms` has no jitter but runs on its own crystal. `fit_robot_clock` maps one onto the other with a line (offset and drift). Receive delay is never negative, so the line is the lower envelope of (robot time, receive time): it rests on the fastest deliveries. The minimum transport latency stays inside the offset.

A reboot resets `timestamp_ms`, so each run of increasing robot time gets its own line.

- `_lower_envelope_line`: (intercept, slope) of the line under every point that minimises the summed gap. This is a two-variable linear program whose optimum is the lower-hull edge spanning mean(x) (Moon, Skelly and Towsley 1999, skew estimation for network delay measurements).
- `fit_robot_clock`: the line is fitted to receive time minus robot time, so the slope is the drift and stays near zero.
- `cross_correlation_delay`: estimates the lag between two signals on the app clock. It serves stick channels against the ESP32's `a_percent`/`b_percent` (the radio link delay) and BNO055 yaw rate against tag yaw rate.
- `radio_link_delay`: how long a stick move takes to show up in `a_percent`/`b_percent`. Both signals are held between samples and correlated on their first differences.
- `_smooth`: Gaussian blur that treats NaN as zero.

## `recording/sysid_io.py`

`field_transform_constancy` measures how far the camera pose wanders from its median over the session. The tripod does not move, so anything above a millimetre or two means a field re-init or a bumped tripod.

## AprilTag pose selection

- Rest pitch: the floor-up vector in the body frame while the robot rests with a tag showing. Upright it leans back by the rest pitch (the robot sits nose down on its wedge tip). Upside down it is body -z.
- IPPE disambiguation: the index of the IPPE solution to trust, from solutions sorted by reprojection error. A solution that puts the camera under the floor is dropped when the other does not. Of the rest, the one whose implied floor up lies closest to the reference wins, because the other IPPE solution is the planar flip. The reference is `floor_up_camera` when known; without it the reference is camera -z, which only decides when the two solutions differ by more than 15 degrees against it. Otherwise, or when the two lie within 2 degrees of a known floor, the lowest reprojection error wins.
- Floor normal learning: the floor's up direction in the fixed camera frame is learned from the tags themselves. Every frame gives two candidates, one per IPPE solution. The right ones all agree; the flipped ones move with the robot's position and heading. Until one direction has the support of most frames and no rival cluster does, `estimate()` stays empty and selection falls back to reprojection error. After that it tracks slowly, so a wheelie or a flip does not drag it.

## Sysid scripts

### `playground/calibration/analyze_apriltag_mcap.py`

Solves field-plane robot poses from an `apriltag_track.py` image recording. It replays the recorded camera images, re-runs the AprilTag detection (`apriltag_detect`, the same code the live preview used), and solves the (t, x, y, yaw, visible) truth CSV that `fit_plant_calib.py` consumes. Two solves, mirroring capture:

1. The floor extrinsic from the `/floor/image` lock burst (board visible).
2. The robot tag pose in that floor frame, per `/camera/image` frame.

Images are recorded raw, so the floor lock, detector tuning, intrinsics, and yaw offset can be corrected and re-run without re-driving the robot. `ROBOT_DETECT_SCALE` upsamples robot-tag detection; the floor board is solved at 1x.

Transmitter data recorded alongside the frames is carried through to the truth CSV as extra columns:

- A read-only stick capture (radio connected, no `--drive`) adds `tx_sample_t`, `ch0..chN`, joined per frame by timestamp.
- An `apriltag_track.py --drive` excitation adds the active scripted command (`cmd_lin`, `cmd_ang`, `trainer_lin`, `trainer_ang`, `label`), zero-order-held onto each frame time. `fit_plant_calib.py` fits the plant directly from this single CSV.

```
source scripts/activate_python.sh
python playground/calibration/analyze_apriltag_mcap.py playground/calibration/out/apriltag_track.mcap \
    --out playground/calibration/out/truth_log.csv
# re-solve with a corrected yaw offset (overrides the value recorded at capture)
python playground/calibration/analyze_apriltag_mcap.py rec.mcap --yaw-offset-deg 90 --plot fit.png
# also emit a Foxglove overlay (camera frames + solved pose markers via TF)
python playground/calibration/analyze_apriltag_mcap.py rec.mcap --overlay-mcap overlay.mcap
```

### `playground/calibration/check_sysid_recordings.py`

Checks Mr Stabs Mk2 drive recordings before they go into the MuJoCo plant fit. It reads each MCAP once and reports, per recording, whether it can feed `smooth_tag_poses.py` and `fit_mujoco_plant.py`, and what is missing if it cannot. Nothing here fits anything.

Checks, each PASS / WARN / FAIL:

- file: the MCAP has a summary. A missing one means it was copied while still recording or the app died mid-write.
- camera: frame rate from `/camera/frame_meta` (60 fps target) and the longest gap (`CAMERA_GAP_FAIL_S`).
- video: `/camera/video` against `/camera/frame_meta`. The ZED X One records its footage into the MCAP, not an SVO. Over 100% is normal (the encoder starts at camera open); under it means the encoder dropped frames.
- field: a field-to-camera transform exists and stays put. Each field init redefines the field frame, so a session is a run of locks. The report gives how many there were, how long the longest held, and whether the camera wandered within it (more than 5 mm or 0.5 deg means a bumped tripod).
- robot tags: share of camera frames with a robot AprilTag, overall and while the sticks are off center, and the tag's pixel size (`TAG_MIN_PX`). The smoother's window gate is 70% coverage. Recordings from the match profile carry no tags; they need `/camera/video` replayed through the sysid keypoint model first (`config/playback/mr_stabs_mk2_sysid_playback.toml`).
- our-robot track: share of frames with an `OUR_ROBOT` marker (YOLO path, match profile).
- robot telemetry: `/robot/espnow_telemetry`, or `/robot/esp32_diagnostics` before 2026-10-25. Events against the 100 Hz telemetry rate, gaps, pack voltage. The fit's command tape is the per-motor `left_cmd`/`right_cmd` from this stream.
- sticks: the maneuver menu, measured from the radio's stick channels, or from the telemetry's `a_percent`/`b_percent` when the radio was not on USB (`sticks_from_telemetry`). Seconds of straight holds per level and direction, creep, spins, arcs, reversals, punches, and hard stops, against `MENU_TARGETS`. `stick_centers` takes each channel's resting value as the value held longest.
- SVO (ZED 2i only, with `--svo-dir`): an SVO whose frame stamps overlap this recording, and how many frames it holds.

```
python playground/calibration/check_sysid_recordings.py data/recordings/*2026-10-11*.mcap \
    --svo-dir data/svo --plots playground/calibration/out/mujoco_sysid/checks
```

Exit status is 1 if any recording has a FAIL. Stick channels (`LINEAR_CH`) are CH1 linear and CH2 angular, +/-1024 full scale, from the sysid and zed_box profiles. `PERCENT_TO_CHANNEL` converts telemetry `a_percent`/`b_percent` to radio channel units: CH1 runs opposite to `a_percent` and CH2 with `b_percent` (from `mujoco_sysid/s01_2026-10-15/session.json`).

### `playground/calibration/detect_robot_yolo.py`

Runs the YOLO pose engine over a sysid recording's video and writes our robot's detections: one row per `MR_STABS_MK2` detection with the box, its confidence, and the front and back keypoints in pixels. A frame can have several, because the model sometimes labels the floor marker board as the robot. `smooth_tag_poses.py --yolo` drops those with size gates, keeps the best survivor per frame, projects it onto the field, and uses it where the tag drops out. It is its own stage so the detections can be inspected and the smoother does not need TensorRT.

```
source scripts/activate_python.sh
python playground/calibration/detect_robot_yolo.py data/recordings/<session>.mcap \
    --out playground/calibration/out/mujoco_sysid/<session>/yolo.csv
```

### `playground/calibration/find_flip_accel.py`

Finds the max forward acceleration a robot tolerates before it flips (trainer link only, no tracking). A torque-happy rammer such as Mr Stabs backflips when the throttle is slammed from rest: the command's rise rate, not the top speed, pitches it over. The script ramps the forward command 0 to target at an escalating slew rate and stops at the rate that flips it. The result is the safe forward command slew rate.

No camera, MCAP, or AprilTag. The operator watches the robot and answers whether it flipped. Ctrl-C during a ramp aborts that trial and zeroes the link.

Safety:

- Guard plates ON, competition battery, surface matched to the NHRL arena floor.
- Clear, bounded lane several metres long: each trial ramps to full speed and coasts.
- Driver sticks centered: in trainer mode the radio adds stick input to the command.
- The link is zeroed and disarmed on every exit (normal, Ctrl-C, exception, per-trial hard timeout).

```
source scripts/activate_python.sh
python playground/calibration/find_flip_accel.py --robot mr_stabs_mk2
```

`main.in_ms2` converts the slew rate to m/s^2: d(speed)/dt = d(command)/dt * v_max, since speed is about command * v_max (deadzone and lag aside).

### `playground/calibration/fit_encoder_scale.py`

Fits metres per encoder count from hand-pushed runs over measured distances (runbook E4). Push the robot with the motors disarmed between two marked points, dwelling at each. The script reads the count at each dwell and fits counts against distance.

```
source scripts/activate_python.sh
python playground/calibration/fit_encoder_scale.py \
    --run 0.5  LOG-38.TXT LOG-39.TXT LOG-40.TXT \
    --run 0.75 LOG-42.TXT LOG-43.TXT LOG-44.TXT \
    --run 1.0  LOG-45.TXT LOG-46.TXT LOG-47.TXT \
    --plot out/encoder_scale.png
```

Use several distances. A fixed error (the marks, or the robot not quite stopping on them) is a constant offset in counts. With one distance it lands entirely in the scale; with three it shows up as an intercept, which the script reports. An out-and-back makes the closure error visible.

- `find_dwells`: stretches where the count barely moves. The tolerance is in counts, not a velocity threshold, because a hand push is slow.
- `Fit`: both fits. The through-origin fit is the model (zero distance, zero counts). The differential slope is unaffected by a constant per-pass offset, so when the intercept is real it is the one to believe. `far_dwell_spread_counts` is the per-pass spread among the candidate far dwells and makes wheel creep visible.
- `fit_scale`: counts against distance through the origin, plus an intercept as a diagnostic. An intercept far from zero means a fixed error per pass and inflates the scale fitted from short pushes.

### `playground/calibration/fit_gyro_noise.py`

Fits the gyro noise model from a long stationary recording (runbook E2). The Allan deviation curve falls while averaging still helps, bottoms out where bias instability takes over, and climbs once the bias is wandering. The tau at the minimum says how often the still holds need retaking. The slopes on either side are the process-noise terms the filter needs for heading.

```
source scripts/activate_python.sh
python playground/calibration/fit_gyro_noise.py \
    playground/calibration/out/gyro_bias/LOG-97.TXT \
    --plot playground/calibration/out/gyro_noise.png
```

It runs on one log, so a 30 min recording of a robot doing nothing is the whole experiment, and needs no radio and no clear floor. The curve is trustworthy only out to about a tenth of the record. Yaw is taken along measured gravity, as in E0 and `fit_gyro_scale.py`. Estimators follow IEEE 952:

```
N  angle random walk    sigma = N / sqrt(tau)      slope -1/2, read at tau = 1 s
B  bias instability     sigma = 0.664 * B          slope 0, the floor of the curve
K  rate random walk     sigma = K * sqrt(tau / 3)  slope +1/2, read at tau = 3 s
```

- `MAX_CLUSTER_FRACTION`: clusters longer than this fraction of the record average too few independent samples. The curve is truncated there.
- `overlapping_allan`: overlapping Allan deviation on the integrated angle, `sigma^2(tau) = sum (theta[k+2m] - 2 theta[k+m] + theta[k])^2 / (2 tau^2 (n - 2m))`. Overlapping buys roughly sqrt(3) tighter confidence for the same record.
- `_read_at`: fits the line of the known slope through `band` and evaluates it at the target. `band` defaults to the decade around the target. Pass one explicitly for rate random walk on a record where the +1/2 region has not opened up by tau = 3 s.
- `RRW_RISE_RATIO`: a record only measures rate random walk if its curve has turned up. `AllanCurve.rrw_deg_per_s_sqrt_s` is `None` when the curve never turned up inside the record, which says the record is too short, not that the gyro is good.
- `yaw_rate`: gyro projected onto measured gravity, in deg/s. The record is stationary, so the mean accelerometer vector is gravity.
- `make_plot`: the +1/2 guide is drawn only over the region that rises.

### `playground/calibration/fit_gyro_scale.py`

Fits the gyro scale factor from rotations through a known angle (runbook E3). Integrating the gyro over a rotation of known size and comparing to that size gives one number: how much the gyro over- or under-reports. Every heading estimate is multiplied by it.

```
source scripts/activate_python.sh
python playground/calibration/fit_gyro_scale.py \
    --run 3600 LOG-a.TXT \
    --run -3600 LOG-b.TXT \
    --plot playground/calibration/out/gyro_scale.png
```

`--run` takes the true rotation in degrees, signed: positive counter-clockwise viewed from above. Run both directions. A scale that differs between them is a mounting or axis-sign problem, and averaging the two would bury it.

The yaw axis comes from gravity: at rest the accelerometer reads +1 g on whichever axis points up, and the IMU puts gyro and accel on one right-handed triad, so positive yaw is counter-clockwise from above. That survives a tilted remount.

Bias dominates the error budget on a slow rotation: 0.05 deg/s over a 20 s turn is 1 deg. The bias is taken from the still hold that opens the run and ramped to the one that closes it. A run with only one still hold gets a constant bias and says so.

`TURN_EPS_DPS` is the rate below which the robot counts as stationary when finding where the turn starts and stops. The fit is through the origin over signed angles, so both directions constrain one number.

### `playground/calibration/fit_mujoco_plant.py`

Fits the Mr Stabs Mk2 MuJoCo plant to smoothed sysid sessions, then validates on held-out ones. Inputs are session bundles from `smooth_tag_poses.py` (`playground/calibration/out/mujoco_sysid/<session>/`). Hold out whole sessions, never windows from a training session.

```
# fit on three sessions, validate on a fourth
python playground/calibration/fit_mujoco_plant.py fit \
    out/mujoco_sysid/s01 out/mujoco_sysid/s02 out/mujoco_sysid/s03 \
    --holdout out/mujoco_sysid/s04 --restarts 4

# re-validate an existing fit
python playground/calibration/fit_mujoco_plant.py validate out/mujoco_sysid/fit_<ts> \
    out/mujoco_sysid/s04
```

Validation writes `validation.json` and, under `plots/`, the loss curves, the restarts' parameters, held-out rollouts against the measured robot, and the error per horizon (`auto_battlebot/mujoco_sim/plots.py`). `validate --name X` writes `validation_X.json` and `plots_X/` instead, so one fit can be scored on several holdouts side by side.

`fit` prints its total rollout-steps up front. 1.64e9 steps take about 27 minutes on the RTX 4080 laptop.

### `playground/calibration/fit_plant_calib.py`

Fits the full tank drivetrain plant from a calibration session. Input is one CSV from `analyze_apriltag_mcap.py` run on an `apriltag_track.py --drive` recording. That file carries both halves the fit needs, recorded in one process on one absolute `CLOCK_MONOTONIC` clock, so no time alignment is done here:

- the solved ground-truth pose per frame (t, x, y, yaw, visible)
- the scripted command active at that frame (cmd_lin, cmd_ang, trainer_lin, trainer_ang, label)

The identification is first-order AR(1) (`v[k+1] = a*v[k] + b*cmd[k]`, `tau = -dt/ln(a)`, `gain = b/(1-a)`) plus lag by cross-correlation, driven by AprilTag truth and the labelled excitation protocol. It yields per-direction deadzone, forward/reverse gain asymmetry, separate accel and coast (decel) time constants, the steer-brake coupling, and the actuation lag.

Outputs: a printed report, the `[our_robot]` block to paste into `simulation/kinematic_sim.toml`, a recommended `lifted_deadzone_percent` for the robot config, and a validation plot (`--plot`).

```
source scripts/activate_python.sh
python playground/calibration/fit_plant_calib.py playground/calibration/out/truth_log.csv \
    --plot playground/calibration/out/fit.png
```

`load_session` reads commands from every row (the full timeline, including frames where the tag was not visible) and pose only from visible frames. The sync twitches at the protocol's start and end are a visual cross-check in `--plot`.

Frames before the first issued command have empty command cells; the command there is zero and the label unset.

- `fit_deadzone`: smallest staircase command level (same sign) whose sustained motion exceeds eps. It works per command level (the staircase holds each command about 0.8 s), comparing the median |velocity| over the hold to the threshold.
- `_rise_tau`: the leading rise only.
- `_drive_segments`: steady-state speed from in-frame samples only.
- `fit_drive_segments`: per held-drive segment, steady-state gain and rise tau, bucketed by command sign. Returns `(gains_fwd, gains_rev, taus_fwd, taus_rev)`. Gain is Vss over the effective command (deadzone removed).
- `_decay_segments`: the coasting portion is measured, not interpolated.
- `fit_steer_brake`: fractional forward-speed loss per unit |angular command| during the steer-brake grid. One steady-state forward-speed sample per grid cell (last half of each drive segment); `speed/speed0 = 1 - coeff*|ang|`.
- `_latency_stack`: pools every clean command edge in the session and stacks the acceleration that follows it. Linear edges are timed against dv/dt, angular edges against dw/dt, since the transport path (Crossfire, ESC, mechanical) is shared. Any edge whose response window overlaps a truth gap is dropped. Each edge contributes its post-edge acceleration, sign-flipped to the step direction and scaled by the channel's own accel spread. Lag is the peak of the averaged acceleration pulse.

## `playground/calibration/make_print_tags.py`

Generates print-ready AprilTag PDFs for drivetrain calibration: a floor grid and a robot tag. The PDF export maps pixels-per-mm to DPI, so a 100%-scale print is dimensionally exact. Tags are rendered with `cv2.aruco` (`DICT_APRILTAG_36h11`), matching the `GridBoard` model `apriltag_track.py` uses for pose.

The script prints the estimated marker size in pixels for the given mount height, assuming `apriltag_track.py`'s OAK-1 W at 1080p (1920x1080 at 60 fps). Face-on, 36h11 needs about 18 px per edge to decode; aim for 30 px or more. If the estimate is small, increase `--floor-marker-size` / `--robot-tag-size`, lower the mount, or use 4K.

```
source scripts/activate_python.sh
python playground/calibration/make_print_tags.py --out-dir playground/calibration/print
```

The floor grid is normally the manufactured board (3x5 of 65 mm markers, 15 mm gaps, ids 160..174 numbered right-to-left per row); the script reproduces it only for a reprint. The robot tag (id 20) is what you print here. Print at 100% (no "fit to page") and tape the tag flat on the robot top. `save_pdf` stores the image bilevel so PDF keeps the marker edges lossless.

## `playground/calibration/make_robot_tag_3d.py`

Generates a multi-material 3D-print file for the robot's AprilTag (Bambu Lab AMS), the 3D-print counterpart of `make_print_tags.py`. It emits a white base with the black AprilTag cells raised on top as a second material. Two watertight objects share one coordinate frame, so in Bambu Studio you assign white to the base and black to the tag and slice.

The black geometry comes from the `cv2.aruco` `DICT_APRILTAG_36h11` bit pattern, so it matches what `apriltag_track.py` detects. The script re-detects a top-down raster before writing to verify id and orientation. A white margin gives the quiet zone. The 3MF is written directly (a zip of XML), so it needs no `lxml`.

Outputs, written to `--out-dir`:

```
robot_tag.3mf         two objects (white base + black tag), pre-colored
robot_tag_white.stl   base only
robot_tag_black.stl   tag only
```

```
source scripts/activate_python.sh
python playground/calibration/make_robot_tag_3d.py --out-dir playground/calibration/print_3d
```

Defaults: id 0, 130 mm tag, 2 mm white base, 0.6 mm raised black, 16 mm white margin (about a 1-cell quiet zone). The plate is 162 x 162 mm. In Bambu Studio, import `robot_tag.3mf`, confirm the tag object is black filament and the base is white, then slice. Note the tag's physical edge (`--tag-size`) and heading offset for `apriltag_track.py --tag-size` / `--yaw-offset-deg`. `black_cell_boxes` merges consecutive black cells in a row into one box to keep the mesh small.

## `playground/calibration/plot_jig_runs.py`

`yaw_rate_dps(log)` is yaw about the measured gravity axis in deg/s (`JigLog.gyro_rad_s` is rad/s). It falls back to the z row when there is no still segment to measure gravity from.

## `playground/calibration/plot_sysid_bundle.py`

Plots a smoothed sysid bundle written by `smooth_tag_poses.py`. Writes under `<bundle>/plots/`:

- `overview.png`: the whole session on one time axis. Which source fixed the pose at each moment (tag, YOLO, neither); the smoothed yaw rate against the robot IMU's on a log scale; pitch against the flat band the window gate uses; x and y against the box the near-rail gate assumes; and the fit windows by kind.
- `floor.png`: top-down, every tag and YOLO fix, the smoothed path, the box, and where each fit window starts.
- `dropouts.png`: one top-down panel per tag dropout longer than `--min-gap-s` in which the robot stayed upright, 0.5 s either side. `--compare` overlays a second bundle of the same recording.

```
python playground/calibration/plot_sysid_bundle.py \
    playground/calibration/out/mujoco_sysid/<session>_yolo \
    --compare playground/calibration/out/mujoco_sysid/<session>_tag
```

## `playground/calibration/smooth_tag_poses.py`

- YOLO fixes carry position and heading only; the 2D keypoints carry no height or tilt.
- The video frames YOLO filled in count as camera frames for window coverage, so a dropout the fixes cover reads as covered and a rejected fix still counts against it.

## `playground/calibration/sysid_coverage.py`

Unpaced replays advance on recording time, so a fast replay closes the same windows as a live run would.

## `mujoco_sim/onshape_export.py` frames and mass

The export's root frame is the Onshape assembly frame: the axle runs along `X_asm` and the robot faces `-Y_asm`. The body frame used everywhere else (C++ filter, MJCF, smoother) is FLU at the axle midpoint, so `x = -Y_asm`, `y = +X_asm`, `z = +Z_asm`. The export carries no mass for the motors, ESCs, flight controller, batteries or switch (Onshape mass overrides do not survive it), so only the wheels are lumped from it. The chassis is the Onshape whole-assembly total minus the wheels; see `chassis_from_assembly`.
