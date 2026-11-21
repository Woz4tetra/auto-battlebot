# Plant fit modules

How the jig fit, the plant model, and the MuJoCo distillation produce the parameters `docs/reference/control/plant_backed_control.md` consumes.

## Jig fit: `calibration/jig/jig_fit.py`

Loading, per-phase fits, joint fit, and scoring for velocity jig sessions. `fit_jig_plant.py` is the CLI over this module; `jig_report.py` and `fit_process_noise.py` import from it.

The fit runs in two stages:

- **Stage A, per phase.** Each waveform is designed so one group of parameters dominates, so it fits in near-closed form: deadzone staircases, gains and rise constants from steps, decel from coast tails, coupling and straight-line drift from the grid, and transport delay pooled from an onset stack over every command edge. It also reports per-parameter spread.
- **Stage B, joint by simulation error.** Starting from stage A, minimize multi-step open-loop prediction error over sliding windows at the horizons the filter coasts over.

Transport delay is profiled on a grid rather than handed to the optimizer, which also produces a profile-likelihood curve.

Runs are routed to fits by waveform kind, channel and role, not by a fixed list of experiment ids.

### `fit_coupling`

Steer-brake and angular droop from the E13 grid:

```
v = k_lin * u_lin_eff * (1 - c_sb * |u_ang_eff|)
w = k_ang * u_ang_eff * (1 - c_ad * |u_lin_eff|)
```

Both are regressed through the origin on fractional loss. A coefficient whose spread covers zero does not go in the model.

Angular droop and straight-line drift both make yaw depend on the linear command. They separate on turn symmetry: for a cell at (u_lin, +u_ang) and its mirror at (u_lin, -u_ang), droop is a fractional loss that flips with the turn and drift is additive. The half-difference isolates droop; the half-sum isolates drift.

### `fit_drift`

Straight-line drift: the yaw a pure forward command produces on its own. Two terms, fitted together and reported separately:

- `c_drift`: a left/right gain mismatch, so yaw scales with the linear command.
- `c_drift_bias`: an asymmetric drag torque, so yaw is constant while sliding.

Both flip sign in reverse, so both directions are required.

### `fit_delay`

Transport delay from every clean command edge in the session, pooled. Linear edges are timed against forward acceleration, angular edges against yaw acceleration. The peak of the averaged acceleration pulse sits at the transport delay: for a pure delay in front of a first-order lag, dv/dt is zero until motion starts and jumps to its maximum at the delay.

The estimate is a transport delay only if the clocks were aligned. On an identity clock it is `clock_offset + delay`, and the report says so.

### `joint_fit`

Minimizes windowed open-loop error over the structure's free parameters with a soft-L1 loss. Delay is fixed here and profiled outside. `bounds` overrides `PARAM_BOUNDS` per parameter. The match fit uses it to hold gains and time constants inside the jig error bars widened 3x.

### `profile_delay`

Grid search over the transport delay, refitting the continuous parameters at each point. The profile curve is reported with its minimum: a flat profile means the delay is not identifiable from this excitation.

Resolution is bounded by the analysis grid. On synthetic data with a known 45 ms delay, a 200 Hz grid puts the minimum at 50 ms and a 500 Hz grid puts it at 44 ms. Quote the delay no finer than half a grid step.

## `calibration/jig/velocity_jig.py`

`ClockFit` maps jig microseconds to host seconds as offset plus linear skew. Cross-correlating command edges against measured acceleration recovers only `clock_offset + transport_delay`, so the offset is measured separately.

## `calibration/match/match_windows.py`

- `ReplayTrack`: `svo_index` is strictly increasing with gaps where frames dropped. `stamp_ns` is the original-clock image stamp.
- `load_replay_track`: ticks are associated to `/camera/frame_meta` by log order. The association is verified through the stamp rebase: marker header stamp minus the frame's `image_stamp_ns` must be one constant across the run. A drifting offset (std above 1 ms) raises.
- `CommandLog`: `auto_switch_latched` is a forward-filled bool.
- `load_commands`: decodes the `opentx_transmitter` channel readback into body commands, `lin = (ch0 - ch1) / 2048` and `ang = -(ch0 + ch1) / 2048`, matching `drive_protocol.MixConfig.from_channels` at readback scale 1024. Entries are logged on change; consumers zero-order hold between them.
- `MatchRun`: `role` is a `RunRole` (`fit`, `holdout`, `validation`, `video`). `t` is seconds from the grid start, uniform. `v_body_mps` is the signed body forward speed finite-differenced from pose, and `yaw_rate_radps` is the yaw rate.
- `central_difference_velocity`: body speed and yaw rate by central difference over live neighbors. Edges of a live stretch use one-sided differences, isolated live samples get NaN. The third return is the mask of samples with both neighbors live.

## `control/plant.py`

- `effective_command`: deadzone removal and rescale, per sign.

  ```
  u_eff = 0                           if |u| <= dz
  u_eff = sign(u) * (|u| - dz) / (1 - dz)    otherwise
  ```

- `PlantParams`: field names are the keys in the plant TOML. `k_fwd` is the forward top speed at full command, m/s.
- `ModelStructure` flags:

  | Flag | Meaning |
  | --- | --- |
  | `asymmetric_tau` | Accel and decel time constants differ |
  | `asymmetric_gain` | Forward and reverse max speeds differ |
  | `coupling` | Steer-brake and angular droop |
  | `drift_prop` | Yaw drift proportional to the linear command |
  | `drift_offset` | Yaw drift constant while moving |

  `pinned` lists parameters the joint fit may not move, held at the stage A value. `gradient_free_names` leaves out `delay_s` because profile search owns it.
- `integrate_step`: pose uses the velocity at the start of the substep, then the first-order velocity update follows. The C++ mirror keeps that order; the golden vectors in `tests/plant/` pin it.

  ```
  theta' = theta + w*dt
  x'     = x + (v/w) * (sin(theta') - sin(theta))
  y'     = y - (v/w) * (cos(theta') - cos(theta))
  ```

- `Plant`: steps the model with a zero-order-hold command buffer, as the transmitter does. The fit uses `simulate`, which shifts the command timeline instead. The two differ by less than one step's worth of command when the delay is not a whole number of steps.
- `delayed_commands`: output is zero before the delay has elapsed.
- `make_windows`: `valid` is the per-sample mask of usable ground truth (encoder slip excluded). A window is kept only if every sample it spans is valid. `horizons_s` reports the horizons actually evaluated (33 ms requested on a 5 ms grid is 35 ms).
- `concat_windows`: `origin_run_index` records which run each window came from. It is all or nothing: a partially attributed batch is rejected.
- `WindowErrors`: `along_m` and `cross_m` are meters in the body frame at the window start, `heading_rad` is wrapped to (-pi, pi], all shaped (windows, horizons). `speed_at_start` and `yaw_rate_at_start` are absolute truth values shaped (windows,). `u_lin0` and `u_ang0` are the signed commands at the window start.
- `WindowSet`: open-loop windows cut from one run. Each window reinitializes the model from ground truth at its start and runs open loop to each horizon, which is what the filter does between camera corrections.
- `toml_float`: every emitted value keeps a decimal point.

## MuJoCo distillation: `playground/calibration/distill_mujoco_plant.py`

Distills a fitted MuJoCo plant into the app's grey-box `[plant]` table for Mr Stabs Mk2. The app's EKF and `MotionProfileNavigation` both run `JigPlantModel` (first-order lag, deadzones, delay, steer-brake coupling, stick commands in). The MuJoCo fit (`fit_mujoco_plant.py`) has motor, contact, and inertia terms the app cannot run.

The script drives the fitted MuJoCo model through `ClosedLoopSim` with stick tapes shaped like the velocity jig protocol (deadzone staircases, steps with coasts, reversals, a steer/brake grid), fits the M4 grey-box model to the result the way `fit_jig_plant.py` fits jig data, and scores it on random stick tapes the fit never saw.

- `ClosedLoopSim` runs the firmware mixer and auto-steer, the drivetrain delay, and command shaping, matching what the app's plant is fed (`CommandFeedback`, the stick readback after slew limit and deadzone). The radio link delay is not in `ClosedLoopSim`. It comes from the sessions' clock alignment and is added to the fitted delay afterwards.
- Samples after the chassis pitches more than `--pitch-gate-deg` off its rest pitch are dropped for the rest of that tape. The report lists tapes that lost samples.
- Every restart within `--loss-ratio` of the best MuJoCo loss is distilled too. The spread across those tables shows which grey-box terms the MuJoCo data pins.
- `score_sessions` scores every table on held-out flat windows, in multiples of each session's noise floor.
- `HORIZONS` are the jig fit's: C1 at 100 ms, C2 and C3 at 400 ms. `CURRENT_TABLE` is the table the app runs today, scored alongside.

```
python playground/calibration/distill_mujoco_plant.py \
    playground/calibration/out/mujoco_sysid/fit_<ts> \
    --holdout playground/calibration/out/mujoco_sysid/s04
```

Writes `<fit_dir>/distilled/plant.toml` (the best run's `[plant]` table, radio delay included), `distill.json` (every run's table, sim and session scores, spread, masked tapes) and `plant_steps.png` (step-and-release responses and steady-state speed against stick, the distilled table against the one the profiles carry).

## `playground/calibration/fit_jig_plant.py` options

- `--fit-hz`: a 200 Hz grid resolves the delay to about 5 ms, a 500 Hz grid to about 1 ms, at 2.5x the fit time.
- `--smooth-ms`: a wider window is quieter against encoder count quantization but adds a few ms of apparent transport delay.
- `--pin`: hold a parameter at its stage A value where a dedicated excitation measures it directly.
- `--commands`: `measured` is the radio's own report of what it sent. `requested` is the fallback when a run's readback was mis-decoded.
- `--max-saturation`: 0 rejects any clipped IMU sample. Raise it to keep a run anyway.
- `--bootstrap`: resamples for the learning curve. 0 runs the leave-one-run-out jackknife only.

## `playground/calibration/fit_match_plant.py` options

- `--cmd-lead-ms`: the channel readback reaches the log tens of ms after the RF frame left the radio, so the effective delay on the logged timeline can be negative. A lead moves the delay profile minimum back inside the searchable range. Effective delay on the logged timeline is the fitted delay minus this lead.

## Other plant-fit scripts

- `velocity_jig_drive.py`: `--out` defaults to `<DEFAULT_OUT_ROOT>/<date>-<name>`, so repeated runs on the same day land in one session. The session is the unit leave-one-out validation holds out.
- `fit_plant_calib.py`: `load_session` needs a CSV from an `apriltag_track.py --drive` recording analyzed by `analyze_apriltag_mcap.py`.

### `fit_mrs_buff_mk3_interim_plant.py`

Fits Mrs Buff Mk3's MuJoCo plant to her jig-fitted grey-box plant (`playground/calibration/out/plant_stageA.toml`, measured on the floor with guard plates on), for use until a tagged session exists. The M4 model is driven with jig-shaped command tapes, its poses are treated as measurements, and the MuJoCo plant is fitted to them with the same staged CMA-ES and loss as the session fit. M4 is flat, so this pins drive and scrub, not tipping.

Commands are what the app sends: `DifferentialDriveProcessor` output in [-1, 1], mixed per motor as left = linear - angular, right = linear + angular (positive angular turns counterclockwise). Her ESCs run open loop, so the voltage-source actuator model is the whole command chain after the delay.

```
python playground/calibration/fit_mrs_buff_mk3_interim_plant.py \
    --out playground/calibration/out/mujoco_sysid/mk3_interim
```

Writes `fit.json` (`load_fit_params` reads it), `holdout.json` (per-horizon errors and response checks against M4) and `fit.log`.

### `make_plant_golden_fixture.py`

Generates the golden-vector fixture for the C++ `JigPlantModel` test. It steps the Python reference `Plant` (zero-order-hold buffer, 2 ms substeps) over a scenario and writes commands plus expected states as a C++ header. The C++ test replays the commands through `JigPlantModel::propagate` and matches to 1e-9.

The scenario covers a quiet start, a forward step, a reversal through the deadzone, a pure spin, a combined command that drives the steer-brake factor to its zero clamp (c_sb 2.70 crosses zero at |u_ang_eff| 0.37), sub-deadzone commands, and a final coast.

```
source scripts/activate_python.sh
python playground/calibration/make_plant_golden_fixture.py
```

### `sim_stop_test.py`

Closed-loop stop test of `MotionProfileNavigation` on Mr Stabs Mk2 against the MuJoCo plant, the last check before a distilled `[plant]` table goes on the robot. Each arm is one `[plant]` table, and every arm drives the same goals.

Each run is the real binary (`build/auto_battlebot`) against `kinematic_sim_server.py`, headless, one goal per run: a static "opponent" at the goal, attack mode with zero terminal speed. Scored from the sim's per-tick trace:

- `stop_along_m`: where the robot came to rest along the final approach, relative to the goal. Negative is short. Within `[-stop_distance, +stop_distance]` counts as on target.
- `peak_along_m`: furthest past the goal it got on the way.
- `arrive_s`: first time inside `stop_distance`. `settle_s`: from then until speed stays under 0.05 m/s.
- `reentries`: times it left the stop circle after arriving.
- `flipped`: pitch left its rest value by more than 60 deg. A flipping arm needs a lower `--max-linear-command`, not a better table.

```
python playground/calibration/sim_stop_test.py \
    --fit-file playground/calibration/out/mujoco_sysid/fit_<ts>/fit.json \
    --arm current=config/mr_stabs_mk2_aer_zed_box.toml \
    --arm distilled=playground/calibration/out/mujoco_sysid/fit_<ts>/distilled/plant.toml
```

Build first (`scripts/build.sh`). Runs are sequential because the C++ side has the sim port hard-coded. Writes `<out>/results.csv` and one trace and log per run; `--out` defaults next to the fit file.

### `write_plant_configs.py`

Pushes the fitted plant numbers from `plant_stageA.toml` into every consumer that hard-codes them. The numbers have three homes: the fit (`playground/calibration/out/plant_stageA.toml`, the only authored one), the kinematic sim (`simulation/sim_mrs_buff_mk3.toml`), and the C++ `[plant]` table (`config/robots/mrs_buff_mk3.toml`), which the our-robot EKF and `MotionProfileNavigation` both read.

- `SIM_FIELDS` takes every term. `delay_ms` is derived from `delay_s`: the sim takes milliseconds under `[latency]`, the controller seconds under `[our_robot]`.
- Fields are matched by name over the whole file, so each name must be unique within a config. `_rewrite` exits on a duplicate.
- It rewrites the numeric literal of each mapped field in place and touches nothing else. A new fit field needs an entry in the tables here.
- `PLANT_FIELDS` omits `angular_droop_coeff`: the plant droops (c_ad = 0.463) but compensating for it measured worse. On a 90-degree turning approach it took terminal error from 0.008 m to 0.067 m and time-to-goal from 1.10 s to 2.83 s. The controller default stays 0.0. `c_drift` and `c_drift_bias` are omitted because model M4 disables them.
- `--check` rewrites nothing and exits 1 if any consumer disagrees with the fit.

```
source scripts/activate_python.sh
python playground/calibration/write_plant_configs.py           # rewrite
python playground/calibration/write_plant_configs.py --check   # report drift, change nothing
```

Mr Stabs Mk2 has no jig fit. Its table comes from the MuJoCo fit through `distill_mujoco_plant.py` and goes into `config/robots/mr_stabs_mk2.toml` only (`MR_STABS_PLANT_CONFIG`); its sim config keeps the MuJoCo plant, so no sim fields are written. `config/robots/mr_stabs_mk2_geometry.toml` is not a fit and the script never writes it.

```
python playground/calibration/write_plant_configs.py --robot mr_stabs_mk2 \
    --fit playground/calibration/out/mujoco_sysid/fit_<ts>/distilled/plant.toml
```

### `simulation/plants/kinematic.py`

`KinematicPlant` is term for term the drivetrain model in `auto_battlebot/control/plant.py`: per-sign deadzone and gain, steer-brake and angular-droop coupling on the steady-state target, asymmetric first-order lag, exact arc integration on 2 ms substeps (`SUBSTEP_S`). The transport delay lives in the server's command ring buffer. `test_kinematic_plant_trajectory_matches_control_plant` fails if the two drift apart.

- The pose is the centre of the footprint disc (`footprint_center_offset`).
- Coupling multiplies the steady-state target, not the achieved speed.
- Accel or decel is chosen per channel by whether the target is further from zero than the current speed. Braking into a reversal uses the decel constant until the sign flips.
