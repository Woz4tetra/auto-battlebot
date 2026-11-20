# Plant fit modules

How the jig fit, the plant model, and the MuJoCo distillation produce the parameters `docs/reference/control/plant_backed_control.md` consumes.

## Fit stages and plant details

Notes on how the jig fit and the plant model produce the parameters used here.

### `calibration/jig/jig_fit.py`

Plant fitting for velocity jig sessions: loading, per-phase fits, joint fit, scoring.

Split out of `fit_jig_plant.py` so the report can call the fit machinery without importing a
command-line script. `fit_jig_plant.py` is the CLI over this module; `jig_report.py` and
`fit_process_noise.py` import from here.

The fit runs in two stages, because a cold-start joint fit over a dozen parameters will find
a local minimum and look convincing while doing it:

- **Stage A, per phase.** Each waveform is designed so one group of parameters dominates, so
  it fits in near-closed form: deadzone staircases, gains and rise constants from steps,
  decel from coast tails, coupling and straight-line drift from the grid, and transport delay
  pooled from an onset stack over every command edge. It also reports per-parameter spread,
  so a parameter that was not actually measured says so.

- **Stage B, joint by simulation error.** Starting from stage A, minimize multi-step
  open-loop prediction error over sliding windows at the horizons the filter coasts over.
  That optimizes exactly the quantity the Kalman filter depends on, which one-step-ahead
  fitting does not: any model missing a delay term can fake one-step accuracy.

Transport delay is not smooth in the objective, so it is profiled on a grid rather than
handed to the optimizer, which also produces a profile-likelihood curve.

Runs are routed to fits by waveform kind, channel and role rather than by a fixed list of
experiment ids, so a new excitation reaches the right fit without editing this file.

#### `fit_coupling`

Steer-brake and angular droop from the E13 grid.

    v = k_lin * u_lin_eff * (1 - c_sb * |u_ang_eff|)
    w = k_ang * u_ang_eff * (1 - c_ad * |u_lin_eff|)

Both are regressed through the origin on fractional loss. A coefficient whose spread covers
zero does not go in the model: three terms that are all real beat six where two are noise.

Angular droop and straight-line drift both make yaw depend on the linear command, so
a grid that only ever turns one way cannot tell them apart. They separate on turn
symmetry: for a cell at (u_lin, +u_ang) and its mirror at (u_lin, -u_ang), droop is a
fractional loss that flips with the turn while drift is additive and does not. The
half-difference isolates droop; the half-sum isolates drift.

#### `fit_drift`

Straight-line drift: the yaw a pure forward command produces on its own.

The guard plates drag asymmetrically on the floor, so the robot arcs when told to go
straight. That friction is in every match, so it belongs in the model rather than being
trimmed away and forgotten.

Two candidate mechanisms with different shapes, fitted together and reported separately:

    c_drift       a left/right gain mismatch, so yaw scales with the linear command
    c_drift_bias  an asymmetric drag torque, so yaw is constant while sliding

Both flip sign in reverse, which is why both directions are required: a forward-only fit
cannot tell an odd function from an even one, and these two terms differ by exactly that.

#### `fit_delay`

Transport delay from every clean command edge in the session, pooled.

Linear edges are timed against forward acceleration, angular edges against yaw acceleration.
The physical path is shared (Crossfire link, ESC, mechanical), which is what lets both stack
together. The peak of the averaged acceleration pulse sits at the transport delay: with a
pure delay in front of a first-order lag, dv/dt is zero until motion starts and jumps to its
maximum right at the delay.

This estimate is only a transport delay if the clocks were aligned. On an identity clock it
is `clock_offset + delay` and the report has to say so.

#### `joint_fit`

Minimize windowed open-loop error over the structure's free parameters.

Soft-L1 loss so a slipped wheel or a wall bump cannot steer the parameters. This is the only
thing standing between a bad window and the fit, so keep the capture-time verdict honest.
Delay is fixed here and profiled outside: the objective is not smooth in it at the
millisecond resolution that matters.

`bounds` overrides `PARAM_BOUNDS` per parameter. The match fit uses it to hold gains and
time constants inside the jig error bars widened 3x, so match data refines the jig answer
instead of wandering to a new one on weaker excitation.

#### `profile_delay`

Grid search the transport delay, refitting the continuous parameters at each point.

The profile curve is the deliverable, not just its minimum: a flat profile means the delay is
not identifiable from this excitation, and that belongs in the report rather than being
hidden behind a single number.

Resolution is bounded by the analysis grid, not by `delay_grid`. On synthetic data with a
known 45 ms delay, a 200 Hz grid puts the minimum at 50 ms and a 500 Hz grid puts it at
44 ms, so quote the delay no finer than half a grid step.

### `calibration/jig/velocity_jig.py`

#### `ClockFit`

Jig microseconds to host seconds, as offset plus linear skew.

Cross-correlating command edges against measured acceleration only ever recovers
`clock_offset + transport_delay`. Measuring the offset separately is what makes the fitted
transport delay a real number instead of a number plus however far the clocks are apart.

### `calibration/match/match_windows.py`

#### `ReplayTrack`

`svo_index` is strictly increasing, with gaps where frames dropped (the constructor raises otherwise). `stamp_ns` is the original-clock image stamp.

#### `load_replay_track`

Ticks are associated to `/camera/frame_meta` by log order. Within one tick the publisher writes frame_meta first, then markers and diagnostics, and the next frame_meta is a frame interval away. The association is verified through the stamp rebase: marker header stamp minus the frame's `image_stamp_ns` must be one constant across the run (the replay clock offset). A drifting offset (std above 1 ms) means the association slipped a tick, and the loader raises.

#### `CommandLog`

`auto_switch_latched` is a bool, forward-filled: the auto switch is a latched physical state, not an event.

#### `load_commands`

Decodes the `opentx_transmitter` channel readback into body commands. `lin = (ch0 - ch1) / 2048` and `ang = -(ch0 + ch1) / 2048`: the tank inverse mix with channel B inverted and angular negated, exactly `drive_protocol.MixConfig.from_channels` at readback scale 1024. Entries are logged on change, and consumers zero-order hold between entries.

#### `MatchRun`

`role` is a `RunRole` (`fit`, `holdout`, `validation`, `video`). `t` is seconds from the grid start, uniform. `v_body_mps` is the signed body forward speed finite-differenced from pose, and `yaw_rate_radps` the yaw rate.

#### `central_difference_velocity`

Body speed and yaw rate by central difference over live neighbors. Edges of a live stretch fall back to one-sided differences and isolated live samples get NaN. The third return is the mask of samples with both neighbors live.

### `control/plant.py`

#### `effective_command`

Deadzone removal and rescale, per sign.

    u_eff = 0                                  if |u| <= dz
    u_eff = sign(u) * (|u| - dz) / (1 - dz)    otherwise

The rescale keeps full command mapping to full effect, so the gain stays the max speed rather
than becoming max speed divided by (1 - dz).

#### `toml_float`

`0` and `5` are integers in TOML, and a reader that asks for a double gets a type error on a
parameter that happened to land on a round number. Every emitted value keeps a decimal point.

#### `PlantParams`

`k_fwd` is the forward top speed at full command, m/s. The field names are the keys in the plant TOML, so the unit stays in this doc instead of the name.

#### `ModelStructure`

| Flag | Meaning |
| --- | --- |
| `asymmetric_tau` | Accel and decel time constants differ |
| `asymmetric_gain` | Forward and reverse max speeds differ |
| `coupling` | Steer-brake and angular droop |
| `drift_prop` | Yaw drift proportional to the linear command |
| `drift_offset` | Yaw drift constant while moving |

`pinned` lists parameters the joint fit may not move, held at whatever stage A measured. A
parameter with its own dedicated excitation and a tight spread is evidence. Leaving it free lets
an objective that cannot see it directly trade it away against something it can.

`gradient_free_names` leaves out `delay_s` even when the structure uses delay, because profile
search owns it.

#### `integrate_step`

Pose uses the velocity at the start of the substep, then the first-order velocity update
follows. The C++ mirror must keep that order; the golden vectors in `tests/plant/` pin it. At a
2 ms substep, using the end-of-substep velocity instead shifts the pose by dt times the
acceleration, under a millimeter at any speed this drivetrain reaches.

    theta' = theta + w*dt
    x'     = x + (v/w) * (sin(theta') - sin(theta))
    y'     = y - (v/w) * (cos(theta') - cos(theta))

#### `Plant`

The fit uses `simulate`, which shifts the command timeline instead of buffering it. `Plant`
exists so the sim and any Python-side runtime experiment step the same model the filter will,
and so a golden-vector test can compare both against the C++ implementation.

One deliberate difference from `simulate`: the buffer holds each command until the next one
arrives, while `delayed_commands` interpolates between command samples. The two agree exactly
when the delay is a whole number of steps and differ by less than one step's worth of command
otherwise. Runtime is the zero-order-hold case, because that is what the transmitter does.

#### `delayed_commands`

Before the delay has elapsed there was no command, so the output is zero rather than the first
command held early.

#### `make_windows`

`valid` is the per-sample mask of usable ground truth (encoder slip excluded, for instance). A
window is kept only if every sample it spans is valid, so one slipped wheel cannot steer the fit
through a window that mostly looks fine.

`horizons_s` reports the horizons actually evaluated. A requested 33 ms on a 5 ms grid is 35
ms, and a table that says 33 while measuring 35 mislabels its own axis.

#### `concat_windows`

`origin_run_index` says which run each window came from, as an index into whatever run list the
caller built the set from. Without it, concatenating runs makes every per-run question
unanswerable: which waveform has the worst residuals, which single run a parameter rests on. The
caller assigns the numbering and `concat_windows` preserves it. It is all or nothing: a
partially attributed batch would silently mis-assign windows to the wrong run, which is worse
than having no attribution at all.

#### `WindowErrors`

`along_m` and `cross_m` are meters in the body frame at the window start, `heading_rad` is
radians wrapped to (-pi, pi], all shaped (windows, horizons). `speed_at_start` and
`yaw_rate_at_start` are absolute truth values shaped (windows,), used as conditioning variables.
`u_lin0` and `u_ang0` are the signed commands at the window start, kept separately because the
drift diagnostic needs the sign: what separates drift from angular droop is whether the effect
flips with the turn direction.

#### `predict_windows`

`simulate` records no initial-state column, so step h lands at trajectory index h - 1.

#### `WindowSet`

Open-loop windows cut out of one run.

Each window reinitializes the model from ground truth at its start and runs open loop to each
horizon. That is exactly what the filter does between camera corrections, so it is what the
fit optimizes and what the noise model characterizes.

## Plant fit and distillation details

### `playground/calibration/distill_mujoco_plant.py`

`distill_mujoco_plant.py`:

Distill a fitted MuJoCo plant into the app's grey-box [plant] table for Mr Stabs Mk2.

The app's EKF and MotionProfileNavigation both run JigPlantModel: first-order lag, deadzones, delay, and steer-brake coupling, with stick commands in. The MuJoCo fit (fit_mujoco_plant.py) has motor, contact, and inertia terms the app cannot run. This drives the fitted MuJoCo model through ClosedLoopSim with stick tapes shaped like the velocity jig protocol (deadzone staircases, steps with coasts, reversals, a steer/brake grid), fits the M4 grey-box model to the result the way fit_jig_plant.py fits jig data, and scores it on random stick tapes the fit never saw.

Stick tapes go into ClosedLoopSim, which runs the firmware mixer and auto-steer, the drivetrain delay, and command shaping. That matches what the app's plant is fed: CommandFeedback is the transmitter's stick readback, after its slew limit and deadzone. The radio link delay is not in ClosedLoopSim, so it comes from the sessions' clock alignment and is added to the fitted delay afterwards. A pure shift adds exactly.

Samples after the chassis pitches more than --pitch-gate-deg off its rest pitch are dropped, and stay dropped for the rest of that tape. The grey-box model is flat and cannot represent a wheelie or a flip, so the table only covers the command range the robot stays down in. The report lists which tapes lost samples.

Every restart within --loss-ratio of the best MuJoCo loss is distilled too. The spread across those tables is the parameter spread the plan hands to the learned sim, and shows which grey-box terms the MuJoCo data pins.

```
    python playground/calibration/distill_mujoco_plant.py \
        playground/calibration/out/mujoco_sysid/fit_<ts> \
        --holdout playground/calibration/out/mujoco_sysid/s04
```

Writes <fit_dir>/distilled/plant.toml (the best run's [plant] table, radio delay included), distill.json (every run's table, sim and session scores, spread, masked tapes) and plant_steps.png (step-and-release responses and steady-state speed against stick, the distilled table against the one the profiles carry now).

- `CURRENT_TABLE`: The table the app runs today, scored alongside so the report shows what the swap buys.
- `HORIZONS`: The jig fit's horizons: C1 is stated at 100 ms, C2 and C3 at 400 ms.
- `BOUNDS`: Mrs Buff's jig fit landed c_sb at 2.70, past PARAM_BOUNDS' 1.5, so the bound is widened rather than left to decide the answer.
- `simulate_tape`: Latched: once the robot has pitched up it lands somewhere the flat model never predicted.

`score_sessions`:

Every table on the held-out flat windows, through the same per-motor tape and inverse mixer the direct grey-box fit uses. Errors in multiples of each session's noise floor.

- `main`: A stick move cannot reach the ESP32 before it is sent: a negative lag is a clock fit or stamping error (2026-10-15 measured -9.5 ms), and added to the drivetrain delay it would write a negative delay_s.
- `main`: The app's delay runs from stick readback to motion: radio link plus drivetrain.

### `playground/calibration/fit_jig_plant.py`

- `main`: The plan's objective horizons, plus 400 ms because that is where acceptance criteria C2 and C3 are stated and a table should measure the horizon it reports.

### `playground/calibration/fit_mrs_buff_mk3_interim_plant.py`

`fit_mrs_buff_mk3_interim_plant.py`:

Fit Mrs Buff Mk3's MuJoCo plant to her jig-fitted grey-box plant, until a tagged session exists.

Mr Stabs Mk2's MuJoCo plant was fit to hand-driven sessions with AprilTags (fit_mujoco_plant.py). Mrs Buff Mk3 has no such session, only the velocity jig's M4 grey-box fit (playground/calibration/out/plant_stageA.toml: speeds, lags, deadzones, steer-brake and angular droop, measured on the floor with the guard plates on). This drives the M4 model with jig-shaped command tapes, treats its poses as measurements, and fits the MuJoCo plant to them with the same staged CMA-ES and loss the session fit uses. M4 is flat, so this pins drive and scrub, not tipping; with the weapon off she does not tip.

```
Commands are what the app sends: `DifferentialDriveProcessor` output (angular priority, the zero
deadzone), linear and angular in [-1, 1]. The radio mixes them per motor as left = linear -
angular, right = linear + angular (positive angular turns counterclockwise, the inverse
`greybox.predict` uses). Her ESCs run open loop: there is no eRPM feedback and no onboard yaw
loop, so the voltage-source actuator model is the whole command chain after the delay.
```

```
    python playground/calibration/fit_mrs_buff_mk3_interim_plant.py \
        --out playground/calibration/out/mujoco_sysid/mk3_interim
```

Writes fit.json (load_fit_params reads it), holdout.json (per-horizon errors and response checks against M4) and fit.log.

- `fit_mrs_buff_mk3_interim_plant.py`: Capped at 0.02 s: the same contact softness applies to walls and opponents, and a softer contact lets the hull sink into them in a collision. M4 is flat, so the floor alone would happily trade stiffness away.
- `fit_mrs_buff_mk3_interim_plant.py`: The ESC's current limit caps acceleration. The datasheet gives 10.4 A stall at 12 V; without this term the fit pinned the rotor inertia at 3x to slow the launch.

### `playground/calibration/make_plant_golden_fixture.py`

`make_plant_golden_fixture.py`:

Generate the golden-vector fixture for the C++ JigPlantModel test.

Steps the Python reference plant (auto_battlebot/control/plant.py, `Plant` class: zero-order-hold command buffer, 2 ms substeps) over a scenario that exercises every term, and writes the commands plus expected states as a C++ header. The C++ test replays the same commands through JigPlantModel::propagate and must match: both sides are double precision with the same substep, so the tolerance can sit at 1e-9.

The scenario covers: quiet start (zero command before the delay elapses), a forward step, a reversal through the deadzone (decel then accel tau), a pure spin, a combined command that drives the steer-brake factor into its zero clamp (c_sb 2.70 crosses zero at |u_ang_eff| 0.37), sub-deadzone commands, and a final coast.

```
Usage:
    source scripts/activate_python.sh
    python playground/calibration/make_plant_golden_fixture.py
```

Paths are resolved from the repo root, so the working directory does not matter.

### `playground/calibration/sim_stop_test.py`

`sim_stop_test.py`:

Closed-loop stop test: MotionProfileNavigation on Mr Stabs Mk2 against the MuJoCo plant.

The last check before a distilled [plant] table goes on the robot. The app plans its brake schedule and feedforward from the [plant] table while the sim moves the robot with the fitted MuJoCo model, so every error in the table shows up as a stop that lands short, overshoots, or oscillates. Each arm is one [plant] table, and every arm drives the same goals.

Each run is the real binary (build/auto_battlebot) against kinematic_sim_server.py, headless, one goal per run: a static "opponent" at the goal, attack mode with a zero terminal speed, so the controller drives there and stops within stop_distance. Scored from the sim's own per-tick trace, which is ground truth:

- stop_along_m: where the robot came to rest, along the final approach, relative to the goal.
  Negative is short. Anything in [-stop_distance, +stop_distance] counts as a stop on target.
- peak_along_m: furthest past the goal it got on the way. Overshoot that it later backed out of
  shows here and not in stop_along_m.
- arrive_s: first time inside stop_distance. settle_s: from then until speed stays under 0.05 m/s.
- reentries: how many times it left the stop circle again after arriving (oscillation).
- flipped: pitch left its rest value by more than 60 deg. Nothing else in the row means anything
  then. The flat [plant] table cannot predict a backflip, so a flipping arm needs a lower
  --max-linear-command, not a better table.

```
    python playground/calibration/sim_stop_test.py \
        --fit-file playground/calibration/out/mujoco_sysid/fit_<ts>/fit.json \
        --arm current=config/mr_stabs_mk2_aer_zed_box.toml \
        --arm distilled=playground/calibration/out/mujoco_sysid/fit_<ts>/distilled/plant.toml
```

Build first (scripts/build.sh). Runs are sequential: the C++ side has the sim port hard-coded. Writes <out>/results.csv and one trace and log per run; --out defaults next to the fit file.

- `write_sim_config`: The MuJoCo plant holds the drivetrain delay itself; the sim's own buffer is the radio part.
- `write_sim_config`: The base config runs the RL environment's 2 ms step; the fit and its checks run at 1 ms.
- `write_cpp_overlay`: Same type as the base, so these merge key by key: arrive stopped at the goal.

### `playground/calibration/write_plant_configs.py`

`write_plant_configs.py`:

Push the fitted plant numbers from plant_stageA.toml into every consumer that hard-codes them.

The plant numbers have three homes: the fit (`playground/calibration/out/plant_stageA.toml`), the kinematic sim (`simulation/sim_mrs_buff_mk3.toml`), and the C++ config chain's `[plant]` table (`config/robots/mrs_buff_mk3.toml`). Only the first is authored. They drifted apart once already, which is how MotionProfileNavigation ended up running Mr Stabs Mk2's time constants on Mrs Buff Mk3, so this script makes the other two derived.

The sim config models the plant, so `SIM_FIELDS` takes every term, including the ones the controller deliberately ignores. `delay_ms` is derived from `delay_s`: the sim takes the transport delay in milliseconds under `[latency]`, where the controller config takes seconds under `[our_robot]`. Fields are matched by name over the whole file, not per section, so each name must stay unique within a config. `_rewrite` exits if a name appears twice.

The `[plant]` table is read by both the our-robot EKF and MotionProfileNavigation, so there is one C++-side target rather than one per consumer.

It rewrites the numeric literal of each mapped field in place and touches nothing else. The hand-written doc comments around those fields explain what each number does and why, which a block regenerator would flatten, so they are left alone. That also means adding a field to the fit does not automatically give it a home here: add it to the tables below and write the comment by hand.

`--check` rewrites nothing and exits 1 if any consumer disagrees with the fit. Run it after a refit, and in CI if the fit ever starts changing on its own.

```
Usage:
    source scripts/activate_python.sh
    python playground/calibration/write_plant_configs.py           # rewrite
    python playground/calibration/write_plant_configs.py --check   # report drift, change nothing
```

Mr Stabs Mk2 has no jig fit. Its table comes from the MuJoCo fit through distill_mujoco_plant.py and goes into `config/robots/mr_stabs_mk2.toml` only; its sim config keeps the MuJoCo plant, so no sim fields are written:

```
    python playground/calibration/write_plant_configs.py --robot mr_stabs_mk2         --fit playground/calibration/out/mujoco_sysid/fit_<ts>/distilled/plant.toml
```

- `MR_STABS_PLANT_CONFIG`: The one Mr Stabs Mk2 plant file. `config/robots/mr_stabs_mk2_geometry.toml` (the ZED Box profiles) is not a fit, so the script never writes it.
- `PLANT_FIELDS`: The controller inverts the plant, so it takes the same terms under its own names. `angular_droop_coeff` is absent on purpose. The plant does droop (c_ad = 0.463) but compensating for it measured worse: on a 90-degree turning approach it took terminal error from 0.008 m to 0.067 m and time-to-goal from 1.10 s to 2.83 s, because the harder turn it commands is exactly what the steer-brake term then charges forward speed for. The controller default stays 0.0 and is not derived from the fit. `c_drift`/`c_drift_bias` are absent because model M4 disables them. The [plant] table uses the fit's own names, so this mapping is the identity. Listed explicitly so a field added to the fit does not silently skip the table.

### `simulation/plants/kinematic.py`

- `SUBSTEP_S`: Internal integration substep. Mirrors auto_battlebot/control/plant.py. At the calibrated 31.7 rad/s top yaw rate this is 0.06 rad of rotation per substep; a whole 33 ms tick is 1.05 rad, where straight-line integration of an arc is wrong by tens of degrees of heading.
- `KinematicPlant`: Term for term the drivetrain model in `auto_battlebot/control/plant.py`, which the velocity jig fits: per-sign deadzone and gain, steer-brake and angular-droop coupling on the steady-state target, asymmetric first-order lag, exact arc integration on 2 ms substeps. The transport delay lives outside the class, in the server's command ring buffer. `test_kinematic_plant_trajectory_matches_control_plant` fails if the two drift apart.
- `KinematicPlant.footprint_center_offset`: The pose is the centre of the disc. The fits behind this plant tracked the keypoint midpoint, which sits within 2 mm of the footprint centre.
- `KinematicPlant.step`: Coupling multiplies the steady-state target, not the achieved speed: turning costs a fraction of the forward command's authority and driving costs a fraction of the turn's, but neither brakes a robot that is already coasting.
- `KinematicPlant._substep`: Pose uses the velocity at the start of the substep, so the arc is exact for the v and w actually held over it.
- `KinematicPlant._substep`: Accel or decel is decided per channel by whether the target is further from zero than the current speed. Braking into a reversal uses the decel constant until the sign flips, as the drivetrain does: friction first, then torque.

### `playground/calibration/fit_jig_plant.py` options

- `--fit-hz`: the pose integration lands the response within half a grid step, so a 200 Hz grid resolves the delay to about 5 ms and a 500 Hz grid to about 1 ms, at 2.5x the fit time.
- `--smooth-ms`: a wider window is quieter against encoder count quantization but rounds the corner of a step, which the fit pays back as a few ms of extra transport delay. Sweep it once on real data.
- `--pin`: use it where a dedicated excitation measures a parameter directly. The window objective only sees that parameter through its effect on everything else, which is where the two disagree worst.
- `--commands`: `measured` is the radio's own report of what it sent, which is what the robot received. Use `requested` when a run's readback was mis-decoded, since a wrong channel sign corrupts the measured stream and leaves the requested one exact.
- `--max-saturation`: 0 rejects any clipped IMU sample, because a clipped sample means an impact and its true value is unknown. Raise it to keep a run anyway.
- `--bootstrap`: resamples for the learning curve, which answers how much more data is needed. 0 runs the leave-one-run-out jackknife only. 8 is a sensible start.

### `playground/calibration/fit_match_plant.py` options

- `--cmd-lead-ms`: the channel readback reaches the log tens of ms after the RF frame carrying the same values left the radio, so the effective transport delay on the logged timeline can be negative. A lead moves the delay profile minimum back inside the searchable range. Effective delay on the logged timeline is the fitted delay minus this lead.

### `playground/calibration/velocity_jig_drive.py` session directory

- `--out` defaults to `<DEFAULT_OUT_ROOT>/<date>-<name>`, so repeated runs on the same day land in one session. The session is the unit leave-one-out validation holds out.

### `playground/calibration/fit_plant_calib.py` input

- `load_session` needs a CSV from an `apriltag_track.py --drive` recording analyzed by `analyze_apriltag_mcap.py`. Read-only stick captures have no command columns.

## Workarounds

- **`distill_mujoco_plant.py` BLAS threads.** `pin_one_blas_thread_per_worker` sets `OMP_NUM_THREADS`, `OPENBLAS_NUM_THREADS` and `MKL_NUM_THREADS` to 1. Workers are spawned rather than forked, so they load BLAS after the variables are set. Without it, `least_squares` runs every worker's SVD on all cores at once.
- **`fit_plant_calib.py` tag polarity.** `_tag_frame_polarity` recovers the sign of each channel from the data. The AprilTag pose frame can be inverted relative to the command frame (a mirrored tag or a 180-degree mount), so a positive command reads as negative measured velocity. Fixing the sign lets both channels stack in the same direction instead of cancelling.
- **`sim_stop_test.py` port.** `CPP_SIM_CONNECTION_PORT` (14882) duplicates the port hard-coded in `src/simulation/sim_connection.cpp`. Changing the C++ port breaks the test until this value follows.
- **`build_mrs_buff_mk3_model.py` CAD floor trim.** `trim_to_cad_floor` cuts every piece off at the structure's lowest CAD point. CoACD hulls run up to a millimetre outside the mesh, and the fork tips sit 0.35 mm above the floor in CAD, so an untrimmed hull reaches under the floor and the robot rests on its forks with the front wheels in the air. Idempotent.
- **`velocity_jig_drive.py` stdin drain.** `drain_stdin` discards keystrokes for `SETTLE_S` (0.35 s) before a prompt accepts one. A double-tapped Enter lands 50-200 ms after the first, so a single flush at prompt time is too early to catch it: the stray press arrives afterwards and satisfies the prompt that just appeared. Without the drain it silently answers the next prompt, so the robot arms and drives while someone still has a hand on it. It does nothing on a piped or redirected stdin (no driver queue, and `termios` would raise), so `--no-prompt` and scripted runs are unaffected. Unverified: no run reproduced the double tap.
- **`velocity_jig_drive.py` `confirm_stopped`.** Recording stops on the operator's Enter, not the jig's `stopped` line. Waiting on the line alone hangs the run when the press is mistimed: a press that lands before the cable is back in never puts a summary on the wire, and the step sits until the 300 s timeout. Gating on Enter costs only the counts, a reporting nicety: `gates()` reads `samples=0` as unknown, and the log on the card is complete either way.
- **`velocity_jig_drive.py` `reopen_after_replug`.** The jig is matched by USB serial number, not device path: `/dev/ttyACM0` can come back as `ttyACM1` if anything else enumerated while the cable was out. `CDC_ENDPOINT_SETTLE_S` (0.5 s) lets the CDC endpoint settle before the port opens.
- **`velocity_jig_drive.py` coast after continuous programs.** `ends_at_rest` gates a coast step (`SETTLE_COAST_S`). A piecewise program ends on a rest cell, so the robot is stopped when the excitation returns. A continuous one (PRBS, chirp, sine) ends on whatever value its last tick lands on, and `play` sends a single zero and returns with the robot at speed. The post-hold countdown then measures a coasting robot, the stillness test rejects it, and the run is thrown out for having no still hold. LOG-134, LOG-135, LOG-137 and LOG-145 were lost this way, each with 0.02 s between the last live command and the start of the hold. The piecewise runs beside them had 3 to 4 s of trailing coast and passed.
- **`velocity_jig_drive.py` press B before unplugging.** The session presses B on the jig, then unplugs. Plugging in resets the board, and a reset lands wherever the firmware happens to be: on LOG-119 it landed mid-write and left a truncated final row and a post-probe on a fresh boot epoch. Pressing B while unplugged closes the file on the card before USB can do anything, so the reset has nothing left to interrupt. The cost is the stop summary, which goes to the wire unbuffered and is lost with the cable out. That loses `n` and `dropped`, both reporting niceties: `gates()` reads `samples=0` as unknown, and the log's own timestamps show the drops. A truncated recording is not recoverable, and the lost counter is.
- **`velocity_jig_drive.py` sidecar log name.** The run TOML's `log_file` is `dest.name`, not the jig's name. A collision after a card wipe renames the file on disk, and a sidecar pointing at the original name would load the wrong log.
- **`velocity_jig_drive.py` channel B read inverted.** The radio has servo-reverse set on drive channel B and reports the channel negated while the ESC on that side compensates. Reading it straight un-mixes every straight-line command into a pure turn, which discarded LOG-57 and LOG-62 on 2026-08-20. The default reads B negated; `--no-read-invert-b` disables it.
