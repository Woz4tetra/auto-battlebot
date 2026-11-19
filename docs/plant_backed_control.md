# Plant-backed control loop

`KalmanMotionEstimator` and `MotionProfileNavigation` are two halves of one loop. The
estimator runs the fitted drivetrain model forward to answer "where is our robot now";
the navigation runs the same model backward to answer "what command produces the speed I
want". Both read their numbers from the same jig fit, so a refit moves the estimator and
the controller together.

![Plant-backed control loop](diagrams/plant_backed_control.svg)

Regenerate the SVG after editing `diagrams/plant_backed_control.dot`:

```bash
./docs/generate_diagrams.sh   # needs graphviz (dot)
```

## Call pattern

`ControlLoop::run_cycle` runs at control rate, driven by `ThreadedControlLoop` (or
`SteppedControlLoop` in sim); perception arrives at about 30 Hz. The two are not in
lockstep, and the split matters:

1. `pump_input()` calls `transmitter_->update()`, which reads the RC channels back as
   `CommandFeedback`. The feedback is normalized [-1, 1] stick, the same space the command
   was sent in. Physical units never cross this boundary: whoever needs m/s and rad/s
   applies the `[plant]` gains themselves, so the drivetrain scaling is defined once. The
   EKF hands the stick command to `JigPlantModel`; the dead-reckoning arm calls
   `plant_stick_to_body_velocity()`, the same gains without the deadzones, coupling, and
   lag a single-frame hold cannot represent.
2. `robot_filter_->predict(now, command_feedback)`, every tick. Inside the estimator this
   pushes a `TimedCommand` into `CommandRingBuffer` and then calls `coast(now)`.
3. `robot_filter_->correct(...)`, only when a new perception frame landed. This runs the
   association front-end and then `MotionEstimatorInterface::update`.
4. `target_selector_->get_target(...)` picks the goal pose.
5. `navigation_->update(...)` returns a `VelocityCommand`.
6. `transmitter_->send(command)`, which is what step 1 reads back on the next tick.

## Estimator: the plant run forward

`KalmanMotionEstimator` (`src/robot_filter/kalman_motion_estimator.cpp`) sits behind the
`MotionEstimatorInterface` seam, so it swaps the state estimation strategy without
touching association.

### Opponents

Four-state linear Kalman filter, `[px, py, vx, vy]` in the field frame, constant-velocity
model with continuous white-noise acceleration. Measurements are position-only and pass a
chi-square innovation gate; `reinit_after_rejects` consecutive rejects reinitialize the
track, on the theory that a consistent reject means the world moved, not the measurement.

Perception latency puts a measurement stamp behind the control clock. Rather than folding
it in at the current state (which biases against the direction of motion), the track
rewinds to the newest of 64 snapshots at or before the measurement stamp, corrects there,
and lets the next `coast()` re-propagate.

### Our robot

Two modes, selected by `our_robot_mode`:

- `DEAD_RECKONING` integrates the last commanded velocity directly, matching
  `RobotTemporalMotionFilter`.
- `EKF` runs a five-state EKF `[x, y, theta, v, w]` propagated by `JigPlantModel`, which
  re-reads the command history at `t - delay_s` and integrates exact arcs in 2 ms
  substeps through per-sign deadzones, asymmetric first-order lag, and the steer-brake and
  angular-droop coupling terms.

Keypoint measurements correct position and heading; blob centroids correct position only.
A heading innovation past pi/2 also falls back to the position rows, because the
front/back keypoint converter can flip a robot and folding a flipped heading in would spin
the whole state.

### coast()

`coast(now)` renders every track forward to the control clock on a copy, so the emitted
state moves between perception frames instead of freezing at the last `correct()`. Our
EKF coasts up to `max_coast_s` of unmeasured age and then holds position while covariance
keeps growing. A track reads `is_stale` once it has gone 0.1 s unmeasured, which is three
frame periods at 30 Hz: normal between-frame coasting stays fresh, a real dropout does not.

## Navigation: the same plant inverted

`MotionProfileNavigation` (`src/navigation/motion_profile_navigation.cpp`) drives to the
goal and arrives at a commanded terminal velocity without overrunning. The terminal
velocity is per behavior mode, because the mode set on the dashboard decides what the mission is:
`attack_terminal_speed_fraction` ships at `1.0` so the robot drives through the opponent at full
speed, and `run_away_terminal_speed_fraction` ships at `0.0` so it arrives stopped at the safe
point. Both are a fraction of the plant's `k_fwd`, clamped to `[0, 1]`, so a refit
rescales them. A mode flip resets the trajectory
state, since the goal and the terminal speed both change at once and carrying the old
reference across would feed the feedforward a `dv/dt` step the plant never asked for.

1. **Distance-to-go brake schedule.** On a first-order plant the residual travel after
   commanding `v_term` is about `v*(tau_decel + latency)`, so the reference speed is
   capped at `v_term + max(0, d - |v|*latency) / tau_decel`. The brake point comes out of
   the measured plant instead of a hand-tuned brake distance. Ramp-up is rate-limited by
   `accel_limit`.
   The distance needs no latency lead of its own: the motion estimator renders every track at
   `now + plant.delay_s`, so the pose it measures from is already where the robot will be when the
   command reaches the wheels. Subtracting `v*latency` as well double-counted the travel and
   braked 10.4 cm early at 2 m/s, against the 0.15 m `stop_distance`
   (`BrakeScheduleDoesNotSubtractALatencyLead` guards it). The horizon still adds `latency_` on
   top of `tau_decel_`, which is the same argument one delay too many, since decay starts from
   the state's own time and `tau_decel_` alone is the derived value. It stays as margin.
   Dropping it takes the horizon from 0.252 s to 0.200 s and raises the reference speed 26% at
   every distance. Nothing has measured that against a stop, so change it with a field or
   replay check, not on the algebra.
2. **Inverse-plant feedforward.** `u_ff = (v_ref + tau*dv/dt) / k_fwd`, with `tau`
   selected by the sign of `dv/dt` so a falling reference actually commands the brake.
   A negative command is rescaled by `k_fwd / k_rev` because reverse is the weaker
   direction. Then the steer-brake loss is divided out and the deadzone inverted exactly.
3. **Speed PI feedback** on `v_ref - v_actual`, with a clamped integrator. This is dropped
   while our track is `is_stale`: a dead-reckoned speed is a function of this controller's
   own command, so closing a loop on it is not feedback. The last measured speed is held
   through the stale stretch so the latency lead keeps working.
4. **Angular channel, same shape.** Turn-lock hysteresis (commit at 135 degrees, release
   at 90) stops the shortest-turn sign from dithering near 180. A PD on heading produces a
   yaw-rate reference, which goes through the same inverse-plant feedforward with the
   regime-appropriate yaw time constant.

Linear command only opens up once the heading error is inside `angle_threshold`;
otherwise the robot turns in place and the linear state is bled so it does not lunge when
the gate opens.

### The two coupling compensations are mutually recursive

The steer-brake term reads the turn command and the droop term reads the linear command.
Each is driven by the previous tick's **uncompensated** counterpart, which breaks the
loop. Feeding the compensated values back instead has a loop gain above 1 and saturates
both channels within a few ticks.

## One fit, two consumers

The fitted numbers live in `playground/calibration/out/plant_stageA.toml` (sessions
2026-08-19 through 2026-08-23, model M4). One C++-side copy carries them: the top-level
`[plant]` table in `config/robots/mrs_buff_mk3.toml`, which `config/_common.toml` loads with
`from = "robots/mrs_buff_mk3"` (`docs/config_extends.md`). It is parsed into `PlantConfiguration`
(`include/plant/config.hpp`) and handed to both consumers after their own sections parse.
The estimator builds a `JigPlantModel` from it; the controller inverts the same fields for
its brake schedule and feedforward. Neither has C++ defaults, so a config with no `[plant]`
is a load error rather than a subsystem running on zeros.

`playground/calibration/write_plant_configs.py` rewrites the numeric literals in
`simulation/sim_mrs_buff_mk3.toml` and `config/robots/mrs_buff_mk3.toml` in place, so the hand-written
notes next to each number survive. `--check` reports drift without changing anything.

Which term does what on each side:

| Fit parameter | Estimator | Navigation |
| --- | --- | --- |
| `k_fwd` 4.88002 | forward gain | divisor mapping reference speed to command |
| `k_rev` 4.3546 | reverse gain | reverse-gain rescale |
| `k_ang` 31.7062 | yaw gain | divisor mapping yaw-rate reference to command |
| `tau_lin_a` 0.14921 | spin-up lag | feedforward of `dv/dt` |
| `tau_lin_d` 0.123461 | coast lag | brake horizon, with `delay_s` |
| `tau_ang_a` 0.173867 | yaw spin-up lag | yaw feedforward, rising |
| `tau_ang_d` 0.0878998 | yaw coast lag | yaw feedforward, falling |
| `delay_s` 0.0522094 | transport delay | latency lead in the brake schedule |
| `c_sb` 2.70197 | steer-brake loss | divided back out of the forward command |
| `c_ad` 0.463423 | angular droop | only when `compensate_angular_droop` is true |

The one deliberate asymmetry is `c_ad`. The estimator models the droop because the plant
has it. The controller reads the same coefficient but ships with
`navigation.compensate_angular_droop = false`, because compensating made the controller
worse: on the 90-degree turning approach, enabling it moved terminal error from 0.008 m to
0.067 m and time-to-goal from 1.10 s to 2.83 s. Compensating buys faster heading
convergence by commanding a harder turn, and the steer-brake term charges forward speed
for exactly that. The heading loop is closed-loop already and gets there without the help.
The coefficient is measured; the compensation is a choice, which is why the switch is a
bool rather than a second copy of the number.

## Where each piece is selected

- `[plant]` in `config/robots/<robot>.toml`, the shared fit both subsystems read. Profiles load
  it with `[plant] from = "robots/<robot>"`; `docs/robot_plant_files.md` lists which profile
  loads which file.
- `[robot_filter.motion_estimator] type = "KalmanMotionEstimator"`, `our_robot_mode = "EKF"`
  in `config/_common.toml`.
- `[navigation] type = "MotionProfileNavigation"` in `config/_common.toml`, so every
  profile gets it by default. Mr Stabs Mk2 has no jig fit and pins `PursuitNavigation` in
  its own profiles; setting `type` there makes the merge replace the whole section, so
  those profiles spell out every pursuit field rather than inheriting any.

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

### `control/plant.py`

#### `effective_command`

Deadzone removal and rescale, per sign.

    u_eff = 0                                  if |u| <= dz
    u_eff = sign(u) * (|u| - dz) / (1 - dz)    otherwise

The rescale keeps full command mapping to full effect, so the gain stays the max speed rather
than becoming max speed divided by (1 - dz).

#### `WindowSet`

Open-loop windows cut out of one run.

Each window reinitializes the model from ground truth at its start and runs open loop to each
horizon. That is exactly what the filter does between camera corrections, so it is what the
fit optimizes and what the noise model characterizes.

## Next steps

- Field-test `MotionProfileNavigation` against `PursuitNavigation` on hardware; the deltas
  so far are from sim (`docs/experiments/control_improvement/stage4_sim_report.md`).
- Pass the plant fit's acceptance criteria so production configs can move our robot off
  dead reckoning onto the EKF arm.

## Config handling

- `apply_plant` hands the shared top-level `[plant]` table to a config after `parse_fields`. It is separate from parsing because the plant is a top-level section and the config's own parser never sees it. Controllers and estimators that do not model the plant (pursuit, fixed velocity, dead reckoning) keep the no-op.
- A missing `[plant]` is not an error at parse time (`plant/config.hpp`): a profile that runs neither the EKF nor `MotionProfileNavigation` needs no plant, and those that do raise their own error when they find it missing.
- `MotionProfileNavigation` throws on a missing `[plant]`. Every plant term it inverts comes from the fit, so there is nothing sensible to fall back on: a defaulted plant would brake and feedforward against a drivetrain no experiment measured.

## Plant parameters

- `PlantParams` mirrors `PlantParams` in `auto_battlebot/plant.py` field for field. Values come from the jig fit (`fit_jig_plant.py`) copied into the `[plant]` table; the fit output stays the source of truth. Model structure selection happens fit-side: a ladder rung disables a term by writing it as zero, so the struct needs no ladder knowledge. Zero tau means no lag, zero delay means no delay.
- `PlantModelInterface` is the process model for the our-robot EKF: commands in, predicted state and Jacobian out. `JigPlantModel` mirrors `auto_battlebot/plant.py` exactly (deadzone, per-sign gain, transport delay, asymmetric first-order lag, arc integration with 2 ms substeps). `plant_model_` in `KalmanMotionEstimator` holds the concrete type so `coast()` can propagate a copy without paying for the finite-difference Jacobian.
- Dead-reckoning arms use the fitted gains alone (`k_fwd`, `k_rev`, `k_ang`): no deadzone removal, no steer-brake or droop coupling, no drift, no lag. They hold one command flat across a single frame and carry no state, so the coupling terms would model a response the arm cannot represent. Use `plant_steady_state()` instead when the caller integrates a command history with lag, as `JigPlantModel` does for the EKF arm. `linear_y` is always zero: a differential drive has no lateral degree of freedom and every transmitter writes zero there.

## MotionProfileNavigation control law

Motion-profiled tracker with inverse-plant feedforward and speed feedback. For a first-order drivetrain it drives to the goal and arrives at a commanded terminal velocity without overrunning.

The terminal velocity is per behavior mode: ATTACK drives through the opponent at full speed, RUN_AWAY arrives stopped at the safe point. Both are configurable, and a mode flip resets the trajectory state because the mission changed. The config value is a fraction of the plant's `k_fwd`, clamped to [0, 1] (0 is a precise zero-velocity stop, 1 is full speed). It is normalized rather than m/s so a refit rescales it instead of leaving a hand-copied speed stale.

1. Distance-to-go velocity profile. The residual travel after commanding v_term on a first-order plant is about v * (tau_decel + latency), so the reference speed is capped at v_term + d / (tau_decel + latency): a coast-aware brake schedule derived from the measured plant, not a hand-tuned brake distance. Ramp-up is rate-limited for a clean launch.
2. Inverse-plant feedforward. Maps the reference speed (m/s) back to a normalized command by dividing by max speed, adds tau_accel * dv/dt to overcome spin-up lag, divides out the steer-brake authority the turn is costing, and applies exact inverse-deadzone.
3. Speed PID feedback. Corrects plant-model error using our filtered speed, so open-loop feedforward accuracy and disturbance rejection stay separated. The feedback is dropped while our track is stale, because a dead-reckoned speed is a function of our own command and closing a loop on it is not feedback. The last measured speed is held through stale stretches so the latency lead keeps working while the speed feedback is off.
4. Angular control in the same shape: turn-lock hysteresis, a PD on heading producing a yaw-rate reference in rad/s, then an inverse-plant feedforward through `max_angular_speed` with the regime-appropriate yaw time constant and the angular droop divided out.

A mission reset clears the trajectory and integrator state that only makes sense within one mission: turn commitment, reference speeds, the speed integral, and the coupling history. It runs when a zero-velocity goal is reached and when the behavior mode flips. It leaves the pose and speed memory alone, since those describe the robot rather than the mission.

## Plant terms in the navigation config

- The measured drivetrain is stamped in by `apply_plant` from the shared top-level `[plant]` table rather than parsed from `[navigation]`. The controller inverts the same fit the our-robot EKF propagates, so both read one table. `k_fwd` / `k_rev` are the divisors mapping a reference speed back to a normalized command, `tau_lin_a` feedforwards dv/dt against spin-up lag, `tau_lin_d` and `delay_s` set the brake horizon, `c_sb` is the steer-brake loss the controller divides back out, and `dz_ang_l` / `dz_ang_r` turn a commanded turn into the effective turn the coupling terms are defined against.
- `steer_brake_floor`: floor on the steer-brake authority multiplier. 1/c_sb = 0.370 is where the fitted linear loss reaches zero and where the fit's own residuals say the shape stops describing the plant, so the compensation saturates there instead of extrapolating into a singularity.
- Angular control is closed in rad/s, the same shape as the linear channel. `PursuitNavigation`'s angular gains are NOT interchangeable: there they are normalized command per radian, here they are rad/s per radian, a factor of `k_ang` apart.
- Droop compensation (fit c_ad = 0.463): the plant multiplies yaw authority by (1 - c_ad * |u_lin_eff|), so the heading loop is weaker at speed than in place. It defaults OFF even though the coefficient is measured, because compensating it makes the controller worse. On the 90-degree turning approach, enabling it takes terminal error from 0.008 m to 0.067 m and time-to-goal from 1.10 s to 2.83 s. Compensating buys faster heading convergence by commanding a harder turn, and a harder turn is exactly what the steer-brake term charges forward speed for. The heading loop is closed-loop already and gets there without the help; the forward speed it spends is not refunded.
- Floor on the droop multiplier: c_ad = 0.463 never reaches zero, so the floor is a guard against a refit pushing the coefficient past 1, not a shape limit like `steer_brake_floor`.

### The coupling compensations are mutually recursive

The droop term reads the linear command and the steer-brake term reads the turn command. Each is driven by the previous tick's UNCOMPENSATED counterpart, which breaks the loop. Feeding the compensated values back instead has a loop gain above 1 and saturates both channels within a few ticks.

## Temporal motion filter

`RobotTemporalMotionFilter` reads only `k_fwd`, `k_rev` and `k_ang` from the shared `[plant]` table. Its prediction holds one command flat over a frame, with no state to carry lag or coupling.

## Transmitter acceleration limit

`max_motor_rpm_per_sec` slew-limits the linear command so wheel surface acceleration stays below the slip and flip threshold. The measured limit is 8.5 m/s^2, which is 8.5 * 60 / (pi * 0.05) = about 3247 RPM/s for the 0.05 m wheel radius. 0 disables it, the default: no profile limits, so the plant is characterized and the RL policy trained without one.

## Notes moved from `src/` comments (2026-11-15)

Each entry names the file and the code it described.

### `src/navigation/motion_profile_navigation.cpp`

Beside `settings.barrier_enable = config.hazard_barrier_enable;`:

The barrier's envelope is the plant fit itself: the same tau_lin_d + delay_s horizon the goal brake schedule inverts, so one refit moves both.

### `src/navigation/motion_profile_navigation.cpp`

Beside `const double authority = std::max(floor, 1.0 - coeff * std::abs(effect));`:

The plant scales the channel by (1 - coeff*|effect|). Below the floor it delivers so little that dividing by the modelled multiplier is extrapolation, not compensation: for the steer-brake term the floor sits just under 1/c_sb, where the fitted linear loss reaches zero and the fit stops describing the plant. Saturating there keeps the command finite and stops the controller from believing it has authority it does not have.

### `src/navigation/motion_profile_navigation.cpp`

Beside `const double tau = (dvdt >= 0.0) ? tau_accel_ : tau_decel_;`:

Inverse-plant feedforward: steady-state fraction plus the lag term tau * dv/dt, using BOTH signs of dv/dt and the regime-appropriate time constant. A rising reference (dv/dt > 0) adds thrust to overcome spin-up lag; a falling reference (dv/dt < 0) reduces throttle or commands reverse, which is what a first-order plant needs to actually track a deceleration instead of coasting past the goal.

### `src/navigation/motion_profile_navigation.cpp`

Beside `const double dwdt = (dt > 0.0) ? (w_ref - prev_w_ref_) / dt : 0.0;`:

Same inverse-plant shape as the linear channel. The yaw spin-up constant is about twice the coast constant, so the feedforward has to know which way the reference is moving: a turn that is winding up needs extra command, a turn that is stopping needs the command pulled back early or the heading sails past.

### `src/transmitter/tank_drive_processor.cpp`

Beside `const auto wheels_in = mix_to_wheels(saturated);`:

The static-friction barrier exists per wheel, not per body axis, so the lifted deadzone is applied in wheel space, which is also the space these channels are sent in.

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
- `KinematicPlant.footprint_center_offset`: The pose is the centre of the disc. The fits behind this plant tracked the keypoint midpoint, which sits within 2 mm of the footprint centre.
- `KinematicPlant.step`: Coupling multiplies the steady-state target, not the achieved speed: turning costs a fraction of the forward command's authority and driving costs a fraction of the turn's, but neither brakes a robot that is already coasting.
- `KinematicPlant._substep`: Pose uses the velocity at the start of the substep, so the arc is exact for the v and w actually held over it.
- `KinematicPlant._substep`: Accel or decel is decided per channel by whether the target is further from zero than the current speed. Braking into a reversal uses the decel constant until the sign flips, as the drivetrain does: friction first, then torque.
