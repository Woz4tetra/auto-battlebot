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
`[plant]` table in `config/_common.toml`, parsed into `PlantConfiguration`
(`include/plant/config.hpp`) and handed to both consumers after their own sections parse.
The estimator builds a `JigPlantModel` from it; the controller inverts the same fields for
its brake schedule and feedforward. Neither has C++ defaults, so a config with no `[plant]`
is a load error rather than a subsystem running on zeros.

`playground/calibration/write_plant_configs.py` rewrites the numeric literals in
`simulation/sim_mrs_buff_mk3.toml` and `config/_common.toml` in place, so the hand-written
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

- `[plant]` in `config/_common.toml`, the shared fit both subsystems read.
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
