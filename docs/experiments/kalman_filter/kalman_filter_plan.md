# Kalman filter plan

Plan for replacing `RobotFrontBackSimpleFilter`'s dead reckoning with a Kalman filter that carries an
explicit uncertainty. Two estimators, one per robot class:

- **Our robot**: an EKF driven by a calibrated plant model. We know the commanded velocity, so the
  prediction step can be physically accurate and the covariance can stay tight through camera
  dropouts.
- **Opponents**: a model-less EKF (constant velocity plus white-noise acceleration). No control
  input, so the estimate lags the truth during maneuvers. That lag is acceptable as long as the
  covariance grows to cover it.

The prediction model for our robot is the load-bearing piece and gets its own part below. The data
comes from `firmware/velocity_jig`, which puts the encoder and IMU on the robot instead of watching
it with an overhead camera.

## Why the jig changes what is measurable

The 2026-07-03 AprilTag calibration
([`control_improvement/stage2_mr_stabs_mk2_calibration.md`](../control_improvement/stage2_mr_stabs_mk2_calibration.md))
produced one number I trust (actuation lag 59 ms) and a set of time constants that each rest on a
single segment. The limitation was ground truth, not fitting: the tag was visible 14-46% of the time
during linear phases because driving forward walks the robot out of the overhead camera's FOV.

The jig removes that failure mode. Encoder and gyro ride on the robot, so ground truth does not
depend on staying in frame, and a run can be as long as the floor and the battery allow. Every
parameter that came back `nan` in stage 2 (forward accel tau, reverse max speed, steer-brake
coupling) is now measurable. In exchange we take on two new problems that the camera did not have:
dead-reckoning drift, and clock alignment between the jig and the host that issues commands. Both
are addressed below.

## Success criteria

Numbers, not adjectives. Each is a gate on the stage that follows it.

| # | Criterion | Threshold | Measured by |
|---|-----------|-----------|-------------|
| C1 | Open-loop model position error, 100 ms horizon | RMSE < 15 mm | Part 1.6 |
| C2 | Open-loop model position error, 400 ms horizon | RMSE < 80 mm | Part 1.6 |
| C3 | Heading error, 400 ms horizon | RMSE < 8 deg | Part 1.6 |
| C4 | Envelope coverage of the noise model | 95% of windows inside the 95% ellipse (2.45 sigma in 2D), per horizon bin from 30 to 500 ms | Part 1.7 |
| C5 | Filter consistency, our robot | mean NEES inside the 95% chi-square band for the state dimension | Part 2.6 |
| C6 | Filter consistency, opponents | mean NIS inside the 95% chi-square band, 2 DOF | Part 3.4 |
| C7 | Our-robot pose error vs jig truth on a live Jetson run | 50th/95th percentile better than `RobotFrontBackSimpleFilter` on the same recording | Part 4.3 |
| C8 | No regression in the nav sim sweep | hit rate and time-to-stop within noise of the Stage 3 baseline | Part 4.4 |

The thresholds in C1-C3 come from the aim-assist requirement, not from what is easy: at 5.6 m/s the
robot covers 560 mm in a 100 ms gap, so a 15 mm prediction error is a 2.7% error on the coast. If the
model cannot hold that, the KF has nothing to coast on and we fall back to holding position.

## Baseline: what the filter does today

`RobotTemporalMotionFilter::update_with_prediction` (`src/robot_filter/robot_temporal_motion_filter.cpp:42`)
propagates an unmeasured track by applying the last commanded velocity directly as true velocity:

```cpp
velocity_field = body_velocity_to_field(cmd.linear_x, cmd.linear_y, yaw);
pose.x += velocity_field.x() * dt;
pose.yaw = wrap(yaw + cmd.angular_z * dt);
```

Five things are wrong with it, and each one is a term the new model adds:

1. **No transport delay.** The command takes ~60 ms to reach the wheels. For the first two frames
   after a command change the robot is still doing the old thing.
2. **No actuator dynamics.** Commanded velocity is treated as achieved velocity. Measured rise time
   constants are 58 ms (accel) and 78 ms (decel), the same order as the frame period.
3. **No deadzone or saturation.** A command of 0.02 produces no motion; a command of 1.0 produces
   whatever the pack can deliver at that state of charge.
4. **No uncertainty.** `Velocity2D` and `Pose` carry no covariance, so downstream code cannot tell a
   fresh measurement from a 400 ms extrapolation except by the `is_stale` flag.
5. **Euler integration of a rotating body.** At 30 Hz and a yaw rate near the calibrated
   61.5 rad/s, one tick is 2 rad of rotation. Straight-line Euler stepping is meaningless there.

Point 5 has a second implication worth checking early: 61.5 rad/s is 3524 deg/s, and the jig gyro is
configured for +/-2000 dps (`IMU_GYRO_RANGE` in `firmware/velocity_jig/include/config.h:53`). If the
calibrated number is real, the gyro clips. See 1.2 for the fix.

## Notation

Field frame is the arena frame used by `FieldDescription`, x right, y up, yaw counter-clockwise from
+x. Body frame is x forward, y left.

```
p     = [x, y]      position, field frame, meters
theta               heading, field frame, radians, wrapped to (-pi, pi]
v                   body forward speed, m/s (tank drive: no lateral term)
w                   yaw rate, rad/s
u     = [u_lin, u_ang]  commanded velocity, the VelocityCommand the transmitter sent
u_eff                   effective command after deadzone and saturation
L_d                     transport delay, seconds, command issue to wheel response
tau_a, tau_d            first-order accel and decel time constants, per channel
h                       prediction horizon (time since the last camera correction)
```

---

# Part 1: The prediction model

This part is the experiment. Everything after it is engineering.

## 1.0 What the model has to do

The KF calls the model once per tick to answer: given the commands I issued and where the robot was
at the last correction, where is it now, and how sure am I? The horizon that matters comes from the
measured dropout distribution (`perception_reliability_may_fights.md`): median gap 56 ms, p90 338 ms,
p95 497 ms. So the model must be accurate from one frame (33 ms) out to about 500 ms, and the noise
model must be honest across that whole range. Past 500 ms we stop trusting it and coast on hold,
which is the existing policy.

The model must also be cheap. It runs inside the perception loop with a 60 ms end-to-end budget, so
no allocation, no iterative solves, fixed-size buffers.

## 1.1 Model structure

Grey box. Every term is a physical effect with a parameter I can defend, and the structure is chosen
so that each term is separately excitable by a phase of the drive protocol.

**Stage 1, static input map.** Per channel and per sign, because forward and reverse are measurably
different on this drivetrain (5.60 vs 4.84 m/s in stage 2):

```
u_eff = 0                                            if |u| <= dz
u_eff = sign(u) * (|u| - dz) / (1 - dz)              otherwise
v_ss  = k_fwd * u_eff       if u_eff > 0
v_ss  = k_rev * u_eff       if u_eff < 0
```

`k_fwd`, `k_rev`, `k_ang` are the max speeds. Deadzone `dz` is per channel and per sign. Stage 2
floored the deadzone at 0.04 because that was the smallest step tested; the jig staircase goes to
0.01 so the floor is real this time.

**Stage 2, transport delay.** A pure delay on the command, implemented as a ring buffer of past
commands. `L_d` is one number for both channels: the path (Crossfire link, ESC, mechanical) is
shared, which is what let stage 2 pool linear and angular onsets into one estimate.

**Stage 3, actuator lag.** First-order, with separate constants for speeding up and slowing down.
The asymmetry is physical: accel is limited by available torque, decel by drivetrain friction plus
whatever braking the ESC does.

```
tau = tau_a if |u_eff(t - L_d) * k| > |v| else tau_d
dv/dt = (k * u_eff(t - L_d) - v) / tau
```

Discretized exactly over a substep dt with alpha = exp(-dt/tau):

```
v_{k+1} = alpha * v_k + (1 - alpha) * v_target
```

**Stage 4, cross-coupling.** Two candidate terms, both unmeasured today:

```
v_target  <- v_target * (1 - c_sb * |u_ang_eff|)     steer-brake: turning costs forward speed
w_target  <- w_target * (1 - c_ad * |u_lin_eff|)     angular droop under linear load
```

Both are hypotheses until the combined lin+ang grid (1.4) says otherwise. If a coefficient fits
within its own confidence interval of zero, it comes out of the model. I would rather ship three
terms that are all real than six where two are noise.

**Stage 5, kinematics.** Exact arc integration, not Euler. For constant v and w over a substep:

```
theta' = theta + w*dt
x'     = x + (v/w) * ( sin(theta') - sin(theta) )
y'     = y - (v/w) * ( cos(theta') - cos(theta) )
```

with the |w| < 1e-6 branch falling back to the straight-line form. v and w are not actually constant
across a 33 ms tick because of the first-order lag, so propagate with internal substeps of 2 ms
(16 substeps per frame). At 61 rad/s that is 0.12 rad per substep, where the constant-w assumption
holds to well under a millimeter. Cost is 16 sin/cos pairs per tick, which is nothing next to
inference.

**Parameter vector.** Priors come from stage 2 run `175805`, and the confidence column is what the
jig has to improve on.

| Parameter | Prior | Prior confidence | Excited by |
|-----------|------:|------------------|------------|
| `dz_lin_fwd`, `dz_lin_rev` | <= 0.04 | floored, not measured | staircase |
| `dz_ang_l`, `dz_ang_r` | <= 0.04 | floored | staircase |
| `k_fwd` (m/s) | 5.60 | medium, extrapolated | steps, max hold |
| `k_rev` (m/s) | 4.84 | low-medium | steps, max hold |
| `k_ang` (rad/s) | 61.5 | medium, possibly inflated by keypoint flips | spin hold |
| `tau_lin_a` (s) | 0.058 | low, one segment | steps |
| `tau_lin_d` (s) | 0.078 | low, one coast | coast tails |
| `tau_ang_a` (s) | 0.066 | low, one segment | angular steps |
| `tau_ang_d` (s) | unmeasured | none | angular coasts |
| `L_d` (s) | 0.059 | high | every command edge |
| `c_sb` | unmeasured | none | lin+ang grid |
| `c_ad` | unmeasured | none | lin+ang grid |

## 1.2 Instrumentation and ground truth

The jig logs one row per sample at 1 kHz (`SAMPLE_RATE_HZ`), IMU internal ODR 1.66 kHz:

```
t_us, count, gx, gy, gz, ax, ay, az        (raw int16 IMU, 4x quadrature count)
```

Ground truth position comes from unicycle dead reckoning on encoder arc length and gyro yaw. Before
that is trustworthy, five calibrations have to happen, in this order.

**(a) Fix the sensor ranges before recording anything.** Two configured ranges are too small for this
robot:

- Gyro at +/-2000 dps clips at 34.9 rad/s. The stage 2 fit says max yaw rate is 61.5 rad/s. Even if
  that number is inflated, the margin is gone. Switch `IMU_GYRO_RANGE` to the ISM330DHCX 4000 dps
  extension (69.8 rad/s) for all angular phases.
- Accel at +/-8 g clips on impacts and, more insidiously, on centripetal acceleration alone. An IMU
  mounted 50 mm off the yaw axis at 35 rad/s sees `w^2 * r` = 61 m/s^2 = 6.2 g of pure centripetal
  before any real acceleration. Switch to +/-16 g.

Either way, log the fraction of samples where `|raw| > 32000` per axis and print it in the fit
report. A saturated channel silently biases every downstream fit, and stage 2 already taught us what
silently-degraded ground truth costs.

**(b) Encoder scale, counts to meters.** Roll the robot by hand over a tape-measured 3.000 m
straight line, 10 times in each direction. Fit meters per count by least squares through the origin
and report the standard error. Ten repeats at a 3 m baseline should give scale to better than 0.3%.
Also record the reverse direction separately as a check on encoder direction sign, not because scale
should differ.

**(c) IMU lever arm.** If the IMU is mounted at radius `r_imu` from the yaw axis, a pure spin
produces a lateral acceleration `w^2 * r_imu`. Run pure spins at four yaw rates, regress measured
lateral accel against measured `w^2`, and the slope is `r_imu`. Do the same for the encoder wheel: a
trailing wheel mounted off the yaw axis accumulates counts during a pure spin, with rate
`w * r_enc_perp`. Regress encoder rate against gyro rate during pure spins to recover `r_enc_perp`.
Both lever arms then get removed in the ground-truth solver, so combined lin+ang segments are usable
instead of being thrown away.

**(d) Gyro bias and noise.** Before and after every recording, hold the robot still for 30 s. Bias is
the mean over that window. The difference between the pre and post estimates bounds the in-run bias
drift, and any run where that difference implies more than 1 deg of accumulated heading error gets
rejected. Separately, one 30 minute stationary log gives an Allan deviation curve, from which we read
angle random walk (deg/sqrt(hr)) and rate random walk. Those two numbers are the gyro's contribution
to the heading uncertainty in 1.7, so they are worth the half hour once.

**(e) Gyro scale factor.** Mount the robot on a marked pivot, rotate through exactly 10 full turns by
hand in each direction, and compare the integrated gyro angle to 3600 deg. Scale error under 1% is
fine; anything larger is a real correction because heading error feeds cross-track error linearly.

**Ground truth construction.** With scale, bias, and lever arms in hand:

```
w(t)      = (gz(t) - bias) * scale
theta(t)  = theta_0 + integral w dt                    (trapezoidal, 1 kHz)
ds(t)     = (count(t) - count(t-1)) * meters_per_count
v(t)      = ds/dt - w(t) * r_enc_perp                  (lever-arm removed)
p(t)      = p_0 + integral [v cos(theta), v sin(theta)] dt
```

**Drift budget.** Heading drift dominates position drift, because cross-track error is roughly
`v * h * theta_err`. With a gyro bias residual of 0.02 deg/s after (d), heading error after 20 s is
0.4 deg, which at 3 m/s over the last second is 21 mm of cross-track. That is acceptable for
validating a 500 ms model. It would not be acceptable for validating a 30 s trajectory, which is why
the validation metric in 1.6 is windowed, not whole-run.

**Closure test.** Every session includes at least two runs that end back at a marked start point,
same heading. Report the closure error. A run whose closure error exceeds 1% of the path length gets
excluded from fitting, and the reason gets investigated (wheel slip, lift-off, or clipping).

**Encoder slip detection.** The trailing wheel can slip under hard accel or lift during a wheelie.
Cross-check encoder-derived longitudinal acceleration against IMU longitudinal acceleration (after
removing the `w^2 * r` and `alpha x r` terms). Windows where the two disagree by more than 3 sigma of
their normal difference get flagged and excluded from the fit, the same way stage 2 excluded tag
dropouts. Report the excluded fraction per run. If it is large under exactly the phases we care about,
that is a finding, not a nuisance.

## 1.3 Time alignment between commands and jig data

This is the single most likely way to get a wrong answer, so it gets its own section.

**The problem.** Commands are logged on the host with `CLOCK_MONOTONIC`
(`playground/calibration/calib_lib/drive_protocol.py`). The jig logs `time_us_64()`, microseconds
since RP2040 boot. The two clocks have an unknown offset and an unknown relative rate.

**The identifiability trap.** The obvious approach is to cross-correlate command edges against
measured acceleration and call the peak the actuation latency. That estimate is
`delta = clock_offset + L_d`. Cross-correlation cannot separate the two, so an unmodeled 40 ms clock
offset shows up as a 40 ms error in the transport delay, which then poisons every time constant fit
against it. Stage 2 avoided this by having the camera and the command log share one process and one
clock. The jig does not have that luxury.

**Primary fix: probe the jig clock over USB.** Add a `TIME` command to the firmware console that
replies with the current `time_us_64()` immediately. The console is idle-only (it answers `BUSY`
while recording, `src/main.cpp:540`), which is fine: probe before pressing A and again after pressing
B. Protocol per probe set:

1. Host records `t0 = CLOCK_MONOTONIC`, sends `TIME`, reads the reply, records `t1`.
2. Keep the sample if `t1 - t0` is within the lowest decile of round trips (min-RTT filtering, the
   same idea NTP uses). Take 200 probes; the survivors have symmetric transport within a few hundred
   microseconds.
3. Estimate host time of the jig timestamp as `(t0 + t1)/2`, with residual uncertainty `(t1 - t0)/2`.

Fit offset and skew from the pre-run and post-run probe sets by a straight line through the surviving
pairs. RP2040 crystals are typically 30 ppm, which is 0.9 ms of drift over a 30 s run, so skew matters
at the margin and is cheap to remove. Target alignment uncertainty is under 2 ms, which is 3% of a
60 ms transport delay.

**Verification.** Probe three times in a session (before, between runs, after) and check that the
linear fit residuals stay under 2 ms. If they do not, the USB path is not symmetric and we fall back
to the method below with its weaker claim.

**Fallback if the firmware change is unavailable.** Run a dedicated sync phase: a hard alternating
command square wave at 2 Hz for 10 s. Cross-correlate against measured acceleration to get `delta`.
This gives correct *relative* timing for everything within a session (the fits for tau, gains, and
deadzone are all valid), but `L_d` is then only identifiable up to the clock offset. In that case,
carry the stage 2 value of 59 ms for `L_d`, fix it during the fit, and say so in the report. Do not
present a delay estimate from this path as a measurement.

**Coarse alignment step, either way.** Before the joint fit, align by normalized cross-correlation of
`d|v|/dt` (from the jig) against the command edge train, at 1 ms resolution, with parabolic
interpolation around the peak. Report the peak SNR the way stage 2 did. This catches gross mistakes
like a mis-selected log file, and it initializes the delay search in 1.5.

**Polarity check, every session.** Stage 2 discovered both channels were anti-correlated with their
command because the solved pose frame was inverted. Guard against a repeat: start every session with
a scripted 1 m forward drive and a 90 deg left turn, and assert that encoder counts and integrated
gyro yaw both increase. Abort the session on a sign mismatch instead of discovering it in the fit.

## 1.4 Excitation design

Ground truth no longer depends on the camera FOV, so the binding constraints are floor space, battery
sag, and operator safety. Design to those.

Standing conditions for every run: guard plates on (their ground friction is part of the plant), pack
voltage logged at the start and end of each run, and runs ordered so that no parameter is
systematically fit from the tail of a pack. Record pack voltage as a covariate and check for a trend
in 1.5.

| Phase | Excitation | Duration | Yields |
|-------|-----------|---------:|--------|
| `polarity` | 1 m forward, 90 deg left | 5 s | sign conventions, abort gate |
| `bias_pre` / `bias_post` | stationary | 30 s each | gyro bias, in-run drift bound |
| `lever` | pure spins at 4 rates | 40 s | `r_imu`, `r_enc_perp` |
| `dz_lin` | staircase 0.01 to 0.10 in 0.01, 2 s hold, both signs | 30 s | linear deadzone, unfloored |
| `dz_ang` | same staircase on angular | 30 s | angular deadzone |
| `step_lin` | steps to 0.25/0.5/0.75/1.0, both signs, 2 s hold, return to zero between | 60 s | `k_fwd`, `k_rev`, `tau_lin_a`, `tau_lin_d` |
| `step_ang` | same on angular | 60 s | `k_ang`, `tau_ang_a`, `tau_ang_d` |
| `grid` | 4x4 grid of held (u_lin, u_ang) combinations, 2 s each | 40 s | `c_sb`, `c_ad` |
| `prbs_lin` | PRBS, amplitude 0.6, bit period 60 ms | 30 s | broadband validation set |
| `prbs_ang` | PRBS on angular, same | 30 s | broadband validation set |
| `prbs_both` | independent PRBS on both channels | 30 s | coupling validation set |
| `chirp` | 0.2 to 8 Hz logarithmic sweep, amplitude 0.4 | 30 s | frequency response cross-check |
| `driving` | operator drives the robot as in a match | 60 s | realistic-distribution holdout |

Two changes from the stage 2 protocol matter:

- **Do not slew-limit the step phases.** Stage 2's forward accel tau was unfittable because
  `build_protocol` slew-limited every forward command increase and then excluded the ramp, so the hold
  began after the robot had already spun up. The whole point of a step is to see the plant's own rise.
  Keep the slew limiter for the operator-driven phase and the safety path, drop it for `step_lin`.
  Size the steps so the robot stops within the available floor, which is now the only reason to be
  gentle.
- **PRBS and chirp are new.** Steps excite one frequency band well. A PRBS at a 60 ms bit period
  covers roughly 0.2 to 8 Hz, which brackets the 58-78 ms time constants and the 33 ms frame period.
  These runs are the honest validation set: they are held out of the fit entirely.

Repeat the full battery three times per robot per session. Three independent sessions per robot,
ideally on different days and different packs, so that session-to-session spread is measurable rather
than assumed.

Safety: the trainer link adds to stick input, so the human driver's sticks stay centered; the runner
zeroes channels and disarms on every exit path including timeout and Ctrl-C. That behavior already
exists in `calib_lib/drive_protocol.py` and gets reused.

## 1.5 Identification methodology

**Two stages, because a joint fit from a cold start on 12 parameters will find a local minimum and
look convincing while doing it.**

**Stage A, per-phase fits for initialization and physical sanity.** Each phase is designed to make one
group of parameters dominant, so each can be fit in near-closed form. This is what
`fit_plant_calib.py` already does, and it ports directly:

- Deadzone: median sustained |velocity| per command level, pick the level where it first clears a
  motion threshold set at 5 sigma of the stationary noise.
- Gains: regress steady-state velocity against `u_eff` over held segments. Now interpolation instead
  of extrapolation, because with the jig we can hold full command.
- Time constants: fit the leading contiguous 15-85% band of each rise, one exponential per segment.
  Keep the stage 2 fix: fit only the first in-band run, never re-entries, or one late steady-state
  sample inflates tau by 10x.
- Delay: pooled onset stack. Every clean command edge, linear timed against forward acceleration and
  angular against yaw acceleration, stacked and peak-picked.

Report per-parameter spread across segments and across runs. A parameter whose across-segment
standard deviation is comparable to its value is not measured, and the report should say so, the way
stage 2 said `nan`.

**Stage B, joint fit by simulation error.** Take stage A as the initial guess and minimize the
multi-step simulation error over sliding windows, which is directly the quantity the KF depends on:

```
J(theta) = sum over windows i, horizons h in H
             w_h * || p_gt(t_i + h) - p_model(t_i + h | x_gt(t_i), u[t_i, t_i+h], theta) ||^2_W
           + same for heading
```

Each window reinitializes the model state from ground truth at `t_i`, then runs open loop for `h`.
Window starts are strided every 50 ms; `H` = {33, 66, 100, 200, 300, 500} ms. This is
simulation-focused identification: it optimizes exactly the multi-step accuracy we need, not the
one-step-ahead accuracy that any model with a delay term can fake.

Details that decide whether this works:

- **Weighting `W`.** Position residuals in meters and heading residuals in radians are not
  commensurate. Normalize each by the empirical residual scale from stage A, then check that the fit
  does not move much when the weights are perturbed 2x.
- **Loss.** Soft-L1 (`scipy.optimize.least_squares(loss="soft_l1")`) so that a slipped-wheel window or
  a wall bump does not steer the parameters. Report how many windows the loss downweights.
- **Delay.** `J` is not smooth in `L_d` at 1 ms resolution. Grid search `L_d` over 0 to 120 ms in 2 ms
  steps, refit the continuous parameters at each, and take the profile minimum. That also produces the
  profile likelihood curve for `L_d`, which is the honest confidence interval.
- **Identifiability.** After convergence, compute the Jacobian at the optimum, then the parameter
  correlation matrix and the condition number of `J^T J`. Any pair with |correlation| > 0.95 is not
  separately identifiable from this data: fix one of them from stage A and report it as fixed rather
  than fit. I expect `k_fwd` and `dz_lin_fwd` to fight, and `tau_lin_a` to fight `L_d` if the
  excitation lacks fast edges.
- **Covariates.** Refit with pack voltage as a linear modifier on `k_fwd` and `k_ang`. If the
  coefficient is significant, report max speed as a function of pack state instead of a constant, and
  decide separately whether the runtime model should carry it. My prior is that it matters and that
  we still ship the constant, with the variation folded into process noise.

**Cross-validation.** Fit on `dz_*`, `step_*`, `grid`. Validate on `prbs_*`, `chirp`, `driving`, which
never enter the objective. Report train and test error separately in every table. Also do a
leave-one-session-out fit to quantify session-to-session transfer, since that is the error we will
actually pay in a match.

## 1.6 Model validation

**Primary metric: windowed open-loop prediction error as a function of horizon.** For each window
start `t_i` and horizon `h`, decompose the error in the body frame at `t_i`:

- along-track (meters)
- cross-track (meters)
- heading (radians)

The decomposition matters because the error sources differ. Along-track error comes from speed and
delay errors. Cross-track error comes mostly from heading error and grows faster. Reporting only
Euclidean position error hides which term is broken.

Report per horizon bin: bias (mean), RMSE, and the 50th, 95th, 99th percentiles, split by held-out
phase. Also report residual autocorrelation: strong autocorrelation at the frame period says the
model has a structural term missing, and no amount of process noise tuning fixes that honestly.

**Model comparison ladder.** Fit and evaluate five nested models on identical data and report the
error reduction each term buys:

| Model | Terms | Purpose |
|-------|-------|---------|
| M0 | commanded velocity applied directly (today's filter) | the baseline we must beat |
| M1 | M0 + transport delay | how much is just delay |
| M2 | M1 + first-order lag, symmetric tau | is asymmetry needed |
| M3 | M2 + accel/decel asymmetry + fwd/rev asymmetry | expected shipping model |
| M4 | M3 + steer-brake and angular droop coupling | is coupling real |

Adopt the simplest model that meets C1-C4. A term that improves 400 ms RMSE by less than 5% does not
ship. This project has one consumer and one robot class per config; extra parameters are extra ways
to be wrong in a match.

**Sanity checks that are worth doing explicitly.**

- Compare fitted `k_ang` against the geometric bound `2 * k_fwd / track_width`. If the fit exceeds the
  bound the fit is wrong, or the track width is.
- Compare fitted `k_ang` against the stage 2 value of 61.5 rad/s. If the gyro says the real number is
  well under that, then the camera-based value was inflated by yaw keypoint flips, which is a finding
  about the perception pipeline, not just about the plant, and it belongs in the report.
- Compare `L_d` against the stage 2 value of 59 ms. Agreement between two independent methods on
  independent hardware is the strongest evidence we will get for that number.

## 1.7 Noise model

The point of this section is a covariance that is right, not one that is small.

**Step 1: collect residuals.** Use the validation runs (PRBS, chirp, driving) and the final model.
For each window start and horizon, store the body-frame error triple plus the conditioning variables:
speed at window start, |commanded angular| over the window, number of command edges in the window,
and pack voltage.

**Step 2: identify the error growth law.** Fit the empirical variance per axis against horizon on a
physically meaningful basis:

```
sigma^2(h) = a*h + b*h^2 + c*h^3
```

Each coefficient corresponds to a mechanism, so the fit tells us which mechanism dominates rather
than just producing a curve:

- `c*h^3` is continuous white-noise acceleration with PSD `q`, giving `sigma_p^2 = q h^3 / 3`. This is
  the standard model for unmodeled accelerations.
- `b*h^2` is a velocity scale-factor error: a constant fractional speed error `eps` gives
  `sigma_p = |v| h eps`.
- `a*h` is a random-walk term, which shows up in heading through gyro angle random walk and feeds
  cross-track error.
- A constant term, if the fit demands one, is delay jitter: `sigma_p ~ v * sigma_delay`, roughly
  constant in `h` once the delay has elapsed.

Fit separately for along-track, cross-track, and heading, and separately in speed bins, because I
expect `b` (scale-factor) to dominate at high speed and `c` to dominate near zero.

**Step 3: build Q.** Map the fitted coefficients into a continuous-time process noise matrix for the
EKF state, so the runtime Q is derived rather than tuned by hand:

```
Q(dt, x, u) = G(x) * diag(q_v, q_w) * G(x)^T * f(dt)  +  scale-factor term proportional to v^2
```

with the scale-factor and delay-jitter terms entering as state-dependent inflation. Write the exact
matrix in the implementation doc once the coefficients exist; the structure follows the mechanism
that the step-2 fit says dominates.

**Step 4: consistency testing.** This is the acceptance gate (C4), and the definition needs to be
precise because "within 2 sigma" is ambiguous in 2D.

- For a scalar axis, 2 sigma is 95.4% coverage.
- For a 2D position with a full covariance, the 2 sigma ellipse contains only 86.5% of the mass. The
  95% ellipse is at 2.448 sigma (chi-square, 2 DOF).

So the criterion is stated as: **95% of windows fall inside the 95% confidence ellipse**, checked by
Mahalanobis distance `d^2 = e^T P^-1 e` against the chi-square distribution with the right DOF. Report
three things per horizon bin:

1. Coverage table at the 68/95/99 levels, empirical vs nominal.
2. Mean NEES with its 95% chi-square band for the window count.
3. A QQ plot of `d^2` against chi-square, which shows tail behavior that a coverage number hides.

**Step 5: tails and impacts.** Combat means getting hit. Impacts produce accelerations far outside the
Gaussian body, so report the p99 and p99.9 residuals separately and do not try to cover them with a
larger Q, which would ruin the estimate everywhere else. Two mitigations to evaluate:

- The jig accelerometer detects impacts directly. Whether the deployed robot will carry an IMU is a
  separate decision, but if it does, an impact flag can inflate Q for a bounded window.
- Without an IMU, the filter sees an impact as a large innovation. Gate it (Part 2.5), and if
  innovations stay large for several frames, reinitialize rather than fight.

Report what fraction of match time falls in the impact tail so the decision is informed by a number.

**Deliverables of Part 1.** A parameter TOML consumed by both the C++ filter and
`simulation/kinematic_sim_server.py`, a `Q` specification, a validation report in this directory with
the coverage tables and the ladder comparison, and the raw jig logs archived alongside.

---

# Part 2: Our-robot Kalman filter

## 2.1 Formulation

EKF. The dynamics are nonlinear through the heading rotation and the deadzone, and the state is small
enough that a UKF's extra sigma-point evaluations buy little. If the linearization turns out to be the
limiting error, a UKF is a drop-in later.

State:

```
x = [ px, py, theta, v, w ]
```

Keeping `v` and `w` as states rather than treating the plant output as a known input is deliberate.
The plant model predicts them, but measurements can correct them, so plant error (a low pack, a
carpet, a bent guard plate) gets absorbed instead of accumulating.

Prediction uses the Part 1 model with the delayed effective command as input, substepped at 2 ms.
The Jacobian `F` is analytic: the arc-integration derivatives with respect to theta, v, w, and the
first-order lag derivatives with respect to v, w.

## 2.2 Delay handling, both directions

Two different delays, easy to confuse, both real:

- **Actuation delay `L_d` (~60 ms).** A command issued now affects the robot later. Handled by a ring
  buffer of past commands, read at `t - L_d` during prediction. Buffer length 256 at 30 Hz covers 8 s,
  which is more than enough and costs nothing.
- **Observation delay (perception latency).** The pose that arrives now describes where the robot was
  when the shutter opened, tens of milliseconds ago. Correcting the current state with a stale
  measurement biases the estimate in the direction of motion, which at 5.6 m/s is a real error.

Handle the observation delay by keeping a short state buffer (last ~20 frames of state and covariance
plus the commands between them), applying the correction at the measurement's own timestamp, and
re-propagating forward to now. This is standard fixed-lag retrodiction. Measure the actual sensor
timestamp path first: if the existing pipeline already stamps measurements at capture time and the
filter runs on that stamp, most of this is already correct and the work is verifying it rather than
building it.

## 2.3 Measurement model

Two measurement types, from `RobotFrontBackSimpleFilter`'s existing outputs:

- **Keypoint pose**: position and heading. `h(x) = [px, py, theta]`.
- **Blob centroid**: position only, larger R.

Three properties of this sensor need explicit handling:

**Heading is 180-degree ambiguous.** Front/back keypoint assignment can flip. A flipped measurement
produces a `pi` innovation, which a naive EKF integrates into a spinning estimate. Options, in order
of preference: reject any yaw innovation whose magnitude exceeds a gate near `pi/2` and accept the
position part only; or track both hypotheses and let the accumulated likelihood pick. Start with the
gate, because it is simple and the plant model gives a strong heading prior between corrections. Log
how often the gate fires; if it fires often, that is a perception bug worth fixing upstream.

**Noise is anisotropic and range-dependent.** The camera measures range and bearing, and the pose is
then projected into the field frame. A fixed diagonal R in field coordinates is wrong at the arena
edges. Model R in camera coordinates (separate range and cross-range variances) and rotate it into the
field frame per measurement.

**There is a known projection bias.** Keypoint projection assumes robots sit flat on the field plane,
so a robot with height off the plane projects to a biased position that depends on where it is
relative to the optical axis. That is a bias, not noise; do not try to cover it with R. Characterize it
from the joint jig session (2.4) as a function of image position, and either correct it or report it as
an error floor.

**Estimating R.** From the joint session: compare the live perception output against jig ground truth,
bin by range and image position, and take the sample covariance. Do this from a recording made **live
on the Jetson**, not by replaying the SVO on a desktop. Desktop replay warps frames a few percent
relative to the live path, which would show up as a fake measurement bias.

## 2.4 Joint jig and camera session

One session gives both the model validation set and the measurement noise characterization:

1. Robot carries the jig, runs the full excitation battery inside the arena, in view of the ZED.
2. The Jetson records the SVO plus the MCAP with perception outputs, filter state, and commands, live.
3. Clock alignment: jig to host by the `TIME` probe (1.3), host to Jetson by NTP or by a shared
   command log if the same host issues commands.

Products: measured R, the projection bias map, the dropout distribution under real motion, and a
ground-truthed A/B test bed for C7.

## 2.5 Gating, dropouts, reinitialization

- **Gate** every measurement on normalized innovation squared against a chi-square threshold at the
  measurement dimension. Rejected measurements are counted, not silently dropped, and the count is a
  diagnostic.
- **Dropout policy.** Predict through gaps with growing covariance. Past a max-coast timeout (start at
  400 ms, near the measured p90-p95 of dropout gaps), stop predicting forward and hold with a
  covariance that keeps growing. Predicting a 5.8 s gap with a 60 ms model is fiction, and the
  existing filter already learned this lesson with the stale-identity decay.
- **Reinitialization.** If innovations are gated out for N consecutive frames, or the trace of P
  exceeds a threshold, reset the state to the next accepted measurement with a large initial P. Fast
  recovery beats a graceful wrong answer.
- **Numerics.** Joseph-form covariance update, symmetry enforcement after each step, angle wrapping in
  every innovation and in the state, and a floor on the diagonal of P so it cannot collapse and lock
  out measurements.

## 2.6 Filter validation

- **Synthetic first.** Simulate with known Q and R, verify the mean NEES lands inside the chi-square
  band over 100 Monte Carlo runs. A filter that fails this is broken as an implementation and there is
  no point running it on real data. This becomes a GoogleTest in `tests/`.
- **Against jig truth.** Position and heading error percentiles versus `RobotFrontBackSimpleFilter` on
  the same recording (C7).
- **On replays without truth.** Held-out measurement prediction: withhold every other detection, then
  score the predicted pose against the withheld measurement. Plus runtime NIS statistics and track
  continuity.

---

# Part 3: Opponent filter

## 3.1 Model

No control input, so the process model has to cover maneuvering with noise. Start with constant
velocity plus continuous white-noise acceleration:

```
x = [ px, py, vx, vy ]
```

Field-frame Cartesian velocity, not body-frame, because opponent heading is unreliable and we do not
need it for interception. Derive a heading from the velocity vector when one is needed, and mark it
low confidence at low speed.

Explicitly not in the initial version: Singer, coordinated turn, IMM. Each is a real improvement for
maneuvering targets and each is more machinery. Adopt one only when the CV+CWNA filter fails C6 in a
specific, identified way, and say which way.

## 3.2 Getting q from data

The tuning parameter is the acceleration PSD `q`. Measure it rather than guessing: take opponent
tracks from the six May 2026 NHRL fight replays, differentiate the position sequences with a
smoothing derivative, and compute the empirical acceleration distribution. Then `q` follows from the
variance and correlation time of that acceleration, per opponent class if the classes differ.

Do this per opponent where the data allows, since each eval recording is one opponent. A drum spinner
and a control bot do not maneuver alike, and a per-opponent `q` is cheap to configure.

## 3.3 Lag is acceptable, but bound it

A model-less filter lags a maneuvering target by roughly one correlation time of the maneuver. That is
fine for aim assist as long as the covariance covers it. What is not fine is a lagging estimate with a
confident covariance. So the acceptance test is coverage (C6), not error magnitude, and the report
should state the measured lag in milliseconds at a few maneuver magnitudes so downstream nav can
account for it.

## 3.4 Validation

- NIS consistency over the replay set (C6).
- Held-out measurement prediction error versus horizon, same method as our robot.
- Track continuity through the measured dropout distribution: how often does the track survive a p90
  gap, and how often does identity swap between opponents.

---

# Part 4: Integration and experiment order

## 4.1 Code layout

| Piece | Location | Notes |
|-------|----------|-------|
| Jig log reader, units, saturation flags | `auto_battlebot/velocity_jig.py` | shared, linted, typed |
| Plant model in Python (for fitting and sim) | `auto_battlebot/plant.py` | one implementation, used by fit and by the sim |
| Excitation runner | `playground/calibration/velocity_jig_drive.py` | built; waveforms from `waveforms.toml`, logs downloaded with a sidecar per run |
| Plant identification | `playground/calibration/fit_jig_plant.py` | ports the per-phase fits from `fit_plant_calib.py` |
| Process noise fit | `playground/calibration/fit_process_noise.py` | produces the coverage tables |
| EKF, our robot | `include/robot_filter/robot_ekf_filter.hpp`, `src/robot_filter/` | registered in the filter factory |
| Opponent filter | same class, model-less branch, or a second class | decide once the our-robot version exists |
| Parameters | TOML, consumed by both C++ and the sim | one source of truth for plant numbers |

Shared analysis code goes in the `auto_battlebot` package so it is linted and typed; the CLI drivers
sit next to the existing calibration scripts. The C++ plant model has to mirror `auto_battlebot/plant.py`
exactly, so a golden-vector test (same inputs, same outputs to 1e-9) belongs in `tests/`.

Remember `cmake -S . -B build` after adding source files; the source glob is not `CONFIGURE_DEPENDS`.

## 4.2 Order of work

1. Firmware: add `TIME`, switch gyro to 4000 dps and accel to 16 g, add saturation counters. Small,
   and it gates everything else.
2. Bench calibrations: encoder scale, gyro bias and scale, Allan variance, lever arms (1.2).
3. Clock alignment probe and verification (1.3). Do not record excitation data until residuals are
   under 2 ms.
4. First excitation session, one robot. Run the ladder (M0-M4), report.
5. Noise model and coverage (1.7). Gate C1-C4.
6. Joint jig plus camera session on the Jetson (2.4). Produces R, the bias map, and the A/B bed.
7. EKF implementation with synthetic NEES test (2.6).
8. A/B against `RobotFrontBackSimpleFilter` on the joint session (C7).
9. Opponent filter, `q` from replays (Part 3).
10. Nav sim sweep for closed-loop impact (C8).

## 4.3 Risks

- **Encoder slip under exactly the conditions we care about.** Hard accel is both the interesting
  regime and the most likely to slip. Mitigation: the IMU cross-check in 1.2, and reporting the
  excluded fraction per phase. If it turns out that most hard-accel windows are excluded, the encoder
  is the wrong sensor for that regime and we fall back to IMU-integrated velocity over short windows.
- **Clock alignment worse than expected.** Falls back to a fixed `L_d` from stage 2 and a weaker claim
  (1.3).
- **Plant varies more across packs and surfaces than the model covers.** Detect via the pack-voltage
  covariate and leave-one-session-out cross-validation. Mitigation is process noise, not more
  parameters.
- **Yaw rate exceeds even the 4000 dps range.** Then the angular parameters are fit from unsaturated
  segments only, and the report says which.
- **The camera measurement is worse than the model over the horizons that matter.** That would be a
  good problem: it means the filter should weight the model heavily, and it makes the case for keeping
  an IMU on the deployed robot.

## Next steps

1. Add `TIME`, the 4000 dps / 16 g ranges, and per-axis saturation counters to
   `firmware/velocity_jig`.
2. Write `auto_battlebot/velocity_jig.py` and confirm it round-trips an existing `LOG-N.TXT`.
3. Run the bench calibrations in 1.2 and record the numbers in
   `docs/experiments/kalman_filter/jig_calibration.md`.
4. Record a first session with `velocity_jig_drive.py`, then fit and read the report's
   "what to collect next" table before planning the rest.
5. Run the first excitation session and report the M0-M4 ladder.
