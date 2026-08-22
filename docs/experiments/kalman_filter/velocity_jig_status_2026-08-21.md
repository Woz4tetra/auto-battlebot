# Velocity jig status, 2026-08-21

Where the plant model stands after the 2026-08-19 and 2026-08-20 sessions, what the data
does and does not support, and what to record next. Companion to
[`velocity_jig_runbook.md`](velocity_jig_runbook.md), which is the procedure, and
[`kalman_filter_plan.md`](kalman_filter_plan.md), which is the why.

Short version: eight of the twenty-two experiments are done, the bench constants are solid,
and the plant fit fails all three acceptance criteria. The largest single cause is that the
coupling grid has zero passing runs, so two model terms have no data at all. The second is
that one session is not enough to measure session-to-session spread, which is the number the
plan actually wants.

## What is measured

| Exp | Result | Where |
|---|---|---|
| E1 clock probes | residuals 0.004 to 0.007 ms, skew 6 to 8 ppm | every run sidecar |
| E2 gyro noise | ARW 0.51 deg/sqrt(hr), B 0.0024 dps, tau_min 58 s | `out/gyro_bias/LOG-97.TXT` |
| E3 gyro scale | 0.9987, two directions agree to 0.02% | `out/gyro_scale/` |
| E4 encoder scale | 4.183811e-05 m/count, 0.14% | `out/encoder_scale/` |
| E6 polarity | resolved, `mix = tank`, `reverse_angular = true` | `session.toml` |
| E7/E10 deadzones | crossings bracketed but not resolved, see below | `LOG-57`, `LOG-65` |
| E8/E11 steps | `k_fwd` 3.353 +/- 0.215, `k_ang` 32.16 +/- 0.572 | `LOG-62`, `LOG-66`, `LOG-69` |
| E9 coast | `tau_lin_d` 0.089, `tau_ang_d` 0.086 | `LOG-36`, `LOG-70` |
| E14-E17 holdouts | recorded, short of the runbook's durations | `LOG-76` to `LOG-91` |

E1 needs no separate experiment. Every run carries two 200-probe clock fits, and the
residuals sit 300x inside the 2 ms bar.

## The k_ang finding

Stage 2 put max yaw rate at 61.5 rad/s from camera keypoints. The jig measures
**32.16 +/- 0.572 rad/s** from 16 held segments, 1.8% relative, the best-determined
parameter in the set. The ratio is 1.91.

The raw counts refute 61.5 independently of any fit. `LOG-69` recorded at a 2293.76 dps
full scale and peaked at 2054 dps, 90% of scale without clipping. 61.5 rad/s is 3524 dps and
would have railed the channel for the entire top step. It did not.

A factor near two is what a yaw keypoint flip produces. This belongs in the perception
report, not only here.

## Gyro noise, E2

From `LOG-97`, 30.5 min stationary, 1,827,509 samples, no gaps and no saturation. Analysis
is `playground/calibration/fit_gyro_noise.py`, plot in `out/gyro_noise.png`.

```
axis    bias dps         ARW           B   tau_min           RRW
                deg/sqrt(hr)         dps         s    deg/hr^1.5
gx        0.2722      0.5347     0.00104     182.7           n/a
gy       -0.2792      0.5498     0.00170     182.7           n/a
gz       -0.4682      0.5118     0.00241      57.6          40.3
yaw       0.4648      0.5102     0.00241      57.6          40.2
```

The curve holds a clean -1/2 slope from 1 ms out to about 20 s, floors, and turns up on yaw
past 58 s. `yaw` and `gz` coincide because gravity reads along -z, so the projection is
essentially -gz; the sign difference is the projection, not a disagreement.

Three consequences:

- **A 10 s still hold pins the yaw bias to 0.0027 deg/s**, worth 0.08 deg of heading drift
  over a 30 s run. That is small enough to ignore against the 8 deg heading target.
- **tau_min is 58 s, so the current 5 s hold sits well inside the averaging region.** Longer
  holds keep helping up to about a minute. This is independent support for raising `--hold-s`
  to 10, which was already needed for a different reason (below).
- **RRW is not measurable from 30 minutes on gx and gy.** Their curves had not turned up by
  183 s, the longest cluster a 30 min record supports. That bounds their drift from above
  rather than measuring it. Yaw did turn up, so its 40.2 deg/hr^1.5 is real. A 2 h record
  would close the other two.

## How the fit works

Ten steps, two of which do the real work. All equations here are plain text, not LaTeX.

| step | what happens | output |
|---|---|---|
| 0 | bench constants set the units | `jig_calibration.toml` |
| 1 | load, resample, gate, split train/holdout by declared role | run set |
| 2 | Stage A: one closed-form estimator per experiment | values with `+/-` and `n` |
| 3 | delay profile, grid search | cost curve |
| 4 | Stage B: joint fit over windowed open-loop error | fitted parameters |
| 5 | score by horizon, train and holdout | error tables |
| 6 | acceptance criteria C1, C2, C3 | pass or fail |
| 7 | model ladder M0 to M6 | which rung the data supports |
| 8 | leave-one-session-out | generalization across sessions |
| 9 | cross-checks against geometry and stage 2 | sanity |
| 10 | write params and HTML report | `plant_params.toml`, `jig_fit.html` |

### The model

State is `(x, y, theta, v, w)`. Deadzone removal runs per sign, rescaled so the gain still
means max speed rather than max speed divided by `(1 - d)`:

```
u_eff = sign(u) * max(|u| - d, 0) / (1 - d)

  d = dz_lin_fwd  when u >= 0
  d = dz_lin_rev  when u <  0     (dz_ang_l / dz_ang_r on the angular channel)
```

Steady-state targets for a held command:

```
v_target = k_fwd_or_rev * u_lin_eff * (1 - c_sb * |u_ang_eff|)
w_target = k_ang        * u_ang_eff * (1 - c_ad * |u_lin_eff|)
```

Coupling multiplies the target, not the achieved speed. Turning costs a fraction of the
forward command's authority; it does not brake a robot that is already coasting.

Velocity follows a first-order lag, discretized exactly rather than by Euler:

```
alpha = exp(-dt / tau)
v_next = alpha * v + (1 - alpha) * v_target

  tau_lin = tau_lin_a  when |v_target| > |v|   (speeding up)
  tau_lin = tau_lin_d  otherwise               (slowing down)
```

The tau switch is evaluated per substep, so braking into a reversal uses the decel constant.
That is what the drivetrain does: friction acts before torque does.

Pose integrates along a circular arc, not a straight segment:

```
theta' = theta + w * dt
x'     = x + (v / w) * ( sin(theta') - sin(theta) )
y'     = y - (v / w) * ( cos(theta') - cos(theta) )
```

with a straight-line fallback when `|w| <= STRAIGHT_W`, to avoid dividing by zero. Pose uses
the velocity at the start of the substep and the velocity update follows it, matching the
C++ ordering.

### Stage A: one estimator per experiment

Each parameter comes from the excitation built for it, in dependency order, because each
step needs the one before it.

**Deadzone.** Each staircase rung gives a median steady speed `s_i` at level `L_i`. A rung
counts as moving when

```
s_i > max( floor, 5 * sigma / sqrt(n_i) )
```

The `sqrt(n)` matters: the test is on a median over a hold of `n` samples, so the threshold
is 5 sigma of the mean, not of a single sample. The deadzone is linear interpolation between
the last still rung and the first moving one. **The reported uncertainty is the rung
spacing**, which is why the 0.01 staircase reported `+/- 0.0098` no matter how many times it
ran, and why finer steps buy more than more repetitions.

**Gain.** Held segments give pairs `(|u_eff|, s)`. Least squares through the origin:

```
k = sum_i (x_i * y_i) / sum_i (x_i * x_i)      x_i = |u_eff,i|,  y_i = s_i
```

Forced through the origin because `u_eff = 0` must give zero speed by construction. The
quoted `+/-` is the standard deviation of the per-segment ratios `y_i / x_i`, not the
standard error of the slope, so it does not shrink automatically as `1/sqrt(n)`. That is why
`k_rev` at `+/- 38%` from n=4 needs more segments rather than longer ones.

**Rise constant.** Log-linear over the 15% to 90% band of the rise:

```
ln( v_ss - v(t) ) = ln(v_ss) - t / tau_a
```

fitted over the first contiguous in-band run only. This was the stage 2 fix that mattered
most: one late steady-state sample dipping back into the band flattens the slope and inflates
tau tenfold, turning a 55 ms rise into a reported 1.1 s.

**Decel constant.** Same idea on coast tails after the command drops below 0.02:

```
slope = polyfit( t, ln|v(t)|, 1 )
tau_d = -1 / slope
```

truncated where speed falls under `max(3 * eps, 0.1 * v_0)`, so the noise floor does not drag
the tail flat.

**Coupling.** Two linear systems on the grid cells, with gains and taus already known:

```
v / (k * u_lin_eff) = 1 - c_sb * |u_ang_eff|
w / (k_ang * u_ang_eff) = 1 - c_ad * |u_lin_eff|
```

`fit_drift` builds the design matrix `[x, sign(x)]` and solves both columns at once, because
fitting them one at a time lets straight-line drift get absorbed into angular droop. Without
cells at both angular signs the two columns are collinear and the system is singular, which
is exactly what `grid is not symmetric in angular sign` reports.

**Delay, the onset stack.** For every command edge with `|delta u| > 0.15`, take a causal
backward difference of velocity, normalize by its standard deviation, sign-flip by edge
direction so accelerations and decelerations reinforce instead of cancelling, and average:

```
S[lag] = mean over edges of  sign(delta u) * accel[t_edge + lag] / sigma_accel
```

Then smooth with `[0.25, 0.5, 0.25]`, subtract the pre-edge baseline, and interpolate a
parabola around the peak for sub-sample resolution. The difference is causal on purpose: a
centered difference leaks response one sample before the edge and biases the delay earlier
than physically possible.

This pooled 873 edges at SNR 10.1, which is why 33.0 ms is the trustworthy delay number.

### Stage B: the joint fit

**Windows.** Cut every 50 ms from each run. Each window carries the measured state at `t0`
and the command sequence over the horizon. Delay is applied by shifting the command matrix
when the window is cut, not inside the model. That is what stops M0 from silently inheriting
M1's delay and scoring identically.

**The residual.** Simulate open loop from the measured `(x, y, theta, v, w)` at `t0`, then
decompose position error into the body frame at window start:

```
along = dx * cos(theta0) + dy * sin(theta0)
cross = -dx * sin(theta0) + dy * cos(theta0)
```

The decomposition is load-bearing. Along-track error comes from speed and delay, cross-track
comes mostly from heading, and a single Euclidean number hides which one is broken. Heading
residuals are wrapped into `(-pi, pi]`.

**The objective.** Residuals are scaled so meters and radians are commensurate:

```
r = [ along / 0.05, cross / 0.05, heading / 0.05 ]
```

so 50 mm of position error weighs the same as 0.05 rad, which is 2.9 degrees. Then
`scipy.optimize.least_squares` with a soft-L1 loss:

```
cost = sum_i rho( r_i^2 )        rho(z) = 2 * ( sqrt(1 + z) - 1 )
```

Soft-L1 grows linearly instead of quadratically once a residual is large, so one slipped
wheel or wall bump cannot steer the parameters. `x_scale="jac"` renormalizes by Jacobian
columns, which is what lets a 0.005 s tau and a 50 rad/s gain sit in the same parameter
vector.

Cost is a raw sum over windows, not a mean. The M2 fit's 38651 over 2460 windows is worse
per window than the M4 fit's 15629 over 1404, so these numbers do not compare across runs
with different window counts.

**Why delay is profiled outside.** The objective is not smooth in delay at millisecond
resolution, because the command matrix is re-cut at each grid point and the cost surface is
piecewise. A gradient method stalls on it. Hence a grid search that refits the continuous
parameters at every point. Resolution is bounded by `--fit-hz`, not by the grid step: on
synthetic data with a known 45 ms delay, a 200 Hz grid puts the minimum at 50 ms and a 500 Hz
grid at 44 ms. Never quote the delay finer than half a grid step.

The horizon table prints 35 and 65 ms rather than the requested 33 and 66 because at 200 Hz
the analysis grid is 5 ms and they snap to it.

### Why this fit misbehaved, in those terms

**Compensating pair.** Over one window, the integral of `1 - exp(-t/tau)` trades against `k`
almost exactly. A 0.6 s hold leaves about 0.2 s of plateau after the rise is excluded, so the
objective sees a ridge rather than a minimum and the optimizer slides along it. That is
`k_fwd` 3.35 to 5.69 moving together with `tau_lin_a` 0.131 to 0.544.

**Delay and tau are also degenerate.** A response with time constant tau delayed by `T_d`
looks much like one with a larger tau and no delay, over a 500 ms horizon. That is why the M2
profile ran to the 0 ms boundary once tau had inflated: the lag was already supplying the
phase.

**Bounds are symptoms.** With no angular excitation the cost gradient with respect to
`k_ang` is near zero, so the optimizer wanders until it hits `PARAM_BOUNDS`. `dz_ang_l = 0.25`
and `tau_ang_a = 0.6` are exactly the upper bounds in `plant.py`.

**The weight check is a conditioning probe.** Doubling the heading weight perturbs the
objective, and the resulting parameter shift is a finite-difference estimate of how sensitive
the fit is to weighting. A well-conditioned fit is dominated by data and nothing moves. The
53.9% shift on M4 against 0.3% on M2 is the same story the ladder tells.

**Residual autocorrelation 0.84** says the residuals violate the independence that least
squares assumes. The parameters may still be near optimal, but the implied uncertainties are
optimistic, because correlated residuals carry less information than their count suggests.

## Why the plant fit fails

Three fits were run. The best holdout came from M2 with the angular runs readmitted.

```
                          C1 @100ms   C2 @400ms   C3 heading @400ms
target                       < 15 mm     < 80 mm            < 8 deg
M4, angular excluded          22.0        119.5              38.18
M2, angular readmitted        20.3        105.4              41.41
M4, ladder, delay fixed       19.9        119.5              29.96
```

### Model complexity is not the problem

The ladder is unambiguous. Holdout error is best at M2 and degrades from M3 up, while train
error keeps falling:

```
model | train 500 ms | holdout 500 ms | params
M2    |        102.2 |          124.8 |  6
M3    |         91.7 |          147.6 | 11
M4    |         91.5 |          147.6 | 13
M6    |         89.0 |          142.0 | 15
```

Train improves 10% from M2 to M3 while holdout degrades 18%. With 7 training runs, the
asymmetry terms fit run-specific detail. M2 is the most complex rung this data supports.

### What is actually missing

**The coupling grid has no passing runs.** `c_sb` and `c_ad` report `0 grid cells, need 3`.
The single attempt, `LOG-71`, was discarded on two gates that were both false positives:

- *Stick contamination 0.065.* Every held stretch above the 0.05 bar had `tgt_lin = 1.0`.
  `to_channels` gives angular priority and clips linear to `1 - abs(ang)`, so a 1.0 cell at
  the 0.13 cap went out as 0.87. Median error across all 56 held stretches was 0.0029,
  identical to the clean runs. The sticks were centered; the tool clipped its own command.
- *IMU saturation, ay x6 and az x3.* Nine isolated 1 ms samples spread over 94 seconds, no
  two adjacent, all pegged at 32764 against an 8 g range. An impact is a burst with ringing.
  This is tank-steer chatter through the guard plates.

**Both angular step runs were excluded on still-hold detection, not on data quality.**
`find_still_segments` requires 5.0 s unbroken while `DEFAULT_HOLD_S` commands 5.0009 s, so
there is no margin:

- `LOG-66`: two accelerometer samples out of 5001 dipped 0.008 g outside tolerance. Gyro
  never exceeded 1.43 dps and the encoder never moved a count. Longest unbroken stretch 2.90 s.
- `LOG-69`: the encoder ticked **once**, 41.8 microns. The +/-25 ms window test marks 50
  samples as moving. Longest unbroken stretch 1.62 s.

`LOG-97` quantifies how normal that is. Over 30.5 minutes of a robot doing nothing the
encoder logged **359 transitions** across a 7-count span, 0.29 mm of apparent travel, and
that alone splits the log into 61 "still" segments with the longest only 2.7 minutes. On a
5 s hold with a 5.0 s minimum, failure is roughly a coin flip.

### Delay is not identified

The Stage A onset stack is stable and well-supported: **33.0 ms from 873 onsets at peak
SNR 10.1**, clock measured. The Stage B profile is not. Across three runs it returned 98 ms
from a single-sample spike, then 0.0 ms on the search boundary. Its cost surface spans only
15% across the whole 0 to 120 ms sweep, so delay is barely identifiable from this data.

Hold delay at the Stage A value with `--no-delay-profile` until there is more data.

### Residual autocorrelation

0.84 to 0.87 at 500 ms in every fit. The residuals are strongly self-correlated, so structure
is missing rather than noise dominating.

### Leave-one-session-out is uninformative

The 2026-08-19 session contains one run, a 4-second `lin_coast`. Holding out 2026-08-20
therefore fits the model on that single run. Both LOSO numbers are artifacts of having
effectively one session.

## Two reporting bugs found

**The delay cross-check calls any pair of numbers agreement.** It printed "39.0 ms apart",
"59.0 ms apart", and "26.0 ms apart" across three runs, each followed by *"Two independent
methods on independent hardware agreeing is the strongest evidence available for this
number."* The template fires unconditionally. Do not quote it.

**`section_next` overflowed on a flat learning curve.** `(TARGET_REL / rel) ** (1 / slope)`
with a slope near -0.004 exceeds float64. Fixed in `5bb8a41` by falling back to the root-N
estimate when the measured slope is flatter than -1e-3. The clamp could be tighter: at -1e-3
`tau_ang_a` still extrapolates to 71 million runs. `-1e-2` would push that row to the
fallback too.

## Changes made, uncommitted

### `firmware/velocity_jig/include/config.h`

```c
#define IMU_ACCEL_RANGE LSM6DS_ACCEL_RANGE_16_G       // was 8_G
#define IMU_GYRO_RANGE ISM330DHCX_GYRO_RANGE_4000_DPS // was 2000_DPS
ACCEL_G_PER_LSB = 0.000488f   GYRO_DPS_PER_LSB = 0.140f
```

Full scale goes from 2293.76 to 4587.52 dps and 7.995 to 15.99 g. Confirmed live in
`LOG-97`'s header. Resolution halves on both channels, so the per-LSB noise floor doubles.
That is the right trade because a clipped sample's true value is unknown and biases every
fit that reads it.

`ISM330DHCX_GYRO_RANGE_4000_DPS` is `0b0001`, a different bit pattern from the other
`LSM6DS_GYRO_RANGE_*` values. `setGyroRange` writes the low four bits of `CTRL2_G` with no
validation, so it works here but would silently select 250 dps on any other LSM6DS part.

### `playground/calibration/waveforms.toml`

| edit | from | to | why |
|---|---|---|---|
| `defaults.coast_s` | 1.2 | 2.0 | 22 decel constants, costs 0.30 m regardless |
| `defaults.angular_cap` | 0.13 | 0.24 | 0.13 was `8.0 / 61.5` using the refuted camera `k_ang` |
| `lin_deadzone` | 0.01-0.10 @ 0.01 | 0.001-0.030 @ 0.001, hold 1.0 s | crossing landed in the first interval |
| `ang_deadzone` | 0.01-0.10 @ 0.01 | 0.001-0.030 @ 0.001, hold 2.0 s | left crossed in the first interval too |
| `ang_coast` | default hold | hold 2.0 s | pivots in place, time is free |
| `coupling_grid.linear` | max 1.0 | max 0.76 | `1 - 0.24`, so no cell exceeds mix authority |
| `lin_prbs` | 5 s | 30 s | runbook E14 |
| `ang_prbs`, `combined_prbs` | 15 s | 30 s | runbook E15, E16 |
| `lin_chirp` | 7 s | 30 s | runbook E17 |

**Linear hold stays at 0.6 s.** An earlier draft raised it to the runbook's 2 s. That was
wrong. At the fitted `tau_lin_a` of 0.1309 s, 0.6 s is 4.6 time constants and 98.98% settled,
and a full-command step costs 1.88 m including the coast. Two seconds would be 15 tau and
6.57 m, more floor than this robot has and more settling than a first-order rise can use.
The runbook's 2 s predates the measured tau; what it guarded against was stage 2's 0.2 s,
which is 1.5 tau and 78% settled.

Precision on `k_fwd` and `k_rev` comes from segment count, not segment length. `k_rev` is
+/- 38% from n=4. Three reps takes that to n=12 and roughly 22%; three sessions to n=36 and
about 13%.

### `playground/calibration/fit_gyro_noise.py`

New. Overlapping Allan deviation per gyro axis plus yaw projected along measured gravity,
with ARW, bias instability and RRW per IEEE 952. Reports RRW as `n/a` rather than fitting a
+1/2 line through a region that is still falling.

### Caveat on `angular_cap = 0.24`

It assumes `rate_limit = 8.0 rad/s` in `jig_calibration.toml` is a real E5 measurement. E5
has not been run: `r_enc_perp` and `r_imu` are both still `0.0`. Confirm before trusting it,
and note that the encoder slip gate was deleted in `95103e8`, so nothing will flag scrubbing
automatically. Watch the wheel.

## Next steps

### Before any session

Flash the new ranges and verify:

```bash
cd firmware/velocity_jig && ~/.platformio/penv/bin/pio run -t upload && cd -
```

Press **C** on the jig. One accel axis must read about 1.00 g. Then one throwaway run, with
a deliberate Ctrl-C partway through to confirm the disarm path:

```bash
source scripts/activate_python.sh
python playground/calibration/velocity_jig_drive.py \
    --waveform lin_step_full --reps 1 --hold-s 10 --name "smoke test, new ranges"
```

Check the runway against your floor before committing. `lin_step_full` has 7 operator stops,
so the constraint is the worst single segment, 1.88 m, not the cumulative figure printed:

```bash
python playground/calibration/velocity_jig_drive.py \
    --waveform lin_step_full --dry-run \
    --params playground/calibration/out/plant_params_m2.toml
```

Without `--params` the predictor uses built-in defaults and reports a 5.60 m/s peak, about
70% high for this robot.

`--hold-s 10` is the **stationary** hold and costs zero runway. It is a different knob from
`hold_s` in `waveforms.toml`, which is the commanded hold. Ten seconds is what makes the
LOG-66 and LOG-69 style exclusions go away, and E2 says the bias keeps improving out to 58 s.

### Session block

Fit battery, 3 reps, about 90 min wall clock:

```bash
python playground/calibration/velocity_jig_drive.py \
    --waveform lin_deadzone --waveform lin_step_full --waveform lin_coast \
    --waveform ang_deadzone --waveform ang_step_full --waveform ang_coast \
    --waveform coupling_grid \
    --reps 3 --hold-s 10 \
    --name "mrs buff mk3, pack 1"
```

Holdouts, 3 reps:

```bash
python playground/calibration/velocity_jig_drive.py \
    --waveform lin_prbs --waveform ang_prbs --waveform combined_prbs \
    --waveform lin_chirp --waveform ang_chirp \
    --waveform lin_sine --waveform lin_triangle \
    --reps 3 --hold-s 10 \
    --name "mrs buff mk3, pack 1"
```

`--reps 3` replays the identical PRBS sequence. Seeds live in `waveforms.toml` and there is
no `--seed` override, so E16's "different seeds each repetition" needs the seed edited
between runs.

Operator driving, twice with two different drivers, and the trim run that has never been
recorded:

```bash
python playground/calibration/velocity_jig_drive.py \
    --waveform operator --reps 1 --hold-s 10 \
    --name "mrs buff mk3, pack 1" --notes "driver: <name>, chasing corner cone"

python playground/calibration/velocity_jig_drive.py \
    --waveform trim --reps 2 --hold-s 10 --name "mrs buff mk3, pack 1"
```

E20 pack state, bracketing a 3 min drawdown. No sidecar carries a pack voltage, so `--notes`
is the only place it lands:

```bash
python playground/calibration/velocity_jig_drive.py \
    --waveform lin_step_full --reps 3 --hold-s 10 \
    --name "mrs buff mk3, pack fresh" --notes "resting 16.8 V"
# drive hard 3 min, then:
python playground/calibration/velocity_jig_drive.py \
    --waveform lin_step_full --reps 3 --hold-s 10 \
    --name "mrs buff mk3, pack drawn" --notes "resting 15.4 V"
```

Three sessions, different days, different packs.

### Fit after each session

```bash
python playground/calibration/fit_jig_plant.py \
    playground/calibration/out/<session> \
    --ladder --no-delay-profile --bootstrap 8 \
    --name "mrs buff mk3, session N" \
    --out playground/calibration/out/plant_params_v2.toml \
    --report playground/calibration/out/jig_fit_v2.html
```

The one check that says whether the session worked: compare Stage A's `k_fwd` against the
joint fit. M2 agreed to within one sigma; M4 was 11 sigma out. If M2 stops agreeing,
something changed and it is worth diagnosing before recording more.

### Still outstanding

- **E5 lever arms.** No fit script. `r_enc_perp` and `r_imu` are both `0.0`, and the
  `angular_cap = 0.24` change depends on `rate_limit = 8.0` being real. The two regressions
  can be attempted on the existing pure spins in `LOG-69` rather than recording new data.
- **E19 closure runs.** Never run. The only check on the dead-reckoning ground truth that
  every other fit is scored against. Three ~20 m loops, tape-measured, pass under 200 mm.
- **E21 arena joint session.** Also the place to confirm the `k_ang` finding directly: if
  the camera reports about twice the gyro during the same spin, that is the keypoint flip
  caught in the act.
- **E22 weapon delta.** Last, after the plant model is fit.
- **Longer E2** if RRW on gx and gy is wanted. Two hours would do it. Low priority; yaw is
  the axis the filter integrates and yaw is measured.
