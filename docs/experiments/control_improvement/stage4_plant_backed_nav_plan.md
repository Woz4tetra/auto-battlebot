# Stage 4 plan: repoint MotionProfileNavigation onto the jig-measured plant

`MotionProfileNavigation` is an inverse-plant controller running on the wrong plant. Its
constants are Mr Stabs Mk2's, carried over from `stage2_mr_stabs_mk2_calibration.md`, and three
of the effects the velocity jig measured on Mrs Buff Mk3 are not in the control law at all. This
plan is the nav-side consumer of the jig fit in `playground/calibration/out/plant_stageA.toml`.
The filter-side consumer is
[`plant_model_poc_plan.md`](../kalman_filter/plant_model_poc_plan.md); the two are independent
and can land in either order.

## Why this is viable even though the plant fit fails acceptance

The jig fit fails C1/C2/C3, which are the EKF's criteria: 400 ms of open-loop position and
heading prediction (163 mm and 36 deg on holdout against 80 mm and 8 deg targets). The nav does
not need that. It needs steady-state gains, deadzones, time constants, and the transport delay.
Every one of those comes from Stage A, where a dedicated jig excitation measures it directly:

| Parameter | Value | Support |
|---|---|---|
| `k_fwd` | 4.880 m/s | +/- 0.313, n=12 |
| `k_rev` | 4.355 m/s | +/- 0.744, n=12 |
| `k_ang` | 31.71 rad/s | +/- 0.425, n=32, 1.3% relative |
| `tau_lin_a` | 0.149 s | +/- 0.074, n=13, weak |
| `tau_lin_d` | 0.123 s | +/- 0.063, n=63 |
| `tau_ang_a` | 0.174 s | +/- 0.036, n=15 |
| `tau_ang_d` | 0.088 s | +/- 0.026, n=43 |
| `delay_s` | 0.0522 s | onset stack, 3486 edges, SNR 22.4 |
| `c_sb` | 2.702 | +/- 0.106, n=96 |
| `c_ad` | 0.463 | +/- 0.143, n=12 |

Multi-step open-loop error is dominated by residual structure (lag-1 autocorrelation 0.91) that
a feedback controller re-closes every 33 ms tick. The controller only has to be right about
where the plant is heading, not about where it will be in 400 ms with no corrections.

## The four gaps

### 1. The plant constants are another robot's

| Nav config default | Value | Jig value for Mrs Buff Mk3 | Error |
|---|---|---|---|
| `max_linear_speed_fwd` | 5.6 | 4.880 | 15% high |
| `max_linear_speed_rev` | 4.84 | 4.355 | 11% high |
| `tau_accel` | 0.058 | 0.149 | 2.6x low |
| `tau_decel` | 0.078 | 0.123 | 1.6x low |
| `latency` | 0.060 | 0.0522 | 15% high |

Two consequences, both pushing the same way:

- **Brake horizon.** `compute_reference_speed` uses `tau_decel + latency`, currently 0.138 s
  against a measured 0.176 s. At a fixed distance-to-go the reference speed is 27% too high, so
  the robot brakes too late and overshoots.
- **Active braking is under-driven.** The Stage 3 fix was to keep the full-signed `tau * dv/dt`
  feedforward term so a falling reference commands reverse thrust. With `tau_decel` 1.6x low,
  that term is 1.6x too small. The overshoot Stage 3 measured as 0.006 to 0.046 m in sim was
  measured on a sim plant that shares the same wrong taus, so it does not bound the real number.

### 2. Steer-brake coupling is invisible to the controller

`c_sb` = 2.702 is the largest unmodeled effect. The plant delivers

```
v_target = k_fwd * u_lin_eff * max(0, 1 - 2.702 * |u_ang_eff|)
```

so forward authority is gone entirely at `|u_ang_eff|` = 0.370. On top of that,
`saturate_velocity` in `src/transmitter/drive_mixing.cpp` gives angular priority and clips
linear to `1 - |angular|` (`velocity_saturation_limit = 1.0` in `config/_common.toml`). The jig
fit logged post-clip body commands, so `c_sb` is a physical loss stacked on the mixer clip, not
a re-measurement of it.

Run the numbers on the Stage 3 sweep config (`angular_kp` 0.4, `max_angular_command` 0.25,
`angle_threshold` 1.7 rad = 97 deg):

- The angular cap engages at 35.8 deg of heading error.
- At the cap, `u_ang_eff` = 0.238 and the steer-brake multiplier is 0.358.
- The mixer clip independently drops linear to 0.75.
- Net forward authority between 35.8 deg and the 97 deg drive gate: **26.8% of what the nav
  commands**, with no cap the controller is aware of.

Uncap the angular command and it is worse: at `angular_kp` 0.4 the forward authority reaches
exactly zero at 54.5 deg of heading error. The nav would run its speed PID against a robot that
is not moving forward at all, wind the integrator, and dump the whole thing the instant the
heading closes.

### 3. Angular control has no physical units

The linear channel is computed in m/s and normalized at the end. The angular channel is a bare
PD on radians producing a normalized command, with no notion of rad/s. Now that `k_ang` = 31.71
rad/s is the best-determined parameter in the set, the angular loop can be written the same way
as the linear one, which makes three things available that are currently unreachable:

- **Angular feedforward.** `tau_ang_a` 0.174 s and `tau_ang_d` 0.088 s. The accel constant is
  2.0x the decel constant, so a turn spins up slowly and stops fast. A symmetric P controller
  overshoots the heading on the way in and stalls on the way out.
- **Angular droop.** `c_ad` = 0.463: at full linear command the yaw rate drops to 54% of
  commanded. The heading loop is effectively half-gain at speed and full-gain in place.
- **A meaningful cap.** `max_angular_command` 0.25 currently means "a quarter of whatever the
  radio does". It means 7.9 rad/s.

Caveat: Stage 2's camera measurement put max yaw at 61.5 rad/s, 1.9x the jig value, and that
disagreement is unresolved. The jig number is the one to use. `LOG-69` recorded at 2293.76 dps
full scale and peaked at 2054 dps without clipping; 61.5 rad/s is 3524 dps and would have railed
the channel for the whole top step. The raw counts refute 61.5 without reference to any fit.

### 4. The speed estimate degrades to a bias during perception dropouts

`estimate_forward_speed` differences our pose across one tick and projects onto heading. During
a perception dropout the filter's `coast()` still moves our-robot's pose, but it moves it by
dead reckoning the last `CommandFeedback` velocity. Two problems:

- **The feedback loop opens.** `v_actual` becomes a function of the nav's own command, so
  `u_fb = speed_kp * (v_ref - v_actual)` stops being feedback. Perception dropouts are not rare:
  the p90 gap is 340 ms, which is 1.66 m of travel at 4.88 m/s.
- **The dead-reckoned speed is scaled wrong.** `opentx_transmitter.cpp:99` builds the feedback
  velocity from `max_motor_rpm * pi * wheel_diameter / 60`, which is 1500 * pi * 0.05 / 60 =
  3.927 m/s. The jig measured 4.880 m/s. Forward speed is understated 19.5%. Yaw rate is
  `2 * 3.927 / 0.195` = 40.3 rad/s against a measured 31.71, overstated 27%.

The nav also ignores `RobotDescription::is_stale`, which the filter sets after 100 ms unmeasured,
and `RobotDescription::velocity`, which the filter already populates.

Worse case: when the filter drops our-robot's track entirely, `update()` falls back to
`last_known_our_pose_`, the pose delta is exactly zero, `v_actual` is 0, and the nav responds by
removing the latency lead (`d_eff` = full distance) and commanding `speed_kp * v_ref` of extra
thrust. Blind and accelerating is the wrong failure mode.

## Step 0: fix the sim first

`simulation/sim_mrs_buff_mk3.toml` says in its own comment that it is an uncalibrated
placeholder, and it is wrong by more than the effects being chased:

| Sim | Placeholder | Measured |
|---|---|---|
| `max_linear_speed` | 2.0 | 4.880 fwd / 4.355 rev |
| `max_angular_speed` | 6.0 | 31.71 |
| `tau_linear` | 0.15 | 0.149 accel / 0.123 decel |
| `tau_angular` | 0.08 | 0.174 accel / 0.088 decel |
| `[latency] command_ms` | 60.0 | 52.2 |
| `steer_brake_coeff` | unset (0) | 2.702 |
| angular droop | not implemented | 0.463 |

Yaw rate is 5.3x low and forward speed 2.4x low. Any gain tuned against this sim is tuned
against a different vehicle. Before touching the controller:

**Done.** `Plant.step` now mirrors `auto_battlebot/plant.py` term for term and
agrees with it to 1e-14 over a 3.66 s command sequence that exercises every term. Four changes
were needed, not the one the first draft of this plan predicted:

1. `angular_droop_coeff` on the yaw target, mirroring the existing `steer_brake_coeff` line.
2. Per-sign deadzone (`deadzone_linear_rev`, `deadzone_angular_right`). The forward and reverse
   linear deadzones differ by 2.3x, and the old `_apply_deadzone` was symmetric.
3. **2 ms substepping with exact arc integration.** This one is not optional. The old code took
   one straight-line Euler step per 33 ms tick. At the corrected 31.7 rad/s yaw gain that is up
   to 1.05 rad of rotation in a single step, and integrating an arc as a straight line over 60
   degrees is wrong by roughly the quantity being measured: on 1.5 s of driving while turning,
   one step per tick and 2 ms substeps disagree by 0.055 m of position and 8.8 degrees of
   heading, against stop-accuracy metrics in the 0.03 to 0.10 m range. The same error was
   present in the Mr Stabs Mk2 sim at 61.5 rad/s, where a tick is 2.05 rad and the disagreement
   is 18 degrees of heading. Every Stage 3 number was computed through it.
4. `simulation/sim_mrs_buff_mk3.toml` filled from `plant_stageA.toml`, plus `max_ticks = 900` so
   runs terminate the way the Mr Stabs config does instead of relying on the sweep timeout.

The sweep harness also had to be revived. It had been dead since the config refactor, with three
independent breakages, which is why the Stage 3 table was never re-run:

- `BASE_CPP_CONFIG` pointed at `config/simulation/headless_sim`, renamed to `kinematic_sim`.
- All four sweeps' `sim_config` pointed at `simulation/kinematic_sim_mr_stabs_mk2.toml`, renamed
  to `sim_mr_stabs_mk2.toml`.
- MCAP recording is off throughout the sim config chain, so every run scored empty even after
  the paths were fixed. `write_cpp_overlay` now sets `mcap.enable = true` itself rather than
  depending on the base config.

Note that the sweeps target Mr Stabs Mk2 by default. The baseline below was taken with
`--sim-config simulation/sim_mrs_buff_mk3.toml`.

### Step 0 baseline, Mrs Buff Mk3 measured plant

Taken with the nav still on Mr Stabs Mk2's constants (`v_max_fwd = 5.6`, brake horizon 0.138 s),
so this is the number Step 1 has to beat, not a result.

| stop | terminal err (m) | terminal vel (m/s) | overshoot (m) | t_to_goal (s) | wall |
|---|---:|---:|---:|---:|---:|
| short_baseline | 0.102 | 0.0 | 0.321 | 0.37 | 0 |
| short_tracker | 0.072 | 0.0 | **0.000** | 0.40 | 0 |
| center_baseline | 0.061 | 0.0 | 0.389 | 0.57 | 0 |
| center_tracker | 0.096 | 0.0 | **0.000** | 0.90 | 0 |
| far_diag_baseline | 0.101 | 0.0 | 0.333 | 1.10 | 0 |
| far_diag_tracker | **0.030** | 0.0 | **0.000** | 0.93 | 0 |

The tracker still holds its Stage 3 headline: zero overshoot on all three geometries against a
consistent 0.32 to 0.39 m for the baseline. Terminal error is now a split decision (tracker wins
short and far_diag, loses center), where on the old sim it won everywhere.

| ram | commanded v_term | impact speed (m/s) | min dist (m) |
|---|---:|---:|---:|
| vt_char_0p0 | 0.0 | 0.25 | 0.001 |
| vt_char_1p5 | 1.5 | 3.03 | 0.002 |
| vt_char_3p0 | 3.0 | 4.50 | 0.007 |
| vt_char_4p5 | 4.5 | 4.50 | 0.006 |

Contact speed still rises with the command but saturates at 4.50 m/s, and `v_term = 0` no longer
reaches an exact stop (0.25 m/s, against 0.00 on the old sim). Both are consistent with a brake
horizon 27% shorter than the plant needs, which is exactly what Step 1 corrects.

| track | mean err (m) | RMS err (m) |
|---|---:|---:|
| circle_slow_baseline | 0.224 | 0.263 |
| circle_slow_tracker | 0.319 | 0.354 |
| circle_fast_baseline | 0.659 | 0.689 |
| circle_fast_tracker | **0.227** | **0.234** |

The fast circle is where the tracker's margin grew: 0.234 RMS against 0.689, a 66% cut, where on
the old sim it was 13%. The slow circle reversed, with the tracker now 35% worse than baseline.
Both are turning-while-driving cases, so both are dominated by the steer-brake coupling the
controller does not model. Step 2 is the one to watch here.

## Steps 1 to 5: the controller

Land these one at a time and re-run the three sweeps after each, so each effect gets its own
number instead of one combined delta.

### Step 1: repoint the constants (config only, no code)

```toml
[navigation]
type = "MotionProfileNavigation"
max_linear_speed_fwd = 4.880
max_linear_speed_rev = 4.355
tau_accel = 0.149
tau_decel = 0.123
latency = 0.052
deadzone = 0.0     # unchanged, see below
```

Leave `deadzone` at 0. The jig drove the ESCs raw (`calib_lib/drive_protocol.py` sends commands
with no lifted deadzone, by design, so the physical deadzone could be measured rather than
pre-compensated), and it measured 1.10% forward and 2.53% reverse. The deployed transmitter
already lifts wheel commands past `lifted_deadzone_percent = 5`. Pasting the jig deadzone into
the nav double-compensates.

Open question worth one bench run: `k_fwd` was fitted on the raw path, so the deployed input map
near zero is not the one the gain was fitted against. The 5% lift is larger than either measured
deadzone, so the deployed map has a step at zero. This matters for creeping speeds near the goal
and for nothing else.

### Step 2: steer-brake compensation

`compute_command` runs `compute_angular_command` before `compute_linear_command` already, so the
angular command is in hand when the linear command is built. Pass it in and divide out the loss:

```cpp
// The plant multiplies forward authority by (1 - c_sb * |u_ang_eff|), so a command issued
// during a turn arrives smaller than it left. Divide it back out, floored: above
// |u_ang_eff| = 1/c_sb the plant delivers no forward motion at all and no amount of command
// changes that.
const double u_ang_eff = effective_command(u_ang, dz_ang_l_, dz_ang_r_);
const double authority = std::max(steer_brake_floor_, 1.0 - c_sb_ * std::abs(u_ang_eff));
u /= authority;
```

Then clamp as today. Two guards this needs:

- **`steer_brake_floor_`, config, default 0.3.** The fit's own note says the linear loss shape is
  wrong above `|u_ang_eff|` = 0.37, which is exactly where the term goes singular. Do not
  extrapolate the model into the region the fit says it does not describe. Below the floor,
  compensation saturates and the controller stops pretending it has forward authority.
- **Do not compensate the mixer clip.** `saturate_velocity` clips to `1 - |angular|` after the
  nav's clamp. Trying to pre-inflate past that just saturates twice. If the residual is a
  problem, the fix is a lower `max_angular_command`, not more command.

`c_sb` = 2.702 exceeds the 1.5 upper bound in `PARAM_BOUNDS`. That bound is a fit-side loss-shape
problem, not a reason to distrust the value (n=96, +/- 0.106). Store it in the nav config as a
plain double with no bound.

### Step 3: angular loop in rad/s

Rewrite `compute_angular_command` in the same shape as the linear channel:

```
w_ref  = clamp(angular_kp * angle_error + angular_kd * d(angle_error)/dt, +/- max_yaw_rate)
tau    = (dw_ref/dt >= 0) ? tau_ang_a : tau_ang_d
u_ang  = (w_ref + tau * dw_ref/dt) / (k_ang * max(droop_floor, 1 - c_ad * |u_lin_eff|))
```

This is a units change to `angular_kp` and `angular_kd`, so both need retuning. Their current
values are normalized-command per radian; the new ones are rad/s per radian. `angular_kp` = 0.4
at the old scale is roughly 12.7 rad/s per rad against `k_ang`, which is a reasonable starting
point for the new gain.

`max_angular_command` becomes `max_yaw_rate` in rad/s. The Stage 2 reasoning behind the 0.25 cap
was that an uncapped turn rotates too far per 30 Hz tick; at 31.71 rad/s the cap is 7.9 rad/s,
which is 15 deg per tick. Keep that number and re-derive the cap from it rather than carrying the
normalized value across the units change.

The droop compensation has the same singularity structure as Step 2 (`c_ad` = 0.463 means yaw
authority floors at 54%, never zero), so its floor is a formality. Add it anyway for symmetry.

Note the two compensations are mutually recursive: linear compensation depends on the angular
command and angular compensation depends on the linear command. Resolve it in one pass using the
previous tick's counterpart command rather than iterating. At 30 Hz with both commands
rate-limited, one tick of staleness is smaller than the parameter uncertainty.

### Step 4: take velocity from the filter

Replace `estimate_forward_speed` with a read of `RobotDescription::velocity`, projected onto
heading:

```cpp
const double v_measured = robot.velocity.vx * std::cos(our_pose.yaw)
                        + robot.velocity.vy * std::sin(our_pose.yaw);
```

and gate the speed feedback on `is_stale`:

- **Fresh:** use `v_measured`, run the speed PID as today.
- **Stale, or our-robot missing:** freeze the integrator and drop `u_fb` to zero. Keep the
  feedforward, which is open-loop by construction and stays valid. Keep the latency lead using
  the last good `v_actual` rather than collapsing `d_eff` to the full distance.

This is a strict improvement over the pose finite difference even before the plant EKF lands,
because the current code silently reports 0 m/s in exactly the case where it should report "no
measurement".

Ordering constraint: `our_robot_mode = "ekf"` is rejected at config validation until a plant
model is registered (`motion_estimator_config.hpp:81`), so our-robot velocity comes from dead
reckoning for now. The gating above is what makes that safe. Once
[`plant_model_poc_plan.md`](../kalman_filter/plant_model_poc_plan.md) lands and the EKF arm is
live for our robot, this step gets a real measured velocity with no further nav change.

### Step 5: reverse gain, and a dead branch

`compute_linear_command` picks `v_max` by `sign(v_ref)`, but `compute_reference_speed` clamps
`v_ref` to `[0, max_linear_speed_fwd_]`. `v_ref` is never negative, so the reverse branch never
runs and every braking command is normalized by the forward gain.

- **Bug:** during active braking the command goes negative through the `tau * dv/dt` term while
  still being divided by `k_fwd` = 4.880. The plant delivers reverse at `k_rev` = 4.355, so the
  brake is 11% weaker than the controller believes.
- **Fix:** choose `v_max` from the sign of the assembled command, not from `v_ref`. Compute
  `u_ff` with the forward gain, then rescale by `k_fwd / k_rev` when the result is negative.

While in there, delete the `(v_ref >= 0.0)` branch or make `v_ref` genuinely signed. Leaving a
branch that cannot be taken is what hid this.

## One source of truth

The plant numbers now live in three places: `plant_stageA.toml` (fitted),
`simulation/sim_mrs_buff_mk3.toml` (sim), and the `[navigation]` section (controller). They have
already drifted apart once, which is how the controller ended up running Mr Stabs Mk2's taus on
Mrs Buff Mk3.

Do not add a fourth. The cheapest fix that holds: a small script under
`playground/calibration/` that reads `plant_stageA.toml` and writes the derived blocks into the
sim config and a `config/plant/mrs_buff_mk3.toml` fragment the nav config extends, run as part of
the fit rather than by hand. Names differ between consumers (`k_fwd` vs `max_linear_speed_fwd`,
`tau_lin_a` vs `tau_accel`), so the mapping has to live somewhere; a generator is the place.

## Acceptance

Re-run all three Stage 3 sweeps after Step 0 to set the baseline, then after each of Steps 1 to 5.

```bash
source scripts/activate_python.sh
cmake -S . -B build && ./scripts/build.sh
for s in stage3_stop stage3_ram stage3_track; do
  python playground/control_stage0/sim_sweep.py \
    --sweep playground/control_stage0/sweeps/$s.toml \
    --out playground/control_stage0/sweep_out/$s
done
./scripts/lint
```

Report the metric and the delta per step, not a combined number. Targets, against the Step 0
baseline rather than against the void Stage 3 table:

- **Stop:** terminal position error and overshoot both improve at Step 1. If they do not, the
  brake-horizon reasoning is wrong and the rest of the plan is built on it.
- **Turning approach:** Step 2 is the only step that changes anything here, and the existing
  sweeps barely exercise it because their goals are close to straight ahead. Add a fourth sweep,
  `stage4_turn_approach.toml`, with start headings 45, 90 and 135 deg off the goal bearing.
  Without it Step 2 has no test.
- **Heading:** Step 3 should cut heading overshoot on the turn-in-place phase. Watch for the
  `angular_kp` units change swamping the effect; retune before comparing.
- **Dropout:** Step 4 needs a scenario the sweeps do not have. Set
  `[perception] dropout_prob` in the sim config to produce gaps near the measured p90 of 340 ms
  and check that the commanded speed falls rather than rises during a gap.

Two of the five steps have no test today. Write those two sweeps before writing the code for
Steps 2 and 4.

## Known limits

- **`tau_lin_a` is the weakest number used here.** +/- 0.074 on 0.149, with 10 of 24 segments
  yielding nothing. It only feeds the ramp-up feedforward, which the speed PID corrects and which
  no acceptance metric scores directly. If Step 1 makes launch behavior worse, this is the first
  suspect and the fix is more E8 data, not a hand tune.
- **`k_ang` disagrees with Stage 2 by 1.9x** and that is unresolved. The jig value is used here
  on the strength of the raw dps counts.
- **`c_sb`'s loss shape is wrong above `|u_ang_eff|` = 0.37.** The floor in Step 2 is what keeps
  the controller out of that region. Fixing the shape is fit-side work.
- **Session count.** Four sessions (2026-08-19 through 2026-08-23) is enough to see
  session-to-session spread but not enough to trust it; leave-one-session-out was uninformative
  on the earlier two-session fit.
- **These are Mrs Buff Mk3's numbers with guard plates on.** The plates' ground friction is part
  of the fitted plant. Characterizing or running without them invalidates the fit.

## Next steps

1. Add `angular_droop_coeff` to the sim plant and fill `simulation/sim_mrs_buff_mk3.toml` from
   `plant_stageA.toml`. Re-run the three sweeps for a new baseline.
2. Land Step 1 as a config change and re-run. This is the highest ratio of expected improvement
   to risk in the plan.
3. Write `stage4_turn_approach.toml` and the dropout scenario, then implement Steps 2 and 4.
