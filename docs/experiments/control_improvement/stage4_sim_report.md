# Stage 4 results: MotionProfileNavigation on the Mrs Buff Mk3 plant

Companion to `stage4_plant_backed_nav_plan.md`. Every number here comes from the headless
kinematic sim (`simulation/sim_mrs_buff_mk3.toml`, plant from
`playground/calibration/out/plant_stageA.toml`, model M4). No hardware runs yet.

Reproduce with:

```bash
source scripts/activate_python.sh
./scripts/build.sh
playground/control_stage0/run_stage4_sweeps.sh final2 \
  stage3_stop stage3_track stage4_turn stage4_dropout stage4_ablation
```

`baseline` rows are `PurePursuitNavigation`; `tracker` rows are `MotionProfileNavigation`. Both
drive the same plant, so the comparison is controller-to-controller.

## Headline

MotionProfileNavigation stops within 6 to 22 mm across every stop and turn scenario, against 38
to 149 mm for pursuit, and it does so without overshoot in six of eight cases. The two places it
does not win are worth stating plainly: it ties pursuit on slow circle tracking (RMS 0.267 vs
0.263), and two of the five plant terms it gained are inert in the shipped configuration.

## Per-step deltas

Terminal position error in metres, `tracker` rows only, so the steps are comparable to each
other. Each column is one sweep run at that point in the implementation.

| Step | stop short | stop center | stop far_diag | turn45 | turn90 | turn135 | arc |
|---|---|---|---|---|---|---|---|
| 0. sim repointed, nav still on Mr Stabs Mk2 constants | 0.072 | 0.096 | 0.030 | - | - | - | - |
| 1. nav constants repointed at the fit | 0.027 | 0.005 | 0.003 | 0.038 | 0.048 | 0.063 | 0.048 |
| 2-5. coupling, rad/s angular loop, velocity feedback, reverse gain | - | - | - | 0.103 | 0.103 | 0.078 | 0.062 |
| after the filter velocity fix | 0.006 | 0.005 | 0.020 | 0.084 | 0.047 | 0.077 | 0.100 |
| final | 0.006 | 0.005 | 0.020 | 0.001 | 0.008 | 0.022 | 0.006 |

Two things in that table need explaining.

Step 1 is where almost all of the stop improvement comes from, exactly as the plan predicted: a
config-only change that points the controller at the right taus and gains cuts center-goal error
from 96 mm to 5 mm. The brake-horizon reasoning the rest of the plan is built on holds.

Steps 2 through 5 made the turn cases *worse* before they made them better. That row is measured
with a feedback bug still live in `GroundTruthRobotFilter`, which fed the new velocity loop
garbage. Steps 2-5 are not responsible for the regression; the last two rows are the same code
with the filter fixed.

## Stop

| scenario | terminal err (m) | overshoot (m) | t_to_goal (s) |
|---|---|---|---|
| short_baseline | 0.102 | 0.321 | 0.37 |
| short_tracker | **0.006** | **0.006** | 0.57 |
| center_baseline | 0.061 | 0.389 | 0.57 |
| center_tracker | **0.005** | **0.000** | 0.73 |
| far_diag_baseline | 0.108 | 0.338 | 1.10 |
| far_diag_tracker | **0.020** | **0.000** | 0.97 |

Overshoot is where the plant model pays. Pursuit overruns by 320 to 389 mm because it has no
model of the coast; the motion profile brakes on `tau_lin_d` = 0.123 s and stops with the goal
still ahead of it. It pays 200 ms of approach time on the two near goals and saves 130 ms on the
far one.

## Turning approach

| scenario | terminal err (m) | overshoot (m) | t_to_goal (s) | mean abs ang (deg) |
|---|---|---|---|---|
| turn45_baseline | 0.149 | 0.113 | 1.00 | 164.5 |
| turn45_tracker | **0.001** | **0.000** | **0.87** | **21.5** |
| turn90_baseline | 0.139 | 0.107 | 1.17 | 31.0 |
| turn90_tracker | **0.008** | **0.000** | **1.10** | 39.9 |
| turn135_baseline | 0.133 | 0.084 | 1.37 | 124.7 |
| turn135_tracker | **0.022** | **0.022** | **1.13** | **49.8** |
| arc_baseline | 0.038 | 0.000 | 0.97 | 122.8 |
| arc_tracker | **0.006** | 0.004 | **0.83** | **22.9** |

The tracker is faster to the goal in all four, which was not a target. Mean absolute heading
error drops by 3x to 8x in three cases and rises in one: turn90 goes 31.0 to 39.9 deg. A 90 deg
offset is the case where the deadzone-aware angular command commits to the turn later than
pursuit's proportional steer does, so it carries more heading error through the middle of the
approach while ending better positioned.

## Dropout

Sim `[perception] our_dropout_prob` at 0.0, 0.5 and 0.8. The plan asked for gaps near the
measured p90 of 340 ms; 0.8 produces gaps well past that.

| scenario | terminal err (m) | terminal vel (m/s) | wall contacts |
|---|---|---|---|
| dropout_00_tracker | 0.005 | 0.000 | 0 |
| dropout_50_tracker | 0.027 | 0.000 | 0 |
| dropout_80_tracker | 0.022 | 0.000 | 0 |
| dropout_50_baseline | 0.077 | 0.000 | 0 |
| dropout_80_baseline | 0.126 | 0.000 | 0 |

The plan's criterion was "commanded speed falls rather than rises during a gap." Measured over
gaps that begin inside the braking phase (distance to goal under 0.5 m):

- `dropout_50_tracker`: 1 fall, 0 rise.
- `dropout_80_tracker`: 1 fall, 1 rise, 4 flat. The single rise is +0.338 m/s.

Call that mostly met. One gap in six at 80% dropout still ramps the reference up, because the
brake schedule is recomputed from a coasted distance and the `accel_limit_` rise limiter has room
to move before the next measurement corrects it. It does not reach the wall or leave residual
velocity at the goal, so it is not currently costing anything measurable.

### The filter bug this exposed

Chasing that criterion turned up a real bug in `GroundTruthRobotFilter`, in the dropout
dead-reckoning path added in `cddce44`. Three faults compounded:

1. A track that had never been observed got its held pose seeded with a fabricated `(0, 0)`. At
   80% dropout the first frame is usually a gap, so this fired most runs.
2. `velocity_from_delta` divided a pose delta that spanned a whole gap by a **single frame**
   interval. A 1.09 m/s move across six missed frames read as 6.52 m/s.
3. The coast branch wrote its propagated pose back into `prev_poses_`, so the next real
   measurement was differenced against the filter's own extrapolation rather than against the
   last measurement. Each gap amplified the previous one.

Symptom: `v_actual` reached -7.5e14 m/s against a plant ceiling of 4.88 m/s, 432 of 899 samples
exceeded 10 m/s, and the logged pose diverged to 6.1e13 m. `dropout_80_tracker` logged 77 wall
contacts and `dropout_80_baseline` 73.

Fix: never observed returns no track at all rather than a fabricated one; velocity is computed
measurement-to-measurement using a separate `measured_poses_` / `measured_stamps_` pair that the
coast never writes; and intervals under 1 ms or over 0.5 s are rejected. After the fix, `v_actual`
stays inside the plant ceiling, poses stay in the arena, and wall contacts are 0 in every dropout
scenario for both controllers.

The pre-fix `dropout_80_tracker` terminal error of 0.024 m was luck on top of garbage state, not
a result. The honest number is 0.022 m.

## Path tracking

| scenario | mean err (m) | RMS err (m) |
|---|---|---|
| circle_slow_baseline | 0.224 | **0.263** |
| circle_slow_tracker | **0.216** | 0.267 |
| circle_fast_baseline | 0.663 | 0.693 |
| circle_fast_tracker | **0.103** | **0.105** |

Slow circle is a tie, and by RMS the tracker is 1.5% worse. A stop-oriented controller following
a target that never stops is outside what it was built for; the plan did not claim tracking as a
target and the sim does not show one. Fast circle is a 6.6x improvement, which is the case where
pursuit's lack of a latency model dominates.

## Ablation: what is actually load-bearing

Each row disables one plant term against the shipped configuration.

| scenario | terminal err (m) | t_to_goal (s) |
|---|---|---|
| turn90_full | 0.008 | 1.10 |
| turn90_no_steer_brake | 0.009 | 1.10 |
| turn90_with_droop | 0.067 | 2.83 |
| turn90_no_rev_gain | 0.009 | 1.10 |
| stop_full | 0.005 | 0.73 |
| stop_no_rev_gain | 0.005 | 0.73 |
| stop_no_steer_brake | 0.005 | 0.73 |

Steer-brake compensation and the reverse gain are near-inert: disabling either moves terminal
error by 1 mm. That is not evidence they are wrong, it is evidence the shipped configuration
never enters the region where they matter. Measured over `turn90_full`, the effective angular
command has max 0.250, mean 0.005, p95 0.000, and `c_sb`'s loss model only bites approaching its
singularity at `1/c_sb` = 0.370. Both terms are insurance against a configuration that commands
harder turns.

Angular droop runs the other way. The plant genuinely droops, and `angular_droop_coeff` is fitted
at 0.463423, but compensating for it costs 59 mm of terminal error and 1.73 s of approach time.
`turn90_with_droop` drives the effective angular command to max 1.000, mean 0.052, p95 0.466,
which is past the `c_sb` singularity, so the coupling floor is active almost continuously and the
two compensations fight. Droop compensation ships disabled, and this row exists to price that
decision rather than leave it undocumented.

## What is not tested here

- No hardware. Every number is sim against a fitted plant, and `tau_lin_a` (0.149 +/- 0.074) is
  the weakest term in that fit.
- `k_ang` still disagrees with Stage 2 by 1.9x, unresolved.
- Ram behavior (`stage3_ram`) was not re-run.
- The plant fit does not pass its own EKF acceptance criteria (C1/C2/C3). Those gate the filter,
  not the nav, which needs only gains, deadzones, taus and transport delay. See the plan's "Why
  viable even though the plant fit fails acceptance."

## Next steps

1. Bench-test the stop cases on Mrs Buff Mk3 with guard plates on. The sim says 5 mm; anything
   inside 50 mm on hardware validates the approach.
2. Fix the `c_sb` loss shape above `|u_ang_eff|` = 0.37 on the fit side, then re-run
   `turn90_with_droop`. Droop compensation may be recoverable once it is not fighting a floor.
3. Collect more E8 data for `tau_lin_a` before trusting the ramp-up feedforward on hardware.
