# RUN_AWAY overshoot: diagnosis

Step 1 of `docs/hazard_avoidance_plan.md`. The question was whether the robot overshoots the
safe point because the goal moved or because the brake was late. Measured on the 2026-08-23
Jetson fights with `playground/control_stage0/run_away_overshoot.py`, then reproduced (and
not reproduced) in the kinematic sim.

## Answer

The brake was late, and the shape is a limit cycle rather than a single ballistic overshoot.
The goal held still through the worst excursions.

The driver of the cycle is the our-robot speed estimate, not the control law. Given a correct
speed, the same controller on the same geometry stops on the goal to 2 mm with zero overshoot.

## Method

`run_away_overshoot.py` joins four diagnostic channels on one time axis: `motion_profile_nav`
(`profile`, `poses`, `command`, `stop`), `safest_point_target` (`run_away_target`, `solver`),
`runner/navigation` (resolved behavior mode), and `opentx_transmitter/switch_states`. It splits
the recording into contiguous RUN_AWAY segments, then splits each segment into *goal epochs*:
stretches over which the held goal did not move by more than 0.1 mm. Overshoot is measured
inside an epoch, so a moving goal cannot masquerade as one.

```bash
source scripts/activate_python.sh
python playground/control_stage0/run_away_overshoot.py \
    data/recordings/auto_battlebot_mrs_buff_mk3_jetson_2026-08-23_22-07-22.mcap
```

Output: `run_away_segments.csv` plus a four-panel plot per segment (distance to held goal with
goal changes marked, `v_ref` against `v_actual`, feedback availability, transmitted command).

## The five leads, in the order the plan listed them

**1. `retarget_improvement_m` ships at zero.** Real, and now fixed. `run_away_mode_plan.md:664`
specified 0.15; the default in `include/target_selector/config.hpp` is 0.0 and no config in the
tree set it. On the 21-37-58 recording the goal moved 10.7-13.1 times per second, median epoch
one tick, median jump 4-5 mm. That is the solver plateau, not opponent motion. `config/_common.toml`
now sets `retarget_improvement_m = 0.15`.

It is not the overshoot, though. The two segments with the largest excursions had **zero** goal
changes: 22-07-22 held one goal for 10.9 s and still receded 0.467 m after closing to 0.021 m,
and all three 20-36-51 segments had zero goal changes.

**2. Filtered speed reads low.** Confirmed, and it is the dominant term. Comparing the EKF's
`v_actual` against a 150 ms chord speed from the logged poses, over ticks where the robot was
actually moving (chord > 0.5 m/s):

| recording | ratio `|v_actual| / chord` |
|---|---|
| 20-36-51 (before `q_along = 10.0`) | 0.30 - 0.34 |
| 22-07-22 (after) | 0.72 |

The `q_along` change (commit 6822f1b, 2026-08-23 21:37) more than doubled the ratio, so the
20-36-51 observations predate it and the 22-07-22 ones postdate it. A low speed reading hurts
twice in `compute_reference_speed` and `compute_linear_command`: `d_eff = distance - |v_actual| *
latency` gives back too little latency lead, and `u_fb = speed_kp * (v_ref - v_actual)` adds
throttle. Both push past the goal.

**3. Open-loop stretches.** Ruled out. `speed_is_measured == false` on 0.0 - 5.7% of RUN_AWAY
ticks, in at most one run per segment.

**4. Heading-gate re-entry.** Confirmed as the recovery half of the cycle. The gate was off for
28.6% and 58.6% of ticks in two of the three 20-36-51 segments.

**5. Solver plateau jitter.** Same phenomenon as lead 1. The winning constraint family was stable
(`three_wall` on 20-36-51, `perpendicular_walls_point` in sim); the center wandered within it.

## The mechanism

From the 22-07-22 trace, one lap of the cycle:

1. The robot closes on the goal at 1.6 - 1.7 m/s chord speed.
2. Inside `stop_distance` (0.15 m) `compute_command` returns `{0, 0, 0}`. The command is cut,
   not reversed, so the drivetrain coasts `tau_lin_d * v` = 0.12 * 1.65 = 0.20 m, plus 0.05 s of
   actuation latency, past the goal.
3. The goal is now behind the robot: heading error jumps to 130-160 degrees, above the 1.7 rad
   (97 degree) gate, so `linear_x` is forced to 0 and `prev_v_ref_` and `speed_integral_` reset
   while the robot turns around. Distance keeps growing on the turn arc, out to 0.49 m.
4. The gate reopens, `accel_limit` ramps `v_ref` at 8.5 m/s^2 (visible as +0.283 m/s per tick at
   dt = 0.0333) toward what the schedule asks at that range: 0.48 / (0.123 + 0.052) = 2.7 m/s.
5. The robot arrives fast again. Go to 1.

Each lap re-injects the energy the previous lap failed to shed. What reads as "overshoot" is the
amplitude of this cycle, which is why the number varies (0.21, 0.33, 0.47, 0.86 m) rather than
converging on a single stopping distance.

## What the simulator can and cannot show

The clean kinematic sim does not reproduce it. Same controller, same plant fit, RUN_AWAY pinned
via the new `[transmitter].behavior_mode`, opponent static at (0.6, 0.6), robot starting at
(-0.9, -0.9):

| run | terminal pos err (m) | terminal vel (m/s) | overshoot (m) |
|---|---|---|---|
| clean | 0.002 | 0.000 | 0.000 |
| +60 ms observation latency | 0.078 | 0.000 | 0.030 |
| +15 mm position noise | 0.049 | (noise) | 0.009 |
| +30% our-robot dropout | 0.002 | 0.000 | - |
| all three | 0.080 | (noise) | - |

`terminal_vel_mps` under position noise is an artifact: the sweep derives speed from the
*observed* pose deltas, so 15 mm of noise at 30 Hz reads as ~0.92 m/s of nonexistent motion.

The reason the sim stays clean is structural. `config/simulation/kinematic_sim.toml` uses
`GroundTruthRobotFilter`, which hands navigation an exact velocity. The real stack runs
`RobotFrontBackFilter` with the Kalman/EKF arm, and that estimator is the thing under suspicion.
The sim bypasses it, so it cannot exhibit the failure. Worst degraded-sim overshoot was 0.05 m
against 0.21 - 0.86 m on the robot: different regime, not a weaker version of the same one.

## Changes made, and changes tried and rejected

Kept:

- `config/_common.toml`: `retarget_improvement_m = 0.15`. Removes the 10-13 Hz goal churn.
- `SimTransmitter` gained a `behavior_mode` config field, so a sweep can pin RUN_AWAY. The real
  transmitter reads it off a switch; the sim had no way to select the mission at all.

Tried and reverted, because the only evidence available says they cost more than they save:

- **Latched stop** (hold the cut command until the robot is `stop_release_distance` from the goal).
  Overshoot 0.009 -> 0.000 under position noise, but terminal position error 0.049 -> 0.127 m,
  because a noisy `distance` dips under `stop_distance` early and latches the robot short.
- **Brake into the hold** (run the reference to zero through `compute_linear_command` instead of
  cutting). Clean-run terminal position error 0.002 -> 0.136 m: with an accurate plant the coast
  lands on the goal and braking wastes the last 8 cm. It can only pay off when arrival speed is
  too high, which is exactly the case the sim cannot produce.
- **Lower `accel_limit`.** A clean trade, not a win: 8.5 -> 2.0 m/s^2 takes overshoot 0.050 ->
  0.000 and terminal position error 0.037 -> 0.105 m in the degraded sim. And the degraded sim's
  0.05 m overshoot is not the 0.5 m one being chased, so tuning here does not transfer.

## Next measurement

The fix belongs in the estimator, and the missing measurement is the one
`config/_common.toml` already asks for in the `q_along` comment: run `fit_process_noise.py` on a
session that contains autonomous RUN_AWAY motion, rather than a hand-set value swept against a
chord reference. The check afterwards is the ratio in lead 2 on a fresh recording: if
`|v_actual| / chord` reaches ~1.0, re-run `run_away_overshoot.py` and confirm the goal epochs
stop showing approach-then-recede.

Until then, hazard-avoidance tuning happens in the sim, where the controller does arrive where it
aimed (0.002 m, zero overshoot), so the geometry layer is being tuned honestly.
