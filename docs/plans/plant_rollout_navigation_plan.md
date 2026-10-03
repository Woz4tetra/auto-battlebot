# PlantRolloutNavigation implementation plan

Goal: a navigation type that drives Mr Stabs Mk2 into the opponent from behind, or from the side
when the rear is out of reach, while staying out of walls, hazards and the opponent's weapon arc.
It plans by rolling candidate stick sequences through the fitted drivetrain plant and scoring the
predicted paths against where the opponent will be.

Background:

- `docs/experiments/control_improvement/navigation_planner_survey.md` ranks this design first
  (Rank 1) and holds the evidence for it over MPC, A*, DWA and learned policies.
- `docs/plans/mujoco_warp_mr_stabs_plan.md` and `learned_sim_environment_plan.md` build the sim
  this plan tunes in.
- `docs/experiments/control_improvement/hazard_avoidance_report.md` and
  `docs/plant_backed_control.md` describe the layers this one reuses.

## Requirements

1. Never drive into a wall, the floor hole or the house bot.
2. Stay out of the opponent's weapon arc at its predicted pose and heading over the planning
   horizon, not just its current one.
3. Strike from the rear when the rear is reachable.
4. Strike from the side when it is not. A side strike scores worse than a rear strike but better
   than not attacking, so a cornered opponent facing out still gets hit.
5. Never make contact inside the opponent's weapon arc.
6. Fit the 250 Hz control loop without blocking it.

## What exists, and corrections to the survey

| Item | State | Where |
| --- | --- | --- |
| Plant model | `JigPlantParams`, `plant_steady_state()`, `JigPlantModel` (EKF process model, 2 ms substeps, finite-difference Jacobian) | `include/plant/` |
| Python plant mirror | `auto_battlebot/control/plant.py`, with a golden-fixture parity test generator | `playground/calibration/make_plant_golden_fixture.py` |
| Delay-predicted state | Already there. `KalmanMotionEstimator::coast()` renders every track at now + `render_lead_s()`, which is the plant's `delay_s`, so navigation receives each robot where it will be when the next command reaches the wheels | `include/robot_filter/kalman_motion_estimator.hpp` |
| Our-robot track | 5-state EKF `[x, y, theta, v, w]` through `JigPlantModel` when `our_robot_mode = "EKF"` (set in `config/_common.toml`). A heading innovation past pi/2 falls back to position-only rows | same |
| Opponent track | 4-state constant-velocity filter `[px, py, vx, vy]`. Heading is the last measurement's rotation, or the velocity direction while coasting. No yaw rate | same |
| Opponent propagation | `opponent_mode` defaults to `HOLD` and no config sets it, so opponents are pinned at their last measured pose today | `include/robot_filter/motion_estimator_config.hpp` |
| Coast cap | `max_coast_s = 0.2` in `config/_common.toml` (the survey says 0.5; that is out of date) | `config/_common.toml` |
| Hazard layers | `HazardAvoidance`: `steer_around` (tangent waypoints) and `limit_command` (velocity barrier on `hard_radius`) | `include/navigation/hazard_avoidance.hpp` |
| Walls | Reactive `apply_wall_reverse` inside each navigation; no wall term in any planner | `pursuit_navigation.hpp`, `motion_profile_navigation.hpp` |
| Target selection | `TargetSelection{pose, label, mode}`; `NearestTarget` in ATTACK, `SafestPointTarget` in RUN_AWAY | `include/target_selector/` |
| Navigation seam | `update(RobotDescriptionsStamped, FieldDescription, const TargetSelection&) -> VelocityCommand`, registered with `REGISTER_CONFIG` and built in `make_navigation` | `include/navigation/navigation_interface.hpp`, `src/navigation/config.cpp` |
| Sim | Kinematic and MuJoCo backends behind `simulation/plants/`; opponents `static`, `straight`, `circle`, `random_walk`, `replay`; per-episode `EPISODE` line with `fell_in`, `wall_hits`, `block_hits`, `min_clearance` | `simulation/` |
| Opponent heading measurement | The learned NHRL keypoint model (synthetic-data training) supplies front and back keypoints for arbitrary robots | keypoint model path |

## Design

### Data flow per replan

```
robots (rendered at now + delay) ──┬─> opponent predictor ──> per-step opponent disc + weapon arc
                                   │
                                   ├─> mode layer (STAGE / COMMIT / EXIT / CONTAIN) ──> goal
                                   │
                                   └─> candidate generator ──> rollout stepper ──> scorer ──> winner
winner stick sequence ──> played back at 250 Hz ──> HazardAvoidance::limit_command ──> transmit
```

The opponent predictor and the goal are computed once per replan. Every candidate reuses them.

### 1. Opponent track with heading

Specified in `docs/plans/opponent_motion_estimation_plan.md`. What this plan relies on:

- `OpponentTrack` becomes `[px, py, vx, vy, theta, omega]`, with heading from the learned
  front and back keypoints and the same pi/2 flip gate the our-robot track uses.
- `RobotDescription` gains `PoseUncertainty` (position, heading and yaw-rate sigmas, infinity
  when unknown), so navigation can widen the weapon arc when heading is uncertain.
- `predict_opponent()` in `include/robot_filter/opponent_prediction.hpp` advances an opponent
  estimate by `dt`, along an arc or a straight line, stopping at walls. Section 2 builds on it.
- `opponent_mode = "KALMAN"` in `config/_common.toml`. Without it every opponent is held at its
  last pose and there is nothing to extrapolate.

### 2. Opponent predictor

For each rollout step k at lookahead t_k = k * dt, compute once per replan:

- **Center.** `predict_opponent()` at t_k, holding position past
  `opponent_prediction_cap_s` (start 0.5 s, the TIGERs cap).
- **Footprint radius.** Half the opponent's footprint diagonal, from
  `[robot_filter.robot_size_meters_per_label]` or the detected size, plus
  `n_sigma * position_sigma_m` from the prediction.
- **Heading interval.** `theta + omega * t_k`, widened on each side by
  `n_sigma * heading_sigma_rad + yaw_reserve * t_k`. `yaw_reserve` is how much faster than its
  current yaw rate the opponent could turn; its upper bound is the opponent's maximum yaw rate.
  Once the interval reaches 2 pi the weapon covers every direction.
- **Weapon arc.** Per opponent label, from a config table `[navigation.opponent_weapons]` keyed
  by label name: a `WeaponShape` enum (`FRONT_ARC` or `FULL_CIRCLE`, in
  `include/enums/weapon_shape.hpp` per the enum convention) and a half-angle for `FRONT_ARC`.
  Labels not listed default to `FRONT_ARC` at 60 degrees. Full-body spinners and robots the
  keypoint model cannot orient use `FULL_CIRCLE`.

The danger zone at step k is every point within the footprint radius plus
`weapon_reach_m` of the center whose bearing from the opponent falls inside the heading interval
widened by the weapon half-angle.

### 3. Rollout stepper

A Jacobian-free stepper on `JigPlantParams`, in `include/plant/plant_rollout.hpp`:
`plant_steady_state()` for the target speeds (deadzone, steer-brake, droop and drift included),
then the sign-selected first-order lag, then one exact arc step. Default step 20 ms, 25 steps,
0.5 s horizon. It does not reuse `JigPlantModel::propagate`, whose 2 ms substeps and eleven-call
finite difference are built for the EKF.

- **Initial state.** Our rendered pose plus `v` and `w` from the rendered velocity. The render is
  already at now + delay, and the commands in flight during the delay are already in it, so
  candidate commands apply from step 0 without a separate predictor.
- **Dead-reckoning arm.** The same code runs on a dead-reckoned render; the prediction is then
  open loop.
- **Parity.** Add the stepper to `auto_battlebot/control/plant.py` and extend the golden fixture
  so C++ and Python agree to float tolerance. The Python copy is what the tuning search and the
  sim-side analysis use.

### 4. Candidate generator

A candidate is two or three segments, each a stick pair `(linear, angular)` held for 0.1 to
0.25 s, totalling the horizon. Default budget 128, split across five sources:

| Source | Default count | How |
| --- | --- | --- |
| Previous best | 4 | Last winner shifted by the elapsed time, plus three small perturbations |
| Goal-aimed | 24 | Turn toward a goal, then drive at it. Goals: the mode layer's goal plus random points around it. Turn-segment length comes from heading error over `k_ang` |
| Fixed menu | 60 | Straight at three speeds each way, arcs left and right at three curvatures, spin each way, reverse then turn, turn then drive, full brake |
| Escape | 8 | Brake, back straight away from the nearest threat, spin to face it |
| Random | remainder | Uniform stick pairs and segment lengths, seeded per replan |

All counts are config fields. The survey's compute and candidate-count experiments set them.

### 5. Scoring

Cost is the sum of per-step costs over the rollout plus an end cost. Lowest total wins.

**Hard rejects** (any step):

- our footprint disc inside a wall margin: `wall_margin_m` while staging, `commit_wall_margin_m`
  during a commit, never below what `limit_command` needs
- inside any hazard's `inflated_radius`
- contact with the opponent from inside its weapon arc
- predicted pitch past `max_pitch_rad`, once a plant with pitch is available (MuJoCo-backed
  rollouts only; the grey-box plant has no pitch)

**Per-step soft costs:**

| Term | Cost |
| --- | --- |
| Wall proximity | `w_wall * f(distance)`, margin scaled by speed (`(min(v, v_max)/v_max)^2 * margin_gain`, the TIGERs rule) |
| Hazard band | `w_hazard` between `hard_radius` and `inflated_radius`, as the barrier already splits them |
| Weapon arc | `w_arc * decay(t_k)` while inside the step-k danger zone without touching |
| Effort | `w_effort * |u_k - u_{k-1}|` |

**Contact.** Contact at step k means our footprint disc overlaps the opponent's step-k footprint
disc while our heading is within `our_weapon_half_angle` of the bearing to the opponent. Let
`phi` be the bearing from the opponent to us, relative to the opponent's predicted heading:
`|phi| = 0` is dead ahead, `|phi| = pi` is directly behind. Outside the weapon arc, contact adds a
negative cost:

```
u = (|phi| - arc_half_angle) / (pi - arc_half_angle)        in [0, 1]
reward = -(w_side + (w_rear - w_side) * smoothstep(u))       with w_rear > w_side > 0
```

Both ends must be rewards. Hovering outside the arc has zero contact cost, so a side strike with
any positive cost would never beat it and the planner would stall against a cornered opponent.
Only the first contact in a rollout is rewarded.

**End cost:** distance from the goal, bearing error to the goal, and, in ATTACK, alignment with
the opponent's rear axis (the directional-pursuit terminal shape from the survey).

**Selection:** the new winner replaces the committed plan only if it beats the committed plan's
re-scored cost by `hysteresis_margin`. If every candidate is rejected, play the escape candidate
with the fewest violations and let `limit_command` handle the rest.

### 6. Mode layer

A small state machine inside the navigation. It depends on the planner's own scores, so it does
not belong in a target selector.

| Mode | Goal | Contact reward | Enters when |
| --- | --- | --- | --- |
| STAGE | A point `stage_distance_m` behind the opponent's predicted heading, clamped inside the wall margin | Off | Default in ATTACK |
| COMMIT | The opponent's rear (or side) contact point | On, wall margin shrinks to `commit_wall_margin_m` | The best candidate makes contact outside the arc within the horizon |
| EXIT | A point `exit_distance_m` from the opponent, outside its arc | Off | After contact, or when the committed plan's contact disappears from the horizon |
| CONTAIN | The point outside the arc that covers the opponent's open side | On | The stage point is unreachable (inside the wall margin or a hazard) and no side contact is feasible |

CONTAIN carries a timeout, `contain_timeout_s`, after which side contact gets an extra reward so
the robot does not wait out a match. The value depends on how NHRL judging treats passivity and
is a tuning decision.

In RUN_AWAY the goal is `SafestPointTarget`'s point, contact rewards are off and every other cost
applies.

### 7. Execution between replans

The winner is a stick sequence, so the 250 Hz loop plays it back by elapsed time since the
replan, then passes the command through `HazardAvoidance::limit_command` against the measured
speed, exactly as `MotionProfileNavigation` does today. Closed-loop correction comes from
replanning at perception rate. If the sim shows drift between replans, add the speed PI from
`MotionProfileNavigation` on top of the played command as a second step.

### 8. Scheduling and compute

Replan when the rendered opponent or our-robot measurement stamp changes (a new perception
frame), else every `replan_period_ticks` ticks. Run it inside `update()` on the control thread
first and log replan time each call. If p99 replan time breaks the budget below, move replanning
to a worker thread that hands over completed plans, with playback staying on the control
thread.

Budget: p99 control tick under 4 ms on the Orin NX CPU with YOLO running. The survey's lower
bound for 128 candidates by 25 steps is about 0.7 ms; the real per-step cost is unknown until
step 2 below measures it.

### 9. Diagnostics and visualization

- A `plant_rollout_nav` diagnostics module: mode, winner cost broken down by term, candidates
  rejected by reason, replan time, contact angle `phi` of the winner, whether the winner came
  from the previous plan.
- `NavigationVisualization` gains an optional polyline for the chosen rollout and the opponent's
  predicted centers, published through the existing Foxglove path.

### 10. Config

`PlantRolloutNavigationConfiguration` in `include/navigation/config.hpp`, registered as
`"PlantRolloutNavigation"` in `src/navigation/config.cpp`, with `apply_plant()` copying the
`[plant]` table so the rollouts, the barrier settings and the EKF read one plant. Every weight,
margin, count and timeout named above is a field. A `[navigation.opponent_weapons]` subtable
keys weapon shape by label.

## Sim, opponents and tuning

**Opponent panel.** Add sim opponent behaviors, all with a heading so the planner's arc logic is
exercised:

| Behavior | Parameters | What it tests |
| --- | --- | --- |
| `pursuit` | lead time, reaction delay, top speed, turn rate; reads our ground-truth pose | Weapon-facing chaser; the rear is hard to reach |
| `corner` | which corner; backs in, then keeps facing out | Side strikes and CONTAIN |
| `spinner` | top speed; `FULL_CIRCLE` weapon | No safe angle; contact only via the weapon-arc override being correct |
| `replay` | human opponent tracks from NHRL and MassD (exists) | Real driving |
| `random_walk`, `circle`, `straight` | exist | Cheap regression cases |

Hold out some parameter settings and some replay tracks as the sim validation set.

**Scorer.** Extend the `EPISODE` line with: rear hits, side hits, hits taken (contact inside the
opponent's arc), seconds inside the arc, time to first hit, wall and hazard contacts, and nose
lifts on the MuJoCo backend. The same metric code must also read real recordings
(`learned_sim_environment_plan.md`, layer 4).

**Search.** CMA-ES over the cost weights, margins and mode thresholds, a dozen or so continuous
parameters. Each candidate runs the C++ binary in lockstep against the sim across the training
panel. Measure episodes per hour per core first; that decides the population size and whether
to run many sim processes in parallel. Count a hit taken, a fall or a flip as a loss.

**Baselines.** `PursuitNavigation` and `MotionProfileNavigation` on the same panel. The new type
has to beat both on hits landed per hit taken, on held-out opponents.

**Real confirmation.** Sim results count only after the closed-loop gate in
`learned_sim_environment_plan.md` passes. Then the top two or three tuned configs run on the
robot against a hand-driven opponent (Mrs Buff Mk3 or the old robot) on the apartment floor,
scored with the same metric code.

## Steps

0. **Prerequisites** (on the checklist): the MuJoCo plant fit passes validation on the
   apartment-floor sessions, and PursuitNavigation in sim matches it on the real robot.
1. **Opponent heading in the estimator.** Steps 1 to 6 of
   `docs/plans/opponent_motion_estimation_plan.md`: the 6-state filter, uncertainty output,
   `predict_opponent()`, the playback regression, the prediction evaluation, and enabling
   `KALMAN`.
2. **Rollout stepper and compute benchmark.** Section 3 with the parity fixture. Then the
   survey's compute experiment on the Orin NX with YOLO running: p50 and p99 for 32 to 256
   candidates and 15 to 30 steps. This fixes the defaults or forces the worker thread.
3. **Predictor and scoring as pure functions.** Sections 2 and 5, unit-tested directly in
   `tests/navigation/`: the arc covers the right bearings and widens with lookahead, a rear
   contact scores below a side contact, which scores below no contact, a front contact is
   rejected, wall and hazard rejects fire, `FULL_CIRCLE` rejects every contact.
4. **Candidate generator.** Section 4, tested for counts per source, segment lengths summing to
   the horizon, and deterministic output for a fixed seed.
5. **PlantRolloutNavigation without the mode layer.** Goal fixed at the opponent's rear,
   execution and barrier per section 7, config and factory, diagnostics. Sim runs against
   `static` and `straight` opponents to check it drives, avoids walls and hazards, and hits from
   behind.
6. **Mode layer.** Section 6. Sim runs against `pursuit` and `corner`.
7. **Opponent panel and scorer.** The sim additions above, plus held-out splits.
8. **Baselines and tuning.** Baseline runs, then the CMA-ES search, then held-out evaluation.
9. **Real-robot runs.** After the closed-loop gate. Write-up in
   `docs/experiments/control_improvement/`.

## Risks and open questions

- **Heading quality on new robots.** The learned keypoints decide the arc. If flips are common on
  a given opponent, its arc should fall back to `FULL_CIRCLE` automatically when
  `heading_sigma_rad` stays high; measure this on recordings before relying on side and rear
  logic.
- **Plan flapping.** The survey flags it for trajectory sampling. Hysteresis is the first
  defense; the margin is a tuned parameter.
- **Coarse candidate set.** If no candidate threads two hazards and the opponent, the survey's
  Rank 2 (SMPPI sampling around the previous plan) replaces the fixed menu and random draws. The
  stepper, predictor and scorer stay.
- **Sim exploitation.** The search will find whatever the sim gets wrong, most likely traction and
  nose lift. Randomize over the passing parameter spread from the MuJoCo fit, count flips as
  losses, and trust only held-out and real results.
- **Pure-pursuit opponents always face us.** Against them a rear window opens only when we are
  faster or they turn. That makes them a good stress test and a poor stand-in for a human; the
  replay tracks cover the human side.
- **Contain timeout.** How long to wait before a forced side strike depends on judging rules,
  not on the planner. It needs a decision from the driver.
- **Our weapon geometry.** `our_weapon_half_angle` and the contact model assume a forward-facing
  weapon. Check it against Mr Stabs Mk2's actual weapon before tuning.
