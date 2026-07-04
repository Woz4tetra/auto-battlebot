# Control Improvement Plan: Mr Stabs Mk2

Goal: Mr Stabs Mk2 precisely follows a commanded trajectory, within the limits of perception accuracy,
reaches the goal in optimal time, and never hits a wall. Terminal velocity is a trajectory parameter:
a precise stop ends at zero velocity, a ram ends at contact velocity. The zero-velocity stop is the
diagnostic that proves the robot can actually control its trajectory.

## Scope

- In scope: the control law and trajectory generation, using the measured plant.
- Out of scope: perception and sensing. Better sensing comes from better model training, tracked
  separately in `perception_improvement_plan.md`. Here perception accuracy and dropout rate are fixed
  constraints the controller must live within, not variables we tune. Perception accuracy sets the position
  tolerance the controller aims for.

## The plant we are controlling

From `mr_stabs_mk2_calibration.md` (run 175805). Time constants are low confidence (one clean segment
each, robot left frame) and should be re-recorded, but they set the shape of the problem.

| Property | Value | Control implication |
|----------|-------|---------------------|
| max linear speed fwd / rev | 5.6 / 4.84 m/s | asymmetric; reverse is a weaker brake than forward drive |
| max angular speed | 61.5 rad/s | rotation is not the bottleneck |
| deadzone | ≤ 0.04 | small; feedforward-lift it |
| actuation latency | 60 ms | command bites 60 ms late: 0.34 m of travel at top speed |
| accel time constant | 0.058 s | accel is fast, near a step |
| coast (decel) time constant | 0.078 s | passive coast distance at top speed ≈ v·τ ≈ 0.44 m |

The single number that dominates precise stopping:

```
stopping lead ≈ latency travel + coast distance ≈ 0.34 m + 0.44 m ≈ 0.78 m at top speed
```

That is ~3.5 robot lengths (robot ≈ 0.22 m). If we command "stop at the goal" when we reach the goal, we
overrun by ~0.78 m. To stop on target we must start braking ~0.78 m early, or actively brake (reverse)
rather than coast. Fast accel plus a long coast means overrun on stop, not sluggish start, is the enemy.

## What already exists

Stages 0 and 1 of the earlier plan are largely done. This plan builds on them, it does not restart them.

- **Baseline measured** (`stage0_baseline_report.md`): latency 49-57 ms p50 (over the 60 ms budget by
  p95), 8-70% near-wall time, and a 13-18 cm flat-plane projection error that points into the wall. That
  projection error is a *perception* cause of wall contact and is punted to the perception plan; the
  coast-and-overrun cause is ours.
- **Plant calibrated** (`mr_stabs_mk2_calibration.md`): the table above, from
  `playground/calibration/fit_plant_calib.py`. Supersedes the earlier rough MCAP fit
  (`playground/control_stage0/fit_plant.py`).
- **Headless sim exists** (`stage1_sim_report.md`): `simulation/kinematic_sim_server.py` is a fast 2D
  kinematic plant behind the sim TCP protocol (first-order lag/coast, actuation-latency buffer, arena
  walls, opponent behaviors), run via `config/simulation/headless_sim.toml` and swept by
  `playground/control_stage0/sim_sweep.py`. It is a faithful **controller** testbed (runs the real
  `PursuitNavigation` under an injected logical clock, deterministic, free-run) but not yet a faithful
  **drivetrain** testbed: its `max_*_speed` and `tau` were informed guesses. Our calibration closes that.
- Known controller findings from the sim: the `steer_brake` sign bug is fixed; the earlier
  "latency causes wall-slamming" result was a tuning artifact (Genesis `angular_kp = 26` leaked into the
  sim); with real gains, no-lead pursuit orbits a static target but misses fast evaders, which motivates
  latency compensation.

The gap this plan fills: pour the calibrated plant into the sim to make it a faithful drivetrain testbed,
add a trajectory-following mission with a zero-velocity stop, and pick a control law that follows the
trajectory precisely and stops on the goal.

## The reframe

Two separable problems. Keep them apart.

- Prediction: where will the robot and target be when the command actually bites (60 ms later). Perception
  noise and dropout live here, and this plan treats them as a fixed constraint (forward-predict our own
  pose from in-flight commands; aim within the perception tolerance).
- Control: given the plant, what command sequence follows the trajectory precisely and arrives at the goal
  at the commanded terminal velocity without overrunning.

Overrun into walls is mostly the coast model leaking into an open-loop controller. We command toward the
goal from a stale pose with a drivetrain that coasts 0.44 m. This is a plant-compensation problem, not a
missing-MPC problem, for the bulk of it.

## Exploration options

Two choices to make: the control law (how commands are computed) and the trajectory scenario (what we
command and validate). Pick a control law and validate it against the scenarios, starting with the
zero-velocity stop.

### Control-law options

Ordered cheapest to heaviest. Each notes what it buys, what it costs, and which measured plant terms it
needs.

1. **Pursuit + feedforward plant compensation** (extend the current `PursuitNavigation`).
   Add three things to the existing carrot-follower: deadzone lift (already have
   `lifted_deadzone_percent`), a velocity feedforward so commanded speed maps to actual speed, and a
   coast-aware brake point that starts decelerating `stopping lead` before the goal.
   - Buys: fastest path to no-overrun and a working zero-velocity stop; reuses shipped code.
   - Costs: heuristic, tuned per trajectory, not provably optimal.
   - Needs: deadzone, max speed, τ_decel, latency. All measured.

2. **Motion-profiled trajectory + inverse-plant feedforward + PID feedback** (recommended default).
   Generate a time-parameterized reference with a trapezoidal or S-curve velocity profile that respects
   v_max and a_max and ends at the commanded terminal velocity (zero for a stop, v_contact for a ram).
   Track it with feedforward from the inverse plant (deadzone + accel model) plus PID on cross-track and
   along-track error. Latency-compensate by forward-predicting the state 60 ms.
   - Buys: precise following, near-time-optimal (the profile is time-optimal for a straight move), a
     natural zero-velocity termination, and a clean split between open-loop accuracy (feedforward) and
     disturbance rejection (feedback). This is the standard robotics motion-control approach and maps
     directly onto the goal.
   - Costs: needs a trajectory generator and an inverse-plant model; more code than option 1.
   - Needs: v_max (fwd/rev asymmetry), a_max (from τ_accel), deadzone, τ_decel, latency. All measured.

3. **Ramsete nonlinear tracker.** A unicycle trajectory tracker with global convergence to a
   time-parameterized pose+velocity reference, two tuning gains.
   - Buys: principled tracking of curved paths with few knobs.
   - Costs: needs a reference trajectory (pair with option 2's generator); does not natively handle the
     tank-drive speed asymmetry or saturation (bolt on feedforward + clamping).
   - Needs: a reference trajectory; otherwise kinematic.

4. **LQR trajectory tracking.** Linearize the plant around the trajectory, optimal linear state-error
   feedback plus feedforward.
   - Buys: systematic gains from the plant, optimal in the LQR sense.
   - Costs: needs a linear plant model and Q/R tuning; still needs feedforward and constraint handling
     bolted on.
   - Needs: linearized first-order plant (have it).

5. **MPC with constraints.** Optimize a short horizon over the plant model with hard constraints on
   velocity, actuation, and walls; objective trades tracking, time, and effort.
   - Buys: handles latency, coast, limits, and wall constraints directly; can be genuinely time-optimal;
     the right tool if the simpler laws cannot meet the spec.
   - Costs: heaviest; a solver in the perception loop (latency budget), and it is only as good as the plant
     model, so re-record the taus first.
   - Needs: full plant model with trustworthy τ_accel / τ_decel.
   - Gate: only if the validated sim shows options 1-2 cannot hit the spec.

### Trajectory scenarios (validation)

1. **Point-to-point, zero terminal velocity ("go to X and stop").** The primary diagnostic. Command a
   straight or curved trajectory to a goal that ends at rest. Pass = stops within the perception-accuracy
   tolerance of the goal, no overshoot, no wall contact, and time within a set margin of the time-optimal
   bound. This is the cleanest pass/fail for trajectory control: it forces accelerate, cruise, brake, and
   stop, and it exercises the 0.44 m coast that causes overrun. Run in sim first, then on the real robot as
   the go/no-go for a control law. This is what tells us whether the real robot can control its trajectory.
2. **Waypoint / curved path following.** Cross-track error along a path (S-curve, figure-8). Tests tracking,
   not just endpoint.
3. **Ram, nonzero terminal velocity.** The combat objective: terminal velocity = contact speed at the
   target. Same controller, different terminal condition.

## Scoring

One scalar per scenario, plus the diagnostics behind it:

- terminal position error (m) and terminal velocity (m/s) [zero-velocity stop]
- cross-track error, RMS and max (m) [path following]
- time-to-goal vs time-optimal lower bound (ratio) [optimal time]
- wall contacts (count) and overshoot past goal (m) [safety]
- control effort and command reversals (smoothness)

## Staged plan

- **Stage 0 (done).** Baseline (`stage0_baseline_report.md`), plant (`mr_stabs_mk2_calibration.md`), and
  perception reliability (`perception_reliability_may_fights.md`). Caveat: taus are low confidence.
- **Stage 1a (done).** Headless kinematic sim as a controller testbed (`stage1_sim_report.md`).
- **Stage 1b (done): faithful drivetrain testbed + the stop mission.** Per-robot calibrated plant configs
  `simulation/kinematic_sim_mr_stabs_mk2.toml` (5.6/4.84 m/s, τ_accel 0.058, τ_decel 0.078, latency 60 ms)
  and `kinematic_sim_mrs_buff_mk3.toml` (uncalibrated placeholder). A "go to X and stop" scenario
  (`playground/control_stage0/sweeps/goto_stop.toml`, a static goal + `navigation.stop_distance`) with
  trajectory-following scoring (terminal position error, terminal velocity, overshoot, time-to-goal,
  wall-contact count) and a stop plot, all in `sim_sweep.py`. Also fixed two pre-existing sweep breakages
  from the config refactor (the `BASE_CPP_CONFIG` path and a stale `label_mapping` in `headless_sim.toml`).
  **Baseline (current `PursuitNavigation` on the calibrated plant): it cannot stop on the goal.** It reaches
  within ~6 cm but arrives at ~5 m/s and overshoots 0.4-1.5 m, ending 0.55-1.84 m from the goal, still
  moving in 4 of 5 runs, and bouncing off walls in the overshoot. This is the number Stage 2 must beat.
- **Stage 2 (in progress): baseline control law.** Three fixes to `PursuitNavigation`, validated on the
  zero-velocity stop in sim:
  1. **Turn hysteresis lock.** `apply_hysteresis` re-committed the turn sign on every flip, so at a ~180 deg
     overshoot (goal directly behind) it chattered left-right under latency and never completed the turn.
     Now it commits once and holds until the error drops below the release threshold.
  2. **Angular command cap** (`max_angular_z`). At the calibrated 61.5 rad/s a committed turn rotates ~116
     deg per 30 Hz tick and overshoots the heading into walls. Capping the command (0.25 -> ~15 rad/s
     effective) keeps the turn stable.
  3. **Coast-aware brake** (`brake_distance`, new config, default 0 = ram). Within `brake_distance` of the
     target the commanded speed ramps to zero, so the robot decelerates into the goal instead of blasting
     through the stop zone at 5.6 m/s and coasting past. The existing velocity ramp did the opposite (full
     speed when close, for ramming).

  Result on `goto_stop.toml` (calibrated plant, 5.6 m/s / 61.5 rad/s kept): terminal position error dropped
  from 0.55-1.84 m to 0.04-0.22 m, terminal velocity to 0, wall contacts from 64-89 to 0-1. The robot now
  drives in, overshoots ~0.26 m, turns and returns, and stops on the goal, the real behavior, converged.
  These params live in the `goto_stop` scenario overrides for now; promote `max_angular_z` and
  `brake_distance` to `main.toml` for the real robot's stop missions. Feedforward/deadzone-lift and the
  motion-profile tracker are still open (Stage 3).
- **Stage 3: precise tracker.** Implement option 2 (motion profile + feedforward + PID). Compare against
  the baseline on all three scenarios. Expect this to be the one that meets the precision + optimal-time
  goal.
- **Stage 4: real-robot validation.** Run the zero-velocity stop on Mr Stabs Mk2. Measure the sim-to-real
  gap. If the gap is large, the low-confidence taus are the first suspect: re-record the plant with the tag
  kept in frame, refit, and re-tune. Do not skip this: the zero-velocity stop on the real robot is the
  whole point.
- **Stage 5: escalate only if needed.** Options 3-5 (Ramsete / LQR / MPC) only if the validated sim shows
  options 1-2 cannot meet the spec. By then the sim is trustworthy, so this is an empirical decision.

## What to do first

Stage 1b is done: the sim runs the calibrated Mr Stabs Mk2 plant and scores the zero-velocity stop, and the
baseline confirms the current controller overshoots ~0.5-1.8 m. Next is Stage 2, option 1: add a
coast-aware brake and velocity feedforward to `PursuitNavigation` so it stops on the goal, and re-run
`goto_stop.toml` to check terminal position error and terminal velocity drop. Run it with:

```
python playground/control_stage0/sim_sweep.py --sweep playground/control_stage0/sweeps/goto_stop.toml \
    --out playground/control_stage0/sweep_out/goto_stop
```

## Key files

- `simulation/kinematic_sim_server.py` — the 2D kinematic plant; its drivetrain model is what the
  calibration must feed. `simulation/kinematic_sim.toml` — the plant params to update from the calibration.
- `config/simulation/headless_sim.toml` — the headless controller-testbed profile.
- `playground/control_stage0/sim_sweep.py` and `playground/control_stage0/sweeps/` — batch sweep + scoring;
  add the zero-velocity stop scenario and trajectory-following metrics here.
- `src/navigation/pursuit_navigation.cpp` / `include/navigation/pursuit_navigation.hpp` — the control law;
  where feedforward, the brake point, and (later) the motion-profile tracker go.
- `include/navigation/config.hpp` — `PursuitNavigationConfiguration`, holds `lookahead_time` (currently
  dead; wire it for latency compensation).
- `src/robot_filter/robot_temporal_motion_filter.cpp` — `update_with_prediction`, self-pose forward
  prediction for latency compensation.
- `playground/calibration/fit_plant_calib.py` — the current plant fit, to re-run after a cleaner recording.
- `config/playback/mr_stabs_mk2_playback.toml` — regression harness entry point.
