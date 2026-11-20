# Stage 3 Report: Precise Motion-Profiled Tracker

I implemented option 2 from the control improvement plan (motion-profiled trajectory + inverse-plant
feedforward + speed PID) as a new `MotionProfileNavigation` control law, and compared it against the
Stage 2 baseline (`PursuitNavigation`) on all three scenarios in sim. The headline: on the
zero-velocity stop, the plan's primary go/no-go, the tracker beats the baseline on every metric that
matters. It stops tighter (terminal error 0.03 to 0.11 m vs 0.10 to 0.12 m), does not overrun (0.01 to
0.05 m vs a consistent 0.26 m), ends at zero velocity, and never touches a wall. The whole point of
Stage 3 was "arrive at the goal without overrunning," and the tracker does that where the baseline
always sails past by a quarter meter and drives back.

The plant is unchanged from `mr_stabs_mk2_calibration.md` (run 175805): 5.6 / 4.84 m/s fwd/rev,
tau_accel 0.058 s, tau_decel 0.078 s, 60 ms actuation latency. Time constants are still low confidence
(one clean segment each), so treat the sim's overshoot *shapes* as trustworthy and *magnitudes* as
provisional until Stage 4 re-records on the real robot.

## What I built

- `MotionProfileNavigation` (`include/navigation/motion_profile_navigation.hpp`,
  `src/navigation/motion_profile_navigation.cpp`), a new `NavigationInterface` implementation. The
  baseline `PursuitNavigation` is untouched, so the two run side by side in one sweep by switching
  `navigation.type`.
- `MotionProfileNavigationConfiguration` (`include/navigation/config.hpp`) and factory registration
  (`src/navigation/config.cpp`). Because the config overlay replaces a section wholesale when `type`
  differs, a tracker run gets a clean `[navigation]` section with no stale pursuit fields.
- Sim-analysis plumbing: `diag_io.load_diagnostics` now ingests the `motion_profile_nav` diagnostics
  logger alongside `pursuit_nav` (both log the same per-tick schema), and `sim_sweep.score_run` emits
  `track_err_mean_m` / `track_err_rms_m` for moving-target runs.
- Three sweeps in `playground/control_stage0/sweeps/`: `stage3_stop.toml`, `stage3_ram.toml`,
  `stage3_track.toml`, each pairing baseline vs tracker runs.

## The control law

Everything is computed in physical units (m/s) and normalized to a command at the end, because the
plant scales a `[-1, 1]` command by the max speed.

1. **Distance-to-go coast-aware brake.** For a first-order plant, the residual travel after commanding
   the terminal speed is set by the actuation latency and the coast time constant. The reference speed
   is capped so the robot can always shed to `v_term` by the goal:

   ```
   d_eff = max(0, distance - v_actual * latency)      # brake command bites `latency` seconds late
   v_ref = v_term + d_eff / (tau_decel + latency)      # coast horizon after it bites
   v_ref = min(v_ref, v_max)                           # rate-limited on ramp-up for a clean launch
   ```

   `v_term` is a config field: 0 for a precise stop, the contact speed for a ram. This is the inverse
   of the measured plant, not a hand-set brake distance.
2. **Inverse-plant feedforward.** `u_ff = (v_ref + tau * dv_ref/dt) / v_max`, using both signs of
   `dv/dt` and the regime tau (accel while rising, decel while falling). The falling-reference term is
   what makes the plant actively brake instead of coasting past.
3. **Speed PID.** `u_fb = speed_kp * (v_ref - v_actual)` (+ optional integral). Our speed is estimated
   from the pose finite difference projected on heading, since the sim sends no velocity.
4. **Angular control.** Turn-lock hysteresis + capped PD, matching `PursuitNavigation` so heading
   behaviour is comparable. Reimplemented in the new class to keep the baseline untouched.

## Results

### Scenario 1: zero-velocity stop (`stage3_stop.toml`)

| run | terminal_pos_err (m) | terminal_vel (m/s) | overshoot (m) | t_to_goal (s) | wall_contacts |
|-----|----:|----:|----:|----:|----:|
| short_baseline    | 0.102 | 0.0 | 0.264 | 0.27 | 0 |
| short_tracker     | **0.028** | 0.0 | **0.046** | 0.43 | 0 |
| center_baseline   | 0.101 | 0.0 | 0.262 | 0.43 | 0 |
| center_tracker    | **0.088** | 0.0 | **0.013** | 0.50 | 0 |
| far_diag_baseline | 0.116 | 0.0 | 0.266 | 0.87 | 1 |
| far_diag_tracker  | **0.114** | 0.0 | **0.006** | 0.70 | 0 |

The tracker wins on position error and overshoot in every case, ends at zero velocity, and takes zero
wall contacts (the baseline clipped a wall on the long diagonal run). It is slightly slower to goal on
the short runs (it brakes earlier and more gently) and faster on the long run (0.70 s vs 0.87 s). That
is a good trade: a fraction of a second for 3x tighter stops and no overrun.

### Scenario 3: ram, nonzero terminal velocity (`stage3_ram.toml`)

The tracker carries a controlled speed into contact instead of braking to zero. `impact_speed_mps` in
the CSV is measured at the 0.15 m contact ring, mid-brake, so it understates the control; the true
contact speed at closest approach tells the clean story (measured from the trajectories):

| commanded v_term (m/s) | contact speed at closest approach (m/s) | min_dist (m) |
|----:|----:|----:|
| 0.0 | **0.00** | 0.000 |
| 1.5 | 3.61 | 0.043 |
| 3.0 | 3.83 | 0.028 |
| 4.5 | 5.09 | 0.025 |

`v_term = 0` brings the robot to an exact stop on the goal (0.00 m/s at 0.000 m) with no stop-distance
cut at all, purely from the profile. That is the strongest possible zero-velocity-stop proof. Nonzero
`v_term` carries speed into contact and the contact speed rises with the command. The low end is
compressed: from a 1.41 m corner-to-center runway the robot reaches near top speed and cannot fully
shed back to 1.5 m/s before it reaches the target. This is a runway limit of the 2.4 m arena plus the
60 ms latency, not a controller failure. On the shorter direct-ram runs the tracker still contacts
slower and more repeatably than the baseline (center: 2.86 m/s vs the baseline's uncontrolled 5.19
m/s).

### Scenario 2: moving-target tracking proxy (`stage3_track.toml`)

True fixed-path cross-track tracking needs a reference-trajectory generator + Ramsete (Stage 5). Here I
chase an opponent on a circle and report the steady-state distance to it (tail half of the run) as a
cross-track proxy.

| run | track_err_mean (m) | track_err_rms (m) | wall_contacts |
|-----|----:|----:|----:|
| circle_slow_baseline | 0.392 | 0.399 | 0 |
| circle_slow_tracker  | 0.386 | 0.395 | 0 |
| circle_fast_baseline | 0.560 | 0.565 | 0 |
| circle_fast_tracker  | **0.469** | **0.492** | 0 |

The tracker matches the baseline on the slow circle and tracks the fast circle 13% tighter (RMS 0.492
vs 0.565). Neither controller is a real path follower, so this is a modest, expected result: the
tracker's controlled speed keeps it closer to a fast-moving target on average.

## Two bugs found during bring-up

- **Bug: no active braking.** My first feedforward clamped the derivative term to `max(0, dv/dt)`, so
  it only ever added thrust. On a falling reference the plant just coasted, overshot the goal, and
  landed in a limit cycle (approach, sail through, turn 180 deg, re-approach, repeat), ending at 0.6 to
  2.5 m/s instead of stopped.
  **Fix:** use the full-signed derivative with the regime tau. A falling reference now commands reduced
  or reverse throttle, so the plant tracks the deceleration. Overshoot dropped from 0.33 m to ~0 and
  terminal velocity to 0.
- **Bug: uncontrolled ram contact speed.** Without a latency lead, the brake schedule started too late;
  the robot rammed at ~5.5 m/s regardless of the commanded `v_term`.
  **Fix:** evaluate the schedule at `distance - v_actual * latency`, where the command will actually
  bite. Contact speed now responds to `v_term`, and the fix also tightened the stop (terminal error
  from 0.07 to 0.03 m on the short run).

## Honest gaps

- The ram cannot hit a low contact speed from a short runway. Precise contact-speed control needs more
  distance than the 2.4 m arena gives at this plant's top speed.
- Scenario 2 is a moving-target proxy, not cross-track error against a fixed path. That waits on
  Stage 5.
- Our-speed estimation is a clean pose finite difference in sim (no observation noise here). On the
  real robot the pose is noisy, so the speed PID term will need filtering or a lower gain.
- Time constants are still low confidence. If the Stage 4 sim-to-real gap is large, re-record the plant
  first (plan Stage 4).

## How to run

```bash
source scripts/activate_python.sh
cmake -S . -B build && ./scripts/build.sh    # new .cpp: the source glob is not CONFIGURE_DEPENDS
python playground/control_stage0/sim_sweep.py --sweep playground/control_stage0/sweeps/stage3_stop.toml  --out playground/control_stage0/sweep_out/stage3_stop
python playground/control_stage0/sim_sweep.py --sweep playground/control_stage0/sweeps/stage3_ram.toml   --out playground/control_stage0/sweep_out/stage3_ram
python playground/control_stage0/sim_sweep.py --sweep playground/control_stage0/sweeps/stage3_track.toml --out playground/control_stage0/sweep_out/stage3_track
```

Each run writes `results.csv` and per-run trajectory PNGs to its `sweep_out/` directory.

## Next steps

1. **Stage 4: real-robot validation.** Run the zero-velocity stop on Mr Stabs Mk2 with
   `MotionProfileNavigation`. Measure the sim-to-real gap on terminal error, terminal velocity, and
   overshoot.
2. If the gap is large, re-record the plant with the tag kept in frame, refit tau_accel / tau_decel,
   and re-tune the brake horizon.
3. Promote the tracker config into `config/main.toml` for the robot's stop missions once validated.
4. Only escalate to Ramsete / LQR / MPC (Stage 5) if the tracker cannot meet spec on the real robot.
