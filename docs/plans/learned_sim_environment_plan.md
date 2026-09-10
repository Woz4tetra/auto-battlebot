# Learned simulation environment plan

Goal: a simulation whose drivetrain and sensing behaviour are fit from recorded
MCAP data, validated against closed-loop Jetson recordings, and used to rank
control algorithms before they touch the robot. The rank order in sim must be
shown to match the rank order on the robot, or the sim is a guess.

Motivation: `MotionProfileNavigation` stops well in sim (stage 4 report, 6 to
22 mm) but on the robot it is not much better than pursuit at reaching the
target and does not arrive at full speed cleanly. The stage 4 report says "no
hardware runs yet". The MassD 2026-08-29 Jetson recordings are those hardware
runs, and nothing has compared the sim against them.

## What exists

| Layer | State | Where |
| --- | --- | --- |
| Plant model | M5 fit on NHRL May match driving passes all gates | `auto_battlebot/control/plant.py`, `playground/calibration/out/plant_match.toml` |
| Plant fit machinery | jig and match loaders, batched windowed open-loop scoring | `auto_battlebot/calibration/{jig,match}/`, `plant.predict_windows` |
| Headless sim | 2D kinematic server, lockstep protocol, sim owns logical time | `simulation/kinematic_sim_server.py`, `simulation/protocol.py` |
| C++ side | real controller binary runs against sim over `ManualClock` and `SteppedControlLoop` | `config/simulation/mrs_buff_mk3_kinematic_sim.toml` |
| Closed-loop real data | 8 Jetson recordings with nav, transmitter, Kalman, pose diagnostics | `data/saved_recordings/MassD_2026-08-29/` |
| Scoring harness | deleted 2026-09-05, restore point `5af3d75` | `control_stage0_retired.md` |

MassD autonomous ticks per diagnostics-only file (`motion_profile_nav` `command`
statuses, 30 Hz):

| recording | command ticks |
| --- | --- |
| 08-48-31 | 74 |
| 09-05-11 | 8164 |
| 09-20-17 | 2579 |
| 09-34-00_fixed | 1995 |
| 09-50-45 | 847 |
| 11-17-24 | 3632 |
| 13-08-16 | 15875 |
| 15-21-21 | 3724 |

About 36,900 ticks, roughly 20 minutes of closed-loop driving on a second
floor with the controller under test. Each tick logs the command sent, the pose
the controller used and whether it was live or cached, the profile state
(`v_ref`, `v_actual`, `w_ref`, distance, angle error, hazard state), and the
Kalman track counts, heading flips and gated measurements.

## Gaps

1. The sim runs the jig fit, not the match fit. `simulation/sim_mrs_buff_mk3.toml`
   is a hand transcription of `plant_stageA.toml` (M4) with no `c_drift` term,
   and `kinematic_sim_server.py` carries its own copy of the plant equations.
   The match fit report's next step 2 (re-run nav sweeps on match constants)
   never happened.
2. The sensing model is per-tick i.i.d. dropout plus Gaussian noise. Real
   dropout is bursty (p90 gap 340 ms, max 5.8 s), the May pipeline missed 25%
   of grabs, and yaw flips are structured, not white.
3. No scorer. `stage0_metrics.py` and `sim_sweep.py` are gone, and `sim_sweep.py`
   launched a ROS master that no longer exists.
4. No sim-to-real agreement check, open loop or closed loop.

## Design

Five layers. Layers 1 and 2 are the "learned from data" part. Layer 5 is what
makes the result trustworthy.

### 1. Plant: one implementation, loaded from the fit

- `kinematic_sim_server.py` imports `auto_battlebot.control.plant.Plant` and
  deletes its own `Plant.step` and `_substep`. The sim venv already installs
  the package (`scripts/setup_simulation.sh`); confirm `pip install -e .` runs
  there.
- Sim TOML `[our_robot]` gets `plant_file = "..."` pointing at a fit TOML.
  Drop the transcribed `max_linear_speed_*`, `tau_*`, `*_coeff`, `deadzone_*`
  keys. `write_plant_configs.py` then only has `config/_common.toml` to rewrite.
- Transport delay stays in the server's command ring buffer, quantised to
  ticks, as now. `delay_s` from the match fit is a timeline artifact and is
  not used; keep the jig value (52 ms).
- Per-episode parameter draw. The measured spread is the point of the
  exercise, so every episode samples a plant from it instead of running the
  nominal fit. Ranges come from the two fits:

  | param | jig (garage) | match (NHRL floor) | draw |
  | --- | --- | --- | --- |
  | `k_fwd` | 4.88 m/s | pinned at widened bound, about 3x slower | log-uniform over the span |
  | `tau_lin_a` | 0.149 s | 0.213 s | uniform |
  | `tau_ang_d` | 0.088 s | 0.017 s | uniform |
  | `c_ad` | 0.46 | 0.77 | uniform |
  | `c_drift` | 0 | 0.47 rad/s | uniform |
  | gain sag | none | 17% over about 60 s (sphinx fight) | linear ramp, slope uniform in [0, 0.3]/min |

  Sag is a multiplicative ramp on `k_fwd`, `k_rev`, `k_ang`. When drive battery
  voltage is logged it becomes an input instead.
- Structure switch stays: a run can pin `ModelStructure` to M4 to reproduce
  old numbers.

### 2. Sensing model fit from the Jetson recordings

Replace the i.i.d. dropout with a process estimated from MassD, keyed by the
signals already logged:

| effect | source | model |
| --- | --- | --- |
| our-robot dropout | `motion_profile_nav` `poses.our_pose_source` live vs cached, `no_robot` | semi-Markov: measured/dropped states, gap lengths from the empirical CDF, conditioned on speed bin and distance-to-wall bin |
| opponent dropout | `kalman_motion_estimator.num_opponent_tracks` per tick | same shape, conditioned on inter-robot distance |
| yaw flips | `kalman_motion_estimator.num_heading_flips` | flip rate per tick vs speed; flips are 180 deg, not Gaussian |
| pose noise | pose residual against a smoothed track on measured ticks | Gaussian per axis, std by speed bin |
| perception latency | `frame_meta.image_stamp_ns` to `opentx_transmitter` `send` log stamp | empirical distribution, sampled per frame |
| frame drops | `frame_meta` interval | Bernoulli per frame from the observed drop rate (25% in May, measure MassD) |
| radio nonresponsive | `playground/calibration/out/match_fit/nonresponsive_windows.csv` | input dropout spans, rate 0.02/min from the May fit |

Output is one `sensing_model.toml` per venue written by a new
`playground/control_sim/fit_sensing_model.py`, loaded by the sim server.
Shared loaders go in `auto_battlebot/recording/` next to the existing
diagnostics loader.

The important property is that dropout is conditioned on state. A robot near a
wall or merged with the opponent loses its track more often, which is where
the overshoot-coast and blind-reverse incursions happen.

### 3. Scenarios from recordings

- Opponent motion: extract opponent tracks from the NHRL May and MassD
  `/robot_markers` into the replay CSV format the sim already reads
  (`OpponentConfig.behavior = "replay"`).
- Initial conditions: sample our start pose, opponent pose and behaviour mode
  from the starts of real autonomous segments (auto switch `ch15 == 1024`,
  see `playground/mcap_auto_percentage.py`).
- Keep the synthetic scenarios (`stop`, `turn`, `track`) as unit tests of
  specific behaviours. They are not the ranking set.

### 4. Scorer that runs on real and sim files alike

- Metric code in `auto_battlebot/control/metrics.py`, reading only the
  diagnostics topics both real and sim recordings carry
  (`motion_profile_nav`, `opentx_transmitter`, `robot_markers`,
  `kalman_motion_estimator`). Same function scores a MassD file and a sim run.
- Metrics per approach segment: time to goal, terminal distance, overshoot,
  peak speed reached versus `terminal_velocity`, command saturation fraction,
  hazard incursions, wall contacts, fraction of ticks on cached pose.
- `playground/control_sim/run_sweep.py` replaces `sim_sweep.py`: builds a
  config overlay, starts `kinematic_sim_server.py` and `build/auto_battlebot`,
  waits for `max_ticks`, scores. No ROS master. Enable `[mcap]` in the overlay,
  the base sim config has it off.
- Restore `summarize_sweeps.py` from `5af3d75` if the column picker is still
  the right shape; otherwise write a smaller one.

### 5. Sim-to-real agreement gates

Two checks, both before any controller work.

Open loop. Feed each MassD command tape through the plant and score
`predict_windows` at 400 ms against the recorded pose. Needs a live-file
loader: MassD files carry pose and command together, so the SVO join in
`match_windows.py` is not needed, but the window cutting, contact gates and
maneuver classes are. Add `auto_battlebot/calibration/match/live_windows.py`
reusing `build_match_windows`. Score three plants: jig A, match B, and a MassD
refit C. The A-to-B-to-C spread on a third floor is the floor-to-floor
variation and sets the domain randomisation ranges above.

Closed loop. For every MassD autonomous segment longer than 3 s: initialise the
sim from the recorded pose and speed, set the same target, load the same
controller config the Jetson ran, run to the segment length, score both with
the layer 4 metrics. Compare per-segment distributions.

Proposed pass bars (to be revised once the first numbers exist):

- Open loop: match plant B on MassD within 1.5x its NHRL holdout number
  (147 mm at 400 ms). If it is worse than that, refit per venue and treat
  venue as a randomisation axis, not a constant.
- Closed loop: median time to goal and median terminal distance within 20% of
  the real segments, cached-pose fraction within 10 points, and the same sign
  of difference between pursuit and motion-profile on the segments where both
  ran.

Until the closed-loop gate passes, sim results are not evidence for a
controller decision.

## Steps

1. Live-file window loader and open-loop scoring on MassD. Half a day.
   Deliverable: a table like the match fit report's, A/B/C per recording.
2. Residual structure check. Regress M5 residuals on the NHRL holdout and
   MassD against yaw rate, speed times yaw rate, time since segment start.
   Heading error sits at 2 to 4x the noise floor on arcs and spins, so a term
   is missing. If a residual has structure, add the rung to `plant.py`. If
   it is white, stop: a learned residual would fit perception artifacts on
   4,000 samples with 60 mm of noise.
3. Sim imports `plant.Plant`, loads `plant_match.toml`, adds the per-episode
   draw. Re-run the stage 4 scenarios on the match fit as a regression check.
4. Sensing model fit and integration. Compare the sim's cached-pose fraction
   and gap CDF against MassD directly; this is the one layer that can be
   validated without a controller in the loop.
5. Scorer and sweep runner.
6. Closed-loop gate on the MassD segments.
7. Controller evaluation under the randomised plant. First candidates:
   short-horizon sampling MPC over the M5 model (the plant steps in 2 ms
   substeps, cheap enough for a few hundred rollouts per tick on the Orin),
   and a policy trained in the randomised sim. Both use the plant forward
   rather than inverted into feedforward.

## Data to add on the next outing

- Drive battery voltage over CRSF telemetry into `/diagnostics`. The INA219
  stream in the MassD files is the Jetson's own 3S pack (11.96 V at rest), not
  the drive battery, so it cannot explain gain sag.
- A gyro on the robot. Measured yaw rate removes the heading noise floor that
  caps the fit today and is a direct signal for the controller.

## Known limits

- The fit target is the perceived plant through desktop playback, which
  carries the 2 to 3% rectification warp. For the sim that is the right
  target, since the controller sees the same perceived world, but the sensing
  model must then be fit from Jetson files, not playback.
- MassD segments drove to a fixed target on several runs, so the closed-loop
  gate is strongest on approach and stop, weaker on moving-opponent tracking.
  The NHRL May fights cover tracking but only with the old pursuit controller.
- Deadzones remain jig priors. Match and MassD data cannot resolve them.

## Next steps

Start with step 1. It reuses existing code, needs no hardware, and its result
decides whether the plant is portable across floors or must be a
randomisation axis.
