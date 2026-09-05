# control_stage0, retired 2026-09-05

`playground/control_stage0/` held the simulation sweep harness and the recording analyzers
that produced every number in the Stage 0 through Stage 4 control reports. The directory is
deleted. This is the record of what it did, so the reports around it stay readable and so it
can be restored deliberately rather than rediscovered.

Last commit to touch it: `5af3d75` (2026-09-03). Restore the whole directory with:

```bash
git checkout 5af3d75 -- playground/control_stage0
```

Sweep outputs (`sweep_out/`, 550 files, 33 MB) and `run_away_out/` were moved out of the repo
to `~/Desktop/auto-battlebot-regenerable-2026-09-05/control_stage0/`. They are reproducible
from the sweep specs, which are listed below and live in the same commit.

## What each tool did

| Tool | Job |
|---|---|
| `sim_sweep.py` | Batch driver for the headless kinematic sim. Per run it built a C++ config overlay extending the base headless config, launched the ROS master, the sim server and `build/auto_battlebot`, waited for `[sim].max_ticks`, then scored the MCAP. Metrics came from tick index x `[sim].dt`, not wall-clock, because runs are accelerated. |
| `summarize_sweeps.py` | Printed the scored columns from one or more sweep output dirs, picking columns by scenario name. |
| `run_stage4_sweeps.sh` | Ran the Stage 4 set (`stage3_stop`, `stage3_ram`, `stage3_track` by default) against the Mrs Buff Mk3 plant, then summarized. Flagged runs that produced no MCAP. |
| `stage0_metrics.py` | Stage 0 baseline from existing Jetson fight recordings: end-to-end latency, perception reliability, aim, overshoot and wall contact, plus the share of wall collisions attributable to flat-plane keypoint projection error. |
| `fit_plant.py` | First-order AR(1) plant identification from fight recordings, per axis: `v[k+1] = a*v[k] + b*cmd[k]`, so `tau = -dt/ln(a)` and `gain = b/(1-a)`. Superseded by the velocity jig, but `playground/calibration/fit_plant_calib.py` still extends the same method. |
| `prediction_eval.py` | A/B over one SVO with the filter's velocity-based prediction on and off, measuring how much prediction buys during our-robot detection dropouts. |
| `run_away_overshoot.py` | Per-goal-epoch overshoot from a recorded fight, with the five diagnostic leads on one time axis: goal churn, filtered speed error, open-loop stretches, heading-gate re-entry, solver plateau jitter. |

`diag_io.py` already moved to `auto_battlebot/diag_io.py` in `5af3d75` and is unaffected.
`NAV_HW_IDS` there still gates which nav diagnostics a sweep can score.

## Scored columns per scenario

- **stop**: `terminal_pos_err_m`, `terminal_vel_mps`, `overshoot_m`, `t_to_goal_s`, `wall_contacts`
- **ram**: `navigation.attack_terminal_speed_fraction`, `impact_speed_mps`, `min_dist_m`, `t_contact_s`
- **track**: `track_err_mean_m`, `track_err_rms_m`, `wall_contacts`
- **turn**: `terminal_pos_err_m`, `overshoot_m`, `t_to_goal_s`, `mean_abs_ang_deg`, `facing_pct`
- **dropout**: `terminal_pos_err_m`, `terminal_vel_mps`, `overshoot_m`, `wall_contacts`

## Sweep catalog

Eighteen specs in `sweeps/`. Each carried its own re-run command in a header comment.

| Spec | What it varied |
|---|---|
| `start_positions` | Static opponent at (0.5, 0.5), 60 ms latency, vary our start. Corner vs head-on approach geometry against overshoot. |
| `opponent_behaviors` | Fixed start (-0.6, -0.6), 60 ms latency, vary opponent motion. Static vs moving under a no-prediction pursuit controller. |
| `evasive_speed` | Opponent circles arena center, vary its speed. How far a no-lead pursuit falls behind a faster evader. |
| `crossing_headings` | Opponent drives straight, bouncing off walls, on varying headings at 60 ms. Lead/lag error against a linearly moving target. |
| `latency_x_circle` | Opponent circles at constant speed, vary actuation latency. Contact degrades faster with latency than against a static opponent. |
| `randomwalk_seeds` | Random-walk opponent across RNG seeds. Seed drives both opponent targets and the perception noise stream, so this is run-to-run variance. |
| `goto_stop` | Coast-aware deceleration into a static goal, with an angular command cap so a committed turn does not overshoot. |
| `stage3_stop` | Zero-velocity stop, PursuitNavigation baseline vs the motion-profile tracker. |
| `stage3_ram` | Same controller, nonzero terminal velocity, including `brake_distance = 0`. |
| `stage3_track` | Chase an opponent circling at 0.8 and 1.5 m/s, radius 0.6. |
| `stage4_turn` | Approach a goal starting well off bearing (45 deg and worse). |
| `stage4_ablation` | Turn each compensation off one at a time on the scenario that exercises it. Sim profile runs `GroundTruthRobotFilter`. |
| `stage4_dropout` | Zero-velocity stop while our own track drops out: no latency lead subtracted, plus `speed_kp * v_ref` of extra thrust. |
| `hazard_avoidance` | Wall-hugging holes in a small cage, AER geometry, goal past the west hole so the run must round it. |
| `hazard_baseline` | Same geometries with the avoidance layer off, so the two tables compare row by row. Both hole runs expected `fell_in = 1`. |
| `run_away_overshoot` | RUN_AWAY overshoot reproduction, pinned via `[transmitter].behavior_mode`, scoring `overshoot_m` and `terminal_vel_mps`. |
| `run_away_speed_sensitivity` | Same corner geometry, estimator degraded one axis at a time. The recorded fights had the EKF reading 0.31x of chord speed pre-`q_along` and 0.72x post. |
| `run_away_accel_limit` | Does softening the reference-speed ramp calm the RUN_AWAY re-approach, under degraded perception. |

## Where the results live

The reports stay. Their command blocks now point at paths that no longer exist; the commands
are still correct against a restored checkout.

- `stage0_baseline_report.md` - Stage 0 baseline from Jetson recordings
- `stage1_sim_report.md` - first sim sweeps and the AR(1) plant fit
- `stage3_sim_report.md` - stop, ram and track against the motion-profile tracker
- `stage4_sim_report.md` - Stage 4 set against the Mrs Buff Mk3 plant
- `hazard_avoidance_report.md` - avoidance layer against its ablated baseline
- `run_away_overshoot_report.md` - overshoot on the 2026-08-23 fight
- `run_away_solver_report.md` - safest-point solver comparison

## Consequences

The `/eval` skill's sim-sweep mode has no backing tool until `control_stage0` is restored or
replaced. Its detector/keypoint mode (`training/model_eval/score.py`) is unaffected.
`config/experiments/experiment_playback.toml`, `config/simulation/kinematic_sim.toml` and
`simulation/kinematic_sim_server.py` carry comments naming these scripts; the comments are
now historical.
