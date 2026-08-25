# Hazard avoidance: sim results

What the four layers do to a robot that would otherwise drive into a hole. Implementation and
config reference: `docs/hazard_avoidance.md`. The plan is `docs/hazard_avoidance_plan.md`; the
overshoot investigation it depends on is `run_away_overshoot_report.md`.

All numbers from the kinematic sim against the Mrs Buff Mk3 plant fit
(`playground/control_stage0/sweeps/hazard_avoidance.toml`), 900 ticks at dt = 0.0333 s, 52.2 ms of
actuation latency, no perception degradation.

## The failure baseline

Hazards in the world model with every avoidance layer switched off
(`playground/control_stage0/sweeps/hazard_baseline.toml`, which sets the ablation itself so the
comparison stays reproducible rather than describing a build that no longer exists):

| run | fell in | min clearance (m) | episode ended (s) |
|---|---|---|---|
| clean | no | - | 29.97 (ran to the tick cap) |
| hole on path | **yes** | -0.0039 | 0.47 |
| hole and block | **yes** | -0.0026 | 0.50 |

Half a second. The robot leaves the corner, accelerates at the opponent, and the straight line
goes through the hole.

## With the layers on

Same geometries plus the run-away and house-bot cases. `min_dist_m` is the closest the robot got
to the thing it was chasing -- the column that says whether avoidance cost the mission.

| run | hazards | fell in | min clearance (m) | block hits | min dist to goal (m) | time lost (s) |
|---|---|---|---|---|---|---|
| attack_clean | 0 | no | - | 0 | 0.016 | - |
| attack_hole | 1 | no | 0.206 | 0 | 0.014 | -0.23 |
| attack_opponent_on_lip | 1 | no | 0.215 | 0 | 0.005 | -0.10 |
| runaway_clean | 0 | no | - | 0 | 0.002 | - |
| runaway_hole | 1 | no | 0.246 | 0 | 0.002 | -0.17 |
| housebot_static | tracked | no | 0.049 | 0 | 0.019 | -0.23 |
| housebot_crossing | tracked | no | 0.051 | 0 | 0.017 | -0.16 |
| hole_and_block | 2 | no | 0.172 | 0 | 0.008 | -0.26 |

No fall-ins, no contacts, clearance positive everywhere, and every run still reaches its goal
inside 2 cm. Time lost is negative in every row, which is not a claim that avoidance is free:
routing around a hazard changes the approach geometry, and on these particular starts it happens
to line the robot up better. Read it as "no measurable cost on these eight cases", not as a
speed-up.

The house-bot rows clear by 5 cm against 17-25 cm for the static ones. That is the configured
difference: `hazard_tracked_margin_m` is 0.05 against `hazard_static_margin_m` 0.10, on purpose,
because a house bot parked mid-field with a large keep-out disc makes big parts of the arena
unreachable.

## No-hazard regression

The feature has to be invisible on a field with nothing to avoid. Stage 3 sweeps, unchanged
configs, `hazard_count = 0` throughout:

| run | terminal pos err (m) | terminal vel (m/s) | overshoot (m) |
|---|---|---|---|
| short_tracker | 0.006 | 0.0 | 0.006 |
| center_tracker | 0.005 | 0.0 | 0.000 |
| far_diag_tracker | 0.020 | 0.0 | 0.000 |

Tracking (`stage3_track`): `track_err_mean_m` 0.219 slow / 0.105 fast, matching the stage 3
numbers.

## Two geometry bugs the sweep caught

Both were found by running the sweep, not by reading the code, and both are worth stating because
the plan's description of option 4 does not rule them out.

**Aiming at the tangency point aims at the keep-out rim.** The first implementation put the
waypoint exactly on the inflated circle, which is a point with zero clearance by construction. The
robot drove there and ground along the edge: `housebot_static` reported 131 block contacts and
never reached the opponent. Taking the tangent against a circle
`hazard_waypoint_clearance_m = 0.06` wider fixed it -- 131 contacts to 0 -- and has a second
benefit: the run from the waypoint to the true goal is then unblocked against the keep-out itself,
so the goal returns on its own.

**A carrot waypoint makes the robot orbit.** The intermediate fix carried the waypoint out along
the tangent bearing to the goal's range instead of stopping at the tangency point. That turns
tangent steering into a heading the robot can hold forever: `housebot_static` circled the hazard,
`min_dist_m` 1.247, and `hole_and_block` fell in. The tangency point terminates because reaching
it clears the segment; a carrot does not. The plan's original wording -- drive to the tangent point
-- was right, and only needed the clearance margin.

## A fixture mismatch worth keeping

The house-bot runs first reported `min_clearance = -0.0000` with dozens of block contacts, and the
cause was not code. The sweep set the sim's `hazard_radius` to 0.20 m while the C++ side derives
the keep-out from the track's own footprint, which `GroundTruthRobotFilter` reports as
0.15 x 0.15 m -- half-diagonal 0.106. The sim's floor was wider than the controller's model of it,
so the controller kept a clearance it believed and the sim scored a scrape.

That is the drift the shared geometry file prevents for static hazards, and it has no equivalent
for tracked ones: both sides derive the radius from the robot, so they agree only if the sim uses
the same footprint. It is now stated in `OpponentConfig.hazard_radius` and in the sweep. Setting
it deliberately larger is a useful test in its own right -- a house bot whose true size the
controller cannot see -- and the sweep reports the scrapes rather than hiding them.

## Replay against a recorded fight

The sim controls the geometry; a replay proves the plumbing on real perception. Dropping a
synthetic hole into the field config and replaying the 2026-05-02 14-12 recording
(`config/hazards/replay_synthetic_hole.toml`, start frame 1540, mrs_buff_mk3 playback base):

- 1851 nav ticks, 1851 `/hazard_markers` messages, run ended on EOF rather than a crash.
- The hole inflated from its raw 0.22 m to **0.443 m**, using our robot's *measured* half-diagonal
  of 0.123 m plus the 0.10 m margin. That number comes from the real keypoint track, and it is
  larger than the sim's 0.106 -- the inflation path is doing its job on real data, not on a
  hardcoded footprint.
- Layers exercised on real poses: tangent waypoint on 28% of ticks, speed cap on 1%, reverse
  backstop on 14%.
- Tick time (`runner heartbeat` max per second): median 14.3 ms, p90 16.2 ms, one 82.8 ms outlier.
  Desktop, not Jetson, so this is not a deployment latency claim -- but the layers are not
  expensive.

What this does **not** show is whether the robot would have avoided the hole. In playback the
`PlaybackTransmitter` replays the recorded driver's sticks, so navigation's output is computed and
logged but never moves anything. The robot in the recording drove over the synthetic hole's centre
(closest approach 0.006 m) because the human driving it in May could not see an imaginary hole.
That is also why the reverse backstop reads 14% here rather than the near-zero a sweep shows: the
recorded trajectory spends real time inside a keep-out the controller never got to steer out of.

The trajectory question belongs to the sim, where the commands actually drive the plant.

## Reproducing

```bash
source scripts/activate_python.sh
python playground/control_stage0/sim_sweep.py \
    --sweep playground/control_stage0/sweeps/hazard_avoidance.toml \
    --sim-config simulation/sim_mrs_buff_mk3.toml \
    --out playground/control_stage0/sweep_out/hazard_avoidance
```

```bash
python playground/control_stage0/sim_sweep.py \
    --sweep playground/control_stage0/sweeps/hazard_baseline.toml \
    --sim-config simulation/sim_mrs_buff_mk3.toml \
    --out playground/control_stage0/sweep_out/hazard_baseline
```

To watch instead of reading the table, set `[viewer] enable = true` in the sim TOML. A sweep row
saying "minimum clearance 0.04 m" does not tell you whether the robot picked the wrong side and
cut back across the hazard.

## Not measured here

- **Real arena geometry.** No shipping config sets `hazards_file`; the hole's true shape and
  position are still open questions. Everything above uses synthetic geometry.
- **The estimator.** The sim runs `GroundTruthRobotFilter`, so the tracked-hazard path is fed an
  exact house-bot pose and velocity. On the robot it comes from the same filter whose speed output
  reads 28% low (see `run_away_overshoot_report.md`), and a lagging hazard position moves the
  keep-out disc behind where the house bot actually is. `hazard_prediction_horizon_s` compensates
  for velocity but not for position lag.
- **Field behaviour.** None of this has run on the robot.
