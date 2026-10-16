# RL policy navigation plan

Goal: a navigation type, `RlPolicyNavigation`, that drives Mr Stabs Mk2 with a neural network
policy trained in the MuJoCo sim. It hits the opponent from behind or the side, stays out of the
opponent's weapon arc, and avoids walls and hazards because it was trained to, not because a
separate layer clamps its output. Hazards are policy inputs. `RlPolicyNavigation` does not use
`HazardAvoidance` (section 12).

This plan replaces `docs/plans/plant_rollout_navigation_plan.md`. The rollout planner needed an
accurate plant online and a hand-built mode layer (STAGE, COMMIT, EXIT, CONTAIN) with tuned cost
weights. A policy needs the plant only during training and learns the mode behaviour from a reward.

Background:

- `docs/plans/mujoco_warp_mr_stabs_plan.md`: the rigid-body model and the CMA-ES fit.
- `docs/sysid_guide.md`: how the 2026-10-15 plant was fitted and scored.
- `docs/plans/learned_sim_environment_plan.md`: sensing model, scorer and sim-to-real gates. This
  plan reuses its layers 2, 4 and 5.
- `docs/plans/opponent_motion_estimation_plan.md`: the 6-state opponent filter with heading, yaw
  rate and uncertainty, and its batched Python mirror for training. A hard dependency: section 3
  reads its output and section 4 runs its mirror.

## Requirements

1. Play aggressively. Holes, wall blocks and the house bot cost reward, but a policy that wins
   more hits for an occasional hazard risk is better than one that hangs back. Hits landed per
   hit taken is the primary score; hazard avoidance is secondary.
2. Stay out of the opponent's weapon arc, using its predicted heading, not just its current one.
3. Make contact with our wedge. Mr Stabs Mk2 is a wedge, so the front is the strike surface:
   meet the opponent front first, under its rear when the rear is reachable, under its side when
   it is not. Contact on our sides or rear earns nothing.
4. Handle perception the way it is: dropouts with p90 340 ms gaps, 180 degree heading flips,
   4 to 9 mm median position error, 40 to 80 ms of latency.
5. Handle the plant the way it is: the 2026-10-15 fit is accurate to 13 mm at 0.4 s and drifts
   past 1 s; braking, nose lift and floor grip are the least trusted parts.
6. Run inside the 250 Hz control loop with no measurable cost (target under 50 us per inference
   on the Orin NX CPU).
7. Every observation the C++ side builds must match the Python training side to float
   tolerance, checked by a test.

## What exists

| Item | State | Where |
| --- | --- | --- |
| Navigation seam | `update(RobotDescriptionsStamped, FieldDescription, const TargetSelection&) -> VelocityCommand`, registered with `REGISTER_CONFIG` | `include/navigation/navigation_interface.hpp`, `src/navigation/config.cpp` |
| Hazards as data | `FieldDescription::hazards`: discs with `center`, `object_radius` (raw), `inflated_radius`, `hard_radius`, `source` (`STATIC` or `TRACKED`) and velocity, assembled from `[field_filter].hazards_file` plus the tracked house bot | `include/data_structures/field.hpp`, `include/hazards/` |
| Hazard layers | `HazardAvoidance::steer_around`, `limit_command`, `apply_reverse`. Not injected: `MotionProfileNavigation` and `PursuitNavigation` each own one as a `hazards_` member built from their own config; `NoopNavigation` and `FixedVelocityNavigation` have none | `include/navigation/hazard_avoidance.hpp` |
| Command path | `VelocityCommand` (normalized linear and angular) -> `DifferentialDriveProcessor` saturation -> OpenTX slew limit (`max_motor_rpm_per_sec`) -> CRSF -> firmware mixer with yaw-rate loop | `src/transmitter/` |
| Firmware mirror | `FirmwareMixer` with `YawController`, heading hold, reverse gain scaling; scalar Python | `auto_battlebot/mujoco_sim/firmware.py` |
| Batched sim | `WarpRollout`: N worlds on the GPU, captured step graph, tape-driven (open loop only) | `auto_battlebot/mujoco_sim/rollout.py`, `warp_kernels.py` |
| Closed-loop CPU sim | `ClosedLoopSim`: sticks through `FirmwareMixer`, arena walls, wall blocks, moving blocks | `auto_battlebot/mujoco_sim/closed_loop.py` |
| Lockstep sim server | Runs the real C++ binary against a Python plant, `MujocoPlant` backend, opponents `static`, `straight`, `circle`, `random_walk`, `replay`, `EPISODE` line | `simulation/kinematic_sim_server.py`, `simulation/plants/` |
| Plant fit | `fit_2026-10-15_yolo_brake`, 6 configs installed from its distilled table | `playground/calibration/out/mujoco_sysid/` |
| Robot keypoints | Front and back keypoints per robot; opponent heading is the last measurement's rotation; `opponent_mode` is `HOLD`, so no opponent velocity extrapolation | `include/robot_filter/` |
| Ground-truth rig | ZED Box tripod over the apartment floor, AprilTag on Mr Stabs Mk2 (ids 41, 76), YOLO fill, `smooth_tag_poses.py` bundles | `config/mr_stabs_mk2_sysid_zed_box.toml`, `docs/sysid_guide.md` |

Gaps this plan fills:

1. `WarpRollout` replays fixed tapes. RL needs the command chosen each step from the world's own
   state, so the firmware mixer, the transmitter slew and the delay must run inside the GPU step.
2. No opponent body, weapon zone or contact classification in the batched sim.
3. No sensing model. The sim knows true poses; the policy on the robot sees EKF tracks with
   dropouts, flips and latency.
4. No apartment-floor recordings of autonomous driving with ground truth. The 2026-10-10 AER
   files have no video and the track came from the same perception the nav steered by.

## Design

### 1. Data flow

```
                      training (Python, GPU)                         deployment (C++, Orin NX)
true world state ──> sensing model ──> observation builder        robots + field + target
        ^                                     │                          │
        │                              policy (MLP, 50 Hz)          observation builder (same spec)
        │                                     │                          │
MuJoCo Warp step <── firmware mixer <── slew + delay <── action     policy (same weights, mean action)
                                                                         │
                                                       VelocityCommand -> transmitter (no hazard layer)
```

The observation builder is specified once (section 3) and implemented twice, in torch for
training and in C++ for deployment. A golden fixture keeps them equal.

### 2. Action and timing

- **Action:** `[linear, angular]` in [-1, 1], the same normalized `VelocityCommand` fields every
  navigation emits. Training samples from a tanh-squashed Gaussian; deployment uses the mean.
- **Policy rate:** 50 Hz (every fifth tick of the 250 Hz loop). The loop repeats the last action
  between policy steps. 50 Hz is above the 30 to 60 Hz perception rate, so every new frame is
  seen within 20 ms, and it keeps the history buffers short.
- **What runs after the action:** on the robot, `DifferentialDriveProcessor` saturation, the
  OpenTX slew limiter and the firmware. In sim, the same three, ported to Warp kernels (section 5).
  Nothing in navigation edits the action.

### 3. Observation

All positions and velocities are in our body frame (x forward, y left), so the policy never sees
absolute field coordinates and cannot memorize an arena. Scales in brackets are the divisors
applied before the network.

**Our robot (6):**

| Field | Count | Notes |
| --- | --- | --- |
| forward speed [2.5 m/s] | 1 | from the EKF track velocity |
| yaw rate [15 rad/s] | 1 | |
| seconds since our last measurement [0.5 s] | 1 | 0 on a fresh frame |
| our track stale flag | 1 | `is_stale` |
| behaviour mode one-hot | 2 | ATTACK, RUN_AWAY from `TargetSelection::mode` |

**History (32):** the last 8 policy steps (160 ms, longer than the worst-case 80 ms latency) of
the action taken (2 each) and of the measured forward speed and yaw rate (2 each). This is what
lets the policy infer the day's plant: a reverse veer, a sagging battery, a slick floor (RMA,
Kumar et al. 2021).

**Target opponent (19):** the robot that `TargetSelection` picked, as rendered by the 6-state
opponent filter (`opponent_motion_estimation_plan.md`, section 6 holds the field contract).

| Field | Count | Notes |
| --- | --- | --- |
| position | 2 | [2 m] |
| distance | 1 | [2 m] |
| bearing sin, cos | 2 | |
| opponent heading relative to ours, sin and cos | 2 | filtered `theta` |
| opponent velocity | 2 | [2.5 m/s], wall-stopped |
| opponent yaw rate | 1 | [15 rad/s], filtered `omega` |
| position, heading and yaw-rate sigmas | 3 | scaled and capped at 1, so "unknown" reads as 1 |
| footprint half-diagonal | 1 | [0.2 m] |
| seconds since its last measurement | 1 | [0.5 s] |
| stale flag | 1 | |
| weapon shape one-hot | 2 | `FRONT_ARC`, `FULL_CIRCLE` |
| weapon half-angle | 1 | [pi]; 0 for `FULL_CIRCLE` |

Weapon shape and half-angle come from a config table keyed by opponent label
(`[navigation.opponent_weapons]`), with a `WeaponShape` enum in `include/enums/weapon_shape.hpp`.
Unlisted labels default to `FRONT_ARC` at 60 degrees. A label on the filter's heading-blind list
is always `FULL_CIRCLE`, whatever the table says.

The heading sigma is how the policy learns to widen its idea of the weapon arc when the
opponent's front is uncertain. Nothing hand-codes that widening.

**Range rays (48):** 24 rays at 15 degree spacing from our axle centre, range capped at 1.5 m,
two channels per ray:

- solid: the nearest wall, wall block or house bot along the ray
- hole: the nearest hole edge along the ray

Value is `distance / 1.5`, 1.0 when nothing is in range. Rays give the policy the free space
around it in a form that does not depend on how many hazards the arena has.

**Nearest hazards (36):** the 4 nearest hazard discs by edge distance, each:

| Field | Count |
| --- | --- |
| centre position | 2 |
| raw radius (`object_radius`) [0.3 m] | 1 |
| kind one-hot: hole, wall block, tracked | 3 |
| velocity | 2 |
| present flag | 1 |

The discs give exact geometry and the house bot's velocity, which rays cannot. The builder uses
`object_radius`, never `inflated_radius`, so changing the inflation config cannot shift the
policy's inputs.

**Total: 141 floats.** The spec lives in `auto_battlebot/rl/obs_spec.py` with a version string.
The exported policy file carries the version and the C++ side refuses to load a mismatch.

### 4. Sensing model

The policy on the robot sees filtered tracks, rendered at now plus `render_lead_s()`, not true
poses. Training feeds it the same thing. The two robots are handled differently:

**Opponent: measurement model plus the real filter.** True opponent pose becomes a simulated
keypoint measurement, which goes through the batched Python mirror of the 6-state opponent filter
(`opponent_motion_estimation_plan.md`, section 7). The mirror is parity-tested against the C++,
so the policy trains against the estimator it runs with: its lag, its coast, its sigmas and its
response to flips. Only the measurement needs a fitted model.

**Our robot: fitted error model.** The our-robot EKF runs the plant model inside it and is not
mirrored. Its rendered output is approximated as truth plus a fitted error: per axis, bias and
AR(1) noise with its standard deviation by speed bin. During a dropout gap the sim coasts at
constant velocity from the last estimate, capped at `max_coast_s` (0.2 s), then sets the stale
flag, which is what the EKF does.

**Measurement model, both robots,** fitted from the apartment-floor recordings (section 9)
against the AprilTag ground truth:

- position and heading noise by speed bin
- heading flips as a 180 degree event rate by speed (and by label, for opponents)
- dropouts: semi-Markov measured and dropped states, gap lengths from the empirical CDF,
  conditioned on the distance between the robots and to the walls (where tracks are lost)
- latency: sampled per episode from 40 to 80 ms, applied as a measurement delay
- frame rate: a new measurement every 1/30 to 1/60 s, sampled per episode

Output is `sensing_model.toml`, written by `playground/rl/fit_sensing_model.py` together with the
opponent filter's evaluation (its step 5), loaded by the training environment. This is layer 2
of `learned_sim_environment_plan.md`, fitted on the new recordings rather than MassD.

### 5. Training environment

`auto_battlebot/rl/env.py`: a batched environment on MuJoCo Warp, 2,048 to 8,192 worlds.

**World contents:**

- Mr Stabs Mk2: the fitted MJCF from `build_mjcf`, as in `WarpRollout`.
- Arena: four walls and up to 3 wall blocks as mocap bodies, so each world places its own walls
  per episode without per-world model geometry. The arena is drawn per episode: rectangles
  1.5 to 2.5 m a side.
- Holes: not geometry. A Warp kernel tests the chassis centre against each hole lip, as
  `HazardMonitor` does, and ends the episode on a fall.
- House bot: 0 or 1 per episode, a mocap cylinder on a slow random walk.
- Opponent: a mocap body with a box footprint from its label's size, driven kinematically by the
  opponent behaviour (section 6). Kinematic means it pushes us with infinite mass; see Risks.

**Per-step chain, all in Warp kernels inside one captured graph:**

1. Action buffer (written by the policy every 10 physics steps at the 2 ms step, 50 Hz).
2. Saturation and the OpenTX slew limit, ported from `opentx_transmitter.cpp`.
3. Command latency ring buffer, length drawn per world.
4. Firmware mixer and yaw loop, ported from `firmware.py` to a Warp kernel. Same gains, same
   reverse scaling, same idle threshold. The gyro input is the sim's yaw rate.
5. Drivetrain delay and command shaping (`shape_command`: deadzone, curve, side gains, pack
   voltage), then the existing motor `control` kernel, changed to read the per-world command
   instead of a tape.
6. `mjw.step`.
7. Contact classification, hole test, flip test, reward and done flags.

**Physics step:** 1 ms today. Benchmark 2 ms against 1 ms on the 2026-10-15 holdout windows
(step 3 below). Use 2 ms if position error at 0.4 s changes by under 2 mm.

**Contact classification:** for each contact between our chassis geoms and the opponent body:

- `phi`: bearing of the contact point from the opponent's centre, relative to its true heading.
  `|phi| = 0` is dead ahead, `pi` directly behind.
- inside the opponent's weapon arc if `|phi| < half_angle`, or always for `FULL_CIRCLE`
- our strike if the contact point lies on the wedge: body x above `our_strike_x_m` and within the
  wedge's half-width. Measure both from the Mr Stabs Mk2 collision hulls
  (`simulation/assets/robots/mr_stabs_mk2/collision/`)

### 6. Opponents

Each world draws one behaviour per episode. Every behaviour has a heading, so the weapon arc is
exercised.

| Behaviour | Parameters drawn per episode | What it trains |
| --- | --- | --- |
| `static` | pose | approach and strike basics |
| `straight`, `circle` | speed, radius | leading a moving target |
| `random_walk` | speed, turn rate | unpredictable motion |
| `pursuit` | lead time 0 to 0.3 s, reaction delay 50 to 250 ms, top speed 0.8 to 2.5 m/s, turn rate | a weapon-facing chaser; the rear is hard to reach |
| `corner` | which corner; backs in, keeps facing out | side strikes against a cornered opponent |
| `spinner` | speed; `FULL_CIRCLE` | no safe angle; contact must be worth it or avoided |
| `replay` | a human opponent track from NHRL, MassD, the 2026-10-10 Mrs Buff Mk3 runs and the new recordings | real driving |

Hold out 20% of the `replay` tracks and one parameter corner of `pursuit` (the fastest top speed
with the shortest delay) as the evaluation set. They are never trained on.

### 7. Reward

Per policy step (20 ms):

| Term | Value | Notes |
| --- | --- | --- |
| Wedge strike, outside the arc | `+(3 + 7 * smoothstep(u))` | contact on our wedge only. `u = (|phi| - half_angle) / (pi - half_angle)`: 3 for a side hit, 10 for dead rear. First contact per engagement only; an engagement ends after 0.5 s apart |
| Wedge push | `+1 per s` | wedge in contact with the opponent outside its arc, after the strike: keeps pushing instead of backing off |
| Hit taken | `-10` | any contact inside the opponent's arc, whichever part of us touched |
| Time in the danger zone | `-1 per s` | inside reach of the opponent's arc without touching |
| Approach shaping | `gamma * Phi(s') - Phi(s)` with `Phi = -dist(us, stage point) / 1 m` | stage point 0.25 m behind the opponent along its heading. Potential-based, so it speeds learning without changing what the best policy is (Ng et al. 1999) |
| Hole | `-10`, episode ends | one rear hit pays for one fall, so the policy takes the risk when a hit is likely and not otherwise |
| Wall block, house bot contact | `-0.5` per contact event | |
| Wall contact | `-0.2` per event, waived while the wedge is on the opponent | pinning the opponent against a wall is a win, not a wall hit |
| Flip | `-20`, episode ends | upside down or pitch past 80 degrees for 0.5 s |
| Nose lift | `-0.5 per s` | pitch past 30 degrees nose down |
| Command change | `-0.01 * |a_t - a_{t-1}|` | smooths chatter |

RUN_AWAY episodes (20% of episodes, mode flag set): strike terms off, `Phi` uses the distance
to the opponent's arc instead (larger is better), every hazard and flip term stays.

Episodes last 20 s and truncate with bootstrapping. Weights above are starting values; change
them only from evaluation results (section 10), never from training curves alone.

### 8. Domain randomization

Every episode draws a plant, a sensing setting and an arena. Starting ranges, to be revised by
the closed-loop gate (section 10, G2):

| Quantity | Range | Source |
| --- | --- | --- |
| `skid_mu`, `wheel_mu_slide`, `wheel_mu_torsion` | 0.6x to 1.5x the fit | floor varies between venues; NHRL is not the apartment |
| effective track width | 0.9x to 1.1x | traded against grip in the 2026-10-15 fit |
| efficiency | 0.68 to 0.80 | restart spread |
| motor resistance | 0.85x to 1.15x | |
| `armature_scale` | 0.9 to 1.2 | |
| joint damping | full search range | restarts disagreed by 45% of the range |
| CG height, pitch inertia | 0.9x to 1.2x | makes nose lift more likely than the fit says; the fit under-predicts it (29 real-only lift windows of 77) |
| reverse veer | one side's reverse gain 0.85 to 1.0 | the firmware held 1.12 to 1.6x L/R in reverse on 2026-10-15 |
| pack voltage | across the logged pack range | battery sag |
| command latency | 40 to 80 ms total | |
| perception | section 4 model, error scale 0.7x to 1.5x | |
| opponent size, speed | per label table, 0.8x to 1.2x | |
| arena | 1.5 to 2.5 m sides, 0 to 3 hazards of radius 0.1 to 0.3 m, 0 or 1 house bot | |

### 9. Recordings on the apartment floor

These recordings serve three uses: fitting the sensing model (section 4), the closed-loop
sim-to-real gate (G2), and later the policy's own real-world evaluation (G6).

**Setup:**

- ZED Box on the tripod rig used for the 2026-10-15 sysid sessions, field locked with the floor
  marker array (ids 160 to 174), then the array removed.
- Live perception is the match path: the YOLO keypoint model for both robots, the EKF, and
  `MotionProfileNavigation` driving. Ground truth comes from the AprilTags, detected offline from
  the recorded video, so the nav sees the same perception it would in a match.
- Video must be in the MCAP. Run `check_sysid_recordings.py` on the first file before recording
  the rest; the 2026-10-10 AER files lost their video to the broken decoder.
- Transmitter type `Esp32DiagnosticsOpenTxTransmitter`, so the firmware's yaw loop and
  per-motor commands are logged alongside the nav.
- Tags on both robots: Mr Stabs Mk2's existing tag and one on Mrs Buff Mk3.
- Hazards: a hazards file for the apartment floor (`config/hazards/apartment_floor.toml`) with
  two taped circles on the floor. They will not swallow the robot, but every entry is scored
  from the ground truth.

**Sessions** (about 10 minutes each, three of them so one can be held out):

1. `MotionProfileNavigation` against Mrs Buff Mk3 placed at 8 to 10 static spots, including
   near the taped hazards and against a wall.
2. `MotionProfileNavigation` against Mrs Buff Mk3 driven by hand: straight runs, circles, turning
   to face, backing into a corner.
3. A repeat of session 2 on a different battery charge.

The baselines run with their hazard layers as they are today, since they are what the policy has
to beat.

### 10. Evaluation gates

| Gate | Passes when | Decides |
| --- | --- | --- |
| G1 sensing model | sim gap-length CDF and position error p50 and p90 within 20% of the held-out session | whether section 4 is trusted |
| G2 closed-loop sim-to-real | `MotionProfileNavigation` in sim, run from each real segment's start state against the replayed opponent, matches the real time to target, overshoot, terminal distance and hazard entries: each real metric inside the sim's 10th to 90th percentile under randomization | the randomization ranges: widen any range whose metric falls outside the band, then re-check |
| G3 parity | C++ observation builder and MLP output equal the Python ones on a golden fixture to 1e-5 | whether a sim result can be trusted on the robot |
| G4 lockstep | the C++ binary with `RlPolicyNavigation`, run in `simulation/kinematic_sim_server.py` with the `MujocoPlant` backend, scores within 10% of the Python evaluation on the same seeds | catches anything the parity fixture misses: timing, the 250 Hz hold, the transmitter |
| G5 sim evaluation | on the held-out opponents and arenas, against `MotionProfileNavigation`: more wedge hits per hit taken, fewer seconds in the danger zone. Hole falls and wall-block contacts are reported, not gated | whether it goes on the robot |
| G6 real | on the apartment floor against hand-driven Mrs Buff Mk3, with the same metric code: wedge hits per hit taken at least the baseline's. Taped-hazard entries are reported next to the hit count | whether it replaces `MotionProfileNavigation` in the match config |

The metric code lives in `auto_battlebot/rl/metrics.py` and reads recordings and sim rollouts
alike, as `learned_sim_environment_plan.md` layer 4 describes.

### 11. Training

- **Algorithm:** PPO with a squashed Gaussian policy, clipped objective, GAE (lambda 0.95,
  gamma 0.99), 24 steps per world per update. Use `rsl_rl` (ETH's PPO used with Isaac Lab),
  pinned in `pyproject.toml` on x86_64 only. If its runner does not fit the Warp environment in
  a day, write a 300-line PPO in `auto_battlebot/rl/ppo.py` instead.
- **Networks:** actor MLP 141 -> 256 -> 256 -> 128 -> 2, ELU. Critic same shape with privileged
  inputs added: true poses, true opponent heading, the episode's plant parameters, true time to
  the next dropout end. The critic is dropped at export.
- **Observation normalization:** running mean and std, frozen at export and stored in the policy
  file.
- **Curriculum:** advance a stage when the rolling success rate passes 70%.

  | Stage | Opponents | Hazards | Sensing | Randomization |
  | --- | --- | --- | --- | --- |
  | A | `static` | none, 2.5 m arena | true state | nominal plant |
  | B | adds `straight`, `circle`, `random_walk` | up to 1 hole | true state | half ranges |
  | C | adds `pursuit`, `corner` | full | section 4 model | half ranges |
  | D | adds `spinner`, `replay` | full | section 4 model | full ranges |

- **Compute:** the sysid fit ran about 1e6 physics steps per second on the RTX 4080. At 2 ms
  physics and 50 Hz policy that is about 1e5 policy steps per second, so 1e9 policy steps take
  about 3 hours. This is an estimate; step 4 measures it. Develop on the laptop, run long jobs on
  megamind through the queue:

  ```bash
  venv/bin/python training/gpu_queue.py submit --name rl_nav_D --by <agent> \
    --profile rl_nav_ppo --eta 12h -- \
    venv/bin/python training/rl/train_nav_policy.py --stage D --resume <run>
  ```

  The first run has no history, so `--eta 12h`. RL runs on one GPU; the queue still runs one job
  at a time.
- **Outputs:** `runs/rl_nav/<timestamp>/` with checkpoints, training curves and the exported
  policy file.

### 12. RlPolicyNavigation in C++

**Files:**

| File | Holds |
| --- | --- |
| `include/navigation/rl_policy_navigation.hpp`, `src/navigation/rl_policy_navigation.cpp` | the navigation |
| `include/navigation/rl_observation.hpp`, `src/navigation/rl_observation.cpp` | observation builder, a pure function of (robots, field, target, history) |
| `include/navigation/mlp_policy.hpp`, `src/navigation/mlp_policy.cpp` | loader and forward pass with Eigen; no TensorRT for a 100k-parameter MLP |
| `include/enums/weapon_shape.hpp` | `WeaponShape { FRONT_ARC, FULL_CIRCLE }` |
| `tests/navigation/test_rl_observation.cpp`, `test_mlp_policy.cpp` | golden-fixture parity, spec version check |

**Behaviour per tick:**

1. Find our robot and the target opponent in `robots`. If our robot has no pose, return zero and
   log `no_robot`, as the other navigations do.
2. Every fifth tick: build the observation, run the MLP, store the mean action and push it into
   the history.
3. Return the stored action as the `VelocityCommand`.

**No `HazardAvoidance`.** `RlPolicyNavigation` has no `HazardAvoidance` member and calls none of
its methods or the free functions behind them: no `steer_around`, no `limit_command`, no
`apply_reverse`, no `push_out_of_hazards`, no `apply_wall_reverse`. The policy's action goes to the
transmitter unedited. Hazards reach the policy only as observations: `update()` receives
`FieldDescription::hazards` like every navigation, and the observation builder reads it.

`HazardAssembler` keeps running, since it builds that list. `MotionProfileNavigation` and
`PursuitNavigation` keep their own `HazardAvoidance` unchanged, as the baselines.

**Config** (`RlPolicyNavigationConfiguration` in `include/navigation/config.hpp`, registered as
`"RlPolicyNavigation"`):

```toml
[navigation]
type = "RlPolicyNavigation"
policy_file = "data/models/nav_policies/rl_nav_<run>.bin"
policy_period_ticks = 5
our_strike_x_m = 0.08

[navigation.opponent_weapons]
mrs_buff_mk3 = { shape = "FRONT_ARC", half_angle_deg = 60 }
opponent = { shape = "FRONT_ARC", half_angle_deg = 60 }
```

**Policy file:** a small binary written by `auto_battlebot/rl/export.py`: a JSON header (spec
version, layer sizes, activation, normalization stats, training run id, git hash) followed by
float32 weights. The loader checks the spec version and layer sizes.

**Diagnostics** (`rl_policy_nav` module): action, value estimate (export the critic's value head
only if it uses no privileged input; otherwise omit), inference time, observation age, nearest
hazard edge distance, whether the target opponent is stale, policy run id.
`NavigationVisualization::path` shows our pose to the target, as today.

## Steps

0. **Record** the three apartment-floor sessions (section 9). Write the apartment hazards file
   first. Check the first file has video.
1. **Ground truth and sensing model.** Offline tag detection on the new videos for both robots,
   bundles through `smooth_tag_poses.py`, then `fit_sensing_model.py`. Gate G1.

   **Opponent filter, in parallel:** `opponent_motion_estimation_plan.md` steps 1 to 6. Its
   step 3 (Python mirror) must finish before step 4 here, its step 5 (evaluation and noise fit,
   which uses these same recordings) before step 7, and its step 6 (enable `KALMAN`) before
   steps 5 and 9. That plan's step order table has the details.
2. **Observation spec and parity.** `obs_spec.py`, the torch builder, the C++ builder, the golden
   fixture. Gate G3 for the builder. This comes early because every later step trains against
   the spec.
3. **Firmware and transmitter on the GPU.** Warp kernels for the slew, delay, mixer and yaw
   loop. Test against the scalar `FirmwareMixer` on the logged sticks of the 2026-10-15 sessions:
   per-motor command RMS difference under 0.5%. Benchmark 1 ms against 2 ms physics.
4. **Environment.** Walls, hazards, opponents, the opponent filter mirror fed by the measurement
   model, contact classification, rewards, randomization, resets. Measure policy steps per second at 2,048, 4,096 and 8,192 worlds.
5. **Closed-loop gate G2.** A Warp or CPU port of `MotionProfileNavigation` is not needed: run
   the real C++ binary in the lockstep sim with `MujocoPlant` and the sensing model, from the
   real segments' start states. Set the randomization ranges from the result.
6. **Train stages A and B.** Check by video (the overlay renderer's style, top-down) that the
   policy drives at the target and strikes. Fix reward bugs here, before adding hazards.
7. **Train stages C and D.** Evaluate on the held-out set against `MotionProfileNavigation`.
   Gate G5.
8. **C++ navigation.** `RlPolicyNavigation`, config, factory, diagnostics, export. Gate G3 for
   the MLP, then G4 in the lockstep sim.
9. **Real runs.** Apartment floor, first against a static Mrs Buff Mk3 with `action_scale` 0.5
   in the transmitter saturation, then full speed against a hand-driven one. Gate G6. Write-up in
   `docs/experiments/control_improvement/`.
10. **Retrain on real data.** Add the G6 recordings to the sensing model and the `replay`
    opponents, refit the randomization ranges from any metric that fell outside the sim band,
    retrain from the last checkpoint.

## Risks and open questions

- **No hazard layer, by choice.** The policy is the only thing between the robot and a hole, and
  the reward is set to take that risk for hits. If hole falls in G5 or taped entries in G6 cost
  more matches than the extra hits win, raise the hole penalty and retrain; the penalty is the
  one dial. The driver's trainer-enable switch remains the kill.
- **Sim exploits.** A policy finds whatever the sim gets wrong. Most likely candidates: braking
  without nose lift, grip on turns, and pushing a kinematic opponent. The CG and grip ranges in
  section 8 and the flip penalty cover the first two. Watch stage D videos for wall-riding and
  shoving.
- **Kinematic opponent.** A mocap opponent has infinite mass, so pushing it never works and being
  pushed always works. That is conservative for us. If G6 shows the policy avoiding contact the
  real robot would win, give the opponent a free body with a simple drive.
- **Opponent heading quality.** Flips put the weapon arc on the wrong side. The sensing model
  injects flips at the measured rate, so the policy learns caution when heading jumps; check on
  recordings of new opponents that the flip rate is in the trained range.
- **Observation drift between C++ and Python.** Any change to the spec bumps its version, which
  breaks the loader until the policy is retrained. That is intended.
- **NHRL floor.** All recordings are from the apartment floor. The friction ranges in section 8
  are a guess for the arena; the first NHRL outing should record a session for G2 on that floor.
- **The opponent filter is a hard dependency.** Today `opponent_mode = HOLD` gives no velocity,
  yaw rate or sigmas. The policy is trained against the 6-state filter through its mirror, so a
  policy must never run with `HOLD`: `RlPolicyNavigation` refuses to initialize unless
  `opponent_mode = "KALMAN"`. Any later change to the filter, including phase 2, means a
  retrain.

## Next steps

1. Write `config/hazards/apartment_floor.toml` and tape the two circles.
2. Read the wedge's leading edge and half-width off the collision hulls for `our_strike_x_m`.
3. Record session 1 and check it has video before recording sessions 2 and 3.
