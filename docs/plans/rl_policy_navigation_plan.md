# RL policy navigation plan

Goal: a navigation type, `RlPolicyNavigation`, that drives Mr Stabs Mk2 with a neural network
policy trained in the MuJoCo sim. It hits the opponent from behind or the side, stays out of the
opponent's front arc, and avoids walls and hazards because it was trained to, not because a
separate layer clamps its output. Hazards are policy inputs. `RlPolicyNavigation` does not use
`HazardAvoidance` (section 12).

This plan replaces `docs/plans/plant_rollout_navigation_plan.md`. The rollout planner needed an
accurate plant online and a hand-built mode layer (STAGE, COMMIT, EXIT, CONTAIN) with tuned cost
weights. A policy needs the plant only during training and learns the mode behaviour from a reward.

Background:

- `docs/plans/mujoco_warp_mr_stabs_plan.md`: the rigid-body model and the CMA-ES fit.
- `docs/sysid_guide.md`: how the 2026-10-15 plant was fitted and scored.
- `docs/plans/sensing_model_plan.md`: the measurement model and our-robot EKF error model the
  training environment samples. Steps 1 to 4 done 2026-10-16.
- `docs/plans/learned_sim_environment_plan.md`: scorer and sim-to-real gates. This plan reuses
  its layers 4 and 5.
- `docs/experiments/kalman_filter/opponent_motion_estimation_report.md`: the 6-state opponent filter with heading, yaw
  rate and uncertainty, and its batched Python mirror for training. A hard dependency: section 3
  reads its output and section 4 runs its mirror.

## Requirements

1. Play aggressively. Holes, wall blocks and the house bot cost reward, but a policy that wins
   more hits for an occasional hazard risk is better than one that hangs back. Hits landed per
   hit taken is the primary score; hazard avoidance is secondary.
2. Stay out of the opponent's front arc, using its predicted heading, not just its current one.
   Every opponent is treated the same: its weapon covers 60 degrees either side of its heading.
   The half-angle is one constant in the training environment, not a policy input or a config
   field.
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
8. Swapping the training method, the policy network or the C++ inference library must not
   change `RlPolicyNavigation`. Only what a bake-off needs gets built; the rest are slots
   (sections 11 and 12).

## What exists

| Item | State | Where |
| --- | --- | --- |
| Navigation seam | `update(RobotDescriptionsStamped, FieldDescription, const TargetSelection&) -> VelocityCommand`, registered with `REGISTER_CONFIG` | `include/navigation/navigation_interface.hpp`, `src/navigation/config.cpp` |
| Hazards as data | `FieldDescription::hazards`: discs with `center`, `object_radius` (raw), `inflated_radius`, `hard_radius`, `source` (`STATIC` or `TRACKED`) and velocity, assembled from `[field_filter].hazards_file` plus the tracked house bot | `include/data_structures/field.hpp`, `include/hazards/` |
| Hazard layers | `HazardAvoidance::steer_around`, `limit_command`, `apply_reverse`. Not injected: `MotionProfileNavigation` and `PursuitNavigation` each own one as a `hazards_` member built from their own config; `NoopNavigation` and `FixedVelocityNavigation` have none | `include/navigation/hazard_avoidance.hpp` |
| Command path | `VelocityCommand` (normalized linear and angular) -> `DifferentialDriveProcessor` saturation -> OpenTX slew limit (`max_motor_rpm_per_sec`) -> CRSF -> firmware mixer with yaw-rate loop | `src/transmitter/` |
| Firmware mirror | `FirmwareMixer` with `YawController`, heading hold, reverse gain scaling; scalar Python | `auto_battlebot/mujoco_sim/firmware.py` |
| Grey-box plant | first-order lag drivetrain model in numpy; fitted parameters in each profile's `[plant]` | `auto_battlebot/control/plant.py` |
| Batched sim | `WarpRollout`: N worlds on the GPU, captured step graph, tape-driven (open loop only) | `auto_battlebot/mujoco_sim/rollout.py`, `warp_kernels.py` |
| Closed-loop CPU sim | `ClosedLoopSim`: sticks through `FirmwareMixer`, arena walls, wall blocks, moving blocks | `auto_battlebot/mujoco_sim/closed_loop.py` |
| Lockstep sim server | Runs the real C++ binary against a Python plant, `MujocoPlant` backend, opponents `static`, `straight`, `circle`, `random_walk`, `replay`, `EPISODE` line. With a sensing model and `SimKeypointModel`, the real keypoint filter runs on simulated measurements | `simulation/kinematic_sim_server.py`, `simulation/plants/`, `config/simulation/mr_stabs_mk2_kinematic_sim_sensing.toml` |
| Plant fit | `fit_2026-10-15_yolo_brake`, 6 configs installed from its distilled table | `playground/calibration/out/mujoco_sysid/` |
| Opponent filter | 6-state `KALMAN` mode, on in production since 2026-10-16; batched Python mirror parity-tested against the C++ | `include/robot_filter/`, `auto_battlebot/perception/opponent_filter.py` |
| Sensing model | batched `MeasurementModel` and `OurEkfErrorModel`; `provisional.toml` fitted from the 2026-10-15 sysid truth and eight `KALMAN` match replays | `auto_battlebot/perception/sensing_model.py`, `simulation/sensing_models/provisional.toml` |
| Ground-truth rig | ZED Box tripod over the 1.52 m test box, AprilTag on Mr Stabs Mk2 (ids 41, 76), YOLO fill, `smooth_tag_poses.py` bundles | `config/mr_stabs_mk2_sysid_zed_box.toml`, `docs/sysid_guide.md` |

Gaps this plan fills:

1. `WarpRollout` replays fixed tapes. RL needs the command chosen each step from the world's own
   state, so the firmware mixer, the transmitter slew and the delay must run inside the GPU step.
2. No opponent body, front-arc zone or contact classification in the batched sim.
3. The sensing model is provisional: Mr Stabs noise comes from offline YOLO on one sysid session,
   and Mr Stabs stands in for the opponent. Sessions with live detections refit it (section 9).
4. No recordings of live perception with ground truth on the ZED Box, and no second robot for
   closed-loop sessions against an opponent.
5. No evidence yet that a policy beats `MotionProfileNavigation` on this task. The toy test
   (step 1) answers that before the Warp environment is built.

## Design

### 1. Data flow

```
                    training (Python, GPU)                           deployment (C++, Orin NX)
true world state ──> sensing model ──> observation builder        robots + field + target
        ^                                     │                          │
        │                             policy (50 Hz)                ObservationBuilderV1 (same spec)
        │                                     │                          │
MuJoCo Warp step <── firmware mixer <── slew + delay <── action     NetworkPolicy -> RtNeuralRuntime
                                              │                          │   (same weights, mean action)
                                     export: model bundle ─────────>  SquashedDecoder
                                                                         │
                                                       VelocityCommand -> transmitter (no hazard layer)
```

The observation builder is specified once (section 3) and implemented twice, in torch for
training and in C++ for deployment. A golden fixture keeps them equal. The model bundle
(section 12) is the only thing that crosses from training to the robot.

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

**Target opponent (16):** the robot that `TargetSelection` picked, as rendered by the 6-state
opponent filter (`docs/experiments/kalman_filter/opponent_motion_estimation_report.md`, "Contract for RlPolicyNavigation").

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

The heading sigma is how the policy learns to widen its idea of the front arc when the
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

**Total: 138 floats.** The spec lives in `auto_battlebot/rl/obs_spec.py` with a version number.
The model bundle's manifest carries the version and the C++ side refuses to load a mismatch.

### 4. Sensing model

Specified and built in `docs/plans/sensing_model_plan.md`. What the environment uses from it:

- **Opponent:** `MeasurementModel` turns the true pose into a delayed, noisy, sometimes missing
  measurement with heading flips. That measurement goes through the batched mirror of the
  6-state opponent filter, so the policy trains against the estimator it runs with: its lag, its
  coast, its sigmas and its response to flips.
- **Our robot:** `OurEkfErrorModel` adds fitted AR(1) error to truth, coasts through dropouts and
  sets the stale flag after `max_coast_s` (0.2 s), which is what the EKF does.
- **Model file:** `simulation/sensing_models/provisional.toml` until the test-box sessions
  (section 9) refit it. Per-world randomization ranges are in the sensing plan's section 4.
- **Toy environment:** uses the same two classes, so the viability test (step 1) runs on the same
  sensing as the full environment.

Gate G1 is the sensing plan's gates S1, S2 and S4.

### 5. Training environment

`auto_battlebot/rl/envs/warp.py`: a batched environment on MuJoCo Warp, 2,048 to 8,192 worlds.
It is the second `VecEnv` after the toy (section 11), so everything built against the toy runs on
it unchanged.

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
- inside the opponent's front arc if `|phi| < half_angle`, where `half_angle` is 60 degrees for
  every opponent
- our strike if the contact point lies on the wedge: body x above `our_strike_x_m` and within the
  wedge's half-width. Measure both from the Mr Stabs Mk2 collision hulls
  (`simulation/assets/robots/mr_stabs_mk2/collision/`)

### 6. Opponents

Each world draws one behaviour per episode. Every behaviour has a heading, so the front arc is
exercised.

| Behaviour | Parameters drawn per episode | What it trains |
| --- | --- | --- |
| `static` | pose | approach and strike basics |
| `straight`, `circle` | speed, radius | leading a moving target |
| `random_walk` | speed, turn rate | unpredictable motion |
| `pursuit` | lead time 0 to 0.3 s, reaction delay 50 to 250 ms, top speed 0.8 to 2.5 m/s, turn rate | a chaser that keeps its front toward us; the rear is hard to reach |
| `corner` | which corner; backs in, keeps facing out | side strikes against a cornered opponent |
| `replay` | a human opponent track from NHRL, MassD, the 2026-10-10 Mrs Buff Mk3 runs and the opponent sessions (section 9) | real driving |

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
| `skid_mu`, `wheel_mu_slide`, `wheel_mu_torsion` | 0.6x to 1.5x the fit | floor varies between venues; NHRL is not the test box |
| effective track width | 0.9x to 1.1x | traded against grip in the 2026-10-15 fit |
| efficiency | 0.68 to 0.80 | restart spread |
| motor resistance | 0.85x to 1.15x | |
| `armature_scale` | 0.9 to 1.2 | |
| joint damping | full search range | restarts disagreed by 45% of the range |
| CG height, pitch inertia | 0.9x to 1.2x | makes nose lift more likely than the fit says; the fit under-predicts it (29 real-only lift windows of 77) |
| reverse veer | one side's reverse gain 0.85 to 1.0 | the firmware held 1.12 to 1.6x L/R in reverse on 2026-10-15 |
| pack voltage | across the logged pack range | battery sag |
| command latency | 40 to 80 ms total | |
| perception | `MeasurementModel` per-world randomization | `sensing_model_plan.md` section 4 |
| opponent size, speed | per label table, 0.8x to 1.2x | |
| arena | 1.5 to 2.5 m sides, 0 to 3 hazards of radius 0.1 to 0.3 m, 0 or 1 house bot | |

### 9. Recordings

Two kinds, each recorded when its hardware is free. Neither blocks training: the toy test and
stages A to D run on the provisional sensing model and the 2026-10-15 plant. A policy does not go
on the robot before gate G2, which needs the second kind.

**Test-box sensing sessions.** Mr Stabs Mk2 alone, driven by hand, in the 1.52 m test box once it
is free (not before 2026-10-30). They refit the sensing model from live detections (sensing plan
step 5) and give gate S4 its data.

Setup:

- ZED X One S on the tripod about 0.75 m above the box floor, looking straight down over the
  centre (`docs/sysid_guide.md`, "Hardware setup"). At the 2026-10-15 mount (1.04 m up, 23 degrees
  off vertical) the tag fell below its 24 px decode limit on the far side, and frames without the
  tag have no truth.
- A new profile, `config/mr_stabs_mk2_sensing_zed_box.toml`:
  - the match perception path from `_zed_box`: YOLO keypoints, the static gate, the `KALMAN`
    opponent filter
  - `Esp32DiagnosticsOpenTxTransmitter`, so the firmware's yaw loop and per-motor commands are
    logged
  - `NoopNavigation` with autonomy off, as in the sysid profile
  - field size 1.52 m
- Tag 76 stays on the robot. Truth comes from the tag, detected offline by replaying the video
  through `config/playback/mr_stabs_mk2_sysid_playback.toml`, then `smooth_tag_poses.py`.
- Measure the printed tag and correct `tag_size_m` (0.064 m is the CAD value) in both profiles
  before the first session.

Each session:

1. Start recording, put the floor marker array (ids 160 to 174) in the box, press field init,
   take the array out. Re-lock the field at the start of every session: the field offset between
   sessions is unmeasured (sensing plan section 2), and two locks give its first spread.
2. Drive for 10 minutes:
   - full-speed straight runs, forward and reverse; the bin above 1.5 m/s had 671 samples on
     2026-10-15
   - spins in place
   - runs along each wall at 5 to 30 cm from it
   - still for a few seconds at a time in each corner
3. Copy the file and run `playground/calibration/check_sysid_recordings.py` on it before the
   next session. The 2026-10-10 AER files lost their video to the broken decoder.

Record two sessions. The second is held out whole; the gates score against it.

They give:

- Mr Stabs position, heading and flip noise from live detections, which include the Jetson's
  frame drops and timing
- latency, from `frame_meta.image_stamp_ns` to the stamp at which the filter consumed the frame
- frame drops on the ZED Box
- our-robot EKF error by gap age (gate S4)
- our-robot dropouts against the walls

Opponent dropouts keep coming from the NHRL and MassD replays, and Mr Stabs keeps standing in for
opponent noise.

The static gate stays on, because it is on in matches. It removes a detection that stays within
8 cm for 4 s, so the corner stills show up as dropouts. Its `static_detection_gate` diagnostics
let the fit tell those apart from detector misses.

**Opponent sessions.** When a second robot with a tag is available. They feed gate G2, later G6,
and the `replay` opponents. Same profile with `MotionProfileNavigation` and autonomy on, plus a
hazards file for the venue with two taped circles; the circles will not swallow the robot, but
every entry is scored from the ground truth. About 10 minutes each:

1. `MotionProfileNavigation` against the opponent parked at 8 to 10 spots, including near the
   taped hazards, against a wall and in corners. Turn the static gate off for this session
   (`[keypoint_filter.static] enable = false`), or the parked opponent disappears after 4 s and
   the navigation loses its target.
2. `MotionProfileNavigation` against the opponent driven by hand: straight runs, circles, turning
   to face, backing into a corner.
3. A repeat of session 2 on a different battery charge, held out.

The baselines run with their hazard layers as they are today, since they are what the policy has
to beat.

### 10. Evaluation gates

| Gate | Passes when | Decides |
| --- | --- | --- |
| G1 sensing model | the sensing plan's gates S1, S2 and S4 pass on the held-out test-box session, with Mr Stabs standing in for the opponent | whether section 4 is trusted |
| G2 closed-loop sim-to-real | `MotionProfileNavigation` in sim, run from each real segment's start state against the replayed opponent, matches the real time to target, overshoot, terminal distance and hazard entries: each real metric inside the sim's 10th to 90th percentile under randomization. Needs the opponent sessions | the randomization ranges: widen any range whose metric falls outside the band, then re-check |
| G3 parity | the C++ observation builder equals the Python one on a golden fixture to 1e-5, and the exported bundle's self-test passes in C++ | whether a sim result can be trusted on the robot |
| G4 lockstep | the C++ binary with `RlPolicyNavigation`, run in `simulation/kinematic_sim_server.py` with the `MujocoPlant` backend, scores within 10% of the Python evaluation on the same seeds | catches anything the parity fixture misses: timing, the 250 Hz hold, the transmitter |
| G5 sim evaluation | on the held-out opponents and arenas, against `MotionProfileNavigation`: more wedge hits per hit taken, fewer seconds in the danger zone. Hole falls and wall-block contacts are reported, not gated | whether it goes on the robot |
| G6 real | against a hand-driven opponent, with the same metric code: wedge hits per hit taken at least the baseline's. Taped-hazard entries are reported next to the hit count | whether it replaces `MotionProfileNavigation` in the match config |

The metric code lives in `auto_battlebot/rl/metrics.py` and reads recordings and sim rollouts
alike, as `learned_sim_environment_plan.md` layer 4 describes.

### 11. Training

**Toy first.** Before the Warp environment exists, a toy environment answers whether RL is worth
building at all (step 1). `auto_battlebot/rl/envs/toy.py`:

- the grey-box plant from `auto_battlebot/control/plant.py`, ported to torch, with the Mr Stabs
  profile's `[plant]` parameters
- a 2.5 m box and one opponent, `static` first
- about 10 observations: opponent distance and bearing, opponent heading relative to ours, our
  speed and yaw rate, distance to the nearest wall
- the section 7 terms that need no contact geometry: strike from behind or the side, hit taken
  inside the front arc, approach shaping
- 4,096 worlds on the laptop GPU
- optional sensing: `MeasurementModel` on `provisional.toml` and the opponent filter mirror

It is the first `VecEnv`, so the learners, policies, evaluator and exporter built against it
carry over to the Warp environment unchanged.

**Seams.** Python code lives in `auto_battlebot/rl/`. Each seam is a `Protocol` with a registry
keyed by name, the way the C++ factories key on `type`. A training config picks one entry from
each.

| Seam | Contract | Built first | Slots |
| --- | --- | --- | --- |
| Source | `VecEnv`: `reset(seed)`, `step(action)` with auto-reset of finished worlds, `EnvSpec` with named observation tensors and `obs_spec_version`. `OfflineDataset`: `sample(batch)` | `envs/toy.py` | `envs/warp.py`; `data/mcap_dataset.py` for offline methods; gymnasium and rsl_rl env adapters |
| Policy | `act(obs, state, deterministic) -> (action, state)`, `export() -> BundleSpec`. Normalization inside; recurrent state as named tensors; scripted baselines implement it too, so the evaluator scores them with the same code | `mlp_gaussian`, `pursuit` (scripted) | `mlp_deterministic`, `gru_gaussian`, `discrete_q` |
| Learner | built bound to its source; `iterate() -> stats`, `state_dict()`. One `iterate()` is one unit of training, which hides the on-policy, off-policy, offline and evolutionary differences | `ppo`, `fasttd3` | `sac`, `dqn`, `bc`, `iql`, `cmaes`; adapters for rsl_rl, Stable-Baselines3, skrl |
| Exporter | `BundleSpec` to a model bundle directory (section 12). Runs the bundle's self-test in Python before writing it | `export/bundle.py`, `export/rtneural.py` | one writer per new C++ runtime |
| Evaluator | any `Policy` on fixed, seeded held-out scenarios. `metrics.py` also reads lockstep MCAPs, so the C++ path reports the same numbers | `evaluate.py`, `metrics.py` | |

A library that owns its own training loop (rsl_rl's runner, Stable-Baselines3's `learn()`) gets
our environment through an env adapter. Its adapter's `iterate()` runs one of the library's update
rounds, and its trained actor is wrapped as a `Policy`, so evaluation and export are ours either
way.

The training loop names no algorithm, network or simulator:

```python
source = SOURCES[cfg.source.type](cfg.source, device)
policy = POLICIES[cfg.policy.type](source.spec, cfg.policy)
learner = LEARNERS[cfg.learner.type](policy, source, cfg.learner)
while not budget.spent():
    log(learner.iterate())
    if budget.eval_due():
        evaluate(learner.policy, cfg.eval.scenarios)
save_checkpoint(learner.state_dict())
write_bundle(learner.policy.export(), out_dir)
```

- **Algorithm:** PPO with a squashed Gaussian policy, clipped objective, GAE (lambda 0.95,
  gamma 0.99), 24 steps per world per update, written in plain torch in `learners/ppo.py`
  (about 250 lines) so the toy reads end to end. FastTD3 is the second arm on the toy: same
  scenarios, same env-step and wall-clock budgets, three training seeds per arm, median and range
  reported, never the best run. The winner moves to the Warp environment. `rsl_rl` comes later
  through a learner adapter as a reference curve for our PPO, not as the training loop.
- **Networks:** actor MLP 138 -> 256 -> 256 -> 128 -> 2, ELU, tanh on the output. Critic same
  shape with privileged inputs added: true poses, true opponent heading, the episode's plant
  parameters, true time to the next dropout end. The critic is dropped at export.
- **What a policy may contain:** whatever RTNeural runs (section 12): a sequential stack of
  Dense, GRU, LSTM, Conv1D and Conv2D layers with tanh, ReLU, sigmoid, softmax, ELU or PReLU.
  No LayerNorm and no skip connections. Check the FastTD3 reference actor before the bake-off;
  if it uses LayerNorm, train that arm without it, or it needs a second C++ runtime.
- **Observation normalization:** running mean and std, frozen at export and folded into the first
  Dense layer (`W' = W / std` per input column, `b' = b - W' * mean`), so the C++ side needs no
  normalization step. No observation clipping: the spec's scales and caps already bound every
  input.
- **Curriculum:** advance a stage when the rolling success rate passes 70%.

  | Stage | Opponents | Hazards | Sensing | Randomization |
  | --- | --- | --- | --- | --- |
  | A | `static` | none, 2.5 m arena | true state | nominal plant |
  | B | adds `straight`, `circle`, `random_walk` | up to 1 hole | true state | half ranges |
  | C | adds `pursuit`, `corner` | full | section 4 model | half ranges |
  | D | adds `replay` | full | section 4 model | full ranges |

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
  model bundle.

**Python files:**

| File | Holds |
| --- | --- |
| `auto_battlebot/rl/env.py`, `data.py` | `EnvSpec`, `Step`, `VecEnv`, `OfflineDataset` |
| `auto_battlebot/rl/obs_spec.py` | observation layout and version (step 2) |
| `auto_battlebot/rl/envs/toy.py`, `envs/warp.py` | the toy (step 1) and the full environment (step 4) |
| `auto_battlebot/rl/policies/` | `mlp.py`, `scripted.py`, registry |
| `auto_battlebot/rl/learners/` | `ppo.py`, `fasttd3.py`, registry |
| `auto_battlebot/rl/export/` | `bundle.py` (manifest, self-test), `rtneural.py` |
| `auto_battlebot/rl/evaluate.py`, `metrics.py` | scenario sets, batched evaluation, metrics shared with MCAPs |
| `playground/rl_toy/train_toy.py` | toy CLI, one TOML config per arm |
| `playground/rl/make_rl_golden_fixture.py` | writes `tests/rl_policy/rl_policy_golden_data.hpp` |
| `training/rl/train_nav_policy.py` | full-environment CLI, run through the GPU queue |
| `tests/python/test_rl_*.py` | PPO on a two-state toy, export round trip through the self-test |

### 12. RlPolicyNavigation in C++

**Model bundle.** A directory written by `auto_battlebot/rl/export/`:

```
data/models/nav_policies/rl_nav_<run>/
  manifest.toml         policy kind, observation spec, decoder, execution, networks
  actor.rtneural.json   the network, in RTNeural's JSON model format
  selftest.bin          16 observation and action pairs from PyTorch
```

```toml
format_version = 1
run_id = "rl_nav_2026-11-02_ppo_s0"
git_hash = "<commit>"
policy_kind = "NETWORK"
obs_spec_version = 1
execution = "INLINE"
policy_period_ticks = 5

[decoder]
kind = "SQUASHED"
scale = [1.0, 1.0]

[[networks]]
name = "actor"
runtime = "RTNEURAL"
file = "actor.rtneural.json"
inputs = [{ name = "obs.vector", shape = [138] }]
outputs = [{ name = "action", shape = [2] }]

[selftest]
file = "selftest.bin"
tolerance = 1e-4
```

The manifest decides the policy kind, runtime, decoder, observation spec and execution mode.
Config only names the bundle, so a bundle cannot be paired with the wrong decoder or observation
layout.

**Module.** A new module, `rl_policy/`, behind four interfaces. `RlPolicyNavigation` sits on top
and only finds the robots, schedules ticks and logs. Each interface has one implementation now;
the slots are what a later bake-off arm would add, each as a class, an enum value and a test, with
no change to the navigation.

| Interface | Picked by | Built now | Slots |
| --- | --- | --- | --- |
| `NetworkRuntimeInterface`: named float tensors in and out, `reset()`; `run()` never allocates | `networks.runtime` | `RtNeuralRuntime` | hand-rolled Eigen graph (LayerNorm, skip connections), ONNX Runtime, OpenCV DNN, TensorRT, LibTorch |
| `PolicyInterface`: `load(bundle)`, `reset()`, `act(observation) -> ActionCommand` | `policy_kind` | `NetworkPolicy`; `ConstantPolicy`, a fixed action for the bench and the lockstep sim before any trained bundle exists | `PlannerPolicy` (TD-MPC2), `WorldModelPolicy` (DreamerV3) |
| `ActionDecoderInterface`: the network's raw output to a stick command | `decoder.kind` | `SquashedDecoder`: tanh, then scale | `ClippedDecoder`; `DiscreteTableDecoder` for the DQN family |
| `ObservationBuilderInterface`: robots, field, target and history to named tensors, as a pure function | `obs_spec_version` | `ObservationBuilderV1` | one per new spec version; old bundles keep loading |

Execution is `INLINE` only: `act()` runs on the control thread every `policy_period_ticks`,
inside requirement 6's 50 us. A `WORKER` mode for policies that cannot fit (planners, image
inputs) is a slot. It would add up to one policy period of delay, which training would have to
include.

**Recurrent state** lives inside the runtime. RTNeural keeps GRU and LSTM state itself, and
`reset()` zeros it. `NetworkPolicy::reset()` calls it on `initialize()` and whenever the target
opponent changes. A later stateless backend such as ONNX Runtime carries its state tensors inside
its own runtime class, so policies never see the difference.

**RTNeural.** BSD-3-Clause.

- Fetched with `FetchContent` like toml++ and CLI11, pinned to a commit hash.
- Built with `RTNEURAL_EIGEN` against the system Eigen 3, not a bundled copy.
- Included as `SYSTEM`, so its headers stay out of `-Werror`.
- Loaded with `RTNeural::json_parser::parseJson<float>`, which builds the layer stack from the
  file. A new architecture made of supported layers deploys by copying a bundle.
- `export/rtneural.py` writes RTNeural's TensorFlow-style JSON from PyTorch modules, transposing
  Dense weights. When step 5 starts, check the schema against RTNeural's `python/model_utils.py`
  and that `parseJson` accepts ELU as a Dense activation. The self-test catches any layout
  mistake.

**Checks at `initialize()`.** Each throws with the mismatch named, so a bad bundle stops the app
at startup rather than mid-match:

- `format_version`, `policy_kind`, `runtime`, `decoder.kind` and `execution` parse as known
  enum values. The enums list only what is built, so a bundle that needs anything else fails
  here with the valid values listed.
- The builder for `obs_spec_version` exists, and its tensors match the network's inputs by name
  and shape.
- RTNeural's loaded model has the manifest's input and output sizes.
- The self-test passes: every pair in `selftest.bin` goes through the policy and decoder on this
  box and matches PyTorch within the tolerance.
- `opponent_mode` is `KALMAN` (see Risks).

**Behaviour per tick:**

1. Find our robot and the target opponent in `robots`. If our robot has no pose, return zero and
   log `no_robot`, as the other navigations do.
2. Every `policy_period_ticks` ticks: build the observation, call `act()`, and push the action
   into the history.
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
our_strike_x_m = 0.08

[navigation.policy]
bundle = "data/models/nav_policies/rl_nav_<run>"
```

**Diagnostics** (`rl_policy_nav` module): action, inference time, observation age, nearest hazard
edge distance, whether the target opponent is stale, the bundle's `run_id` and self-test result.
`NavigationVisualization::path` shows our pose to the target, as today.

**Files:**

| File | Holds |
| --- | --- |
| `include/rl_policy/tensor.hpp` | `TensorSpec`, `Tensor`, `TensorSet`, allocated once at load |
| `include/rl_policy/policy_bundle.hpp`, `src/rl_policy/policy_bundle.cpp` | manifest parse and checks |
| `include/rl_policy/network_runtime_interface.hpp`, `src/rl_policy/runtime_registry.cpp` | the interface, `make_network_runtime()` |
| `include/rl_policy/runtimes/rtneural_runtime.hpp`, `src/rl_policy/runtimes/rtneural_runtime.cpp` | the only file that includes RTNeural headers |
| `include/rl_policy/policy_interface.hpp`, `src/rl_policy/policy_registry.cpp` | the interface, `make_policy()` |
| `include/rl_policy/policies/network_policy.hpp`, `constant_policy.hpp`, with `.cpp` under `src/rl_policy/policies/` | the two policy kinds |
| `include/rl_policy/action_decoder_interface.hpp`, `decoders/squashed_decoder.hpp`, `src/rl_policy/decoder_registry.cpp` | the interface, `SquashedDecoder`, `make_action_decoder()` |
| `include/rl_policy/observation_builder_interface.hpp`, `observations/observation_v1.hpp`, `src/rl_policy/observations/observation_v1.cpp`, `src/rl_policy/observation_registry.cpp` | the interface, the spec v1 builder, `make_observation_builder()` |
| `include/rl_policy/selftest.hpp`, `src/rl_policy/selftest.cpp` | `run_selftest()` |
| `include/rl_policy/action_history.hpp` | fixed-size ring of past actions |
| `include/rl_policy/config.hpp` | `RlPolicyConfiguration`: the `[navigation.policy]` table |
| `include/enums/policy_kind.hpp` (`NETWORK`, `CONSTANT`), `network_runtime_kind.hpp` (`RTNEURAL`), `action_decoder_kind.hpp` (`SQUASHED`), `policy_execution.hpp` (`INLINE`), plus `include/enums.hpp` | one value per built implementation |
| `include/navigation/rl_policy_navigation.hpp`, `src/navigation/rl_policy_navigation.cpp` | the navigation |
| `include/navigation/config.hpp`, `src/navigation/config.cpp` | `RlPolicyNavigationConfiguration`, `REGISTER_CONFIG`, the `make_navigation()` branch |
| `CMakeLists.txt` | the RTNeural `FetchContent` block |
| `tests/rl_policy/test_policy_bundle.cpp` | wrong version, unknown enum, missing file, shape mismatch: each fails with a named error |
| `tests/rl_policy/test_rtneural_runtime.cpp` | a tiny Dense, ELU, tanh network against hand-computed values; no allocation in `run()` |
| `tests/rl_policy/test_network_policy.cpp` | self-test pass and fail, `reset()` |
| `tests/rl_policy/test_observation_v1.cpp` | golden parity with `obs_spec.py`; stale target, no hazards, our robot against a wall |
| `tests/rl_policy/rl_policy_golden_data.hpp` | generated by `playground/rl/make_rl_golden_fixture.py` |
| `tests/rl_policy/bundles/tiny_mlp/` | a 4-input bundle for the tests |
| `tests/navigation/test_rl_policy_navigation.cpp` | `no_robot`, refuses `HOLD`, a failed self-test refuses to start |

CMake's source glob is recursive and `CONFIGURE_DEPENDS`, so the new directories need no build
file changes beyond the RTNeural block.

**Dependency rules:**

- `RlPolicyNavigation` includes only the four interfaces and `rl_policy/config.hpp`, never a
  concrete policy, runtime, decoder or builder.
- Only `rtneural_runtime.cpp` includes RTNeural. A second backend would get its own `.cpp` and,
  if it is an optional library, a `BUILD_WITH_*` option that follows the `BUILD_WITH_ZED` pattern.
- Nothing in `rl_policy/` includes `navigation/`.

## Steps

0. **Record when the hardware is free** (section 9). Test-box sensing sessions once the box is
   free, not before 2026-10-30; opponent sessions once a second robot is available. Steps 1 to 8
   do not wait on them.
1. **Toy viability test** (section 11). Python seams, `envs/toy.py`, `ppo`, `mlp_gaussian`,
   `pursuit`, the evaluator. Stop at the first no:
   1. With true state, does PPO hit a static opponent from behind or the side in over 90% of
      episodes, within about 30 minutes on the laptop?
   2. Does it beat `pursuit` on hits per hit taken and time to first hit?
   3. With `MeasurementModel` on `provisional.toml` and the opponent filter mirror, does it still
      beat `pursuit`? A no here means trying `gru_gaussian` before going further.
   4. FastTD3 against PPO, three seeds each. The winner's learner goes to step 4.
2. **Observation spec and parity.** `obs_spec.py`, the torch builder, `ObservationBuilderV1`, the
   golden fixture. Gate G3 for the builder. This comes before the environment because every later
   step trains against the spec.
3. **Firmware and transmitter on the GPU.** Warp kernels for the slew, delay, mixer and yaw
   loop. Test against the scalar `FirmwareMixer` on the logged sticks of the 2026-10-15 sessions:
   per-motor command RMS difference under 0.5%. Benchmark 1 ms against 2 ms physics.
4. **Environment.** Walls, hazards, opponents, the opponent filter mirror fed by the measurement
   model, contact classification, rewards, randomization, resets. Measure policy steps per second
   at 2,048, 4,096 and 8,192 worlds. The step 1 learner, policy and evaluator run on it unchanged;
   only the source registry gains `warp`.
5. **C++ skeleton** (section 12). The `rl_policy/` module with `RtNeuralRuntime`,
   `NetworkPolicy`, `ConstantPolicy`, `SquashedDecoder`, the bundle loader, the self-test and
   `RlPolicyNavigation`. Done when `ConstantPolicy` drives in the lockstep sim and a tiny exported
   bundle passes its self-test in C++. Independent of steps 2 to 4.
6. **Train stages A and B.** Check by video (the overlay renderer's style, top-down) that the
   policy drives at the target and strikes. Fix reward bugs here, before adding hazards.
7. **Train stages C and D.** Evaluate on the held-out set against `MotionProfileNavigation`.
   Gate G5.
8. **Export and parity.** Export the winner's bundle. Gate G3 for the full policy, then G4 in the
   lockstep sim.
9. **Sensing refit** from the test-box sessions (sensing plan step 5). Gate G1. Retrain from the
   last checkpoint if the refit moves any sensing quantity outside the ranges trained on.

   **Opponent filter:** done 2026-10-16
   (`docs/experiments/kalman_filter/opponent_motion_estimation_report.md`). Its evaluation used the
   Mr Stabs sysid sessions; rerun `evaluate_opponent_prediction.py` on the opponent sessions so the
   tuned noise comes from a real opponent.
10. **Closed-loop gate G2, then real runs.** Both need the opponent sessions. For G2, run the real
    C++ binary with `MotionProfileNavigation` in the lockstep sim with `MujocoPlant` and the sensing
    model, from the real segments' start states, and set the randomization ranges from the result;
    retrain if they move. Then real runs: first against a parked opponent with `action_scale` 0.5
    in the transmitter saturation, then full speed against a hand-driven one. Gate G6. Write-up in
    `docs/experiments/control_improvement/`.
11. **Retrain on real data.** Add the G6 recordings to the sensing model and the `replay`
    opponents, refit the randomization ranges from any metric that fell outside the sim band,
    retrain from the last checkpoint.

## Risks and open questions

- **No hazard layer, by choice.** The policy is the only thing between the robot and a hole, and
  the reward is set to take that risk for hits. If hole falls in G5 or taped entries in G6 cost
  more matches than the extra hits win, raise the hole penalty and retrain; the penalty is the
  one dial. The driver's trainer-enable switch remains the kill.
- **No second robot yet.** G2 and G6 need the opponent sessions. Until then the randomization
  ranges are section 8's starting values and nothing has checked the closed loop against reality.
  Training can run through stage D, but a policy does not go on the robot before G2.
- **The toy can say no.** If step 1 fails question 1 or 2 after a few hours of reward and
  start-state tuning, the RL build stops there and `MotionProfileNavigation` stays.
- **Sim exploits.** A policy finds whatever the sim gets wrong. Most likely candidates: braking
  without nose lift, grip on turns, and pushing a kinematic opponent. The CG and grip ranges in
  section 8 and the flip penalty cover the first two. Watch stage D videos for wall-riding and
  shoving.
- **Kinematic opponent.** A mocap opponent has infinite mass, so pushing it never works and being
  pushed always works. That is conservative for us. If G6 shows the policy avoiding contact the
  real robot would win, give the opponent a free body with a simple drive.
- **One arc for every opponent.** A full-body spinner has no safe side, and the policy treats it
  as a 60 degree front arc anyway. A heading-blind label reads as a heading sigma at the cap, so
  the policy sees that it does not know where the front is. If spinners cost hits in G5 or G6,
  add the weapon shape back as an input and retrain.
- **RTNeural limits the architectures.** Sequential stacks only: no LayerNorm, no skip
  connections, no attention. Every method in the step 1 bake-off fits. A method that needs more
  (SimBa- or BRO-style networks, planners, transformers) needs a second runtime first; the
  hand-rolled Eigen graph is the cheapest for LayerNorm and skip connections.
- **Opponent heading quality.** Flips put the front arc on the wrong side. The sensing model
  injects flips at the measured rate, so the policy learns caution when heading jumps; check on
  recordings of new opponents that the flip rate is in the trained range.
- **Observation drift between C++ and Python.** Any change to the spec bumps its version, which
  breaks the loader until the policy is retrained. That is intended.
- **NHRL floor.** All recordings are from the test box. The friction ranges in section 8 are a
  guess for the arena; the first NHRL outing should record a session for G2 on that floor.
- **The opponent filter is a hard dependency.** `opponent_mode = HOLD` gives no velocity, yaw
  rate or sigmas; production runs `KALMAN` since 2026-10-16. The policy is trained against the 6-state filter through its mirror, so a
  policy must never run with `HOLD`: `RlPolicyNavigation` refuses to initialize unless
  `opponent_mode = "KALMAN"`. Any later change to the filter, including the body-frame model
  prototyped in `phase2_body_frame.py`, means a retrain.

## Next steps

1. Step 1: build the toy and answer question 1.
2. Read the wedge's leading edge and half-width off the collision hulls for `our_strike_x_m`.
3. Before the test box is free: write `config/mr_stabs_mk2_sensing_zed_box.toml`, measure the
   printed tag, and plan the camera remount to 0.75 m straight down.
