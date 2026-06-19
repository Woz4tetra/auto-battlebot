# Control Improvement Plan: Mrs Buff MK3

Goal: ram the opponent at maximum velocity, stop overrunning into walls, and work with a worst-case
60ms end-to-end latency. Improve aiming by predicting where the opponent will be, not where it was.

## Current state (from code review)

Two findings that change the approach:

1. Opponent velocity is already estimated but never used to aim. `RobotTemporalMotionFilter::estimate_velocities`
   computes opponent velocity (finite-difference + EMA) and our velocity (from command feedback).
   `NearestTarget::get_target` then targets `robot.pose` raw, ignoring velocity.
2. `lookahead_time` is a dead parameter. It is parsed, stored, and logged in
   `PursuitNavigation::initialize()`, but never read in `compute_pursuit_command`. The class docstring
   claims "Predicts opponent position using velocity"; the code does not.

The prediction primitives exist and sit unused.

## The reframe

Split into two problems and keep them separate:

- Where will things be when my command actually bites? This is the latency + prediction problem.
  All perception noise lives here.
- Given a predicted target, what command rams hardest without overrunning into a wall? This is the
  control law.

The overshoot is mostly the first problem leaking into the second. We command toward where the
opponent was 60ms ago, from where we were 60ms ago, with a drivetrain that coasts. This is a
stale-target problem, not a missing-MPC problem. No controls expertise required to fix the bulk of it.

## Why we overshoot into the wall

Three compounding causes, ordered by how cheaply we can attack them:

1. Stale target and stale self-pose. 60ms at 2 m/s is 0.12m, more than half a robot length (0.22m).
   We aim behind the opponent, miss, and the miss vector points at the wall.
2. Open-loop drive. Untuned AM32 ESC with asymmetric deadzone (+20 vs -10) means commanded velocity
   does not equal actual velocity. The controller does not know how fast it is really going.
3. No coast model. `stop_distance` just zeroes the command, but a robot at 2 m/s with low friction
   keeps sliding.

Nuance for a rammer: we want to contact at max velocity, so we should not brake before contact. Wall
hits happen when we miss. The highest-value fix is aiming better so we connect, not braking earlier.
Braking logic should only engage when there is no opponent between us and the wall.

## Staged plan

### Stage 0: Measure before building

Turn the two unknowns into numbers from existing MCAP/SVO recordings in playback mode.

- Perception reliability: % of frames with a valid opponent track, and track-ID-switch rate. Instrument
  the existing `is_stale` flag and `RobotDescriptionsCache`. This tells us how long the predictor must
  coast through dropouts.
- Actual end-to-end latency: decompose it, do not guess the 60ms. Perception (~10ms keypoint + ~30ms
  pipeline) + Crossfire (20ms) + ESC/mechanical. The mechanical/ESC tail is the part prediction cannot
  compensate.
- Overshoot baseline: count wall contacts, time-to-contact, and % time aimed at opponent, to have a
  baseline to beat.

### Stage 1: Fast headless 2D sim

Do not tune control in Genesis or Unity. Rendering is not needed to tune a pursuit law. Build a tiny
2D sim that models the things that actually hurt:

- Configurable latency (command delay buffer).
- ESC deadzone + asymmetry + a velocity-tracking lag.
- Friction/coast (first-order velocity model, not instant stop).
- Opponent motion replayed from real fight recordings (extract opponent trajectories from MCAP). This
  is the highest-fidelity prediction test without building an opponent AI.
- Injected perception dropouts and track switches at the rates measured in Stage 0.

Define a scalar score: contact velocity, time-to-contact, wall-hit count, % time aimed. Batch-run over
many recorded scenarios. This is the iteration loop, seconds not minutes, and it plugs into the
existing playback regression harness.

### Stage 2: Use the prediction we already compute

Low effort, high value. Forward-predict both robots over the measured latency horizon:

- Opponent: constant-velocity using the EMA velocity already estimated. Over 60ms a constant-velocity
  model is sufficient.
- Self: forward-simulate our own pose from in-flight commands sent but not yet observed (Smith-predictor
  idea). `update_with_prediction` already does most of this math for gap-filling.
- Aim the controller from predicted-self at predicted-opponent.

Wire `lookahead_time` up to actually do this. This change may eat most of the overshoot.

### Stage 3: Intercept geometry from problem-space knowledge

The arena is a known flat square and hitting non-opponents is fine, so skip obstacle avoidance
entirely. The only constraint is walls.

- Bias the approach heading so the wall sits behind the opponent. A miss then overruns into the
  opponent or open field, not a wall.
- No path planner needed, just an intercept-angle choice.

### Stage 4: Robustify against bad sensing

- Replace the EMA with a constant-velocity Kalman filter on the opponent. It gives velocity and an
  uncertainty estimate.
- Use the uncertainty: when high (just reacquired, or track jittering), widen the "aim within footprint"
  tolerance and do not commit to full-send.
- Gate the controller: coast on the last predicted trajectory for a bounded time on dropout, then hold
  rather than chase a hallucinated detection. We already hold last pose; add a max-coast timeout and a
  confidence gate.

### Stage 5: Consider MPC / pursuit-with-dynamics only if needed

Only if the validated sim model shows the simpler law still overshoots. By then the sim is accurate
enough to trust. Control sophistication becomes an empirical question, not a theory question.

## What to do first

Stages 0 and 1 together. Everything downstream needs the sim and the baseline numbers. The sim is also
what removes the need for controls expertise: try a pursuit law, measure contact velocity and wall hits
over many recorded fights, keep what scores.

Lean toward Stage 0 instrumentation first. It is quick and tells us how hard the prediction has to work.

## Key files

- `src/navigation/pursuit_navigation.cpp` / `include/navigation/pursuit_navigation.hpp` — control law,
  where `lookahead_time` must be wired in.
- `include/navigation/config.hpp` — `PursuitNavigationConfiguration`, holds the dead `lookahead_time`.
- `src/robot_filter/robot_temporal_motion_filter.cpp` — velocity estimation and `update_with_prediction`.
- `src/target_selector/nearest_target.cpp` — targets raw pose, needs predicted pose.
- `config/playback.toml` — regression harness entry point.
