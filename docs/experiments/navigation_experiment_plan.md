# Navigation Failure Diagnosis: Experiment Plan

## Problem Statement

The pursuit navigation algorithm overshoots its target heading and continues spinning in the same direction instead of correcting. The goal is to systematically identify the root causes and develop a better algorithm.

## System Architecture

```
Camera (RGB) --> KeypointModel --> RobotFilter --> Navigation --> Transmitter --> SimRunner --> Physics
                                  (pose + vel)    (VelocityCmd)   (TCP)          (diff drive)
```

Key files:

- `src/navigation/pursuit_navigation.cpp` -- the pursuit algorithm under test
- `include/navigation/config.hpp` -- tunable parameters (Kp, angle_threshold, lookahead, etc.)
- `src/robot_filter/robot_front_back_simple_filter.cpp` -- pose and velocity estimation from keypoints
- `simulation/runner.py` -- physics sim (differential drive, wheel velocity control)

## Hypotheses

### H1 -- System lag

End-to-end latency (image to command) causes the robot to act on stale pose data. At 6 rad/s max angular velocity, even 50 ms of lag produces ~17 deg of overshoot per correction.

### H2 -- Neural net identity swap

The keypoint model occasionally labels the wrong robot as "ours," producing a sudden jump in perceived position/yaw, which sends the controller into a large-error recovery spin.

### H3 -- Angular Kp not tuned

`angular_kp = 3.0` with `max_angular_velocity = 6.0` means the command saturates at `angle_error = 2.0 rad` (115 deg). Below that, it is a linear ramp with no damping -- classic recipe for overshoot.

### H4 -- Front/back yaw ambiguity (180 deg flip)

The robot filter computes yaw from front/back keypoint pairs. If the neural net occasionally swaps front and back, `our_pose.yaw` jumps by pi. The controller then sees a massive angle error and spins hard to "correct" a phantom offset.

### H5 -- Hysteresis logic locks in the wrong direction

The committed-turn hysteresis (lines 148-162 of `pursuit_navigation.cpp`) forces `angle_error = |angle_error| * committed_sign`. If the target crosses to the opposite side while the robot is mid-turn, the hysteresis fights the correct correction and keeps the robot spinning past the target. The release threshold (90 deg) may never be reached if the robot keeps overshooting.

### H6 -- No derivative (D) term -- underdamped angular control

The angular controller is pure proportional. There is no damping on angular velocity, so the robot swings past the desired heading, rebounds, and oscillates. If combined with lag (H1), oscillations grow instead of decaying.

### H7 -- Target switching between multiple opponents

`find_target_robot()` picks the closest opponent every tick. With two opponents in the sim config, the robot can alternate targets each frame as relative distances shift during a turn, causing incoherent steering.

### H8 -- Wide angle threshold allows forward drive while off-target

`angle_threshold = 1.0 rad` (57 deg) means the robot drives forward at large heading errors. At speed, this creates wide arcs that overshoot the target laterally, requiring another large turn to recover.

## Experiments

### Experiment 0: Instrument the loop (prerequisite)

Add diagnostic logging for every hypothesis, leveraging the existing `DiagnosticsLogger` infrastructure. Log per-tick to MCAP:

- `pipeline_latency_ms` -- image timestamp vs command-send timestamp
- `our_frame_id`, `our_x`, `our_y`, `our_yaw_deg` -- robot identity and pose
- `target_frame_id`, `target_x`, `target_y` -- which target, where
- `angle_error_deg`, `committed_turn_sign` -- controller state
- `cmd_linear_x`, `cmd_angular_z` -- output commands

Analysis tool: `scripts/analyze_nav_diagnostics.py` reads MCAP recordings and produces 5-panel diagnostic plots plus summary statistics.

**Status: Complete.**

### Experiment 1: Isolate lag (H1)

- **Setup**: Simulation with one static opponent. Robot starts facing 90 deg away from target.
- **Variable**: `command_delay_ms` in `SimTransmitter` config -- sweep {0, 50, 100, 200} ms.
- **Metrics**: Mean |angle_error|, peak |angle_error|, yaw jumps, spin events.
- **Pass/fail**: If overshoot grows with injected delay, lag is a contributor. Check baseline overshoot at 0 ms to separate from other issues.

Config: `config/experiment1/classes.toml`, sim: `simulation/sim_config_experiment1.toml`

**Status: Complete. See [Experiment 1 Results](experiment1_results.md).**

### Experiment 2: Detect identity swaps and yaw flips (H2 + H4)

- **Setup**: Simulation, record MCAP for 30+ seconds of pursuit.
- **Analysis**: Plot `our_yaw_deg` over time. Flag any jump > 90 deg between consecutive frames as a front/back flip. Correlate flip events with onset of spinning (sustained `|cmd_angular_z| > 0.8`).
- **Ground truth comparison**: Log ground-truth poses from the sim alongside rendered frames. Diff detected vs ground truth yaw.
- **Pass/fail**: If spin events are preceded by yaw flips within 5 frames, perception is the dominant cause.

### Experiment 3: Gain sweep with step response (H3 + H6)

- **Setup**: Simulation, one static opponent. Robot placed at fixed distance, facing 180 deg away (worst case).
- **Sweep**: `angular_kp` in {1.0, 2.0, 3.0, 5.0, 8.0}. For each, record angle_error over time.
- **Metrics**: Rise time, peak overshoot, settling time (to within 5 deg), number of oscillation cycles.
- **Follow-up**: Add a derivative term: `angular_cmd = Kp * error + Kd * d(error)/dt`. Sweep Kd in {0.1, 0.3, 0.5, 1.0} at the best Kp.
- **Pass/fail**: If overshoot decreases monotonically with Kd, the lack of damping is a primary cause.

### Experiment 4: Hysteresis A/B test (H5)

- **Setup**: Simulation, one opponent doing `random_walk`. Record 60 seconds.
- **Variants**:
  - A: Current hysteresis (committed turn sign).
  - B: Hysteresis disabled (`committed_turn_sign_` always 0).
  - C: Shortest-path-only (always use raw `angle_error`, no override).
- **Metrics**: Number of full 360-deg spins, cumulative |angular_z|, time-to-intercept.
- **Pass/fail**: If variant B/C has fewer spins but the same or better intercept time, the hysteresis is net harmful.

### Experiment 5: Target stability (H7)

- **Setup A**: Two opponents (default sim config). Record `target_frame_id` each tick.
- **Setup B**: One opponent only.
- **Metric**: Count of target switches per second. Correlate with spin events.
- **Pass/fail**: If target switches > 2/sec correlate with spins, add target persistence (stick with current target for N frames or until distance exceeds a threshold).

### Experiment 6: Angle threshold sweep (H8)

- **Setup**: Simulation, one moving opponent.
- **Sweep**: `angle_threshold` in {0.2, 0.5, 1.0, 1.5} rad.
- **Metrics**: Average intercept time, number of "spiral" events (driving forward while |angle_error| > 30 deg for > 0.5 s).
- **Pass/fail**: If tighter threshold reduces spiraling without excessively increasing intercept time, the current threshold is too permissive.

## Recommended Experiment Order

After completing Experiment 1, the recommended order is driven by what the data reveals:

1. ~~**Experiment 0** -- instrument once, collect data for all hypotheses.~~ Done.
2. ~~**Experiment 1** -- lag injection. Quantify headroom.~~ Done. Lag is a secondary contributor; baseline already broken.
3. ~~**Experiment 1-GT** -- ground truth perception baseline.~~ Done. Proved controller is broken independently of perception.
4. ~~**Experiment 4 / 4b** -- controller investigation (hysteresis, PD, gain sweep).~~ Done. Identified `quaternion_to_euler` bug as root cause. See [Controller Investigation Results](controller_investigation_results.md).
5. **Re-run experiments with fixed `quaternion_to_euler`** -- validate that the P-only controller now converges.
6. **Re-run with NN perception** -- measure actual impact of H2/H4/H7 now that the yaw bug is fixed.
7. **Experiment 3** -- gain sweep + D term. Only meaningful now that the sensor is correct.
8. **Experiment 6** -- angle threshold. Fine-tuning after other fixes.

## Tooling

- **Sim config**: `simulation/sim_config_experiment1.toml` (one static opponent, controlled start geometry)
- **C++ config**: `config/experiment1/classes.toml` (adjustable `command_delay_ms`)
- **Run**: `./scripts/run_simulation.sh simulation/sim_config_experiment1.toml config/experiment1/`
- **Analyze**: `python scripts/analyze_nav_diagnostics.py data/recordings/auto_battlebot_experiment1_*.mcap`
