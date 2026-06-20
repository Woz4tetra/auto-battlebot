# Stage 1 Report: Fast Headless Sim for Control Tuning

Stage 1 built a fast, headless simulation to tune Mrs Buff MK3's control without hardware, and used
it to surface several real controller bugs. The honest outcome: it is a faithful **controller**
testbed (it runs the real `PursuitNavigation` with the real tuning, accelerated and deterministic),
but not yet a faithful **drivetrain** testbed (the plant is uncalibrated, and the available
recordings can only roughly identify it). This report covers what was built, how to run it, what it
found, and what is still missing.

## Goal

From `docs/experiments/control_improvement/control_improvement_plan.md`: build a sim that bypasses perception and feeds poses straight
to the controller, so control and latency compensation can be iterated in seconds instead of on the
robot. Stage 0 (`docs/experiments/control_improvement/stage0_baseline_report.md`) established the baseline this stage works against:
latency at/over the 60 ms budget, heavy near-wall time, and a ~15 cm flat-plane projection bias.

## What was built

### Headless kinematic sim (the core)

The project already abstracts the "plant" behind a TCP protocol (`simulation/protocol.py`,
`include/simulation/sim_connection.hpp`): the C++ side sends a `VelocityCommand` and receives
poses. Genesis sits behind it as the slow, high-fidelity, image-rendering backend. Stage 1 adds a
fast backend behind the same protocol:

- `simulation/kinematic_sim_server.py` - a 2D kinematic plant (no Genesis, no rendering). Models the
  effects that actually drive overshoot: first-order drivetrain lag/coast, actuation latency
  (sim-tick command buffer), square-arena walls, opponent behaviors
  (static / straight / circle / random_walk / replay), and perception emulation (observation
  latency, position noise, dropout, and the flat-plane projection bias). It owns logical time and
  sends `sim_time` in each response. Typed config via dacite (`simulation/config/kinematic.py`,
  loaded by the generalized `simulation/config/loader.py`).
- C++ providers so navigation gets an arena without perception: `FixedFieldFilter` (configured arena
  size) and `FixedMaskModel` (satisfies the field-init gate). Registered in the existing factories.
- `config/headless_sim.toml` - the C++ profile: `SimRgbdCamera` + Noop perception + `FixedFieldFilter`
  + `GroundTruthRobotFilter` + `SimTransmitter` + `ManualClock`, free-running.

### Injected logical clock

Logical time is single-sourced from the camera frame timestamp (`ClockInterface` / `ManualClock` /
`SystemClock`, config-selected; `sim_time` added to the wire protocol). In the sim, the controller's
`dt` comes from the sim's own time, so runs are deterministic and can go far faster than real time on
hardware the default `SystemClock` returns wall-clock, so deployment is unchanged. This replaced the
earlier real-time-only workaround.

### Free-run loop

`Runner::run()` treats `max_loop_rate <= 0` as free-run (no wall-clock pacing, division guarded);
`headless_sim` uses `0`, so the loop is paced only by lockstep with the sim. The real-robot configs
(30 Hz) are unaffected.

### Batch sweeps and plots

- `playground/control_stage0/sim_sweep.py` - one command launches the ROS master
  (`build/bin/miniroscore`, no Docker), the sim server, and the binary across a sweep, scores each
  run with the Stage 0 loaders, and writes a results table + plots (per-run 4-panel and a combined
  trajectory overlay). Uses the overlay config system: a per-run C++ overlay `extends` the base
  headless config and overrides only swept fields the sim config is deep-merged.
- Six sweep specs in `playground/control_stage0/sweeps/`: opponent behaviors, start positions,
  evasive speed, crossing headings, latency x moving target, random-walk seeds.

### Plant identification

- `playground/control_stage0/fit_plant.py` - fits the kinematic plant (max speeds, coast time
  constants, actuation lag) from recordings, validates by short-horizon replay, and plots one
  open-loop trajectory row per recording.

## How to run it

From the repo root, in the project venv (`source scripts/activate_python.sh`; do not use `uv`):

```bash
# Built-in latency sweep (manages master + sim server + binary, scores, plots)
python playground/control_stage0/sim_sweep.py --out playground/control_stage0/sweep_out

# A specific scenario sweep
python playground/control_stage0/sim_sweep.py \
    --sweep playground/control_stage0/sweeps/evasive_speed.toml \
    --out playground/control_stage0/sweep_out/evasive_speed

# Manual three-process run
build/bin/miniroscore &
python simulation/kinematic_sim_server.py simulation/kinematic_sim.toml &
build/auto_battlebot -c config/headless_sim.toml

# Plant fit from test-field recordings
python playground/control_stage0/fit_plant.py data/recordings/tests/*_repaired.mcap \
    --plot playground/control_stage0/sweep_out/fit_plant_test.png
```

The Genesis sim now runs via `scripts/run_simulation.sh` against `config/genesis.toml` (see config
refactor below).

## What it found

The sim earned its keep mostly by exposing real bugs.

### 1. Wall-clock timing in the controller

`PursuitNavigation`'s PD derivative used wall-clock `dt`. Run accelerated, `dt` shrank to ~0.2 ms, the
D-term exploded, and the robot flew into a wall. Fixed by the injected clock: logical time follows the
sim frame stamp, so `dt` is correct regardless of loop speed.

### 2. Config gain mismatch exposing a `steer_brake` bug (the "fail to launch")

From some start poses the robot would not launch, behavior never seen on the real robot. Root cause
was two layers:

- `headless_sim` inherited `angular_kp = 26.0` from `simulation.toml` (the Genesis profile). The real
  robot uses `angular_kp = 0.4` (`main.toml`).
- `compute_linear_velocity` had `steer_brake = 1.0 - 0.5*|angular_z|`, treating `angular_z` (rad/s,
  unbounded) as if normalized. At kp=26 a 29 deg heading error gives `angular_z ~ 13 rad/s`, so
  `steer_brake ~ -5.6` and the forward command flipped to full reverse, the robot backed into the
  corner. At kp=0.4 it stays positive, which is why the real robot never showed it.

Confirmed empirically: the same failing start reaches the opponent at 1.82 m/s with the real gains.
Fixes: real tuning in `headless_sim`, and a dimensionless/clamped `steer_brake`. Also refactored the
configs (below) so the Genesis tuning can never leak into the sim base again.

### 3. The latency-overshoot finding was a tuning artifact

An early sweep showed near-wall time jumping 0 -> 93% with latency. That was under the wrong kp=26
gains. With the real gains, near-wall is 0% at every latency (0-90 ms) on a static opponent: latency
causes a small overshoot loop that grows modestly with latency, then the controller recovers and
orbits the opponent. So the sim does **not** reproduce the NHRL wall-slamming with realistic gains,
which points at the uncalibrated drivetrain (coast/momentum, ESC deadzone), not the controller.

### 4. Sweep results (real gains)

- Start positions: all five reach the opponent at ~1.8-1.9 m/s, 0% near-wall.
- Opponent behaviors: catches static, straight-crossing, and random-walk catches slow circlers
  eventually fast circlers (>= 1.5 m/s) escape.
- Latency on a static target: time-to-contact rises 1.17 -> 1.33 s across 0-90 ms.

The no-lead pursuit failing on fast evaders is the clearest motivation for prediction (Stage 2).

### 5. Plant identification is hard from fight data

- Competition recordings are unusable: the opponent shoves the robot (motion != command) and damage
  changes the drivetrain.
- The navigation command is not what drives the robot. In trainer mode the driver adds/subtracts, so
  the actual command is the OpenTX channel feedback (`opentx_transmitter/channels`, the post-mix
  output), not the nav suggestion. Correlation of command with motion: nav suggestion 0.07, actual
  channel 0.34.
- Clean test-field recordings still only give a rough fit: driving is gentle (most commands sit in
  the ESC deadzone), perception velocity is noisy (yaw keypoint flips produce +-40 rad/s spikes), and
  the gain is run-dependent (~0.5-1.4 m/s per unit command, far below the ~3.9 m/s the gearing
  allows, consistent with the known friction/ESC losses). The robust extract is the actuation lag,
  ~45 ms.
- The large open-loop replay error (~105 cm) is integration drift, not a broken plant. Re-anchored to
  the observed pose, the fitted plant tracks within ~6 cm over 0.5 s and ~10 cm over 1 s. The sim runs
  closed-loop (the controller re-reads the pose every tick), so short-horizon error is what matters.

### 6. Config refactor

`simulation.toml` no longer defines navigation or the transmitter startup delay it is a shared,
non-runnable sim base. `config/genesis.toml` (new) carries the Genesis-specific tuning, and
`scripts/run_simulation.sh` defaults to it. This is what stops the Genesis gains from contaminating
the headless sim.

### 7. Tooling

Added project ruff + mypy config (mirroring the Pickle-Robot conventions `playground/` excluded),
`dacite`/`mypy` deps, and the `diag_io` loader now also extracts the drive-command channels.

## Current fidelity status

- Faithful controller testbed: real `PursuitNavigation` and tuning, lockstep free-run, deterministic,
  scored by the same Stage 0 analyzer as real fights. Good for control-law and latency work.
- Not yet a faithful drivetrain testbed: the plant `max_*_speed` and `tau` are still informed guesses.
  The recordings give a rough, run-varying fit and the right actuation lag, but not a trustworthy
  single plant. Treat sim overshoot *shapes* as real and *magnitudes* as provisional.

## Key files

- C++: `include/field_filter/fixed_field_filter.hpp`, `include/mask_model/fixed_mask_model.hpp`, the
  clock (`include/time/`, `src/time/`), the `steer_brake` fix and free-run guard
  (`src/navigation/pursuit_navigation.cpp`, `src/runner.cpp`), `sim_time` in
  `src/simulation/sim_connection.cpp`.
- Configs: `config/headless_sim.toml`, `config/genesis.toml`, `config/simulation.toml` (now a base),
  `simulation/kinematic_sim.toml`.
- Python: `simulation/kinematic_sim_server.py`, `simulation/config/kinematic.py`,
  `simulation/config/loader.py`, `playground/control_stage0/sim_sweep.py`,
  `playground/control_stage0/fit_plant.py`, `playground/control_stage0/sweeps/*.toml`.

## Next steps

1. **Calibration run** for the plant: ~15 s of deliberate excitation on the test field (sustained full
   throttle to steady state, then hard yaw steps both directions) with the robot tracked. Then
   `fit_plant.py` gives a clean `max_*_speed` and `tau`. This is the gate to trusting overshoot
   magnitudes.
2. **Stage 2 - latency compensation / prediction**: wire `lookahead_time`, forward-predict both
   robots over the measured latency, and watch the overshoot loop straighten and the fast-evader
   misses shrink in these same sweeps.
3. **Height compensation**: model the flat-plane projection bias is already in the sim use it to
   estimate the benefit before implementing the real fix.
4. Optionally fix the perception yaw (keypoint front/back flips) so the angular plant fit and any
   future ID are not corrupted by +-40 rad/s spikes.
