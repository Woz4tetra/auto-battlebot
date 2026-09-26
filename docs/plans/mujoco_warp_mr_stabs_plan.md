# MuJoCo Warp simulation of Mr Stabs Mk2, tuned from hand-driven recordings

Goal: a rigid-body MuJoCo model of Mr Stabs Mk2 whose unmeasured parameters (floor contact, motor
and ESC, delay) are fit by running thousands of parallel MuJoCo Warp rollouts against overhead
AprilTag recordings of the robot being driven by hand. The mass properties come from CAD and are
held fixed. The fit is judged on held-out sessions against the grey-box plant fit on the same data.

This sits under `learned_sim_environment_plan.md`. That plan fits a kinematic plant and a sensing
model; this one adds the 3D rigid-body layer the kinematic sim cannot represent: pitch (the
backflip), traction limits, the drivetrain's reflected inertia, and later contact with walls.

The input is manual driving only. No chirps, steps or scripted waveforms. The driver follows a
maneuver menu (below) and a live coverage display shows which parts of command space still lack
data.

## What is already known

Mass properties, from the Onshape assembly (screenshot 2026-09-25 23:10) and the Onshape URDF
export, lumped into three bodies. Frame is FLU at the axle midpoint (x forward, y left, z up). The
mapping from the Onshape assembly frame is x = -Y_asm, y = +X_asm, z = +Z_asm, confirmed by the
longitudinal-vertical product of inertia agreeing in sign between the two independent sources.

| Body | Mass | COM (mm) | Inertia about COM (kg m^2) |
| --- | --- | --- | --- |
| Chassis | 459.0 g (CAD 457.3 g + 1.7 g to match the scale) | (31.94, 0.00, 0.86) | Ixx 7.769e-4, Iyy 5.461e-4, Izz 1.2521e-3, Ixz -2.31e-6 |
| Wheel, each | 18.445 g | (0, +/-65.26, 0) | axle 4.2528e-6, radial 2.33e-6 |
| Whole robot | 495.9 g on the scale, 494.22 g in CAD | (29.55, 0.00, 0.80) | roll 9.387e-4, pitch 5.898e-4, yaw 1.449e-3 |

The URDF export alone sums to 328.9 g: the motors, ESCs, flight controller, batteries and switch
come out at zero, probably because their masses are Onshape overrides the exporter does not carry. The chassis above is therefore the Onshape
whole-assembly total minus the URDF wheels, not the URDF chassis.

Drivetrain, from the Repeat Robotics Compact 1806 product page and CAD:

| Quantity | Value | Source |
| --- | --- | --- |
| Gear ratio N | 22.6:1 | repeat-robotics.com/products/repeat-compact-1806 |
| Motor | BL 1806, 2300 KV, so Kt = 60 / (2 pi KV) = 4.152e-3 N m/A | same |
| Gearmotor mass | 44.5 g (motor alone 22.4 g) | same |
| Free speed at the output | 1130 rpm at 3S, 1500 rpm at 4S (ideal KV x V) | same |
| Rotor inertia J_r | 9.3e-7 kg m^2 (9.3 g cm^2) | CAD integration of flux ring, 14 magnets, aluminum end cap, shaft |
| Reflected at the wheel | N^2 J_r = 4.77e-4 kg m^2 per wheel | derived |
| Wheel radius | 25 mm | CAD |

The rotor number is cross-checked by mass: CAD rotor 9.8 g, plus stator laminations, base and
bearings, plus an estimated 3 to 4 g of copper, gives about 22 g against the 22.4 g listed motor.
A steel end cap would push it to about 25 g, so the cap is aluminum. The gearbox in the CAD is a
hollow shell, so the sun, planets and carriers are missing; bounding them at 1 to 2 g inside a 6 mm
radius adds at most about 4% to the rotor term.

Reflected inertia dominates the drivetrain. Each gearmotor acts like J/r^2 = 0.76 kg of extra
translational mass, about 1.5 kg against a 0.5 kg robot, and adds 6.5e-3 kg m^2 to yaw against the
body's 1.45e-3. The backflip follows from it: the chassis sees the drive reaction
F r + (J_w + N^2 J_r) alpha, and with the COM 31.9 mm ahead of the axle the nose lifts at about
0.29 g of forward acceleration. Without the rotor term (the Unity model) the threshold was 1.4 g,
above any traction the wheels have, which is why that model never flipped and the real robot does.

Plant behavior, from the 2026-07-03 and 2026-07-05 AprilTag sessions
(`docs/experiments/control_improvement/stage2_mr_stabs_mk2_calibration.md`):

- Actuation latency about 60 ms (run `175805`, pooled onset stack). The best-measured number.
- Linear decel tau about 0.078 s, accel tau about 0.058 s, angular tau 0.05 to 0.07 s. Each rests
  on one or a few segments.
- Deadzone at or below 0.04 on both channels.
- Reverse spins uncontrollably and settles slower than forward (1.08 to 1.21 m/s against 1.31 to
  1.33 m/s at |command| 0.25). The plant is asymmetric.
- Linear motion loses the tag: the robot leaves the OAK-1 view within a second at anything above
  a quarter stick. Tag detection runs 50 to 68% at 1080p even with 2x upsampling.

One inconsistency this model should settle. The stage-2 fit reports max forward speed 5.6 m/s,
but the gearmotor's ideal free speed is 3.93 m/s at 4S (4.64 m/s on a full LiHV pack). The 5.6 was
a linear extrapolation from commands 0.2 to 0.5, so the command-to-voltage map is probably not
linear (AM32 throttle curve, radio scaling), or the measured speeds are off. The MuJoCo actuator
has a hard voltage ceiling, so the fit will expose which.

Firmware effects that sit inside the plant (`firmware/mr_stabs_mk2/src/main.cpp`):

- A BNO055 heading-hold PID runs when auto-steer is on. It closes a loop around yaw rate, so any
  fit with it on identifies the PID and not the drivetrain.
- A failsafe stops the robot after 5 s of identical radio frames. Hand driving never produces that.

## Recording

### Recorder

`playground/calibration/apriltag_track.py` was deleted in e56a4464 (2026-08-18) when the velocity
jig tool replaced it. Its non-`--drive` mode is what hand driving needs: it records raw frames on
`/camera/image`, the floor board lock on `/floor/image`, and the transmitter channels read-only on
`/transmitter/channels`, all on CLOCK_MONOTONIC, in the layout `auto_battlebot/calibration/apriltag/
apriltag_mcap.py` still defines. `analyze_apriltag_mcap.py` still reads that layout.

Restore it with these changes:

1. **Confirm what `/transmitter/channels` holds.** The MCAP docstring says "stick axes the driver
   was commanding". The fit needs what the robot received, which is the radio mixer output. If the
   recorder reads sticks, either read the mixer output instead or set the radio model to a pure
   pass-through (100% rates, no expo, no trims, no slow-up) for these sessions and record that in
   the session metadata. A mixer the log can't see is a silent model error.
2. **Record raw frames**, not JPEG, at 60 fps. JPEG moves the corner estimates the subpixel
   refinement keys on (see the `apriltag_mcap.py` docstring).
3. **Keep the full 3D tag pose.** `analyze_apriltag_mcap.py` solves the tag pose with PnP and then
   projects to (x, y, yaw). Also write pitch and roll to the truth CSV. Nose lifts during hard
   throttle are the data that pins the COM, the reflected inertia and traction together, and the
   rigid-body model is the first consumer that can use them.
4. **Live coverage display.** Replaces the scripted protocol as the thing that guarantees coverage.
   While recording, bin the (linear, angular) command and the solved body speed and show which
   cells have less than N seconds of steady, in-frame data. The driver drives toward the empty
   cells.
5. **Session metadata.** Pack voltage at start and end, auto-steer state, radio mixer settings,
   surface (which floor, cleaned or not), tire condition, robot mass, and free-text notes. Stored in
   `/calibration/metadata`.

### Setup

- Overhead OAK-1 W and the 3x5 floor GridBoard (ids 160 to 174), locked once then removed, as in
  the stage-2 runs. Robot tag id outside that range.
- Tape the tracked area on the floor, inset from the camera's view edge by a robot length, so the
  driver knows where the tag stays visible.
- If the mount allows, raise the camera or widen the view. Stage 2 found field of view, not
  detection, was the bottleneck on every linear phase.
- Walls: keep the robot off them for the fitting sessions. Wall contact is a later stage.
- **Auto-steer off** for all fitting sessions, so the fit sees the open-loop drivetrain. Record a
  separate small set with it on, to validate a PID model later.
- Radio in pass-through mode per recorder change 1.

### What to drive (maneuver menu)

Each session is 3 to 4 minutes. Steady holds are worth the most, so hold a stick position for at
least a second whenever the space allows. Mix these freely; the coverage display says what is
missing.

| Maneuver | Identifies |
| --- | --- |
| Straight holds at a few throttle levels, forward and reverse, released to zero | gain, command-to-voltage map, decel (coast or brake) |
| Very slow creep, stick barely off center | deadzone, static friction |
| Spins in place at 3 to 4 stick levels, both directions | angular gain, skid and torsional friction |
| Arcs of several radii, figure eights | steer-brake coupling, per-side asymmetry |
| Direction reversals (forward to reverse, left spin to right) | delay versus lag, which smooth driving confounds |
| Smooth ramps up and down | the command-to-voltage curve between the hold levels |
| Throttle punches, getting sharper across the session, stopping at the first nose lift | reflected inertia, COM, traction |

Avoid:

- Leaving the taped area. Frames without the tag are interpolation, not measurement.
- Wall contact.
- Stick jitter near center. It makes the deadzone ambiguous.
- Full throttle plus full turn early in a session, before the punches have shown where the nose
  lifts.

The punches replace `find_flip_accel.py`'s ramp for this purpose: the driver works up to the nose
lift instead of a script. The predicted threshold is 0.29 g. Stop at the first wheelie rather than
the first flip; a lifted nose already carries the information and a flip ends the session.

Volume: aim for 10 to 15 sessions over at least two battery charge levels and, if possible, two
floor surfaces (the test box and a sample of competition floor). Hold out whole sessions for
validation, not windows from sessions used in training.

## Data pipeline

1. `analyze_apriltag_mcap.py` produces the truth CSV per session: t, x, y, z, roll, pitch, yaw, plus
   the channel log carried through.
2. Command tape: the logged channels converted to [-1, 1] with the same scaling the transmitter
   uses. The delay is not applied here; the fit shifts the tape per candidate.
3. Windows: reuse `plant.make_windows` with window lengths of 1 to 2 s. Gate out:
   - windows where tag visibility is below a threshold (start at 70%, tune on the noise floor)
   - windows within a robot length of a wall or the taped edge
   - windows where pitch exceeds a few degrees, except in the separate nose-lift set
   - windows where auto-steer is on (a separate validation set)
4. Initial state per window: pose from the truth CSV, body velocity from a smoothing spline over
   the in-frame detections, wheel speeds from the no-slip relation (v +/- omega * 0.06526) / 0.025.
   The rotor state follows the wheels through the armature.
5. Noise floor: pose residual against the smoothed track on held-still segments and on steady
   holds, per session. Every error metric is reported as a multiple of it, as in the match fit
   report.

## MuJoCo model

`simulation/assets/robots/mr_stabs_mk2/` holds `robot.urdf` with older values (422.8 g base, 15 g
wheels, no rotor inertia). Replace it with an MJCF built from the table above.

- **Chassis**: free joint, `inertial` with the full tensor. Collision from a convex decomposition
  (CoACD) of the chassis collision mesh, plus a small explicit geom at the nose where the robot
  rests. The robot rests on the front skid, since the COM is ahead of the axle.
- **Wheels**: hinge joints on y, `armature = 4.77e-4` (the reflected rotor inertia), collision as
  a 25 mm cylinder the width of the tread.
- **Actuator**: a DC motor with back-EMF is affine in command and joint speed, so it fits MuJoCo's
  `general` actuator directly:

  ```
  torque = (eta N Kt V_batt / R) * u  -  (eta N^2 Kt Ke / R) * qvel
  ```

  with `gainprm[0]` the first coefficient and `biasprm[2]` the second. `forcerange` carries the
  current limit. Deadzone and the throttle curve are applied to the command tape before it becomes
  `ctrl`. Joint `frictionloss` and `damping` carry gearbox friction.
- **Zero-command behavior**: with AM32 complementary PWM and brake-on-stop settings, a zero
  command either shorts the motor (the back-EMF term stays active, which is braking) or lets it
  freewheel (the term drops out). Model both, pick by the recorded AM32 settings, and let the coast
  data confirm.
- **Floor**: a plane with fitted friction. Wheels use `condim` 4 or 6 so torsional and rolling
  friction exist; spins scrub through the skid, so its friction matters as much as the wheels'.
- **Timestep**: start at 1 ms and halve it until the flip threshold and coast tau stop changing.

Sanity checks before any fitting:

- The simulated nose lift under a torque ramp should match the analytic 0.29 g.
- Free speed at a set voltage should match KV x V / N.
- The coast tau should land near the measured 0.078 s with plausible R and braking settings. If no
  plausible R reaches it, the zero-command model is wrong.

## Parameters

Fixed from CAD, perturbed only in the sensitivity check: masses, inertias, COM, geometry.

Fitted, with priors:

| Parameter | Prior or bounds | Notes |
| --- | --- | --- |
| Transport delay | 30 to 90 ms, centered on 60 | profiled on a grid, as `jig_fit` does |
| Deadzone per channel | 0 to 0.06 | stage 2 says at or below 0.04 |
| Command-to-voltage curve | linear plus one curvature term | settles the 5.6 m/s question |
| Pack voltage | measured per session, with a linear sag term | not fitted blind |
| Motor resistance R | 0.05 to 1 ohm | wide; no published value |
| Gearbox efficiency | 0.6 to 0.95 | |
| Wheel joint friction and damping | wide | |
| Left/right gain ratio | 0.8 to 1.2 | reverse spin shows asymmetry |
| Wheel-floor friction (sliding, torsional, rolling) | sled test when available, wide until then | |
| Skid-floor friction | wide | |
| Contact `solref` / `solimp` | MuJoCo defaults, then a narrow search | |
| Armature scale | 1.0, sigma 20% | tests the CAD rotor number |
| Zero-command mode | brake or coast, discrete | from AM32 settings |

About 15 continuous parameters plus two discrete choices.

## Fitting on MuJoCo Warp

- **Batch layout**: one world per (candidate, window) pair. A generation of 128 candidates over 64
  windows is 8,192 worlds. The spike in step 2 measures steps per second on one A6000 and sets the
  real sizes.
- **Per-world parameters**: MuJoCo Warp can vary some model fields per world for domain
  randomization. The spike confirms which of the fields above are supported (friction,
  `gainprm`, `biasprm`, armature, damping). Unsupported fields move to an outer loop over
  candidates.
- **Optimizer**: CMA-ES, gradient-free, so the delay grid and the discrete zero-command choice
  don't need special handling. Several restarts, to see whether they land in the same region.
- **Loss**: open-loop multi-step pose error as in `plant.predict_windows`: position and heading at
  0.2, 0.4 and 1.0 s horizons, Huber-weighted, normalized by the session noise floor. A pitch term
  applies only on the nose-lift windows.
- **Staged release**, so each group is fit on the data that identifies it:
  1. Straight holds and coasts: delay, deadzone, voltage curve, R, efficiency, braking mode.
  2. Spins and arcs: skid and torsional friction, left/right ratio.
  3. Nose-lift windows: armature scale and wheel traction, with everything else frozen.
  4. A final joint pass over all windows, starting from the staged result.
- **GPU queue**: fitting jobs go through `training/gpu_queue.py` like training. First submission
  uses `--profile mujoco-fit --eta 2h`, since the queue has no history for it; later ones pass
  `--work` as rollout-steps so the queue learns a rate.
- **Dependencies**: `mujoco` and `mujoco-warp` (with `warp-lang`) as x86_64-only entries in the
  platform-conditional deps in `pyproject.toml`. The Jetson never runs the fit. Pin versions; the
  MuJoCo Warp API is still moving.

## Validation

The model is accepted when all of these hold on held-out sessions:

1. **It beats or matches the grey-box plant.** Fit `plant.py` (M5) on the same training windows
   and compare position and heading error at 0.4 and 1.0 s. If MuJoCo loses on flat driving, the
   actuator model is wrong, since the rigid body adds physics and should not lose accuracy.
2. **Errors sit near the noise floor.** Report every error as a multiple of it.
3. **The physical numbers are plausible.** R, efficiency and friction inside physical ranges;
   armature scale near 1; delay near 60 ms; coast tau near 0.078 s.
4. **Nose lift matches.** The fitted model lifts the nose on the same held-out punches that lifted
   the real robot, and not on the ones that didn't.
5. **The parameters are identified.** CMA-ES restarts agree. Report the parameter pairs that trade
   off (friction against armature against gain is the likely one) and fix one of each pair from an
   outside measurement if they do.

The spread of parameter sets that pass becomes the randomization range for the sim in
`learned_sim_environment_plan.md`, not a single best fit.

## Code layout

- `auto_battlebot/mujoco_sim/`: MJCF builder from the mass-property table, actuator and
  command-tape mapping, batched rollout, loss. The library code, type-checked. It is covered by the
  `auto_battlebot*` include in `pyproject.toml`.
- `simulation/assets/robots/mr_stabs_mk2/`: the MJCF, the convex collision pieces, and a
  `mass_properties.toml` recording the table above with its provenance (Onshape screenshot date,
  URDF export, the rotor integration and its densities).
- `playground/calibration/apriltag_track.py`: the restored recorder.
- `playground/calibration/fit_mujoco_plant.py`: the fit CLI.
- Recordings and fit output under `playground/calibration/out/`, like the existing plant fits. Not
  `runs/` (training output only) and not `data/`.
- The write-up in `docs/experiments/control_improvement/`.

## Steps

1. **Freeze the mass properties.** Commit `mass_properties.toml` and the lumping script that
   produced it from the URDF export and the Onshape totals. Half a day.
2. **Model and MuJoCo Warp spike.** Build the MJCF, pass the three sanity checks, measure batched
   throughput, confirm which fields vary per world. One to two days. This decides the batch layout.
3. **Restore the recorder** with the five changes above. One to two days, mostly the coverage
   display.
4. **Record.** Two or three evenings of driving to get 10 to 15 sessions.
5. **Truth, windows and noise floor.** Extend the analysis to 3D pose; build and gate windows. One
   day.
6. **Grey-box baseline** on the same windows. Half a day; the code exists.
7. **MuJoCo fit**, staged. Two to three days including iteration.
8. **Validation and report.** One day.
9. **Hand-off**: the passing parameter spread goes into the learned-sim randomization ranges.

## Risks and open questions

- **Field of view.** Stage 2 lost the tag on every fast linear phase. If the camera can't be raised,
  sustained high speed never gets measured and the top of the voltage curve is extrapolated again.
  A second camera or the onboard IMU (once the ESP-NOW link streams it) are the fixes.
- **Closed-loop data.** A human driver reacts to the robot, so commands correlate with past
  disturbances. Open-loop multi-step scoring on the recorded tape limits the bias, but it is why the
  punches and reversals matter: they are the least reactive inputs in the menu.
- **Radio mixer.** If the log turns out to hold pre-mixer sticks and the mixer wasn't in
  pass-through, the recorded sessions can't be used for the actuator fit.
- **Friction identifiability.** Without the sled test, wheel friction and armature can trade off on
  the punch data. The sled test fixes friction from outside.
- **Heading hold.** Fits with auto-steer off describe a robot that never fights with auto-steer
  off. Modeling the PID is a follow-up, validated on the auto-steer-on set.
- **MuJoCo Warp maturity.** If per-world model fields don't cover what's needed, the fallback is an
  outer loop over candidates, which costs throughput but not correctness.
