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

Firmware between the radio and the ESCs (`firmware/mr_stabs_mk2/src/main.cpp`):

- **Auto-steer** (flip switch DOWN, the power-on default) runs a heading-hold PID on BNO055 yaw:
  kp 0.08, ki 0.01, kd 0.01 (percent per degree), 2 degree deadband, output added to the left/right
  mix. It only acts while the turn stick is within 1% of center, and waits 0.25 s after a turn
  before engaging. Any commanded turn passes straight through. The same switch position also turns
  on upside-down detection.
- **Mixing**: left = -a + turn, right = -a - turn, both scaled down together if either exceeds
  100%. The ESC stop threshold is 1% per motor and is tunable over the diagnostics server.
- **Diagnostics stream**: an event stream on the robot's WiFi access point carrying `left_cmd`,
  `right_cmd`, `a_percent`, `b_percent`, `pid_setpoint`, `pid_output`, BNO055 orientation and
  acceleration, stamped in robot milliseconds. It sends at 10 Hz, or every control loop in its
  recording mode.
- A failsafe stops the robot after 5 s of identical radio frames. Hand driving never produces that.

Fitting runs with auto-steer on, the way the robot competes. The PID does not have to be fit or
even modeled for this: the diagnostics stream logs the per-motor commands after the PID and the
mixer, and those are the drivetrain's input. The drivetrain is fit open loop on them. The PID is
reproduced in the sim separately, from the gains above, for closed-loop use.

## Recording

### Recording host and camera

Record on the ZED Box Mini with the ZED X One S (`docs/zed_box_mini.md`), not the OAK-1 W the
stage-2 runs used. What changes:

- **The whole cage fits in view.** The SDK reports rectified intrinsics fx 665, fy 716 at
  1920x1200, a 111 x 80 degree view. Mounted 1.5 m above the floor it covers about 4.3 x 2.5 m, so
  the 2.35 m cage fits with margin. Stage 2's main limitation, the robot leaving the frame on
  every fast linear phase, goes away.
- **Global shutter.** SDK 5.2.3 opens the camera as a "ZED XOne GS", so a fast-moving tag is not
  skewed by a rolling shutter. Set a short manual exposure to also limit motion blur.
- **Smaller tag in pixels.** The cost of the wide view. An 80 mm tag spans about 44 px at 1.2 m,
  35 px at 1.5 m and 27 px at 2.0 m; stage 2 worked at about 40 px with 2x upsampling. Mount as
  low as still covers the cage, and fit a larger tag if the top plate has room (100 mm gives 44 px
  at 1.5 m).
- **Frames are already rectified.** The SDK delivers rectified images with zero-distortion
  intrinsics, which PnP uses directly. Check whether `sl::CameraOne` can also deliver unrectified
  frames; if it can, record those and the factory calibration instead, so the lens model can be
  revised later, as the e-CAM25 path does.
- **Everything plugs into one box.** The camera is on GMSL2, the radio takes the single USB3
  port, the AX210 WiFi joins `MR-STABS` for the firmware stream, and ethernet stays up for ssh
  and for moving recordings off the box.

### Recorder

`playground/calibration/apriltag_track.py` was deleted in e56a4464 (2026-08-18) when the velocity
jig tool replaced it. Its non-`--drive` mode is what hand driving needs: it records raw frames on
`/camera/image`, the floor board lock on `/floor/image`, and the transmitter channels read-only on
`/transmitter/channels`, all on CLOCK_MONOTONIC, in the layout `auto_battlebot/calibration/apriltag/
apriltag_mcap.py` still defines. `analyze_apriltag_mcap.py` still reads that layout.

Restore it with these changes:

1. **A ZED X One source.** The recorder had OAK and generic OpenCV sources. Add one on
   `sl.CameraOne` (pyzed 5.2 is installed for the box's system Python 3.10; confirm the venv can
   import it). The main app cannot record these frames yet: on the box it logs `Video recording
   requested but no encoder started`. Stamp each frame with the SDK image timestamp and also log
   the CLOCK_MONOTONIC time it was retrieved, since the SDK stamps in its own clock and the other
   streams use CLOCK_MONOTONIC. Put the camera serial and the SDK's intrinsics in
   `/calibration/metadata`.
2. **Log the firmware diagnostics stream.** This path is independent of the Crossfire control link:
   the ESP32 hosts its own 2.4 GHz access point (`MR-STABS`), and the box joins it on its AX210.
   The recorder sends `GET /record/start` so the stream sends every control loop instead of at
   10 Hz, holds the server-sent-events stream at `/events` open, and writes every event to a new
   `/robot/diagnostics` topic stamped with the host CLOCK_MONOTONIC receive time as well as the
   robot's `timestamp_ms`. It sends `GET /record/stop` on exit. `left_cmd` and `right_cmd` become
   the fit's command tape. BNO055 orientation gives yaw and pitch on board, a second measurement
   of both.
3. **Keep `/transmitter/channels`** for clock alignment and as a fallback, with the radio on the
   box's USB port. The MCAP docstring says it holds "stick axes the driver was commanding", which
   may be pre-mixer. With the firmware log that no longer matters for the fit, but if the
   diagnostics stream drops events, the fallback needs the radio model in pass-through (100%
   rates, no expo, no trims, no slow-up) to be usable.
4. **Record uncompressed grayscale frames** at 60 fps. JPEG moves the corner estimates the
   subpixel refinement keys on (see the `apriltag_mcap.py` docstring), and AprilTag detection only
   needs one channel. Full-resolution gray is 2.3 MB per frame, 138 MB/s, about 33 GB for a
   4-minute session. The box has about 225 GB free, so move each session to the dev machine over
   ethernet between sessions (about 5 minutes at gigabit). If the write rate doesn't keep up, store
   a lossless crop around the live-tracked tag plus a full frame every half second, with the crop
   offset in each message. `analyze_apriltag_mcap.py` needs `mono8` support either way.
5. **Keep the full 3D tag pose.** `analyze_apriltag_mcap.py` solves the tag pose with PnP and then
   projects to (x, y, yaw). Also write pitch and roll to the truth CSV. Nose lifts during hard
   throttle are the data that pins the COM, the reflected inertia and traction together, and the
   rigid-body model is the first consumer that can use them.
6. **Live coverage display.** Replaces the scripted protocol as the thing that guarantees coverage.
   While recording, bin the (linear, angular) command and the solved body speed and show which
   cells have less than N seconds of steady, in-frame data. The driver drives toward the empty
   cells. The box is headless, so serve it as a small web page or through Foxglove.
7. **Session metadata.** Pack voltage at start and end, auto-steer state, ESC stop thresholds,
   AM32 settings, radio mixer settings, camera height and exposure, surface (which floor, cleaned
   or not), tire condition, robot mass, and free-text notes. Stored in `/calibration/metadata`.

### Setup

- ZED X One S overhead, pointing straight down, high enough to see the whole cage and no higher.
  The 3x5 floor GridBoard (ids 160 to 174) is locked once then removed, as in the stage-2 runs.
  Robot tag id outside that range.
- Mark the edge of the tracked area on the floor if the view does not cover the whole cage.
- Walls: keep the robot off them for the fitting sessions. Wall contact is a later stage.
- **Auto-steer on** (flip switch DOWN), the competition setting. Heading hold is also what makes
  straight runs possible: stage 2 could not measure forward or reverse speed because the robot
  spun off line. A few minutes with auto-steer off are useful as a cross-check of the drivetrain
  fit, but not required.
- Diagnostics stream in recording mode, logged per recorder change 2.
- Radio in pass-through mode, so the fallback path in recorder change 3 stays usable.

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

- Leaving the camera's view, if it does not cover the whole cage. Frames without the tag are
  interpolation, not measurement.
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
2. Clock alignment: map robot `timestamp_ms` onto CLOCK_MONOTONIC by cross-correlating the
   firmware's `a_percent`/`b_percent` against `/transmitter/channels`, then refine by matching
   BNO055 yaw rate against tag yaw rate. The first offset is the radio link delay; the refinement
   checks it. Report both, since the closed-loop sim needs the radio delay and the drivetrain fit
   needs only what follows the firmware.
3. Command tape: `left_cmd` and `right_cmd` from the firmware log, divided by 100. These are the
   per-motor commands after the PID and the mixer, so the drivetrain fit is open loop even with
   auto-steer on. The remaining delay (ESC and mechanics) is not applied here; the fit shifts the
   tape per candidate.
4. Windows: reuse `plant.make_windows` with window lengths of 1 to 2 s. Gate out:
   - windows where tag visibility is below a threshold (start at 70%, tune on the noise floor)
   - windows within a robot length of a wall or the taped edge
   - windows where pitch exceeds a few degrees, except in the separate nose-lift set
   - windows with gaps in the firmware log
5. Initial state per window: pose from the truth CSV, body velocity from a smoothing spline over
   the in-frame detections, wheel speeds from the no-slip relation (v +/- omega * 0.06526) / 0.025.
   The rotor state follows the wheels through the armature.
6. Noise floor: pose residual against the smoothed track on held-still segments and on steady
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
  current limit. `u` is the per-motor command (`left_cmd` or `right_cmd` / 100). Deadzone and the
  throttle curve are applied to the command tape before it becomes `ctrl`. Joint `frictionloss`
  and `damping` carry gearbox friction.
- **Firmware layer**, for closed-loop use only: a per-step function that takes the stick commands
  and the sim's yaw and reproduces `mix_motor_outputs` (the 1% turn threshold, 0.25 s cooldown,
  PID with its 2 degree deadband, and the 100% normalization) to produce `ctrl`. The fit does not
  use it, since the fit replays logged per-motor commands.
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
| Drivetrain delay (firmware to motion) | 0 to 60 ms | profiled on a grid, as `jig_fit` does; the radio part of the 60 ms comes from clock alignment |
| Deadzone per motor | 0 to 0.06 | ESC stop threshold is set to 0.01; stage 2 says at or below 0.04 end to end |
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
   armature scale near 1; radio plus drivetrain delay near 60 ms; coast tau near 0.078 s.
4. **Nose lift matches.** The fitted model lifts the nose on the same held-out punches that lifted
   the real robot, and not on the ones that didn't.
5. **The firmware layer matches.** Driven by the logged sticks and the fitted drivetrain in closed
   loop, the sim's `pid_output` and per-motor commands track the logged ones on straight holds.
   This is the check that the sim will behave like the robot once a controller drives it.
6. **The parameters are identified.** CMA-ES restarts agree. Report the parameter pairs that trade
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
3. **Restore the recorder** on the ZED Box Mini with the seven changes above. Three to four days:
   the ZED X One source, the coverage display and the diagnostics logging. Before the first
   session, check three rates on the box: 60 fps gray frames written to NVMe without drops, the
   diagnostics stream keeping up with the control loop over WiFi, and tag detection at the chosen
   camera height with the robot at full speed.
4. **Record.** Two or three evenings of driving to get 10 to 15 sessions.
5. **Truth, windows and noise floor.** Extend the analysis to 3D pose; build and gate windows. One
   day.
6. **Grey-box baseline** on the same windows. Half a day; the code exists.
7. **MuJoCo fit**, staged. Two to three days including iteration.
8. **Validation and report.** One day.
9. **Hand-off**: the passing parameter spread goes into the learned-sim randomization ranges.

## Risks and open questions

- **Tag size in pixels.** The ZED X One S sees the whole cage, which fixes stage 2's field-of-view
  problem, but the tag shrinks to 27 to 44 px depending on height. If detection drops at speed,
  lower the camera until the view just covers the cage, fit a larger tag, or bin to 960x600 at
  120 fps only if the higher rate buys more than the resolution costs. The BNO055 orientation in
  the diagnostics stream still measures yaw and pitch through any dropout, but not position.
- **SDK clock.** Frames carry SDK timestamps; everything else is CLOCK_MONOTONIC. The recorder
  logs both for each frame, but check the offset stays constant across a session before trusting
  the delay fit.
- **Closed-loop data.** A human driver reacts to the robot, so commands correlate with past
  disturbances. Open-loop multi-step scoring on the recorded tape limits the bias, but it is why the
  punches and reversals matter: they are the least reactive inputs in the menu.
- **Diagnostics stream reliability.** The stream runs over the robot's own WiFi access point from
  the same ESP32 that runs the control loop. If recording mode drops events or slows the loop,
  fall back to the transmitter channels with the radio in pass-through and reproduce the firmware
  layer in the fit, which makes the PID part of what has to be right.
- **Friction identifiability.** Without the sled test, wheel friction and armature can trade off on
  the punch data. The sled test fixes friction from outside.
- **BNO055 yaw.** The PID acts on the BNO055's fused heading, which has its own lag and drift. The
  firmware layer in the sim uses the sim's true yaw until the logged orientation shows how much
  those matter.
- **MuJoCo Warp maturity.** If per-world model fields don't cover what's needed, the fallback is an
  outer loop over candidates, which costs throughput but not correctness.
