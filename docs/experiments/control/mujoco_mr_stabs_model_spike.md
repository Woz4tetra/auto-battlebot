# Mr Stabs Mk2 MuJoCo model: mass properties and Warp spike

Steps 1 and 2 of `docs/plans/mujoco_warp_mr_stabs_plan.md` (deleted 2026-11-18, in git history), run 2026-09-26 on the dev laptop
(RTX 4080 Laptop GPU, mujoco 3.14.0, mujoco-warp 3.14.0, warp-lang 1.17.0). No recordings exist
yet, so nothing here is fit to data.

## Regenerate

```bash
source scripts/activate_python.sh
python playground/calibration/lump_mr_stabs_mass_properties.py   # mass_properties.toml
python playground/calibration/build_mr_stabs_mjcf.py             # collision/ + mr_stabs_mk2.xml
python playground/calibration/mujoco_warp_spike.py               # checks, timestep study, throughput
```

Inputs are in `simulation/assets/robots/mr_stabs_mk2/onshape_export/`: the Onshape URDF, the six
meshes the scripts read, and the mass-properties screenshot.

## Mass properties

| Body | Mass | COM (mm) | Notes |
| --- | --- | --- | --- |
| Chassis | 459.0 g | (31.94, 0.00, 0.86) | assembly minus wheels, +1.7 g to the scale at its COM |
| Wheel, each | 18.445 g | (-0.1, +/-65.26, 0.0) | hub, tread, clamp hub, two M3 screws |
| Whole | 495.9 g | (29.56, 0.00, 0.80) | |

- The wheel lump reproduces the plan's table exactly (18.445 g, axle inertia 4.2528e-6).
- The chassis inertia is the plan's scaled by the mass ratio 459.0 / 457.3: Ixx 7.798e-4 against
  the plan's 7.769e-4 at CAD mass.
- Onshape's panel shows tensor elements, products included. Subtracting the wheels reproduces
  the plan's chassis Ixz of -2.3e-6 only under that reading.

## Tags

I found each tag's marker frame by rasterizing its CAD face as seen from outside and running the
aruco detector on it. The corner order then fixes the frame, and a mirrored tag would show up.

| Tag | Faces | Center (mm, body) | Tilt | Black square |
| --- | --- | --- | --- | --- |
| 76 | up | (35.6, 0.0, 14.2) | 9.6 deg | 64 mm |
| 41 | floor | (35.4, 0.0, -12.1) | 10.3 deg | 64 mm |

- Neither tag is mirrored.
- The plan says 41 is on top. The CAD has 76 on top, next to the top plate. The C++ model and
  the smoother both take up or down from each tag's rotation, never from the order of the ids.
- OpenCV's marker length is the black square, 64 mm. The plan's 80 mm is the white border. The
  profile uses 0.064, with a note to measure the print.

## Model

- The chassis bottom rises toward the front like a wedge. Resting on the wedge tip, the model
  sits 10.9 deg nose-down (11.35 deg from CAD geometry alone). An upright robot's top tag is
  therefore about 21 deg from vertical, not 9.6 deg. Both IPPE selectors use the rest pitch.
- CoACD makes 16 convex pieces of the chassis and plates (concavity 0.15 at the 16-piece cap).
- Each rotor is a body coupled to its wheel at +22.6 by a joint equality, not wheel-joint
  `armature`. Armature gives the right N^2 J_r at the wheel but no reaction on the chassis.
- A `general` actuator only applies the back-EMF term with `biastype="affine"`. Without it the
  torque stays at gain times volts and the robot flies off.

## Nose lift: the plan's 0.29 g uses the wrong rotor term

The plan puts the chassis reaction at F r + (J_w + N^2 J_r) alpha. The chassis actually reacts
to the rate of change of the rotor's angular momentum, J_r N omega_wheel, so the rotor term is
N J_r alpha, 22.6 times smaller. Working through the gearbox torques gives the same result:
stator plus ring reaction = -(tau_wheel + N J_r alpha).

| Rotor term | Lift threshold |
| --- | --- |
| N J_r (momentum) | 1.203 g |
| N^2 J_r (plan) | 0.299 g |
| none | 1.398 g |

The simulated torque ramp lands on the momentum value: 1.219, 1.213 and 1.220 g at dt 1, 0.5
and 0.25 ms, with a near-frictionless skid (the analysis assumes one). So the reflected inertia
does not explain the backflip through steady acceleration. It still makes the effective mass
about 1.5 kg against the robot's 0.5 kg, which matters for everything else.

The plan's 1.4 g no-rotor figure matches mine. What flips the real robot is open. Candidates:
wheel traction above 1.2 (lift needs wheel mu over about 1.2), or a transient such as a reversal
dumping the rotor's energy. The fit's nose-lift stage will show which.

## Sanity checks

| dt | Lift, skid mu 0.01 | Lift, skid mu 0.3 | Top speed at 15.2 V | Decel tau |
| --- | --- | --- | --- | --- |
| 1 ms | 1.219 g | 0.635 g | 4.021 m/s | 97.0 ms |
| 0.5 ms | 1.213 g | 0.865 g | 4.034 m/s | 99.5 ms |
| 0.25 ms | 1.220 g | 1.091 g | 4.036 m/s | 99.3 ms |

- Ideal top speed is 4.050 m/s (KV x V / N at the wheel radius). The model is within 0.7%.
- Decel tau is traction-limited, not resistance-limited. Back-EMF braking at R = 0.2 ohm asks
  for about 60 N at the tread against about 2.5 N of grip. Without traction, R = 0.87 ohm would
  give the measured 78 ms. That is inside the 0.05 to 1 ohm prior, so the zero-command model
  passes.
- With a realistic skid friction, the sliding skid stick-slips and lifts the nose early, by an
  amount that has not converged at 0.25 ms. Nose-lift windows need dt of 0.25 ms or less, or a
  lower-friction skid material in the model. Flat driving converges at 1 ms.
- A full-voltage step from rest wheelies the model over. The top-speed check ramps the voltage.

## MuJoCo Warp

- All nine fitted fields batch per world: `geom_friction`, `geom_solref`, `geom_solimp`,
  `dof_damping`, `dof_frictionloss`, `body_inertia` (rotor, for the armature scale),
  `actuator_gainprm`, `actuator_biasprm`, `actuator_forcerange`. There is no outer loop over
  candidates.
- Throughput with the whole step captured as a CUDA graph: 1.14 M world-steps/s at 1,024
  worlds, 1.64 M at 4,096, 1.78 M at 8,192.
- A generation of 128 candidates x 64 windows x 1.0 s at dt 1 ms is 8.2 M world-steps, about
  4.6 s on this laptop. At 0.25 ms for nose-lift windows it is four times that. The A6000s
  should be faster; measure there before sizing a run.
- GPU and CPU rollouts agree within 1 to 3 cm over 1 s on straight and curving drives. They
  diverge after about 150 ms on a spin in place where the skid stick-slips (float32 against
  float64 on a chaotic contact), so spin windows carry more loss noise.
- `njmax` and `nconmax` are per world in this version (24 contacts, 160 rows).

## Implementation notes

Design notes for the MuJoCo model code under `auto_battlebot/mujoco_sim/`.

### `mujoco_sim/actuator.py`

DC gearmotor as a MuJoCo `general` actuator, and the command tape that drives it.

A brushless motor behind a synchronous-rectifying ESC is, averaged over a PWM period, a voltage
source behind a resistance with back-EMF. At the wheel:

    torque = (eta N Kt / R) * V  -  (eta N^2 Kt Ke / R) * qvel

which is affine in the control and the joint speed, so it is exactly MuJoCo's `general` actuator
with `gainprm[0] = eta N Kt / R` and `biasprm[2] = -eta N^2 Kt Ke / R` (Ke = Kt in SI units).
The control is the applied voltage V; `forcerange` carries the current limit.

The command tape turns the logged per-motor percent (after the firmware mixer) into V: delay,
deadzone, a one-term throttle curve, the left/right gain ratio, and the logged pack voltage.
A zero command either shorts the motor (brake: V = 0, the back-EMF term brakes) or lets it
freewheel (coast). Coast has no fixed-parameter form, so the rollout sets V to the back-EMF
voltage N Ke qvel on those steps, which zeroes the motor torque.

#### `DEFAULT_CURRENT_LIMIT_A`

The default current limit is 35 A, the rating of the ReadyToSky ESC. The phase-current limit the
ESC enforces is not published.

#### `PlantParams`

`curve` is the throttle curvature: `g(x) = x (1 + curve (1 - |x|))`.

### `mujoco_sim/checks.py`

#### `analytic_lift_accel`

`analytic_lift_accel` returns the forward acceleration (m/s^2) at which the skid unloads, from
moments about the axle.

With the robot at its rest pitch: the drive reaction on the chassis, F r plus the spin-up
torque of whatever turns with the wheel, plus the chassis's own inertial load at its COM
height, against gravity at the COM's lever arm.

`rotor` picks the rotor's share of the spin-up torque:

- `"momentum"`: N J_r alpha. The rotor's angular momentum is J_r N omega_wheel, and the
  gearbox housing reacts exactly its rate of change. This is the physics.
- `"reflected"`: N^2 J_r alpha, the plan's first estimate (0.29 g). N^2 J_r is the inertia
  the motor pushes against at the wheel, not what the chassis reacts.
- `"none"`: no rotor at all.

#### `simulated_lift_accel`

Ramp wheel torque until the nose unloads; return the forward accel it took.

The actuators are switched to pure torque (gain 1, no back-EMF) so the ramp is exact, and
the wheels get `wheel_mu` so they don't spin out before the nose can lift. The analytic
threshold assumes a frictionless nose, so the hull and skid get `skid_mu` near zero: with a
realistic 0.3 the sliding skid stick-slips, rocks the chassis, and trips the lift early by an
amount that depends on the timestep (0.64 g at 1 ms, 0.93 g at 0.25 ms against 1.20 g).
The nose counts as lifted once the pitch sits `lift_deg` above rest for 20 ms; the
acceleration is a line fit to the chassis speed over the 100 ms before that. The skid
contact force can't mark it: the sliding skid chatters on and off the floor well below
the threshold.

#### `simulated_top_speed`

Forward speed after driving straight at `volts`; joint friction and damping zeroed.

The voltage ramps up over `seconds` and then holds for one more second: a step to full
voltage from rest wheelies the robot over, as the real one does.

### `mujoco_sim/closed_loop.py`

#### `WALL_HEIGHT_M`

The arena walls are 0.3 m tall, enough that a backflip lands inside the arena.

#### `_add_static_geoms`

Walls get zero friction, like the floor. MuJoCo takes the max of the pair, so the robot's own geom
friction governs every wall contact.

### `mujoco_sim/mass_properties.py`

Loader for a robot's simulation/assets/robots/<robot>/mass_properties.toml.

Mr Stabs Mk2's file is generated by playground/calibration/lump_mr_stabs_mass_properties.py from
the Onshape export, in the FLU body frame at the axle midpoint. Mrs Buff Mk3's comes from
playground/calibration/build_mrs_buff_mk3_model.py, in the FLU frame at her footprint centre at
axle height: her motors drive the back wheels, and belts drive the front ones 1:1 (followers).

### `mujoco_sim/mjcf.py`

MJCF for a drive robot, built from its mass-property table and convex collision pieces.

The chassis on a free joint with the full CAD inertia tensor, two driven wheels on hinges about
y, and one rotor body per gearmotor, coupled to its wheel at the gear ratio by a joint equality.
A robot with belt-driven front wheels (Mrs Buff Mk3) adds two follower wheels, each held to its
side's driven wheel 1:1 by another joint equality; the motors drive only the back pair.

The rotor is a body rather than wheel-joint `armature`. Armature adds N^2 J_r to the wheel's
joint inertia, which is right for how hard the motor must push, but passes no reaction to the
parent body. The chassis does react to the rotor spinning up, by the rate of change of the
rotor's angular momentum, N J_r alpha (not N^2 J_r alpha, which is 22.6 times larger). A rotor
body keeps both right, and the gyroscopic coupling in yaw with it. The planetary gearbox turns
the rotor the same way as the wheel, so the coupling coefficient is +N.

The chassis collides through its CoACD pieces, and Mr Stabs Mk2 adds a small sphere at the nose
skid, where she rests (the COM sits ahead of the axle). A four-wheeled robot has no skid. Robot
geoms carry the friction; the floor's is zero, so MuJoCo's max-combination rule leaves the
robot's value in charge of each contact.

Geom and joint names are stable: the rollout looks them up to batch per-world parameters.

#### `ROTOR_MASS_KG`

The rotor mass is 1e-4 kg. The rotor's mass is already in the CAD chassis; the body only needs to carry inertia.

### `mujoco_sim/onshape_export.py`

#### `body_from_root`

Rotation and translation taking root-frame points into the body frame.

The origin is the axle midpoint: the wheel joints sit on the axle line, and the midpoint
along it is halfway between the two lumped wheel COMs.

#### `extract_tag_frame`

Find a tag's marker frame by rendering its CAD face and running the aruco detector.

The export does not say which in-plane direction is the printed image's "up", so the face
is rasterized as seen from outside (looking along -normal) and decoded; the detected corner
order then fixes the frame. The face is the set of triangles facing `outward` that lie on
the outermost plane.

#### `chassis_from_assembly`

Chassis = Onshape whole assembly minus the wheels, then topped up to the scale.

Returns (chassis at CAD mass, chassis at scale mass). The unmodeled difference between the
scale and CAD (wire, glue, solder) is spread like the rest of the chassis: added at its COM
with the inertia scaled by the mass ratio.

### `mujoco_sim/plots.py`

Figures that make a MuJoCo plant fit and its validation readable at a glance.

``fit_mujoco_plant.py`` calls these from validation, so every validated fit directory gets them:

- ``loss.png``: CMA-ES best loss per generation for every restart, stage by stage. Restarts
  that end at the same loss found the same basin; one far above the rest got stuck.
- ``params.png``: where each restart's best parameters sit inside their search range. A
  parameter the restarts agree on is identified; one scattered across its range is not, and one
  pinned to a bound is fighting the model.
- ``rollouts.png``: held-out windows top-down, the measured robot against the MuJoCo and grey-box
  rollouts from the same start state and the same logged motor commands.
- ``errors.png``: position and heading error at each scoring horizon on the held-out windows, as
  multiples of the measurement noise, MuJoCo against the grey-box.

### `mujoco_sim/rollout.py`

Batched open-loop rollouts of the Mr Stabs Mk2 MJCF on MuJoCo Warp, with a CPU reference.

One world per (candidate, window) pair. Every parameter the fit moves either lives in a Model
field that MuJoCo Warp batches per world (friction, solref, solimp, joint damping and
frictionloss, rotor inertia, actuator gain, bias and force limit) or is folded into the voltage
tape on the host (delay, deadzone, throttle curve, left/right ratio, pack voltage). The
zero-command mode is a per-world flag the control kernel reads each step.

A step is: write ctrl from the tape (the back-EMF voltage on coast steps), mjw.step, record the
chassis pose every `record_every` steps. The whole step is captured once as a CUDA graph and
replayed, which is 3 to 5x faster than launching it from Python.

### `mujoco_sim/session.py`

#### `HORIZON_TOLERANCE_S`

A measurement counts for a horizon when it lands within this of it (60 fps is 16.7 ms).

#### `motor_polarity`

`motor_polarity` is +1 if a positive mean per-motor command drives the robot forward, else -1.

The firmware mixer writes left = -a + turn, and the ESC wiring decides which way that
turns the wheel; the recording settles it by correlating the mean command with the
smoothed forward speed.

### `mujoco_sim/step_response.py`

Mr Stabs Mk2's pitch response to a forward stick step from rest, and an armature refit from it.

A held forward step lifts the nose: the motors' reaction torque pitches the chassis back about
the axle. How far depends on the rotor inertia reflected through the gearbox, which the MuJoCo
fit scales by `armature_scale`, and the sysid fit could not pin it (two restarts, 1.10 and 0.51,
fit equally well). A forward step test pins it: replay each measured stick tape through the
closed-loop sim (`ClosedLoopSim`: firmware mixer, drivetrain delay, command shaping) on open
floor, and pick the armature scale whose peak nose lift matches the measured one.

Pitch follows `pose_from_qpos`: the robot rests nose-down at the collision set's rest pitch, and
lifting the nose makes pitch smaller. Nose lift is the rest pitch minus the lowest pitch reached.

`slew_sweep` runs the same sim with a slew-limited ideal step, to find the loosest host slew limit
(`EspNowTransmitter`, normalized units per second) that keeps the nose down.

#### `loosest_slew`

The highest rate whose lift stays under `limit_deg`, 0 meaning no limit, None if none do.
Assumes lift grows with rate, so it stops at the first rate (from slowest) that fails.

### `rl/envs/warp_nav.py`

#### Contact buffer sizing

`WarpNavEnv` sizes `nconmax` and `njmax` from the geom count. A robot with many collision pieces (Mrs Buff Mk3's 30) pairs each with the floor, walls
and opponent in broadphase, so the buffers grow with the model.

## Next steps

1. Measure the printed tag and correct `tag_size_m` in `config/mr_stabs_mk2_sysid_zed_box.toml`.
2. Run the step 5 dry run, then check the rest pitch against the smoother's still segments
   (expect about 0.19 rad).
3. Rerun `mujoco_warp_spike.py` on an A6000 to size the batch.
