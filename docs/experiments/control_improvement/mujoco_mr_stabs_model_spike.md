# Mr Stabs Mk2 MuJoCo model: mass properties and Warp spike

Steps 1 and 2 of `docs/plans/mujoco_warp_mr_stabs_plan.md`, run 2026-09-26 on the dev laptop
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

## Next steps

1. Measure the printed tag and correct `tag_size_m` in `config/mr_stabs_mk2_sysid_zed_box.toml`.
2. Run the step 5 dry run, then check the rest pitch against the smoother's still segments
   (expect about 0.19 rad).
3. Rerun `mujoco_warp_spike.py` on an A6000 to size the batch.
