# Mrs Buff Mk3 model build details

## Model build (`auto_battlebot/mujoco_sim/build_mrs_buff_mk3_model.py`)

Build the Mrs Buff Mk3 model: mass properties, collision pieces, and the MJCF.

```
Writes, under simulation/assets/robots/mrs_buff_mk3/:
    mass_properties.toml       lumped bodies, wheel geometry, drivetrain, weapon sweep
    collision/hull_*.obj       convex pieces of the chassis structure, body frame
    collision/disk_*.obj       convex pieces of the weapon disk, body frame
    collision/collision.toml   piece list
    mrs_buff_mk3.xml           the model at the nominal PlantParams
```

```
The body frame is FLU at the footprint centre, at axle height: 15.75 mm ahead of the midpoint
between the axles. The Onshape assembly has +X to the robot's left and -Y forward, so
x = -Y_asm, y = +X_asm, z = +Z_asm, the same rotation as Mr Stabs Mk2's export.
```

Mass properties come from Onshape's Mass and section properties panel, not the URDF: the URDF export sums to 708 g against the assembly's 1,332 g, because the ESCs, batteries, wheel cores and weapon motor export at zero mass and the drive motors are not in it at all. The panel reports the inertia tensor about the reference mate connector's origin, not about the COM, so each tensor is shifted to its COM here (parallel axis). The wheel was measured against a connector on its own axle, which keeps that shift small; the screenshots are in onshape_export/.

The weapon does not spin in this model: the disk and its rotating parts are lumped into the chassis, and its CoACD pieces collide as chassis geometry. Its sweep (the cylinder the disk fills when spinning) is recorded for the strike reward.

The drive motors (Blitz Pro 24 mm, 650 KV, 5.2:1) turn the back wheels; 120XL belts turn the front wheels 1:1. No rotor inertia is published for the 2306 motor: 3.0e-6 kg m^2 is Mr Stabs Mk2's CAD-integrated 1806 rotor (9.3e-7) scaled by bell mass and radius, and the fit's armature scale absorbs the error.

```
    python -m auto_battlebot.mujoco_sim.build_mrs_buff_mk3_model            # reuse collision pieces
    python -m auto_battlebot.mujoco_sim.build_mrs_buff_mk3_model --split    # rerun CoACD
```

## Onshape sources

- Assembly to body: the footprint centre sits at Y_asm = 80.29 mm, at the axle height Z_asm = -30.5.
- Whole assembly: Onshape panel readings against the footprint-centre connector (x forward, y left, z up), screenshot 2026-10-25 10:24.
- Weapon: the disk and every part that turns with it, screenshot 2026-10-25 10:23.
- Wheel mass: one wheel (tread, core, hub plate, clamp hub, two M3 screws) against a connector on its axle, z along the axle, screenshot 2026-10-25 10:35. The four wheels are identical. The front wheels' 6 mm shoulder screws are axles that do not turn, so they stay in the chassis.
- Wheel COM in the footprint frame: screenshot 2026-10-25 10:21, back right wheel, same parts.

## Interim plant fit command tapes (`auto_battlebot/calibration/fit_mrs_buff_mk3_interim_plant.py`)

Each window's linear and angular commands are piecewise constant on the plant time step grid. Each window draws one family, the way the jig protocol excites one term at a time: straight steps and reversals, spins, arcs on the steer/brake grid, and staircases near the deadzone. Holds end in zero about a third of the time, so the coast or brake decel is in the data.
