# 2026-09-25 mr stabs mk2 mass properties

## Notes moved from code comments (2026-11-15)

### `playground/calibration/build_mr_stabs_mjcf.py`

- -x sin(t) + z cos(t) = -r  ->  solve for the smallest t in [0, pi/2) (at `amp = np.hypot(x, z)`)
- -x sin t + z cos t = amp cos(t - alpha) with alpha = atan2(-x, z) (at `alpha = np.arctan2(-x, z)`)

### `playground/calibration/lump_mr_stabs_mass_properties.py`

- Onshape "Mass and section properties" on the whole Mr Stabs Mk2 assembly, screenshot 2026-09-25 23:10 (onshape_export/onshape_mass_properties_2026-09-25.png). Assembly frame, mm and g mm^2, inertia about the COM. Unlike the URDF export, this includes the Onshape mass overrides for the motors, ESCs, flight controller, batteries and switch. (at `ONSHAPE_MASS_G = 494.22`)
- Repeat Robotics Compact 1806 gearmotor (repeat-robotics.com/products/repeat-compact-1806). (at `GEAR_RATIO = 22.6`)
- CAD integration of the rotor: steel flux ring, 14 N52 magnets, aluminum end cap, steel shaft. (at `ROTOR_INERTIA_KG_M2 = 9.3e-7`)

