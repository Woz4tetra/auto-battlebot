# Mass property lumping details

## Mass property lumping details

### `playground/calibration/build_mr_stabs_mjcf.py`

- `rest_contact`: -x sin(t) + z cos(t) = -r -> solve for the smallest t in [0, pi/2)
- `rest_contact`: -x sin t + z cos t = amp cos(t - alpha) with alpha = atan2(-x, z)

### `playground/calibration/lump_mr_stabs_mass_properties.py`

- `ONSHAPE_MASS_G`: Onshape "Mass and section properties" on the whole Mr Stabs Mk2 assembly, screenshot 2026-09-25 23:10 (onshape_export/onshape_mass_properties_2026-09-25.png). Assembly frame, mm and g mm^2, inertia about the COM. Unlike the URDF export, this includes the Onshape mass overrides for the motors, ESCs, flight controller, batteries and switch.
- `GEAR_RATIO`: Repeat Robotics Compact 1806 gearmotor (repeat-robotics.com/products/repeat-compact-1806).
- `ROTOR_INERTIA_KG_M2`: CAD integration of the rotor: steel flux ring, 14 N52 magnets, aluminum end cap, steel shaft.
