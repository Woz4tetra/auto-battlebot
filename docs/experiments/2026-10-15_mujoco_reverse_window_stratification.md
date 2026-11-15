# 2026-10-15 mujoco reverse window stratification

## Notes moved from code comments (2026-11-15)

### `playground/calibration/fit_mujoco_plant.py`

Strata are drive direction (forward, reverse, spin, other) crossed with flat and nose-lift.
Each gets an equal share of the limit; a stratum with fewer windows than its share gives the
rest to the others. A plain random draw followed the session's mix, which is mostly forward:
the 2026-10-15 fit got 5 reverse windows out of 64, one of them a reversing turn, and its
MuJoCo model turned 24 deg too far in median over 0.4 s on reverse windows against 4 deg
forward.

