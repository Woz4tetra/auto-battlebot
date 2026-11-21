# Provisional our-robot EKF error fits

Provenance for the `[robots.ours.ekf_error]` tables in the two provisional sensing models. Both come from `auto_battlebot/rl/fit_provisional_ekf_error.py`, which scores the C++ filter's output at control time against sim truth for all five components (along, cross, theta, v, omega). Both are placeholders until the sensing plan step 5 fit replaces them.

## `simulation/sensing_models/provisional_rl.toml`

- Source: 53 lockstep episodes (`lockstep_record`) of the Mr Stabs Mk2 setup.
- Scenario set and the comparison against the sysid table: `docs/experiments/control/rl_navigation_report.md`.

## `simulation/sensing_models/provisional_rl_mrs_buff_mk3.toml`

- Source: 127 lockstep episodes of Mrs Buff Mk3 (`lockstep_record`, 2026-10-25, bundle `rl_nav_mk3_c_near`).
- Heading error runs 2 to 4 times the Mr Stabs Mk2 table, most of it while spinning in place.
- Cause: the filter predicts with the jig plant (31.7 rad/s full spin) and the MuJoCo plant spins at 26.1 rad/s.
