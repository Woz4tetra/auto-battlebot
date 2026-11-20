# Mrs Buff Mk3 sim plant values

Provenance for the plant and latency values in `simulation/sim_mrs_buff_mk3.toml` and `simulation/sim_mrs_buff_mk3_rl.toml`. Key meanings are in `docs/reference/simulation/sim_profiles.md`.

- Source: model M4 of `playground/calibration/out/plant_stageA.toml`, velocity jig sessions 2026-08-19 through 2026-08-23, guard plates on.
- `command_ms = 52.2094`: jig onset stack, 52.2 ms over 3486 edges at SNR 22.4, clock measured.
- `tau_linear_accel = 0.14921`: +/- 0.0738 (n=13). The weakest number in the set.
- `tau_linear_decel = 0.123461`: the coast term. It drives stopping overrun.
- `steer_brake_coeff = 2.70197`: forward authority is gone at `|u_ang_eff|` = 0.370.
