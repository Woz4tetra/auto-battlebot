# Kinematic sim scenario files

`simulation/kinematic_sim_server.py` reads these files. Field meanings per dataclass are in `kinematic_sim_config.md`. This page covers what each scenario sets and why. Pass a file to `./scripts/run_simulation.sh`.

## `simulation/sim_mrs_buff_mk3.toml`

Fast headless kinematic sim for Mrs Buff Mk3 at 30 Hz.

| Field | Meaning |
| --- | --- |
| `[arena]` | Keep in sync with `[field_filter]` in `config/simulation/headless_sim.toml`. |
| `[sim] dt` | 0.0333 s, about 30 Hz. Matches the real controller rate. |
| `[latency] command_ms` | Command issue to wheel response, in ms of sim time. The server quantizes it to whole ticks, so at `dt = 0.0333` it applies as 2 ticks (66.6 ms). Measurement in `docs/experiments/control/2026-11-19_mrs_buff_mk3_sim_plant_values.md`. |
| `[camera] fov, pos, lookat` | Overhead and slightly back, the way the ZED mounts over the cage. The UI draws its overlay on this frame, so the pose decides whether the arena is in shot. The projection-bias model also reads it. |
| `[our_robot]` plant keys | Transcribed from `playground/calibration/out/plant_stageA.toml` (model M4, velocity jig sessions 2026-08-19 through 2026-08-23). Edit the fit, not this file. Measured with guard plates on, so their ground friction is part of the plant. Name mapping below. |
| `max_linear_speed`, `tau_linear`, `tau_angular` | Symmetric fallbacks. Unused once the `_fwd`/`_rev` and `_accel`/`_decel` keys are set. |
| `steer_brake_coeff`, `tau_linear_accel`, `tau_linear_decel` | Fit provenance in the dated experiments doc above. |
| `[perception] sensing_model` | Perception latency, dropouts and noise come from a sensing model (`docs/experiments/control/2026-11-15_kinematic_sim_design.md`). Empty sends the true poses every tick. Pair a model with `config/simulation/kinematic_sim_sensing.toml` on the C++ side so the real keypoint filter and motion estimator see the measurements. `simulation/sensing_models/provisional.toml` is the usual non-empty value. |

Fit name to key name:

| Fit parameter | Key |
| --- | --- |
| `k_fwd`, `k_rev` | `max_linear_speed_fwd`, `max_linear_speed_rev` |
| `k_ang` | `max_angular_speed` |
| `tau_lin_a`, `tau_lin_d` | `tau_linear_accel`, `tau_linear_decel` |
| `tau_ang_a`, `tau_ang_d` | `tau_angular_accel`, `tau_angular_decel` |
| `c_sb` | `steer_brake_coeff` |
| `c_ad` | `angular_droop_coeff` |
| `dz_lin_fwd`, `dz_lin_rev` | `deadzone_linear`, `deadzone_linear_rev` |
| `dz_ang_l`, `dz_ang_r` | `deadzone_angular`, `deadzone_angular_right` |

## `simulation/sim_mrs_buff_mk3_rl.toml`

Mrs Buff Mk3 against the kinematic sim, in the world her RL policy trained in (`auto_battlebot/rl/envs/warp_nav.py`, stage C). Run it with:

```bash
./scripts/run_simulation.sh simulation/sim_mrs_buff_mk3_rl.toml config/simulation/mrs_buff_mk3_rl_sim.toml
```

The policy trained on a MuJoCo model of Mk3 fitted to the jig plant. The interactive server's MuJoCo plant is Mr Stabs Mk2's, so this file runs the jig plant itself on the kinematic model, as `sim_mrs_buff_mk3.toml` does. The plant, camera, latency and `[perception]` keys carry the same meanings as above.

| Field | Meaning |
| --- | --- |
| `[arena]` | Keep in sync with `[field_filter]` of the C++ config pair. |
| `[sim] dt` | 0.02 s, 50 Hz. The RL policy steps every 20 ms and the lockstep eval ticks at this rate. |
| `[latency] command_ms` | Applied as 3 ticks (60 ms) at this `dt`, the RL environment's nominal. |
| `[perception] sensing_model` | `simulation/sensing_models/provisional_rl.toml`: Mr Stabs Mk2's lockstep-fitted EKF error with the measured dropout gaps (`warp_nav.PROVISIONAL_RL_SENSING`). This is the table the Mk3 policies trained on. |
| First `[[opponents]]` | A copy of Mrs Buff Mk3 driven by the RL training opponents. `trained_behaviour` is one of `static`, `straight`, `circle`, `random_walk`, `pursuit`, `corner`, `replay`, or `any` to draw one per client connection (the server prints the draw). `collision_box` is the CAD footprint the C++ pair's robot filter restates. |
| Second `[[opponents]]` | The house bot, parked as in half the training worlds, at its 0.2083 m radius (`HOUSE_RADIUS_M`). The C++ pair expects it as the third ground-truth slot (`NEUTRAL_ROBOT_1`). |
