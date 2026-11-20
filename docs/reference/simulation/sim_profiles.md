# Sim profiles

`simulation/kinematic_sim_server.py` reads these two TOML files. Field meanings are in `kinematic_sim_config.md`. This page holds why each profile sets what it sets.

Run either profile with the C++ side:

```bash
./scripts/run_simulation.sh simulation/sim_mr_stabs_mk2.toml config/simulation/mr_stabs_mk2_rl_sim.toml
./scripts/run_simulation.sh simulation/sim_mr_stabs_mk2_mirror.toml config/simulation/mr_stabs_mk2_rl_sim.toml
```

## `simulation/sim_mr_stabs_mk2.toml`

Mr Stabs Mk2 on the MuJoCo rigid-body plant (`auto_battlebot/mujoco_sim`, `docs/guides/sysid_guide.md`). It matches the world the RL policy trains in (`auto_battlebot/rl/envs/warp_nav.py`, `docs/experiments/control/rl_navigation_report.md`): the 2026-10-15 plant fit, 50 Hz ticks, 60 ms actuation latency, the `provisional_rl` sensing model, a moving opponent our robot can hit, and the house bot. It has no static hazards. The training mix draws 0 to 3, and this profile is the 0 case.

| Key | Meaning |
| --- | --- |
| `[arena]` | Keep in sync with the C++ pairs' `[field_filter]` `size_x` and `size_y`. |
| `[sim] dt` | 0.02 s (50 Hz). The RL policy steps every 20 ms and the lockstep eval ticks at this rate. |
| `[latency] command_ms` | 59.3 ms. The measured total is 60 ms (Crossfire + ESC + mechanical), the RL environment's nominal value. The MuJoCo plant holds the firmware-to-motion part itself (the fit's `delay_s`, 0.65 ms), so this value is the radio part only. Set it to 60 with `plant = "kinematic"`. |
| `[camera] pos, lookat, fov` | Overhead and slightly back, the way the ZED mounts over the cage. The UI draws its overlay on this frame, so the pose decides whether the arena is in shot. |
| `[our_robot] plant` | `"mujoco"`. Set `"kinematic"` to use the run-175805 kinematic fit in the same table. |
| `[our_robot.mujoco] fit_file` | The 2026-10-15 sysid fit the RL environment trains on (`warp_nav.DEFAULT_FIT`). |
| `[our_robot.mujoco] timestep` | 0.002 s, the RL environment's step. Position error against 1 ms is 0.3 mm at 0.4 s. |
| `[perception] sensing_model` | `provisional_rl.toml`: our-robot EKF error fitted from lockstep, plus the measured dropout gaps (`playground/rl/fit_provisional_ekf_error.py`), as the RL environment reads it. Pair a model with `config/simulation/kinematic_sim_sensing.toml` on the C++ side so the real keypoint filter and motion estimator see the measurements. Provenance: `docs/experiments/control/2026-11-19_provisional_ekf_error_fits.md`. |

### Kinematic plant keys

Read only when `plant = "kinematic"`. They are the calibrated Mr Stabs Mk2 plant from run 175805.

| Key | Meaning |
| --- | --- |
| time constants | Low confidence: one clean segment each. Physically sensible. |
| `max_linear_speed` | The full-throttle speed the robot achieves. 5.6 m/s is the symmetric fallback, unused once `_fwd` and `_rev` are set. |
| `max_angular_speed` | 61.5 rad/s. May be inflated by yaw-keypoint flips in the calibration. Revisit if turns look unrealistically fast in sim. |
| `tau_linear`, `tau_angular` | Fallbacks for the `_accel` and `_decel` variants. |
| `tau_linear_decel` | 0.078 s. The coast term, which drives stopping overrun. |

### Opponents

| Entry | Meaning |
| --- | --- |
| Mrs Buff Mk3, `behavior = "trained"` | Driven by the RL training opponents. `trained_behaviour = "any"` draws one per client connection and the server prints the draw. Speeds, radii, pursuit lead and delay, and replay tracks are drawn as in training, so `speed` and `radius` in the entry are unused. `trained_holdout = true` draws the evaluation's held-out pursuit corner and replay tracks instead. |
| Mrs Buff Mk3 `hazard_radius`, `collision_box` | `hazard_radius` makes it an obstacle our robot hits instead of driving through. In the MuJoCo plant it is a box turned with the opponent's yaw, the RL environment's opponent (`warp_nav.OPP_SIZE_M`, the Mk3's CAD footprint). The kinematic plant and the clearance score use the 0.168 m disc, the footprint's half-diagonal. |
| "Go to X and stop" mission | Set `behavior = "static"`, `hazard_radius = 0` and `start_pos` to the goal. `sim_stop_test.py` does this. |
| House bot | Parked as in half the RL environment's worlds, at its 0.2083 m radius (`HOUSE_RADIUS_M`). The C++ pairs expect it as the third ground-truth slot (`NEUTRAL_ROBOT_1`). |

## `simulation/sim_mr_stabs_mk2_mirror.toml`

Mr Stabs Mk2 against Mr Stabs Mk2, both robots running the same RL policy. It is `sim_mr_stabs_mk2.toml` with the opponent swapped for `[[opponents]] behavior = "policy"` (`simulation/policy_opponent.py`). `dt`, `command_ms`, camera, kinematic keys and `[perception]` carry the same values and reasons as above.

| Key | Meaning |
| --- | --- |
| `policy_bundle` | Drives a second Mr Stabs Mk2 on the same MuJoCo plant fit and actuation latency, and attacks our robot. Point it at the bundle named in `config/simulation/mr_stabs_mk2_rl_sim.toml` and the two robots run the same policy. |
| Who runs where | Our robot runs in C++ behind the sensing model and the real filter. The opponent's policy runs in the sim on ground truth, the observation the policy trains on with sensing off. |
| Starts | Mirrored across x = 0, with the house bot on the mirror line. |
| `hazard_radius`, `collision_box` | The opponent's footprint (`[robot_sizes]` `MR_STABS_MK2` in `config/_common.toml`): the box our plant collides with. Its pose is the footprint centre. |
