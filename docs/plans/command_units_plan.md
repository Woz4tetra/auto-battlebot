# Navigation in real units

Navigation outputs body velocity in m/s and rad/s. A new command shaping stage between navigation and the transmitter turns it into stick values in [-1, 1] using the plant.

## Today

`VelocityCommand` has fields named `forward_mps`, `lateral_mps`, and `yaw_rate_radps`, but every producer writes stick values and every consumer reads them as sticks:

| Component | What it writes or reads |
| --- | --- |
| `MotionProfileNavigation` | Divides by the plant: `u_ff = (v_ref + tau * dvdt) / k_fwd`, plus a PI term in stick units |
| `PursuitNavigation` | PD heading loop in stick units (`angular_kp = 3.0`), `max_linear_command` as a stick limit |
| `RlPolicyNavigation` | Writes the policy action straight in: `VelocityCommand{action_.linear, 0.0, action_.angular}` |
| `HazardAvoidance::limit_command` | Converts its m/s bound to sticks with `k_fwd` / `k_rev`, then clamps `forward_mps` |
| Transmitters, drive processors | `saturate_velocity` clamps to [-1, 1], then mixes as sticks (`src/transmitter/drive_mixing.cpp`) |

The plant already has the forward map, `plant_stick_to_body_velocity` (`src/plant/plant_params.cpp`), which the estimator uses to read the commands it sent.

## After

```
navigation ──► BodyVelocityCommand (m/s, rad/s) ──► hazard barrier (m/s) ──► CommandShaper(plant) ──► StickCommand [-1, 1] ──► transmitter
```

- `CommandShaper` is a new module (`include/command_shaper/`, `src/command_shaper/`), with an interface, a plant-backed implementation, and a factory. The plant-backed implementation inverts `plant_stick_to_body_velocity`: `k_fwd` or `k_rev` by sign for linear, `k_ang` for yaw. It then saturates, using the `velocity_saturation_limit` headroom rule that lives in the drive processors today.
- `NavigationInterface::update` returns a body velocity type in real units. `VelocityCommand` keeps its field names, which become true.
- `TransmitterInterface::send` and `DriveProcessorInterface::process` take `StickCommand`. `saturate_velocity` moves into the shaper, and the drive processors keep mixing and the deadzones.
- `HazardAvoidance` works in m/s directly, so its `k_fwd` / `k_rev` conversion goes.
- The control loop calls navigation, then the shaper, then `transmitter_->send`.

Per navigator:

- **`MotionProfileNavigation`** outputs `v_ref + tau * dvdt` plus the PI term scaled by `k_fwd`. The output stays the same once the shaper divides by `k_fwd`.
- **`RlPolicyNavigation`** scales the action by the plant (`k_fwd` or `k_rev`, `k_ang`). The policy was trained on sticks, so the round trip through the shaper has to be exact. A test pins that.
- **`PursuitNavigation`** gets its gains converted: `angular_kp` is multiplied by `k_ang`, and `max_linear_command` becomes `max_linear_speed_mps`. The values change in every pursuit profile.
- **`NoopNavigation`** outputs zero.

## Decisions

1. **Profiles without a `[plant]`.** The shaper needs one. I found `config/simulation/kinematic_sim.toml`, `config/simulation/mr_stabs_mk2_rl_sim.toml`, `config/simulation/mrs_buff_mk3_kinematic_sim.toml`, and `config/playback/mr_stabs_mk2_sysid_playback.toml` without one in a rough pass; I'll confirm with the config loader before changing anything. Options: require `[plant]` everywhere and fail at parse (my pick), or default to unit gains.
2. **Pursuit retune.** Converting the gains keeps today's behaviour only if the plant's `k_ang` and `k_fwd` match the robot. Options: convert numerically and replay to confirm (my pick), or retune by hand.

## Steps

Each step is one commit, with C++ tests passing in `build-test/` and `build/`.

1. Add `StickCommand` to the transmitter and drive processor interfaces, and move `saturate_velocity` into a `CommandShaper` that is an identity for now. No behaviour change.
2. Make the shaper plant-backed and require `[plant]` in every profile.
3. Convert the navigators one at a time, each with a test that the old stick output equals the new real-units output after the shaper: `MotionProfileNavigation`, `RlPolicyNavigation`, `PursuitNavigation`.
4. Move `HazardAvoidance` to m/s.
5. Replay `config/playback/mr_stabs_mk2_playback.toml` and run the kinematic sim before and after, and compare the transmitted sticks.
