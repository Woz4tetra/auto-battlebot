# Kinematic sim config

`auto_battlebot/kinematic_sim/config/kinematic.py` holds the typed config for the headless kinematic sim (`kinematic_sim_server.py`). `auto_battlebot.kinematic_sim.config.loader.load_config` loads and validates it with dacite in strict mode. Each section below is one dataclass. Keep `[arena]` in sync with `config/simulation/kinematic_sim.toml` on the C++ side.

## `KinematicSimConfig`

| Field | Meaning |
| --- | --- |
| `obstacles_file` | Path to a TOML file holding `[[hazards]]` entries. The C++ field filter reads the same file through `[field_filter].hazards_file`, so the simulated floor and the controller's keep-out discs cannot drift apart. Empty means no obstacles. Obstacle geometry is not duplicated in this config. |

## `SimParamsConfig`

| Field | Meaning |
| --- | --- |
| `dt` | Seconds of sim time per tick. |
| `max_ticks` | 0 runs until interrupted. Above 0 closes the connection after N ticks. It always ends the run: it is an explicit cap, not an outcome. |
| `stop_on_outcome` | What to do when an episode ends early. Today that means falling in a hole. `false` (interactive) reports it, respawns our robot at its start pose, and keeps the connection. Falling in latches the plant's velocity to zero, so without a respawn the robot would sit in the hole for the rest of the session. `true` (batch) closes the connection. `sim_sweep` waits for the binary to exit and reads the `EPISODE` line, so an outcome has to end the run for a sweep to score it. |
| `trace_csv` | Per-tick CSV of the applied command and true poses, rewritten on each client connect. Empty means none. `sim_stop_test.py` scores stops from it. |

## `LatencyConfig`

| Field | Meaning |
| --- | --- |
| `command_ms` | Actuation latency, converted to sim ticks. |

## `CameraConfig`

Noop perception ignores the pixels, but the UI draws its overlay on this frame by projecting field points through these intrinsics. At the old 16x16 placeholder the whole overlay landed inside a handful of pixels and the pane read as black. 640x360 is small enough to stay cheap over the socket and large enough to look like the real camera pane.

## `PlantType`

| Value | Meaning |
| --- | --- |
| `kinematic` | 2D first-order lag, the model the C++ filter and nav assume. |
| `mujoco` | Mr Stabs Mk2 rigid body with the firmware mixer. Mr Stabs Mk2 only. |

## `MujocoPlantConfig`

| Field | Meaning |
| --- | --- |
| `fit_file` | `fit.json` from `playground/calibration/fit_mujoco_plant.py`, repo-relative. The lowest-loss run is used. Empty runs the unfit `PlantParams` defaults. |
| `timestep` | The fit and its checks run at 1 ms. |
| `pack_voltage` | Nominal 4S. The fit's logged `vbat` is not replayed here. |
| `flip_switch` | The transmitter's flip switch (`FlipSwitch`). `down` (power-on, competition) runs the heading hold and reverses the throttle while the IMU reads upside down, so a flipped robot keeps driving wedge-first. `middle` turns both off. `up` always reverses. |

## `PlantConfig`

`mujoco` is read only when `plant = "mujoco"`. The kinematic fields are read only when `plant = "kinematic"`. `start_pos`, `start_yaw_deg`, `sprite` and `radius` apply to both.

| Field | Meaning |
| --- | --- |
| `sprite` | Entry in the sprite manifest the viewer draws for this robot. Sprites are named after the robot, not its role, because which one is ours is what this config decides. |
| `max_linear_speed` | Speed at command magnitude 1.0. |
| `max_angular_speed` | Yaw rate at command magnitude 1.0. |
| `tau_linear` | First-order coast time constant. Fit to recordings. |
| `tau_angular` | Angular time constant. Fit to recordings. |
| `radius` | Half robot length, used for the wall clamp. |
| `max_linear_speed_fwd` | Forward gain. Falls back to `max_linear_speed`. |
| `max_linear_speed_rev` | Reverse gain (asymmetry). Falls back to the forward gain. |
| `tau_linear_accel` | Spin-up. Falls back to `tau_linear`. |
| `tau_linear_decel` | Coast and brake. Falls back to `tau_linear_accel`. |
| `tau_angular_accel` | Falls back to `tau_angular`. |
| `tau_angular_decel` | Falls back to `tau_angular_accel`. |
| `steer_brake_coeff` | Forward-speed loss per unit of absolute angular command (physical coupling). |
| `angular_droop_coeff` | Yaw-rate loss per unit of absolute linear command (physical coupling). |
| `deadzone_linear`, `deadzone_linear_rev` | Deadzone as a command fraction, forward and reverse. `_rev` falls back to the forward value when unset. |
| `deadzone_angular`, `deadzone_angular_right` | Deadzone as a command fraction, left and right. `_right` falls back to the left value when unset. |

The fields from `max_linear_speed_fwd` down are the physical-realism fields fit by `playground/calibration/fit_plant_calib.py`. Each defaults to 0, meaning unset: fall back to the symmetric values, so existing configs behave as before.

The deadzone applies to the command the plant receives. `SimTransmitter` hands the navigation command straight to the sim with no drive processor, so unlike the deployed path there is no `lifted_deadzone_percent` upstream and these carry the raw physical deadzone. The sign split matches the jig fit.

## `ProjectionBiasConfig`

| Field | Meaning |
| --- | --- |
| `keypoint_height` | Metres above the field plane. |

## `PerceptionConfig`

| Field | Meaning |
| --- | --- |
| `sensing_model` | Sensing model TOML, repo-relative: perception latency, dropouts, position and heading noise, flips. Empty delivers the true poses every tick with no latency. |
| `randomize` | Draw the model's per-episode randomization (latency, noise scale, flip rate, ...) from `[sim] seed`. `false` runs its nominal values. The frame rate is always the tick rate. |

## `OpponentConfig`

| Field | Meaning |
| --- | --- |
| `behavior` | One of `static`, `straight`, `circle`, `random_walk`, `replay`, `trained`, `policy`. `policy` requires `policy_bundle`. |
| `radius` | Circle radius for `behavior = "circle"`. |
| `sprite` | Manifest entry the viewer draws. `house_bot` for the house bot. |
| `hazard_radius` | Treat this opponent as a moving obstacle of this radius (m), the way the house bot behaves: the plant is clamped out of it and clearance to it is scored. 0 means not an obstacle. On the C++ side the matching track arrives as `NEUTRAL` (see `GroundTruthRobotFilter.neutral_count`) and the hazard assembler turns it into a `TRACKED` keep-out disc. Unlike a static hazard there is no shared geometry file: both sides derive the radius from the robot itself, so this has to be the track's own half-diagonal. `GroundTruthRobotFilter` reports every robot as 0.15 x 0.15 m, which is 0.106. A larger value models a house bot the controller cannot see the true size of, and the sweep will report the scrapes. |
| `collision_box` | The MuJoCo plant's contact shape for this obstacle: a box of `[length, width, height]` in m, length along the opponent's heading, turned with its yaw. Empty is a cylinder of `hazard_radius`. Read only when `hazard_radius > 0`, and only by `plant = "mujoco"`. The kinematic plant and the clearance score always use the `hazard_radius` disc. |
| `trained_behaviour` | `behavior = "trained"` only: the RL training opponents (`rl.envs.trained_opponent`). One of `auto_battlebot.rl.envs.opponents.BEHAVIOURS`, or `any` to draw one per reset as stage D does. Parameters are drawn as in training. |
| `trained_holdout` | `behavior = "trained"` only: draw the evaluation's held-out pursuit corner and replay tracks instead of the training ones. |
| `policy_bundle` | `behavior = "policy"` only: an exported RL bundle, repo-relative, that drives this opponent on its own Mr Stabs Mk2 MuJoCo plant (`auto_battlebot/kinematic_sim/policy_opponent.py`). |

## `ObstacleConfig`

One keep-out disc in the arena, in field-frame metres.

| Field | Meaning |
| --- | --- |
| `kind` | `hole` ends the run when the robot's centre crosses into it. `wall_block` clamps the robot to its boundary and kills forward speed, exactly like an arena wall. |
| `radius` | Raw geometry, before any inflation. The controller inflates by its own half-diagonal, the simulated floor does not. |

## `ViewerConfig`

In-loop OpenCV window. On by default: running the sim by hand means watching it. `sim_sweep` turns it off for every run it drives, so batch runs stay headless and fast.

Rendering costs wall-clock time but cannot corrupt a run. The sim owns logical time and ships `sim_time` in the response header, which the C++ side adopts through `ManualClock`. A slow render makes the sim slower in real time and changes nothing the controller sees.

| Field | Meaning |
| --- | --- |
| `render_every` | Tick decimation. |
| `realtime` | Pace `sim.dt` to a human. `false` free-runs. |
| `mujoco` | Also open MuJoCo's 3D viewer on the plant's contact geometry (`auto_battlebot/kinematic_sim/mujoco_window.py`). Needs `enable = true` and `[our_robot] plant = "mujoco"`. Ignored otherwise. |

## Fixed mask model

`FixedMaskModel` returns a constant non-empty FIELD mask for the headless kinematic sim. The Runner's field-init gate requires a non-empty mask. The field size comes from `FixedFieldFilter`, not from perception, so the constant mask satisfies the gate without running a segmentation model.

## Workarounds

- **Lazy MuJoCo imports.** `kinematic_sim_server.py` imports `MujocoWindow` and `plants/__init__.py` imports `MujocoPlant` inside the branch that needs them. MuJoCo is an x86-only dependency (`pyproject.toml` pins it per platform), so on aarch64 a top-level import would fail with `ModuleNotFoundError: mujoco` and break the kinematic plant.
- **`RlEvalServer._render_camera` is a no-op.** The C++ sim profile reads measurements off the socket, not pixels. The compositor cost lockstep tick time for a render nobody reads.

## Workarounds

- `SimKeypointModel::update` fetches `SimConnection::instance()` on its first call, not in the constructor. `SimRgbdCamera` configures the connection, and factories do not promise which of the two is built first. `instance()` silently creates a default `127.0.0.1:14882` connection when unconfigured, so a constructor lookup could pick the wrong one.
- `tests/python/test_sensing_model.py` compares capture spacing to the frame period with a one-step tolerance. Captures land on the first 1 ms step at or after each frame time, so spacing jitters by up to one step while the clock keeps the frame period.
