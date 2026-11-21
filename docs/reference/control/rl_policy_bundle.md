# RL policy bundle

## Layout

A bundle directory is parsed and checked on load:

```
manifest.toml         policy kind, observation spec, decoder, execution, networks
actor.rtneural.json   the network (NETWORK bundles)
selftest.bin          observation and decoded-action pairs from PyTorch (NETWORK bundles)
```

NETWORK bundles are written by `auto_battlebot/rl/export/bundle.py`. A CONSTANT bundle is written by hand for the bench and the lockstep sim before a trained bundle exists, and for the navigation tests (`tests/rl_policy/bundles/constant_forward/manifest.toml`). `RlPolicyNavigation` drives the action every tick with no network. It has no networks, decoder or self-test, only the action every tick:

```toml
format_version = 1
run_id = "constant_forward"
policy_kind = "CONSTANT"
obs_spec_version = 1
execution = "INLINE"
policy_period_ticks = 5

[constant]
action = [0.3, 0.0]
```

The action is linear then angular, normalized to [-1, 1].

## Observation spec versions

One observation spec version maps robots, field, target and history to named tensors. The output depends on the context and nothing else. A new spec version is a new builder; old bundles keep loading because each bundle names the version it was trained on.

### Spec v1 layout

`ObservationBuilderV1` (`include/rl_policy/observations/observation_v1.hpp`) is the C++ twin of `build_v1` in `auto_battlebot/rl/obs_spec.py`. It writes 138 floats in our body frame (x forward, y left) at our axle centre. The field frame's origin is the field centre. Blocks, in order:

| Block | Floats | Contents |
| --- | --- | --- |
| our robot | 6 | forward speed, yaw rate, measurement age, stale flag, mode one-hot |
| history | 32 | last 8 policy steps, newest first: actions, then measured speed and yaw rate |
| target opponent | 16 | position, distance, bearing, relative heading, velocity, yaw rate, three sigmas, half-diagonal, measurement age, stale flag |
| range rays | 48 | 24 rays, solid and hole channel each |
| nearest hazards | 36 | 4 discs by edge distance |

- `obs_spec.py` holds the scales and caps.
- A hazard's radius is the raw object radius, never inflated. Hazard sorting is stable, so ties keep input order in both implementations.
- Scales and caps: forward speed /2.5 m/s, yaw rate /15 rad/s, measurement age /0.5 s (capped at 1), position /2 m, velocity /2.5 m/s, position sigma /0.1 m, heading sigma /(pi/2), yaw-rate sigma /15, footprint half-diagonal /0.2 m, hazard radius /0.3 m. The three sigmas are capped at 1, so an infinite (unknown) sigma reads 1.
- Our robot block: mode one-hot is ATTACK, RUN_AWAY. History holds 8 steps newest first: (linear, angular) actions x 8, then measured (forward speed, yaw rate) x 8.
- Target opponent block order: position x, y, distance /2, bearing sin, cos, heading relative to ours sin, cos, velocity x, y, yaw rate, the three sigmas, footprint half-diagonal, measurement age, stale flag. With no target opponent the block is all zeros except the three sigmas, the age and the stale flag, which read 1.
- Range rays: 24 rays from heading 0 counter-clockwise at 15 degrees, two channels each (solid, hole), distance /1.5 m, 1.0 when nothing is within 1.5 m. Solid rays see the field walls, wall blocks and tracked hazards (the house bot); hole rays see holes. A ray that starts inside a disc reads 0, and a ray whose origin is on or outside the field wall reads 0.
- Nearest hazards: 4 discs by edge distance, each: centre x, y /2, raw radius /0.3, kind one-hot (hole, wall block, tracked), velocity x, y /2.5, present flag.
- Input units: speeds in m/s, yaw rates in rad/s, `opp_velocity` in the field frame, `field_half` is the half size of the field rectangle, `hist_action` is (linear, angular).
- Any change to the layout, a scale or a cap bumps `OBS_SPEC_VERSION`, which breaks loading of every bundle trained on the old one.
- A layout change is a new spec version: bump the version in `obs_spec.py`, add a new builder, and regenerate `tests/rl_policy/rl_policy_golden_data.hpp` with `playground/rl/make_rl_golden_fixture.py`.

## Manifest checks

`load_policy_bundle` parses `<directory>/manifest.toml` and checks everything that needs no network library: the format version, every enum field against its known values, required fields and tables, that the files it names exist, and that the decoder scale matches the actor's output size. It throws `PolicyBundleError` naming the problem. `policy_period_ticks` counts control-loop ticks between policy steps (5 gives 50 Hz on the 250 Hz loop). NETWORK bundles hold `[decoder]`, `[[networks]]` (the first is the actor) and `[selftest]`; CONSTANT bundles hold `[constant]`.

## Startup checks

`make_navigation` runs `RlPolicyNavigation::initialize()` before it returns, so a bad bundle throws on the main thread and stops the app before the control loop starts.

## Real-time contract

`NetworkRuntimeInterface::run()` never allocates, so it is safe on the control thread. Recurrent state lives inside the runtime, and `reset()` zeros it. `ObservationBuilderV1::build()` does not allocate once its hazard scratch has grown to the field's hazard count. `ActionHistory` is a fixed ring allocated once, newest first. A slot never written reads as zeros, which is how the training environment starts an episode, so a fresh history feeds the policy nothing it did not see in training. `ActionDecoderInterface::decode()` throws `std::invalid_argument` if either span is not `action_size()` long.

## RTNeural runtime

The exporter writes `""` as the activation of the actor's last layer because the decoder applies `tanh`, not the network.

The deployed actor is 138 -> 256 -> 256 -> 128 -> 2 with ELU and no output activation.

## selftest.bin

Little-endian: the bytes `RLST`, uint32 version (1), uint32 pair count N, uint32 observation size D, uint32 action size A, then N x D float32 observations, then N x A float32 decoded actions (`scale * tanh(network output)`), both row-major.

`run_selftest` runs every observation through the runtime and decoder on the box and compares with the PyTorch actions. The runtime is reset before each pair, so a recurrent network sees each observation from a zero state, and again at the end. A size mismatch with the runtime input or the decoder throws `PolicyBundleError`; a numeric disagreement is reported in the result. `NetworkPolicy::load` runs it and throws when it fails, naming the worst pair and its error.

## RlPolicyNavigation

`RlPolicyNavigation` finds our robot and the target opponent, schedules policy steps, and logs. The bundle decides the policy kind, the network runtime, the decoder, and the observation spec.

- Every `policy_period_ticks` control ticks it builds the observation, runs the policy, and stores the action. Every tick it returns the stored action.
- By default nothing edits the action on its way to the transmitter, and hazards reach the policy only as observations.
- With `hazard_barrier_enable` the motion profile's velocity barrier clips `linear_x` every tick against the measured speed. The action history keeps the policy's own output, which is what training saw.
- `initialize()` loads and checks the bundle the first time it runs and throws naming what failed: `opponent_mode` must be `KALMAN`, the manifest must parse with known values, a builder must exist for its `obs_spec_version` and make the tensors the network reads, and the self-test must pass. Every call resets the policy, the history, and the schedule.
- Once our robot has gone unmeasured for longer than `max_our_unmeasured_s` it commands zero, the same as when our robot is missing. The coasted track keeps reporting the last yaw rate, so the policy otherwise holds whatever it was doing when the keypoints dropped, usually a full-rate spin. Motion blur from that spin keeps the keypoint model from reacquiring the robot. Stopping lets it reacquire, and the next policy step runs on the first measured tick.
