# RL policy bundle

## Layout

A bundle directory is parsed and checked on load:

```
manifest.toml         policy kind, observation spec, decoder, execution, networks
actor.rtneural.json   the network (NETWORK bundles)
selftest.bin          observation and decoded-action pairs from PyTorch (NETWORK bundles)
```

NETWORK bundles are written by `auto_battlebot/rl/export/bundle.py`. A CONSTANT bundle is written by hand for the bench and the lockstep sim before a trained bundle exists. It has no networks, decoder or self-test, only the action every tick:

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

- `obs_spec.py` holds the scales and caps. The golden test (`ObservationV1GoldenTest.MatchesObsSpecPyOnEveryCase`) compares all 138 floats against it on every case.
- A hazard's radius is the raw object radius, never inflated.
- A layout change is a new spec version: bump the version in `obs_spec.py`, add a new builder, and regenerate `tests/rl_policy/rl_policy_golden_data.hpp` with `playground/rl/make_rl_golden_fixture.py`.

## Manifest checks

`load_policy_bundle` parses `<directory>/manifest.toml` and checks everything that needs no network library: the format version, every enum field against its known values, required fields and tables, that the files it names exist, and that the decoder scale matches the actor's output size. It throws `PolicyBundleError` naming the problem. `policy_period_ticks` counts control-loop ticks between policy steps (5 gives 50 Hz on the 250 Hz loop). NETWORK bundles hold `[decoder]`, `[[networks]]` (the first is the actor) and `[selftest]`; CONSTANT bundles hold `[constant]`.

## Real-time contract

`NetworkRuntimeInterface::run()` never allocates, so it is safe on the control thread; `RtNeuralRuntimeTest.RunDoesNotAllocate` counts malloc calls over 100 runs. Recurrent state lives inside the runtime, and `reset()` zeros it. `ObservationBuilderV1::build()` does not allocate once its hazard scratch has grown to the field's hazard count (`ObservationV1AdapterTest.BuildDoesNotAllocateAfterWarmup`). `ActionHistory` is a fixed ring allocated once, newest first. A slot never written reads as zeros, which is how the training environment starts an episode, so a fresh history feeds the policy nothing it did not see in training (`ActionHistoryTest.UnwrittenSlotsAreZero`). `ActionDecoderInterface::decode()` throws `std::invalid_argument` if either span is not `action_size()` long.

## selftest.bin

Little-endian: the bytes `RLST`, uint32 version (1), uint32 pair count N, uint32 observation size D, uint32 action size A, then N x D float32 observations, then N x A float32 decoded actions (`scale * tanh(network output)`), both row-major.

`run_selftest` runs every observation through the runtime and decoder on the box and compares with the PyTorch actions. The runtime is reset before each pair, so a recurrent network sees each observation from a zero state, and again at the end. A size mismatch with the runtime input or the decoder throws `PolicyBundleError`; a numeric disagreement is reported in the result. `NetworkPolicy::load` runs it and throws when it fails, naming the worst pair and its error.
