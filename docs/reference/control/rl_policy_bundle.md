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

## selftest.bin

Little-endian: the bytes `RLST`, uint32 version (1), uint32 pair count N, uint32 observation size D, uint32 action size A, then N x D float32 observations, then N x A float32 decoded actions (`scale * tanh(network output)`), both row-major.
