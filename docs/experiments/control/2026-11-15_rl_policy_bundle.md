# RL policy bundle notes

- `PolicyKind::NETWORK` runs the bundle's networks. `CONSTANT` returns the fixed action in its manifest, for the bench and the lockstep sim before a trained bundle exists.
- The squashed decoder computes `action[i] = scale[i] * tanh(raw[i])`: the mean of a tanh-squashed Gaussian policy, which is what deployment runs (not a sample).
- `ObservationV1` inputs mirror `ObsInputs` in `obs_spec.py` with the batch dimension dropped. Infinite sigmas mean unknown and are allowed; `test_observation_v1.cpp` covers the C++/Python twin.
