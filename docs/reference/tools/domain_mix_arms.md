# Domain mix arms

`training/yolo/make_domain_mix_arms.py` builds the training lists for the synthetic domain mix arms (`ARMS`).

## Arm fields

- `randomized`: randomized frames drawn from the pool. `None` takes all of them.
- `domain`: domain frames drawn from the pool after the filters.
- `venues`: defaults to `CAGE_VENUES`, not every venue loaded. Arms that want the basement too pass `ALL_VENUES`.
- `real_repeat`: writes the 452 real frames that many times over. Every other count is a draw from a pool, so this is the one place an arm carries a frame twice on purpose.
- `extra_real`: adds the `--extra-real` frames, written once whatever `real_repeat` says.

Tests: `tests/python/test_make_domain_mix_arms.py`.
