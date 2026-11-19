# Process noise fit (sessions 2026-08-19 to 2026-08-23)

The process noise defaults in `plant_params.hpp` come from `fit_process_noise.py` against `plant_stageA.toml`, sessions 2026-08-19 through 2026-08-23. The `[plant]` table overrides them field by field.

`kPlantTableToml` in `tests/test_config.cpp` carries the Stage A plant values from `plant_stageA.toml` (fit dated 2026-08-23).

Growth-law mechanisms the fit mirrors:

- White-noise acceleration: variance grows as h^3.
- Scale-factor error on the velocity states: h^2, state-dependent.
- Heading angle random walk: h.
- Delay jitter: constant.

The scale-factor and jitter terms grow faster or slower than the linear-in-h accumulation an additive Q can produce, so `process_noise()` injects them at a rate matched at the 400 ms design horizon (the max-coast timeout): exact there, conservative below it.
