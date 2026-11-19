# Process noise fit (sessions 2026-08-19 to 2026-08-23)

The process noise defaults in `plant_params.hpp` come from `fit_process_noise.py` against `plant_stageA.toml`, sessions 2026-08-19 through 2026-08-23. The `[plant]` table overrides them field by field.

`kPlantTableToml` in `tests/test_config.cpp` carries the Stage A plant values from `plant_stageA.toml` (fit dated 2026-08-23).

Growth-law mechanisms the fit mirrors:

- White-noise acceleration: variance grows as h^3.
- Scale-factor error on the velocity states: h^2, state-dependent.
- Heading angle random walk: h.
- Delay jitter: constant.

The scale-factor and jitter terms grow faster or slower than the linear-in-h accumulation an additive Q can produce, so `process_noise()` injects them at a rate matched at the 400 ms design horizon (the max-coast timeout): exact there, conservative below it.

## Growth-law targets at the 400 ms design horizon

`JigPlantModelTest.ProcessNoiseMatchesGrowthLawAtDesignHorizon` accumulates `P = F P F^T + Q` per frame over 400 ms (16 frames at 25 ms) and checks it against the fitted growth law (`fit_process_noise.py`, modelled row):

- Along-track 135 mm, cross-track 86.6 mm, heading 35.1 deg (0.613 rad), decomposed at the start heading. With theta0 = 0, field x is along-track.
- The start state is the steady state of the fit's median operating point, 0.47 m/s and 0.83 rad/s, reached with u_lin 0.10983 and u_ang 0.04294. That keeps the window matched to the conditions the growth law was fit under.
- The band is 0.6x to 1.5x on purpose. The plant's lag damps the `q h^3/3` terms below their undamped mapping, and heading noise reaches cross-track through the Jacobian on top of the fitted cross budget. The test pins the wiring (units, rotation, mechanism terms), not the second-order shape.
