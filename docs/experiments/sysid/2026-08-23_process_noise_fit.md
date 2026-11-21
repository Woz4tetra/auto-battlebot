# Process noise fit (sessions 2026-08-19 to 2026-08-23)

The process noise defaults in `plant_params.hpp` come from `fit_process_noise.py` against `plant_stageA.toml`, sessions 2026-08-19 through 2026-08-23. The `[plant]` table overrides them field by field.

Growth-law mechanisms the fit mirrors:

- White-noise acceleration: variance grows as h^3.
- Scale-factor error on the velocity states: h^2, state-dependent.
- Heading angle random walk: h.
- Delay jitter: constant.

The scale-factor and jitter terms grow faster or slower than the linear-in-h accumulation an additive Q can produce, so `process_noise()` injects them at a rate matched at the 400 ms design horizon (the max-coast timeout): exact there, conservative below it.

## Growth-law targets at the 400 ms design horizon

Fitted growth law at 400 ms (`fit_process_noise.py`, modelled row):

- Along-track 135 mm, cross-track 86.6 mm, heading 35.1 deg (0.613 rad), decomposed at the start heading. With theta0 = 0, field x is along-track.
- The fit's median operating point is 0.47 m/s and 0.83 rad/s, reached with u_lin 0.10983 and u_ang 0.04294.
