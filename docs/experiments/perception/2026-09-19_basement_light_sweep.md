# Basement light sweep, 2026-09-19

Sweep over the meatball basement render's light and albedo settings, scored against the real ZED frame (2026-04-19T17-01-18 frame 4560). Results are in `runs/basement_scene/sweep_light*.json`. The settings live in `training/synthetic/cage/meatball_basement.toml`; their meaning is in `docs/reference/tools/synthgen_cage_specs.md`.

## Result

The plywood `albedo_gain` of 0.9 was fitted jointly with the wall gains, `ambient_strength` and the light height:

- Full-frame L1: 26.4, against 30.3 at MassD's gain of 0.50.
- Floor L1: 4.1.
- SSIM: 0.973.

## Single-parameter findings

- `ambient_strength`: 0.4 chosen. At 0.6 the stone reads 13 L brighter than the target. At 0.3 the floor loses its fill.
- `[lights.grid] height`: 1.6 m above the plywood (2.4 m above the concrete) chosen. 1.3 m left a hot spot on the floor the target does not have. 1.9 m fits the floor slightly better but is above this basement's ceiling.
