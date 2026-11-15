# 2026-11-15 height gate relief method

These facts were source comments in `src/` and moved here on 2026-11-15 so the code carries names and tests instead of prose. Each section names the file and the code it described.

### `src/keypoint_filter/height_gate.cpp`

Beside `const double ring_inner = radius * config_.ring_inner_scale;`:

The floor reference is a ring outside the circle, with a gap so a robot spilling past its inscribed circle does not contaminate its own reference.

### `src/keypoint_filter/height_gate.cpp`

Beside `const bool too_low = relief.elevation < config_.min_elevation_meters;`:

Low is judged on relief against the surrounding floor; high on absolute height above the field plane, because a wall's elevation over its own surroundings says nothing.
