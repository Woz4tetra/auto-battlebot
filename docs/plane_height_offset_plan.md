# Plan: apply estimated plane-height offset for all robots

## Problem

Tracked robot keypoints sit a small height (~6 cm) above the field plane, but the
perception pipeline projects them straight onto the ground plane by ray/plane
intersection. Because the keypoint is above the plane, its projection lands
*farther* from the camera nadir than the robot's true ground position — the error
grows with horizontal distance from the camera. This biases every robot's estimated
position radially outward.

The simulation already models this exact error as a forward bias
(`simulation/kinematic_sim_server.py:223-229`, `Perception._project_bias`), and an
offline metric measures it (`playground/control_stage0/stage0_metrics.py:243-255`),
but **the production C++ pipeline applies no correction**. This plan adds one, gated
by config, applied uniformly to all tracked robots.

## Current behavior (where the fix goes)

Both projection call sites turn a pixel keypoint into an on-plane 3D point with no
height term:

- `src/transform_utils.cpp:247-255` — `project_keypoint_onto_plane()` does
  `pixel_to_camera_ray()` then `intersect_camera_ray_with_plane()`. No offset.
- `src/robot_filter/robot_keypoint_tracker.cpp:70-77` — projects two blob keypoints,
  takes the midpoint as the robot center.
- `src/robot_filter/front_back_keypoint_converter.cpp:36-41` — projects each
  front/back keypoint, later averaged into a pose center.

A repo-wide grep for `keypoint_height|height_offset|plane_height|projection_bias` in
`src/` and `include/` returns no hits — there is no existing parameter to reuse.

## Geometry of the correction

Work in the camera frame, where all projections above already live. Let:

- `n` = unit plane normal (`plane_normal`), pointing from the plane toward the camera.
- `c` = a point on the plane (`plane_center`).
- Camera origin is `0` (camera frame).
- `d = (0 - c) · n = -c · n` → perpendicular camera-to-plane distance is `h_cam = |d|`.
- `p` = the on-plane projected point (current output of `project_keypoint_onto_plane`).
- `h_kp` = estimated keypoint height above the plane (new config value, e.g. 0.06 m).

The camera nadir (foot of perpendicular from the camera onto the plane) is
`nadir = c + (c · n) n`... equivalently `nadir = 0 - d·n`. The true ground point sits
on the line from the nadir to `p`, scaled inward by the ratio of heights (similar
triangles: the physical keypoint is at height `h_kp`, the camera at height `h_cam`):

```
p_corrected = nadir + (p - nadir) * (h_cam - h_kp) / h_cam
```

This is the exact geometric inverse of the flat-plane projection. The sim's linear
form (`p += (h_kp / h_cam) * (p - cam)`) is a first-order approximation of the same
relationship; use the exact form above in C++. Guard `h_cam > h_kp` and
`h_cam > 1e-6`; if not, skip the correction (degenerate camera geometry) and return
`p` unchanged.

Because the correction is applied inside `project_keypoint_onto_plane` (or right
after each call), it automatically covers **all robots** — our robot, opponents,
blob candidates, and front/back keypoints all flow through the same two call sites.

## Implementation steps

### 1. Add the correction helper (`transform_utils`)

In `include/transform_utils.hpp` and `src/transform_utils.cpp`, add:

```cpp
// Shift an on-plane projected point inward toward the camera nadir to compensate
// for the keypoint sitting `keypoint_height` above the plane. Returns the input
// unchanged when geometry is degenerate.
Eigen::Vector3d correct_plane_height_offset(const Eigen::Vector3d &plane_point,
                                            const Eigen::Vector3d &plane_center,
                                            const Eigen::Vector3d &plane_normal,
                                            double keypoint_height);
```

Add an overload of `project_keypoint_onto_plane` that takes `double keypoint_height`
and applies the correction before returning, so existing call sites can opt in by
passing the height. Keep the current signature delegating to the new one with
`keypoint_height = 0.0` (no-op) to avoid churn where correction is not wanted.

### 2. Thread the config value

- `RobotKeypointTrackerConfig` (`include/robot_filter/robot_keypoint_tracker.hpp:13`):
  add `double keypoint_height_meters = 0.0;` (default 0 = feature off, preserves
  current behavior).
- `FrontBackKeypointConverterConfig`
  (`include/robot_filter/front_back_keypoint_converter.hpp:20`): add the same field.
- Parse both from TOML in
  `include/robot_filter/config.hpp:64` (`parse_robot_keypoint_tracker_config`) via
  `parser.get_optional_double("robot_blob_keypoint_height_meters", ...)`, and wherever
  `FrontBackKeypointConverterConfig` is parsed (find with
  `grep -rn front_keypoints include/robot_filter/`). Use a shared key name, e.g.
  `keypoint_height_meters`, so a single value configures both.

### 3. Apply at the call sites

- `robot_keypoint_tracker.cpp:72-73` — pass `config_.keypoint_height_meters` to the
  new `project_keypoint_onto_plane` overload for `point_a` and `point_b`.
- `front_back_keypoint_converter.cpp:38` — same for each keypoint.

### 4. Config

Add `keypoint_height_meters` to the relevant `[robot_filter]` sections in
`config/*.toml` and `config/default_profile/`. Default to `0.06` where the real
keypoint height is known; leave `0.0` (off) in configs that should not change. Match
the sim value (`simulation/kinematic_sim.toml:50`, `keypoint_height = 0.06`) so sim
injection and real correction use the same number.

### 5. Tests

- Unit test `correct_plane_height_offset` in `tests/`: a keypoint at known height and
  horizontal distance produces the expected inward shift; verify it exactly inverts a
  synthetic forward projection; verify the degenerate-geometry no-op path.
- Regression: replay an SVO in playback mode
  (`./scripts/build_and_run.sh -c config/playback.toml`) with the offset enabled and
  confirm tracked positions move inward toward the camera nadir vs. baseline.

## Validation checklist

```bash
./scripts/build_and_test.sh --gtest_filter=TransformUtils.*   # new unit tests
venv/bin/mypy scripts/ simulation/ training/                  # unchanged, sanity
git diff --name-only HEAD | grep '\.cpp$' | xargs -r clang-tidy -p build-test/
./scripts/lint
```

## Notes / decisions to confirm

- **Single height vs. per-robot.** This plan uses one `keypoint_height_meters` applied
  to every robot ("for all robots"). If different robots carry keypoints at different
  heights, the field could later move into `RobotConfig`
  (`include/data_structures/robot.hpp:34`) and be looked up per label; the correction
  helper already takes height as a parameter, so that extension is additive.
- **Where to correct.** Correcting inside the projection step (rather than
  post-filter) keeps the fix upstream of front/back pose fitting and frame-id
  assignment, so `max_jump_distance` gating and velocity estimation all see corrected
  positions.
- **Default off.** Height defaults to `0.0` so no config regresses silently; the
  feature activates only where the TOML sets a nonzero height.
