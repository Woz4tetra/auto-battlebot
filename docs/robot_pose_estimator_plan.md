# Plan: Kalman-filter state estimator for robot tracks

## Goal

Replace `RobotTemporalMotionFilter`'s hold-last-value / commanded-velocity dead-reckon
logic with a real per-track constant-velocity Kalman filter: estimated (not commanded)
velocity, smoothed position, and a real covariance in place of the fixed
`max_jump_distance_` gate. This is the single highest-impact change to the robot
filter pipeline -- see `docs/robot_filter_pipeline.md` for the current pipeline this
plugs into, and the design-review conversation this plan grew out of for the fuller set
of gaps (data association, coupling, shared projection code). Those are explicitly out
of scope here; see "Deliberately out of scope" at the end.

## Where it plugs in

`RobotFrontBackSimpleFilter` bundles two concerns: measurement generation + association
(`FrontBackKeypointConverter`, `RobotKeypointTracker`, `FrameIdAssigner`) and per-track
temporal update (`RobotTemporalMotionFilter`). Only the second is being replaced. Neither
`RobotFilterInterface` nor `RobotFrontBackSimpleFilter`'s public shape changes.

## Extract a motion-filter interface first

`RobotTemporalMotionFilter` has no abstract base today; `RobotFrontBackSimpleFilter`
holds one directly by value. Extract a minimal interface so the two implementations are
swappable via config, which is what makes an A/B comparison against the current
behavior possible without a second `[robot_filter] type` or duplicated measurement/
association code:

```cpp
// include/robot_filter/robot_motion_filter_interface.hpp
class RobotMotionFilterInterface {
   public:
    virtual ~RobotMotionFilterInterface() = default;
    virtual void reset() = 0;
    virtual std::vector<RobotDescription> update_with_prediction(
        std::vector<RobotDescription> inputs, const CommandFeedback &command_feedback,
        double timestamp, FrameIdAssigner &frame_id_assigner, const FieldDescription &field,
        double field_bounds_margin_meters, double our_robot_hold_window_s) = 0;
};
```

`RobotTemporalMotionFilter` implements it unchanged (rename not required, just add
`: public RobotMotionFilterInterface` and `override` on the existing method).
`RobotFrontBackSimpleFilter` holds `std::unique_ptr<RobotMotionFilterInterface>
temporal_motion_filter_` instead of a value member, constructed from a new config field:

```toml
[robot_filter]
type = "RobotFrontBackSimpleFilter"
motion_filter = "kalman"   # or "dead_reckon" (default); see config.hpp changes below
```

This is the mechanism for comparing old vs. new on the exact same recorded input: flip
one config field, replay the same `.jsonl` through `replay_filter_inputs`, diff the
output.

## The estimator

### State

4D constant-velocity state per track, position and velocity in the field XY plane:

```
x = [px, py, vx, vy]^T
```

Orientation is **not** part of the filter state. Keypoint-sourced measurements carry a
real heading (`FrontBackKeypointConverter::get_pose_from_points`); blob-sourced ones
don't (`merge_blob_detections` forces identity rotation -- "orientation from blob
rectangles is intentionally non-semantic"). Filtering yaw would mean smoothing real
signal for keypoint tracks and smoothing noise for blob tracks with the same code path.
Keep passing orientation through unfiltered exactly as today; revisit only if a
concrete need for smoothed heading shows up.

### Predict (constant-velocity model, discretized white-noise-acceleration process noise)

```
F(dt) = [[1, 0, dt, 0],
         [0, 1, 0, dt],
         [0, 0, 1,  0],
         [0, 0, 0,  1]]

Q(dt, q) = q * [[dt^4/4,   0,     dt^3/2, 0     ],
                [0,        dt^4/4, 0,     dt^3/2],
                [dt^3/2,   0,     dt^2,   0     ],
                [0,        dt^3/2, 0,     dt^2  ]]

x- = F(dt) x
P- = F(dt) P F(dt)^T + Q(dt, q)
```

`q` is an acceleration-noise spectral density (m^2/s^3), one tunable per track class:
`q_ours` for `Group::OURS` (agile, human/RC-driven, expect faster accel changes) and
`q_opponent` for everything else. Two config fields, not one.

### Correct (position-only measurement)

```
H = [[1, 0, 0, 0],
     [0, 1, 0, 0]]

y = z - H x-                    (innovation)
S = H P- H^T + R                (innovation covariance)
K = P- H^T S^-1                 (Kalman gain)
x = x- + K y
P = (I - K H) P-
```

`R` (2x2 measurement noise) is set per measurement, not fixed: use a smaller `R` for
keypoint-sourced measurements and a larger one for blob-sourced ones, replacing the
already-documented-but-informal "keypoints are higher quality than blobs" assumption in
`is_blob_suppressed_by_keypoint`'s comment with an actual number. Optionally scale further
by the measurement's `confidence` (lower confidence -> larger `R`) -- confidence is
computed today (`MeasurementWithConfidence`) and currently thrown away after breaking
ties in assignment order; this is the first real use for it.

### Track lifecycle

One `KalmanTrack` per `FrameId`, replacing the three parallel maps across
`FrameIdAssigner` and `RobotTemporalMotionFilter`
(`last_position_per_frame_id_`/`jump_reject_count_per_frame_id_` and
`last_timestamp_per_frame_id_`/`last_measured_timestamp_per_frame_id_`/
`last_description_per_frame_id_`):

- **New track** (`FrameId` measured for the first time, or after being dropped): initialize
  `x = [px, py, 0, 0]` from the measurement, `P` to a large diagonal (position variance from
  `R`, velocity variance large since it's unknown).
- **Every tick**: `predict(dt)`.
- **Matched to a measurement this tick** (association is unchanged, still
  `FrameIdAssigner`): `correct(z, R)`. Output `is_stale = false`.
- **Unmatched this tick**: predict-only, no correction. Output `is_stale = true`. This
  is what makes prediction symmetric across `Group::OURS` and opponents for free --
  today only OURS gets predicted at all (opponents have no `command_feedback` entry to
  dead-reckon from), because prediction currently *requires* a commanded velocity.
  A Kalman predict step needs only the track's own estimated velocity, so every track
  gets a real forward extrapolation on a miss, not just ours.
- **Deletion**: keep the existing `our_robot_hold_window_s_` OURS-only decay as-is for
  this change (`Group::OURS` track dropped after that many seconds unmeasured; everyone
  else held indefinitely). Generalizing decay to all groups is a natural follow-up given
  a per-track miss counter now exists, but is a behavior change beyond "swap the
  estimator" and should land separately so it can be evaluated on its own.

### Preserve the assigner-gating side effect

`docs/robot_filter_pipeline.md` calls this out explicitly: prediction is load-bearing
for two reasons, not one -- it also keeps `FrameIdAssigner::last_position_per_frame_id_`
current via `set_last_position`, so a detection reappearing after a dropout still lands
inside the (still-fixed-for-now, see below) `max_jump_distance_` gate against last
tick's *extrapolated* position rather than a stale one. The new motion filter must call
`frame_id_assigner.set_last_position(frame_id, x.head<2>())` for every track every tick
(predicted or corrected), in the same place `RobotTemporalMotionFilter` does today.
Dropping this would silently regress association continuity through dropouts, not just
prediction smoothness -- easy to miss since nothing in `RobotFilterInterface`'s return
type would signal it.

`FrameIdAssigner`'s gating logic itself (fixed `max_jump_distance_`, greedy
nearest-position matching) does not change in this plan. A real per-track covariance
now exists, which is what a follow-up Mahalanobis-distance gate needs -- but wiring that
through `FrameIdAssigner::assign`'s signature is its own change and should be a separate
plan once the estimator has landed and been validated on its own.

## Concrete files

New:
- `include/robot_filter/robot_motion_filter_interface.hpp` -- the extracted interface above.
- `include/robot_filter/kalman_track.hpp` + `src/robot_filter/kalman_track.cpp` -- `KalmanTrack`:
  owns `x` (`Eigen::Vector4d`), `P` (`Eigen::Matrix4d`), last-measured timestamp, hit/miss
  counters. Methods: `predict(dt)`, `correct(Eigen::Vector2d z, Eigen::Matrix2d R)`,
  `position() const`, `velocity() const`, `covariance() const`. Pure math, no dependency on
  `FrameIdAssigner` or config parsing -- this is what makes it unit-testable in isolation.
- `include/robot_filter/robot_kalman_motion_filter.hpp` + `src/robot_filter/robot_kalman_motion_filter.cpp`
  -- `RobotKalmanMotionFilter : public RobotMotionFilterInterface`, holding
  `std::map<FrameId, KalmanTrack> tracks_`. Same method contract as
  `RobotTemporalMotionFilter::update_with_prediction`, same field-bounds clipping
  (`clip_to_field_bounds`, reused as-is) and `our_robot_hold_window_s` decay.

Modified:
- `include/robot_filter/robot_temporal_motion_filter.hpp` -- add
  `: public RobotMotionFilterInterface` and `override` on `update_with_prediction`/`reset`.
  No behavior change.
- `include/robot_filter/robot_front_back_simple_filter.hpp` -- change
  `RobotTemporalMotionFilter temporal_motion_filter_;` to
  `std::unique_ptr<RobotMotionFilterInterface> temporal_motion_filter_;`.
- `include/robot_filter/config.hpp` (`RobotFrontBackSimpleFilterConfiguration`) -- add:
  - `std::string motion_filter = "dead_reckon";` (or `"kalman"`)
  - `double kalman_q_ours = <tuned default>;`, `double kalman_q_opponent = <tuned default>;`
  - `double kalman_r_keypoint = <tuned default>;`, `double kalman_r_blob = <tuned default>;`
  - parse via `PARSE_FIELD_STRING`/`PARSE_FIELD_DOUBLE` in `parse_fields`, matching the
    existing fields in that struct.
- `src/robot_filter/robot_front_back_simple_filter.cpp` (constructor) -- construct
  `temporal_motion_filter_` as `RobotKalmanMotionFilter` or `RobotTemporalMotionFilter`
  based on `config.motion_filter`.

## Testing

1. **`tests/test_kalman_track.cpp`** (new, GoogleTest, matching `tests/test_robot_keypoint_tracker.cpp`'s
   convention): feed a track a sequence of noisy synthetic measurements along a known
   constant-velocity path; assert the estimated velocity converges toward the true
   velocity and position error shrinks over a few updates. Feed a track a step change in
   velocity; assert it tracks with bounded lag. Feed several predict()-only steps with no
   measurement; assert position advances by `v * dt` per step and `P` grows monotonically.
2. **`tests/test_robot_front_back_simple_filter.cpp`** (existing file) -- add cases with
   `motion_filter = "kalman"` covering the same scenarios already tested for the current
   dead-reckon path (stale flagging, field-bounds clipping on a predicted position,
   OURS-only hold-window decay), so both paths satisfy the same output contract.
3. **Playback regression** -- use this session's `record_filter_inputs`/`replay_filter_inputs`
   (`src/record_filter_inputs.cpp`, `src/replay_filter_inputs.cpp`) to capture one `.jsonl`
   from a representative SVO, then run `replay_filter_inputs` twice against it (config
   toggling `motion_filter`) and diff position/velocity/staleness per tick. This is the
   direct successor to `playground/control_stage0/prediction_eval.py`'s existing
   prediction-quality A/B, extended to cover velocity error now that velocity is an
   actual estimate rather than always zero.

## Rollout

`motion_filter` defaults to `"dead_reckon"` (current behavior unchanged) until the
Kalman path has been validated via the above, then flip the default in
`config/_common.toml`. Keep `RobotTemporalMotionFilter` and the config option around
after the flip rather than deleting it immediately -- cheap fallback if the new
estimator misbehaves on a field the recorded regression set didn't cover.

## Deliberately out of scope (follow-ups this enables)

- **Mahalanobis-gated association**: replace `FrameIdAssigner`'s fixed
  `max_jump_distance_` with a gate on `S` (innovation covariance) now that one exists per
  track. Needs `FrameIdAssigner::assign`'s signature to accept per-track covariance.
- **Global optimal assignment** (Hungarian) instead of greedy nearest-available, in
  `FrameIdAssigner::assign`.
- **Symmetric stale-decay**: drop long-unmeasured opponent tracks too, not just OURS.
- **Confidence/covariance-driven fusion subsuming `is_blob_suppressed_by_keypoint` and
  `suppress_blobs_near_our_anchor`**: once measurements have a real per-source `R`,
  treating keypoints and blobs as two sensors feeding the same track (lower-`R` source
  wins on conflict) replaces both hand-tuned suppression heuristics with one mechanism.
- **Shared 2D-to-3D projector**: `FrontBackKeypointConverter` and
  `RobotKeypointTracker::extract_candidates` each reimplement "unproject pixel, intersect
  plane at known height, transform to field frame." Worth extracting once a third
  consumer needs it or the duplication causes a bug.

Each is a separate plan; this one only replaces the temporal update.
