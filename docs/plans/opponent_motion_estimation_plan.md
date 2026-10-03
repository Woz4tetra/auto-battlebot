# Opponent motion estimation plan

Goal: estimate each opponent's pose (position and heading) and velocity (linear and yaw rate)
with uncertainties, and predict them forward over the planning horizon, so
`PlantRolloutNavigation` can place the opponent's weapon arc where it will be, not where it was
(`docs/plans/plant_rollout_navigation_plan.md`, sections 1 and 2).

The heading measurement comes from the learned NHRL keypoint model, trained on synthetic data,
which gives front and back keypoints for arbitrary robots. This plan does not change perception.

## What exists

All in `src/robot_filter/kalman_motion_estimator.cpp` and its header unless noted.

| Item | Today |
| --- | --- |
| Mode | `opponent_mode` defaults to `HOLD` (`motion_estimator_config.hpp:76`) and no config sets it, so production pins every opponent at its last measured pose. `KALMAN` exists but is unused |
| State | `[px, py, vx, vy]`, field frame, constant velocity, white-noise acceleration with PSD `opponent_accel_psd` (30) |
| Measurement | Position only, from keypoints or blob centroids. Noise from `measurement_noise_for` (keypoint or blob sigma, plus a per-metre-from-camera term) |
| Heading | Not a state. `render_opponent` copies the last accepted measurement's rotation, or uses the velocity direction while stale and faster than `min_heading_speed` (0.3 m/s). No yaw rate: `velocity.omega` is always 0 |
| Gating | Chi-square at `gate_nis` (11.83); reinit after `reinit_after_rejects` (5) consecutive rejects |
| Latency | Snapshot ring of 64 states; a measurement older than the track rewinds, corrects at its own stamp, and `coast()` re-propagates |
| Coasting | Predicts to `max_coast_s` (0.2 s in `config/_common.toml`) of track age, then holds position and grows covariance |
| Render | `coast()` renders at now + `render_lead_s()` (the plant's `delay_s`) and clips position to the field |
| Output | `RobotDescription`: pose, `velocity{vx, vy, omega}`, `is_stale`. No uncertainty |
| Our-robot precedent | The our-robot EKF already corrects heading from keypoints and handles flips: a heading innovation past pi/2 updates position rows only and counts `num_heading_flips` |
| Tests | `tests/test_kalman_motion_estimator.cpp`: CV convergence, coast lead, coast hold, gate and reinit, late-measurement rewind, hold mode, our-robot EKF heading flip |

## Design

### Model choice

Two-wheel robots move along their heading, so a model that couples them (constant turn rate and
velocity) predicts arcs better than constant velocity. It fails on exactly the opponents that
matter most for weapon placement: full-body spinners (heading is meaningless), robots shoved
sideways, invertible robots whose front flips when they flip over, and any frame where the
keypoint model labels the back as the front. A straight-line model with an independent heading
fails gracefully on all of those.

So the filter stays decoupled, and the coupling moves into prediction, where it can be switched
off per frame:

- **Filter (phase 1):** a linear 6-state Kalman filter `[px, py, vx, vy, theta, omega]`. The
  translation block is today's constant-velocity model. The heading block is a separate
  constant-yaw-rate model with its own process noise. Linear, so the existing `ekf_update`,
  snapshot and rewind code carry over with a larger `N` and `theta` marked as an angle state.
- **Prediction:** a pure function (section 4) that advances along an arc using `omega` when the
  velocity direction agrees with the heading, and in a straight line otherwise.
- **Phase 2, only if phase 1 falls short:** a body-frame model `[x, y, theta, u, s, omega]`
  with forward speed `u` and lateral speed `s`. Lateral speed gets small process noise for
  two-wheel robots and large noise for spinners, configured per label. Adopt it only if it beats
  phase 1 on the prediction-error evaluation (step 5).

### 1. State, process model and noise

```
state  = [px, py, vx, vy, theta, omega]
F(dt)  = translation: px += vx dt, py += vy dt
         heading:     theta += omega dt   (wrapped after the update)
Q(dt)  = translation: today's white-noise-acceleration block with opponent_accel_psd
         heading:     white-noise yaw acceleration with opponent_yaw_accel_psd (new)
```

- **Past `max_coast_s`:** hold position and heading, as today, and also stop integrating
  `omega`. Covariance keeps growing.
- **Initialization:** heading from the first measurement that carries one, with variance
  `keypoint_heading_sigma_rad^2`. A track born from a blob starts with heading variance
  `pi^2`, which reads as unknown. `omega` starts at 0 with variance `initial_yaw_rate_sigma^2`
  (new config, start at 20 rad/s).
- **Snapshots:** the snapshot struct grows to the 6-state vector and 6x6 covariance. At 64 slots
  that is about 20 KB per track, fine.

### 2. Measurement update

- **With heading** (keypoint measurement, heading innovation within pi/2, label not
  heading-blind): 3-row update on `[px, py, theta]`, `theta` marked as an angle row, noise
  `diag(sigma_pos^2, sigma_pos^2, keypoint_heading_sigma_rad^2)`.
- **Without heading** (blob, flip, or heading-blind label): 2-row position update as today.
- **Flips:** an innovation past pi/2 updates position only and counts toward a new
  `num_opponent_heading_flips` diagnostic. After `heading_flip_reinit_count` consecutive flips
  (start at 3), re-initialize the heading block at the measured heading, since a run of
  consistent flips means the track had the front wrong. Position and velocity are untouched.
- **Heading-blind labels:** a `[robot_filter.motion_estimator.heading_blind_labels]` list,
  label names, for full-body spinners and robots the keypoint model cannot orient. Their
  heading block is never corrected and its variance stays at the cap, which the navigation
  reads as a full-circle weapon.
- **Measured yaw rate:** none. The keypoint model gives heading per frame only, so `omega` is
  inferred from successive headings. That is enough at 30 Hz for robots turning below about
  half a revolution per frame interval; faster spins alias and are what the heading-blind list
  is for.

### 3. Output with uncertainty

Add to `RobotDescription`:

```cpp
/** One-sigma uncertainties of the rendered estimate. Infinity means unknown. Filled by
 * KalmanMotionEstimator; every other producer leaves the defaults. */
struct PoseUncertainty {
    double position_sigma_m = std::numeric_limits<double>::infinity();
    double heading_sigma_rad = std::numeric_limits<double>::infinity();
    double yaw_rate_sigma_rad_s = std::numeric_limits<double>::infinity();
};
PoseUncertainty uncertainty;
```

`position_sigma_m` is the square root of the larger eigenvalue of the 2x2 position covariance.
`render_opponent` writes the filtered heading to `pose.rotation`, `omega` to `velocity.omega`,
and these sigmas. The velocity-direction heading fallback goes away, since the filter now coasts
heading itself. The our-robot EKF can fill the same struct from its own covariance; that is a
one-line follow-up, not part of this plan.

Add the per-track sigmas and the flip count to the `kalman_motion_estimator` diagnostics, and
the new fields to the Foxglove adapter that publishes robot descriptions, with
`docs/foxglove_recording_format.md` updated to match.

### 4. Prediction helper

A pure function shared by navigation and the offline evaluation:

```cpp
/** Advance a rendered opponent estimate by dt seconds (dt >= 0). Arc motion when the
 * velocity direction is within `arc_alignment_rad` of the heading or its reverse and the
 * heading is known; straight-line otherwise. Position stops at the field walls minus
 * `wall_margin_m`. Sigmas grow with the filter's own process noise over dt. */
OpponentPrediction predict_opponent(const RobotDescription &opponent, double dt,
                                    const OpponentPredictionSettings &settings,
                                    const FieldDescription &field);
```

`OpponentPrediction` holds center, heading, speed, yaw rate and the three sigmas. It lives in
`include/robot_filter/opponent_prediction.hpp` so the estimator, navigation and tests share it.
`PlantRolloutNavigation` calls it once per rollout step per replan, and builds the weapon arc
from the predicted heading and `heading_sigma_rad`.

Wall stop: a constant-velocity prediction of an opponent pinned on a wall drives through the
wall. The helper zeroes the velocity component into any wall the predicted center reaches. The
renderer does the same within `max_coast_s`, which also stops the clip in
`clip_to_field_bounds` from fighting a velocity that points out of the field.

### 5. Turning it on

Set `opponent_mode = "KALMAN"` in `config/_common.toml` once step 4 below passes. Playback and
sim profiles inherit it. `HOLD` stays available as a fallback.

## Evaluation

The question is prediction accuracy at the horizons the planner uses, against later
measurements, on real recordings with the learned keypoint model.

- **Data:** NHRL May and MassD recordings, replayed in playback mode with the learned keypoint
  model, recording the new diagnostics. Hold out whole recordings.
- **Metric:** from each accepted correction at time t, predict to t + h for h in 0.1, 0.2, 0.3,
  0.5 s with `predict_opponent`, and compare against the next accepted measurement within a
  frame of t + h. Report position error (median and p90) and heading error, split by opponent
  label and by motion class (still, straight, arcing, spinning).
- **Baselines:** today's `HOLD` (prediction = last pose) and today's 4-state `KALMAN` with
  velocity-direction heading. Phase 1 has to beat both at 0.2 and 0.5 s.
- **Consistency:** normalized innovation squared (NIS) on accepted updates should average about
  the measurement dimension (2 or 3). Too high means process noise is too low, and the gate is
  rejecting good data.
- **Tuning:** `opponent_accel_psd`, `opponent_yaw_accel_psd`, `keypoint_heading_sigma_rad` for
  opponents, and `arc_alignment_rad`, chosen by grid search on the training recordings against
  the 0.2 and 0.5 s errors plus the NIS check. The filter is linear and small, so a Python
  mirror in `auto_battlebot/perception/` runs the search over logged measurements without the
  C++ binary. A parity test against a C++ golden fixture keeps the two identical.
- **Script:** `playground/opponent_prediction/evaluate_opponent_prediction.py`, one CLI. Output
  goes under `playground/opponent_prediction/out/`, not `runs/`.

## Steps

1. **6-state filter.** Sections 1 and 2 with config fields (`opponent_yaw_accel_psd`,
   `initial_yaw_rate_sigma`, `heading_flip_reinit_count`, `heading_blind_labels`). Tests:
   - heading and yaw rate converge on a target turning at constant rate
   - a single flipped heading updates position only and increments the flip count
   - three consecutive flips re-initialize heading and leave position alone
   - a blob-born track reports infinite heading sigma until a keypoint heading arrives
   - a heading-blind label never corrects heading
   - late-measurement rewind still restores all six states
   - past `max_coast_s`, position, heading and yaw rate all hold
   - existing CV tests still pass with heading rows absent
2. **Uncertainty output.** Section 3: `PoseUncertainty`, render changes, diagnostics, Foxglove
   adapter and format doc. Test that sigmas shrink with measurements and grow while coasting.
3. **Prediction helper.** Section 4, with tests for arc versus straight selection, the wall
   stop, sigma growth, and `dt = 0` returning the input.
4. **Playback regression.** Replay the eval recordings with `KALMAN` on. Compare `num_gated`,
   `num_reinit`, flip counts and track identity swaps against `HOLD`. Nothing downstream
   (target selection, current navigations) should get worse.
5. **Prediction evaluation and noise fit.** The evaluation above, the Python mirror and its
   parity test, the grid search, and a write-up in `docs/experiments/kalman_filter/`.
6. **Enable.** `opponent_mode = "KALMAN"` in `config/_common.toml`.
7. **Phase 2, conditional.** Only if step 5 shows arc or lateral motion errors that the
   prediction helper cannot fix: the body-frame `[x, y, theta, u, s, omega]` EKF, evaluated the
   same way, adopted only if it wins on held-out recordings.

## Risks and open questions

- **Keypoint heading quality on unseen robots.** The learned model is the input that decides
  where the weapon arc goes. Measure per-label flip rate and heading error in step 5 before
  trusting rear and side strike logic against a new opponent.
- **Fast spinners.** Yaw rates above roughly 90 rad/s alias at 30 Hz. The heading-blind list
  handles known spinners; a robot that starts spinning mid-match shows up as heading NIS
  failures, which could switch it to heading-blind automatically. Not in scope until the data
  shows it is needed.
- **Identity swaps.** Association uses position only. Two opponents crossing can swap identities
  and carry the wrong heading. Heading could join the association cost later; out of scope here.
- **Field clipping.** The wall stop in prediction and the existing render clip must agree on the
  margin, or the predicted center jumps when the track coasts.
- **Our-robot uncertainty.** Navigation would also benefit from our own sigmas. Same struct,
  separate change.
