# Opponent motion estimation plan

Goal: estimate each opponent's pose (position and heading) and velocity (linear and yaw rate)
with uncertainties, so `RlPolicyNavigation` (`docs/plans/rl_policy_navigation_plan.md`) sees
where the opponent is, which way its weapon faces, how it is moving, and how sure the filter is
of each. The same filter runs, batched, inside the policy's training environment, so the policy
trains on the filter output it will see on the robot.

This plan used to serve `PlantRolloutNavigation`, which the RL plan replaces. The forward
prediction helper that planner needed (old section 4) is dropped: the policy predicts the
opponent itself from its observation history.

The heading measurement comes from the learned NHRL keypoint model, trained on synthetic data,
which gives front and back keypoints for arbitrary robots. This plan does not change perception.

## Status, 2026-10-16

Steps 1 to 6 are done and step 7 is not triggered. Results, figures and the gate verdict are in
`docs/experiments/kalman_filter/opponent_filter_evaluation.md`. The apartment-floor recordings
with a tag on Mrs Buff Mk3 did not exist yet, so step 5 used the two 2026-10-15 Mr Stabs Mk2
sysid sessions as ground truth, with Mr Stabs standing in for the opponent.

Changes from the design below, each found while building or scoring it:

- **Heading row applied after position.** The translation and heading blocks never correlate, so
  correcting position and then heading equals the 3-row update. Doing it in sequence keeps the
  chi-square gate on position alone: a heading 60 degrees off must not cost the track its
  position fix. The flip test is the heading's gate.
- **Flip test only while the heading is known.** Past a heading sigma of pi/4 a measurement more
  than pi/2 away is as likely an unseen turn as a flip, and is taken. Without this, the first
  headings after a 1 s gap were called flips until three of them forced a re-initialization.
- **Covariance keeps propagating past the horizon.** Position and heading hold, but P propagates
  through the full transition. With an identity transition the position sigma stayed at
  centimetres through a 0.5 s gap and the gate rejected the reacquisition.
- **Variance caps.** Velocity and yaw-rate variance are capped at their initialization priors, so
  a track unseen for minutes does not grow its position variance with t^3.
- **Coast horizon split within a step, and opponent-only.** A step that crosses the horizon
  integrates only its part before it; the first coast after a measurement spans 115 ms of latency
  and lead, so a per-step decision overshot any horizon shorter than that.
  `opponent_max_coast_s = 0.1`, separate from the our-robot `max_coast_s = 0.2`: longer
  horizons overshoot robots that stop or reverse.
- **Opponent measurement sigmas separate from the our-robot ones:** `opponent_heading_sigma_rad`
  and `opponent_position_sigma_m`, since `keypoint_heading_sigma_rad` and
  `keypoint_position_sigma_m` also tune the our-robot EKF.
- **`opponent_measurement_<id>` diagnostic.** Every opponent measurement the estimator receives, in
  either mode, so a recording replays through the mirror offline.
- **Measurement model file.** `playground/opponent_prediction/sensing_model_opponent.toml`, the
  opponent section the RL plan's `fit_sensing_model.py` will merge into `sensing_model.toml`.

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
- **Prediction:** none beyond the filter's own coast to the render time. The policy reads
  `omega`, velocity and the sigmas and learns how the opponent moves next.
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

### 4. Wall stop in the renderer

A constant-velocity coast of an opponent pinned on a wall drives through the wall. Within
`max_coast_s`, `coast()` zeroes the velocity component into any wall the rendered center
reaches, which also stops the clip in `clip_to_field_bounds` from fighting a velocity that
points out of the field. The rendered velocity is what the policy sees, so an opponent against a
wall must read as stopped, not as still driving into it.

### 5. Turning it on

Set `opponent_mode = "KALMAN"` in `config/_common.toml` once step 4 below passes. Playback and
sim profiles inherit it. `HOLD` stays available as a fallback.

### 6. What RlPolicyNavigation reads

The contract between this filter and the policy's observation builder
(`rl_policy_navigation_plan.md`, section 3). Every field is from the rendered estimate, at now
plus `render_lead_s()`.

| `RobotDescription` field | Meaning | Policy observation |
| --- | --- | --- |
| `pose` position | filtered center | position, distance, bearing in our body frame |
| `pose.rotation` | filtered heading `theta` | heading relative to ours, sin and cos |
| `velocity.vx`, `vy` | filtered velocity, wall-stopped | velocity in our body frame |
| `velocity.omega` | filtered yaw rate | yaw rate [15 rad/s] |
| `uncertainty.position_sigma_m` | section 3 | [0.1 m], capped at 1 |
| `uncertainty.heading_sigma_rad` | section 3; at the cap for heading-blind labels and blob-born tracks | [pi], capped at 1 |
| `uncertainty.yaw_rate_sigma_rad_s` | section 3 | [15 rad/s], capped at 1 |
| `is_stale` | past `max_coast_s` | stale flag |

Rules the builder relies on:

- An infinite sigma is written as the cap (1.0 after scaling), never as infinity or NaN.
- A heading-blind label reports `heading_sigma_rad` at the cap and its weapon is `FULL_CIRCLE`
  in the observation, whatever `[navigation.opponent_weapons]` says. The heading-blind list is
  the one source for "cannot be oriented".
- A heading flip that the filter rejected leaves `pose.rotation` on the filtered value. The
  policy never sees the raw flipped measurement, only the filter's response to it.

### 7. Batched mirror for training

The Python mirror (Evaluation, below) is written once, vectorized over tracks in torch, so the
same code serves three callers:

1. the grid search over logged measurements on the CPU
2. the parity test against the C++ golden fixture
3. the RL training environment on the GPU, one filter per world, fed simulated keypoint
   measurements

In the training environment, the opponent's true pose becomes a measurement through the fitted
measurement model (position and heading noise, dropout gaps, 180 degree flips, latency; RL plan
section 4), goes through the mirrored filter, and is rendered at the same lead the C++ uses. The
policy then trains on exactly the estimator it runs against, including its lag, its coast
behaviour and its response to flips, instead of on a fitted approximation of the filter's error.
The filter is linear with 6 states, so 8,192 copies cost little next to the physics step.

## Evaluation

The question is how close the rendered estimate, which is what the policy sees, sits to the
opponent's true pose: on fresh frames, and while coasting through the dropouts the policy has to
drive through (p90 gap 340 ms).

- **Ground truth:** the apartment-floor recordings from `rl_policy_navigation_plan.md`
  section 9, with an AprilTag on Mrs Buff Mk3 detected offline from the video. This is the only
  data with an independent opponent pose, so it scores the filter directly.
- **Volume and opponent variety:** NHRL May and MassD recordings, replayed in playback mode with
  the learned keypoint model, recording the new diagnostics. No ground truth there, so they are
  scored against later measurements, as below. Hold out whole recordings.
- **Metric, against ground truth:** rendered position, heading and yaw-rate error at each tick,
  split by time since the last accepted measurement (0, 0.1, 0.2, 0.35 s) and by motion class
  (still, straight, arcing, spinning). Also the fraction of ticks where the truth falls inside
  2 sigma, per field; the policy reads the sigmas, so they have to mean what they say.
- **Metric, against later measurements:** from each accepted correction at time t, coast the
  filter to t + h for h in 0.1, 0.2, 0.35 s and compare against the next accepted measurement
  within a frame of t + h. Position error (median and p90) and heading error, by opponent label.
- **Baselines:** today's `HOLD` (estimate = last pose) and today's 4-state `KALMAN` with
  velocity-direction heading. Phase 1 has to beat both at 0.1 and 0.35 s.
- **Consistency:** normalized innovation squared (NIS) on accepted updates should average about
  the measurement dimension (2 or 3). Too high means process noise is too low, and the gate is
  rejecting good data.
- **Tuning:** `opponent_accel_psd`, `opponent_yaw_accel_psd` and `keypoint_heading_sigma_rad`
  for opponents, chosen by grid search on the training recordings against the ground-truth
  errors at 0 and 0.35 s, the 2-sigma coverage, and the NIS check. The filter is linear and
  small, so a Python mirror in `auto_battlebot/perception/opponent_filter.py` runs the search over
  logged measurements without the C++ binary. A parity test against a C++ golden fixture keeps
  the two identical. Section 7 reuses the same mirror in training.
- **Measurement model:** the same ground-truth recordings give the keypoint measurement errors
  the training environment injects: position and heading noise by speed, flip rate by speed and
  label, dropout gap lengths, latency. Written to the RL plan's `sensing_model.toml`.
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
2. **Uncertainty output and wall stop.** Sections 3 and 4: `PoseUncertainty`, render changes,
   the renderer wall stop, diagnostics, Foxglove adapter and format doc. Tests: sigmas shrink
   with measurements and grow while coasting; a heading-blind track reports the capped heading
   sigma; an opponent coasting into a wall renders with zero velocity into it.
3. **Python mirror and parity.** Section 7: `auto_battlebot/perception/opponent_filter.py`,
   vectorized over tracks in torch, plus the C++ golden fixture and a parity test to 1e-6 on
   state, covariance and rendered output, including a rewind and a flip re-initialization.
4. **Playback regression.** Replay the eval recordings with `KALMAN` on. Compare `num_gated`,
   `num_reinit`, flip counts and track identity swaps against `HOLD`. Nothing downstream
   (target selection, current navigations) should get worse.
5. **Evaluation and noise fit.** Needs the apartment-floor recordings with the Mrs Buff Mk3
   tag. The evaluation above, the grid search, the measurement model for the RL sensing model,
   and a write-up in `docs/experiments/kalman_filter/`.
6. **Enable.** `opponent_mode = "KALMAN"` in `config/_common.toml`.
7. **Phase 2, conditional.** Only if step 5 shows arc or lateral motion errors at 0.35 s that
   phase 1 cannot fix: the body-frame `[x, y, theta, u, s, omega]` EKF, evaluated the same way,
   adopted only if it wins on held-out recordings. A phase 2 filter also needs its own mirror,
   and a policy retrain.

Order against `rl_policy_navigation_plan.md`:

| This plan | Must finish before RL step | Why |
| --- | --- | --- |
| 1, 2 | 2 (observation spec and parity) | the spec reads `omega` and the sigmas |
| 3 | 4 (environment) | the environment runs the mirror |
| 5 | 7 (training stages C and D) | the measurement model and tuned noise feed the sim |
| 6 | 5 (closed-loop gate G2) and 9 (real runs) | the C++ binary must run the filter the policy trained on |

The apartment-floor recordings (RL step 0) can happen before any of this. They log the raw
`/keypoint_detections`, so the filter can be replayed over them offline with the mirror.

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
- **Field clipping.** The renderer wall stop and the existing render clip must agree on the
  margin, or the rendered center jumps when the track coasts.
- **Mirror drift.** Any change to the C++ filter must land in the Python mirror in the same
  commit, or the policy trains against a different estimator than it runs with. The parity test
  is the guard; a filter change after training also means a retrain.
- **Our-robot uncertainty.** Navigation would also benefit from our own sigmas. Same struct,
  separate change.
