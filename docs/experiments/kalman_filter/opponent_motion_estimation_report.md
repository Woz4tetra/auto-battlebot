# Opponent motion estimation report, 2026-10-16

The opponent filter now estimates each opponent's position, heading, velocity and yaw rate with
uncertainties, and it runs in production (`opponent_mode = "KALMAN"` in `config/_common.toml`).
This replaces `docs/plans/opponent_motion_estimation_plan.md`, which is deleted; this report keeps
the design, the contract `RlPolicyNavigation` reads, and the results. The detailed evaluation,
with every table and figure, is `opponent_filter_evaluation.md` next to this file.

## Outcome

| Measure, sysid ground truth | HOLD (before) | Old 4-state | 6-state filter |
| --- | --- | --- | --- |
| Fresh position error p90, tuning session | 165 mm | 86 mm | 80 mm |
| Position error at 0.1 s p90, tuning session | 272 mm | 192 mm | 178 mm |
| Position error at 0.35 s median, held-out session | 36 mm | 51 mm | 50 mm |
| Heading error once stale, median | 3 to 7° | 77 to 172° | 6 to 9° |
| Heading on spins, median | 52.5° | 53.0° | 31.8° |
| Heading 2σ coverage | none | none | 0.94 to 0.99 |

- The filter halves the short-horizon position tail against HOLD and fixes the old arm's heading,
  which swung to the velocity direction whenever the robot drove backwards or slid.
- It does not beat HOLD on position at 0.35 s. Long dropouts are mostly robots decelerating to a
  stop, and no kinematic model predicts that. The plan's gate required it; Ben amended the gate on
  2026-10-16 to heading at every horizon plus position at 0.1 s, which the filter passes, and the
  0.35 s miss is a known limitation.
- Eight NHRL May and MassD match replays showed no identity swaps, no loss of opponent coverage,
  and at most 1.1% of opponent measurements rejected by the gate.

## What was built

| Piece | Where |
| --- | --- |
| 6-state opponent arm `[px, py, vx, vy, theta, omega]` | `src/robot_filter/kalman_motion_estimator.cpp` |
| `PoseUncertainty` on `RobotDescription` | `include/data_structures/robot.hpp` |
| Opponent config fields | `include/robot_filter/motion_estimator_config.hpp` |
| Batched torch mirror | `auto_battlebot/perception/opponent_filter.py` |
| Golden fixture and its generator | `tests/robot_filter/opponent_filter_golden_data.hpp`, `playground/opponent_prediction/make_opponent_filter_golden_fixture.py` |
| C++ tests (11 new) and Python parity tests | `tests/test_kalman_motion_estimator.cpp`, `tests/python/test_opponent_filter.py` |
| 2σ ring and heading sigma on `/robot_markers` | `src/foxglove_adapters/scene.cpp`, `docs/foxglove_recording_format.md` |
| `opponent_measurement_<id>` diagnostic | every opponent measurement the estimator receives, so replays rerun offline |
| Evaluation, grid search, replays, samples, videos | `playground/opponent_prediction/` |
| Measurement model for the RL sim | `playground/opponent_prediction/sensing_model_opponent.toml` |

## Design

**State and model.** Translation is constant velocity with white-noise acceleration
(`opponent_accel_psd` 30). Heading is an independent constant-yaw-rate block
(`opponent_yaw_accel_psd` 30). The blocks never correlate.

**Measurements.** A keypoint pose corrects position (chi-square gated at `gate_nis`) and then
heading as a separate row, which equals the joint 3-row update because the blocks are
uncorrelated, and keeps a bad heading from costing the track its position fix. Position sigma is
`opponent_position_sigma_m` (12 mm), heading sigma `opponent_heading_sigma_rad` (0.15 rad). Blob
centroids correct position only.

**Flips.** A heading more than pi/2 from the track counts as a front/back flip and corrects
nothing; three in a row re-initialize the heading block. The flip test is skipped while the
heading sigma is at least pi/4, because a wide prior cannot tell a flip from an unseen turn.

**Heading-blind labels.** `heading_blind_labels` never correct heading and report an infinite
heading and yaw-rate sigma. A track born from a blob reports the same until its first keypoint.

**Coasting.** The state extrapolates for `opponent_max_coast_s` (0.1 s) past the last accepted
measurement, then holds; a step that crosses the horizon integrates only its part before it. The
covariance always propagates through the full transition, so the sigma keeps growing and the gate
accepts the reacquisition. Velocity and yaw-rate variance are capped at their initialization
priors. Our robot keeps its own `max_coast_s` (0.2 s), since it predicts from known commands.

**Rendering.** `coast()` renders at now plus the plant `delay_s`, clips to the field bounds, and
zeroes any velocity component pointing into a wall the rendered center has reached.

**Latency.** Measurements older than the track rewind to a snapshot (64 slots), correct at their
own stamp, and re-propagate.

## Contract for RlPolicyNavigation

The observation builder reads the rendered estimate, at now plus `render_lead_s()`:

Scales and caps are set by `auto_battlebot/rl/obs_spec.py`, which both builders implement; this
table follows it.

| `RobotDescription` field | Meaning | Policy observation |
| --- | --- | --- |
| `pose` position | filtered center | position, distance, bearing in our body frame |
| `pose.rotation` | filtered heading `theta` | heading relative to ours, sin and cos |
| `velocity.vx`, `vy` | filtered velocity, wall-stopped | velocity in our body frame |
| `velocity.omega` | filtered yaw rate; 0 when the heading is unknown | yaw rate [15 rad/s] |
| `uncertainty.position_sigma_m` | sqrt of the larger eigenvalue of the position covariance | [0.1 m], capped at 1 |
| `uncertainty.heading_sigma_rad` | infinite for heading-blind labels and tracks with no keypoint yet | [pi/2], capped at 1 |
| `uncertainty.yaw_rate_sigma_rad_s` | infinite in the same cases | [15 rad/s], capped at 1 |
| `is_stale` | no accepted measurement for 0.1 s, or this frame's was gated out | stale flag |

Rules the builder relies on:

- An infinite sigma is written as the cap (1.0 after scaling), never as infinity or NaN.
- A heading-blind label reports `heading_sigma_rad` at the cap.
- A rejected flip leaves `pose.rotation` on the filtered value; the policy never sees the raw
  flipped measurement.
- The position sigma is conservative (2σ coverage about 1.00 on the sysid data), so the policy
  will mostly read it at the cap.

## The mirror

`auto_battlebot/perception/opponent_filter.py` is the same filter in torch, batched over rows,
with config fields that take a float or a per-row tensor. It serves the grid search, the parity
test, and the RL training environment (one filter per world, fed simulated measurements from
`sensing_model_opponent.toml`), so the policy trains on the estimator it runs against. The C++
test replays a scripted scenario (1,120 events: blob birth, flips, a three-flip re-initialization,
a missed rewind, a 0.5 s dropout, a gated jump, a wall) and must match the mirror to 1e-6 on
state, covariance and render. Any change to the C++ opponent arm must regenerate the fixture in
the same commit, and a filter change after training means a retrain.

## Evaluation in brief

- **Ground truth.** The plan called for apartment-floor recordings with a tag on Mrs Buff Mk3;
  they do not exist yet. The two 2026-10-15 Mr Stabs Mk2 sysid sessions stood in: YOLO keypoints
  from the recorded video as the opponent measurements, the tag-only smoother bundle as truth
  (never containing a YOLO fix), the 63 ms perception latency and 52 ms render lead replayed.
  `00-29-00` tuned, `00-20-58` held out.
- **Tuning.** A 480-row grid. Heading values are interior optima; the accel PSD was flat from 3
  to 1000. A coast-horizon sweep picked 0.1 s: at 0.2 s the 0.35 s median was 244 mm, at 0.1 s
  98 mm, with no loss fresh.
- **Gate.** Original: beat HOLD and the old arm at 0.1 and 0.35 s. Failed on held-out position
  at 0.35 s (50 against 36 mm). Amended by decision; passed.
- **Phase 2.** The plan's body-frame EKF `[x, y, u, s, theta, omega]` was triggered by arc
  errors at 0.35 s, built in the mirror (`phase2_body_frame.py`), and tied phase 1 on the holdout
  (120 against 122 mm summed gate medians). Not adopted.
- **Match replays.** NHRL May 11-45, 14-12, 16-18, 17-26 and MassD 11-17, 13-20, 15-21, each in
  HOLD and KALMAN. No identity swaps; flips against the filtered heading reached 8.5% of
  measurements on MassD 11-17, handled by the flip logic.

## Bugs found and fixed while scoring

- **Bug:** past the coast horizon the covariance froze with the state, so after a 0.5 s gap the
  gate rejected the reacquired measurements until `reinit_after_rejects` ran out.
  **Fix:** the state holds and the covariance keeps propagating.
- **Bug:** after a long gap a real turn looked like a front/back flip and was ignored for three
  frames. **Fix:** no flip test while the heading sigma is at least pi/4.
- **Bug:** the hold decision was taken per step, and the first coast after a measurement spans
  115 ms, so any horizon shorter than that overshot. **Fix:** integrate only up to the horizon.
- **Bug:** an opponent unseen for minutes grew its position variance with t^3.
  **Fix:** cap velocity and yaw-rate variance at their priors.

## Samples and videos

- Figures for 0.2 s, 1 s and 3 s dropouts on both sysid sessions and four matches:
  `playground/opponent_prediction/plot_dropout_samples.py`; 12 are committed under
  `assets/opponent_filter_2026-10-15/`.
- Camera-overlay videos, one per scene (apartment floor with tag truth, NHRL May cage, MassD
  arena), each with a 0.2 s, 1 s and 3 s dropout at half speed:
  `playground/opponent_prediction/render_dropout_videos.py`. The videos are not committed; the
  script regenerates them into `playground/opponent_prediction/out/videos/` from the sysid
  recording and the KALMAN replays (`run_replays.py`).

## Limitations

- Position at 0.35 s is worse than HOLD when the opponent stops during a dropout.
- The position sigma is conservative; YOLO's error is a slow bias, so NIS is 0.05 to 0.4 against
  2 to 3 expected. A bias state or a shorter render lead would fix the coverage.
- One robot, one floor, one camera rig, and a keypoint model that has seen Mr Stabs a lot: heading
  error and flip rate here are a floor for unseen opponents.
- Laptop playback drops about one frame per 0.5 s and its clock steps back a few milliseconds per
  re-anchor, which logs "Coast span is -0.00x s" warnings now that opponent tracks commit every
  tick. Live runs use the system clock and do not.

## Next steps

- Record the apartment-floor sessions with a tag on Mrs Buff Mk3 (RL plan section 9) and rerun
  `evaluate_opponent_prediction.py` and `phase2_body_frame.py` on an opponent that is not driven in
  sysid steps.
- Add MassD 11-17's opponent to `heading_blind_labels` if it gets its own label.
- Merge `sensing_model_opponent.toml` into the RL plan's `sensing_model.toml` when
  `fit_sensing_model.py` exists.
