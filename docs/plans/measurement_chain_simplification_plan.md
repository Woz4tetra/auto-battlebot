# Measurement chain simplification

Delete the robot blob path and the staleness fields that predate a stable EKF. When this plan is done, a robot is either in the snapshot with a covariance or it is absent, and every measurement carries a yaw.

## Why

- `is_stale` and `measurement_age_s` describe the same thing as the EKF covariance, only more crudely. Five timers each decide independently when a robot counts as gone.
- The estimator infers whether a yaw is real from `keypoints.empty()`. Test helpers push a dummy keypoint just to switch heading fusion on (`tests/test_kalman_motion_estimator.cpp`, `make_our_pose_measurement`).
- The RL policy was trained with `our_age`, `our_stale`, `opp_age`, and `opp_stale` hard-coded to 0 or false (`auto_battlebot/rl/lockstep_world.py`). On the robot those inputs take values the policy never saw.
- The blob path is already off. Every shipped config sets `[robot_mask_model] type = "NoopRobotBlobModel"`, and on the ZED Box the keypoint model already detects `OPPONENT` and `HOUSE_BOT` with front and back keypoints.

## Chain today

Values are from `config/_common.toml`.

```
model ──► RobotFrontBackFilter ──► KalmanMotionEstimator ──► ControlLoop cache ──► nav / hazards / UI
          blob window 1.0 s        update(): stale = missed    snapshot reuse 0.25 s   RL: zero cmd at 0.15 s
          jump rejects ×5          this frame                                       MPN: stale → no speed
                                   coast(): stale = age > 0.1 s                     hazards: skip stale
                                   our mean freezes at 0.2 s                        UI: "seen" = !stale
                                   opp mean freezes at 0.1 s
                                   ours erased at 0.5 s
                                   opponents never erased
                                   reinit after 5 rejects
```

| Timer | Value | Where | Effect |
| --- | --- | --- | --- |
| `max_coast_s` | 0.2 s | estimator | Stops plant integration for our robot |
| `opponent_max_coast_s` | 0.1 s | estimator | Freezes the opponent mean while its covariance keeps growing to the caps |
| `coast_stale_age_s` | 0.1 s | estimator `coast()` | Sets `is_stale` |
| implicit | 1 frame | estimator `update()` | Also sets `is_stale`, by a different rule from `coast()` |
| `our_robot_hold_window_s` | 0.5 s (struct default 0.15) | front/back filter config, passed into the estimator through the context | Erases our track |
| none | never | estimator | Opponent tracks are never erased; `is_stale` is the only sign one is gone |
| `robot_cache_max_age_s` | 0.25 s | `ControlLoop` `RobotDescriptionsCache` | Replays the previous snapshot when ours or theirs is missing |
| `max_our_unmeasured_s` | 0.15 s | `RlPolicyNavigation` | Commands zero once snapshot age plus `measurement_age_s` exceeds it |
| `reinit_after_rejects`, `max_consecutive_jump_rejects` | 5, 5 | estimator, `FrameIdAssigner` | Two reject counters |
| `our_keypoint_dropout_blob_window_s` | 1.0 s | front/back filter | Suppresses blobs near our last keypoint position |

Findings:

- `RobotDescriptionsCache` only fires when our robot or theirs is absent. Opponents are never erased and ours is held for 0.5 s, so it only covers startup and a 0.25 s tail after our track is erased.
- `MotionProfileNavigation::measure_forward_speed` drops the EKF velocity whenever the robot is stale. Velocity prediction through dropouts is the reason the EKF exists.
- `DeadReckoningMotionEstimator` and `RobotTemporalMotionFilter` (about 125 lines) are used only by `config/playback/mr_stabs_mk2_playback.toml`. `OurRobotMode::DEAD_RECKONING` and `OpponentMode::HOLD` are struct defaults that `_common.toml` overrides. Each one carries its own output, stale, and age code in `update()` and `coast()`.
- `heading_blind_labels` is not set by any config.

## Chain after

```
 YoloKeypointModel  (one model, every class has FRONT/BACK keypoints)
   │  each detection: front + back keypoints → centre, size, yaw
   ▼
 RobotFrontBackFilter
   │  keypoints → pose with a measured yaw  (every measurement, every class)
   │  FrameIdAssigner jump gate             (×5 rejects)
   ▼
 KalmanMotionEstimator  (EKF ours, Kalman opponents)
   │  update():  correct position + heading, or skip
   │  coast():   propagate to now + render lead
   │               mean moves for at most max_coast_s, then holds
   │               covariance keeps growing
   │  expire:    erase any track once σ_pos > limit
   ▼
 RobotDescriptionsStamped
   │  each robot: pose, velocity, uncertainty (σ)
   │  robot missing  ⇔  track expired
   ▼
 ControlLoop
   ├─► RL nav:   our robot missing → zero command
   ├─► MPN:      EKF velocity
   ├─► hazards:  every neutral track
   └─► UI/diag:  "seen" = track exists
```

Two timing knobs remain: `max_coast_s` (how long the mean extrapolates) and the expiry limit (when a track is dropped). The front end keeps one gate, the jump gate.

## Steps

Each step is one commit. Replay one recording before and after each step with `./scripts/build_and_run.sh` on a playback profile, and compare the opponent track coverage and our-robot dropouts in the diagnostics.

### 1. Delete the blob path

- `src/robot_blob_model/` and `include/robot_blob_model/`: the YOLO bbox and Noop blob models and the rectangle keypoint helpers
- `[robot_mask_model]` in `config/_common.toml` and the three `config/simulation/*` files, plus its loader in `src/config/config.cpp`
- `ParallelModelBatch`: it only runs two models side by side; the runner calls the keypoint model directly
- In `RobotFrontBackFilter`:
  - `RobotKeypointTracker`
  - `merge_blob_detections` and the blob overwrite (`blob_overwrite_*`)
  - our-robot blob suppression (`our_keypoint_dropout_blob_*`)
  - the `robot_blob_*` limits
  - `last_suppressed_own_blob` and the `our_blob_present_no_keypoint` diagnostic
- The `robot_blob_keypoints` argument on `RobotFilterInterface::correct` and every implementation
- In `KalmanMotionEstimator`: `measures_heading()`, every `keypoints.empty()` branch (blob measurement noise, `has_speed_ref`, heading-flip counts), and `heading_blind_labels`
- The dummy keypoint in the `make_our_pose_measurement` and `make_opponent_pose_measurement` test helpers, and that row in `docs/plans/workaround_ledger.md`
- The blob sections in `docs/reference/perception/robot_filter_pipeline.md`, `trt_yolo.md`, and `trt_engine.md`

### 2. Delete the dead estimator modes

- `DeadReckoningMotionEstimator`, `RobotTemporalMotionFilter`, and their config registration
- `OurRobotMode::DEAD_RECKONING`, `OpponentMode::HOLD`, `our_tracks_`, and `held_opponent_tracks_`. If no other value is left, the mode enums go too
- Switch `config/playback/mr_stabs_mk2_playback.toml` to the EKF. Git history does not record why it chose dead reckoning, so its replay is the check for this step

### 3. Make track existence the only lost signal

- Add one expiry rule in the estimator that erases any track, ours or theirs, past the limit chosen below
- Delete:
  - `RobotDescription::is_stale` and `RobotDescription::measurement_age_s`
  - `output_stale`
  - `coast_stale_age_s`, and `our_robot_hold_window_s` with its context plumbing
- Delete `RobotDescriptionsCache` and `robot_cache_max_age_s`
- Delete `max_our_unmeasured_s`; `RlPolicyNavigation` commands zero when our robot is missing, as it does today
- Fold `opponent_max_coast_s` into `max_coast_s`
- Once the mean stops moving at `max_coast_s`, report zero velocity and yaw rate. Today `render_our_ekf` keeps `state.v` and `state.w`, so for 0.3 s after the mean freezes the policy sees a robot spinning in place. This closes the `src/navigation/rl_policy_navigation.cpp:189` ledger row
- Update the consumers:
  - `MotionProfileNavigation` always uses the EKF velocity
  - `HazardAssembler` keeps every neutral track
  - the runner's remote UI and the control loop's perception diagnostic count a robot as seen when its track exists
- Rewrite the dropout and coast tests in `tests/test_kalman_motion_estimator.cpp` to assert on expiry, not on the stale flag

### 4. Drop the age and stale observation inputs (nav v2)

Removing `our_age`, `our_stale`, `opp_age`, and `opp_stale` changes the observation layout. They go into the v2 observation in `docs/plans/nav_scoring_and_rl_v2_plan.md`, not into a patch to `observation_v1`. Until then `observation_v1` fills them with the constants the policy was trained on (0, false). The three opponent sigma inputs have the same zero-in-training problem: v2 either trains them with real values or drops them.

## Open decisions

- **Expiry rule.** One option is a σ limit (`max_position_sigma_m`), under which a track coasting fast drops sooner than one sitting still. The other is a single `track_timeout_s`, which is easier to read from logs. My recommendation is the σ limit.
- **Every class has front and back keypoints.** This is true for the ZED Box label set (`MR_STABS_MK2`, `MRS_BUFF_MK3`, `OPPONENT`, `HOUSE_BOT`). `config/_common.toml` lists only our two robots in `label_indices`. Confirm that no shipped profile depends on a class without keypoints before step 1 lands.

## Next steps

1. Pick the expiry rule.
2. Land step 1 with a before and after replay.
3. Land steps 2 and 3, with a replay for each.
4. Carry step 4 into the nav v2 observation work.
