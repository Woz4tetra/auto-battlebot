# Measurement chain simplification replays (2026-11-21)

I replayed `data/svo/tests/2026-08-28/2026-08-28T22-21-16.svo2` from frame 0 after each step of the
measurement chain simplification (commits `f7b96502`, `88293f79`, `0b75bb84`). Real-time mode, MCAP
written without images, scored from `/status/tracks` (20 Hz) and the `kalman_motion_estimator`
diagnostics.

## Run-to-run spread

Real-time replay is not deterministic. The detector output matched frame for frame (5095 frames,
same labels), but two runs of the same step 1 binary gave 1750 and 2067 opponent-measured frames.
Jump-gate decisions follow the predicted positions, which follow wall-clock tick timing. Treat
differences under about 15% in measured-frame counts as noise.

## Mrs Buff profile (`playback/mrs_buff_mk3_playback`)

| Metric | Baseline `9302c8bb` | Step 1 run a | Step 1 run b | Step 3 |
| --- | --- | --- | --- | --- |
| Opponent-measured frames | 1986 | 1750 | 2067 | 1863 |
| Our-measured frames | 3720 | 3559 | 3726 | 3757 |
| Opponent track exists | 99.96% | 99.96% | 99.96% | 55.5% |
| Opponent track live (exists, not stale) | 41.8% | 36.6% | 43.4% | 55.5% |
| Our track exists | 85.0% | 81.5% | 85.0% | 88.7% |
| Our track live | 77.6% | 75.2% | 77.6% | 88.7% |
| Opponent absences (count, p90) | 1, 0.07 s | 1, 0.08 s | 1, 0.08 s | 37, 3.83 s |

- Step 1 (blob path deleted) sits inside the run-to-run spread on every metric.
- Step 3: "live" and "exists" are the same thing now. The opponent is present for 55.5% of ticks,
  up from the 42% that were live before, because short dropouts no longer flag stale. It is absent
  through long gaps (longest 17.7 s) where it used to sit frozen at its last pose.
- Our track is present 88.7% of ticks, up from 85.0%. A track of ours expires on sigma, which a
  slow or parked robot takes much longer to reach than the old 0.5 s hold window.

## Mr Stabs profile (`playback/mr_stabs_mk2_playback`), step 2

The profile ran `DeadReckoningMotionEstimator` before step 2 and the `_common` EKF after.

| Metric | Dead reckoning (step 1 binary) | EKF (step 2) |
| --- | --- | --- |
| Our track exists | 2.8% | 43.0% |
| Our track live | 1.5% | 28.4% |
| Opponent track live | 21.4% | 77.1% |
| Target present | 89.0% | 89.0% |

The dead-reckoning arm emitted almost no our-robot track on this recording, so the EKF switch is
a large gain.

## Position sigma growth

Unmeasured from a 60-frame measured run, `mrs_buff_mk3` plant, `_common` noise:

| Unmeasured for | Opponent | Ours parked | Ours at 0.6 stick |
| --- | --- | --- | --- |
| 0.1 s | 0.13 m | 0.05 m | 0.25 m |
| 0.2 s | 0.32 m | 0.07 m | 0.38 m |
| 0.5 s | 1.09 m | 0.10 m | 0.63 m |
| 1.0 s | 2.22 m | 0.14 m | 0.90 m |

`max_position_sigma_m = 1.0` drops opponents about 0.47 s into a dropout and a driving robot of ours
about 1.2 s in. That is above the p90 opponent gap of about 340 ms
(`2026-05_nhrl_dropout_gap_quantiles.md` in `docs/experiments/perception/`).

## Shared coast horizon

`opponent_max_coast_s` (0.1 s) folded into `max_coast_s` (0.2 s). The 2026-10-15 sysid evaluation
measured the opponent's median position error at 0.35 s as 98 mm with a 0.1 s horizon and 244 mm
with 0.2 s (`kalman_filter/opponent_filter_evaluation.md`). The fold accepts that cost for one
horizon.

## Next steps

- Field-test RL nav with the new missing-robot rule. Between `max_coast_s` and expiry the policy
  now sees our robot stopped, where it used to command zero at 0.15 s.
- If opponent targeting suffers, measure `max_coast_s = 0.1` for both arms rather than restoring
  the split.
