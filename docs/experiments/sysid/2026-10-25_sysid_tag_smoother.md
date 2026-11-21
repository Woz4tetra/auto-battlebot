# Sysid tag smoother, 2026-10-25

Notes on the IMU fusion, hold-out noise selection and gating in `auto_battlebot/perception/tag_pose_smoother.py`. The measurements come from the 2026-10-25 sysid sessions.

## Forward filter and gating

Forward Kalman filter with chi-square gating, then the RTS backward pass.

Axes never interact
except through the joint gate: a knot's measurements pass or fail together, against the
chi-square quantile for as many degrees of freedom as axes measured.

A gated position fix is still taken, with the prediction covariance inflated to the prior,
once `reacquire_s` has passed without an accepted fix or after `max_consecutive_rejects`
rejections in a row. That covers the filter losing the robot across a long dropout, where
every later fix would otherwise fail the gate. The inflation enters `p_pred`, so the RTS
pass sees it as extra process noise on that step and stays consistent.

## Process noise selection

Pick each group's process noise by hold-out: drop every `hold_every`-th position fix,
smooth the rest once per candidate, score the smoothed state against the dropped fixes.

All groups step through their candidate lists together in one batch (entry b uses the b-th
candidate of every group), and each group is scored on its own axes. The groups are
independent in the model, so this equals a separate sweep per group except for the joint
gate. The score is the mean squared held-out residual over its variance, clipped at
`clip` so an outlier fix cannot choose q.

## IMU tilt fusion

The tilt smoother reruns with the robot IMU's tilt. Over 0.5 s intervals where pitch changes by more than 3 degrees, tag pitch against integrated gyro on s05 (2026-10-25) correlates 0.84.

Pitch: the gyro's pitch rate (`gyro_y`, the chip axis along the axle, which picks up no yaw)
at every upright telemetry sample, so the tags only pin its bias. The BNO055's fused Euler
pitch is not used where the gyro is: on 2026-10-25 hard spins pulled it up to 18 degrees off
while the tags and the gyro read the robot flat. The gyro's sign is measured against tag pitch
changes and the fusion skipped when they do not agree.

Roll, and pitch without the gyro: YOLO fixes carry no tilt, and across a long dropout the tilt
state wanders far enough to mark a robot sitting flat as lifting its nose. At each YOLO fix the
BNO055 Euler angle is mapped onto the tag angle by a straight line fitted on upright tag
detections; its residual (about 2 degrees, mostly tag noise) is the sigma.

The IMU lag is the one the yaw-rate check measured.

## Yaw branch with IMU rate knots

Without a rate measurement a dropout leaves only the last fix's rate to carry the angle
forward. With the IMU the prediction is the
better reference: carrying a 3 rad/s rate from the last fix across a 1 s gap lands more than
half a turn off and picks the wrong branch. Across 40 withheld 1 s tag spans the carried
reference flipped the heading by half a turn in 9 of them; the prediction flipped none.

## IMU rate noise

Still-period noise alone understates the IMU's error mid-spin, where a too-tight sigma gates
out good tag fixes; the spin residual after the gain correction (which includes some
tag-rate noise) covers it. Before the correction, the 0.84 gain of clipped spins inflated
it to about 3 rad/s, loose enough that a dropout could hide a whole extra turn.

## Tilt aliasing, 2026-10-25

With wrapped tilt innovations, a pitch rate of one turn per camera frame (377 rad/s at 60 Hz) fits every fix exactly. On 2026-10-25 the cross-validated q let the filter settle there: right at the knots, a random angle between them. The tilt smoother takes unwrapped angles and never wraps its innovations.

## Yaw rate alias on 2026-10-15_00-29-00

Fixes at 60 Hz cannot tell 0 rad/s from a whole turn per frame, so the IMU must decide. A bad rate seeds the alias for 0.5 s and then the IMU reports the truth (still). Gating the rate knots against the aliased state rejected all of them, and the session `data/recordings/auto_battlebot_mr_stabs_mk2_sysid_zed_box_2026-10-15_00-29-00.mcap` spanned hundreds of rad/s for most of its length. The rate knots skip the gate.

## Innovation whiteness

One white-jerk q has to cover stills and punches, so the innovations cannot be white everywhere. The cross-validated pick lands slightly over-responsive, with a lag-1 innovation autocorrelation between -0.5 and 0.1.
