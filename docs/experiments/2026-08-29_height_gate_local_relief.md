# Height gate: local relief (2026-08-29, MassD replay)

Measurements behind `KeypointHeightGate` and its config defaults.

## What it measures

The gate measures local relief, not absolute height: the upper percentile of depths inside the detection's inscribed circle minus the floor immediately around it. Comparing a detection to its own surroundings cancels field-plane fit error, plane drift and depth bias, which otherwise smear the two populations together.

Setup: 2026-08-29 MassD replay, separating arena-logo detections (persistent field-frame clusters) from moving ones.

- Elevation (local relief): AUC 0.928 against moving opponent detections, 0.959 against our own robots.
- Absolute height: AUC 0.903.

## Two sampling rules, both pinned by tests

- Sample inside the detection box, never at the keypoint pixels. Blob keypoints are the ends of a box midline and land on the floor beside a robot. mr_stabs_mk2 read -0.019 m at its keypoint pixels and +0.056 m over its box.
- Take an upper percentile, never a median. The camera views the arena about 23 degrees off horizontal, so even an inscribed circle holds floor alongside the robot and a median measures the floor. The question is "does anything in this detection stand up".

The per-keypoint height annotation uses a median over the sample window centred on (x, y), not a mean, because depth bleeds across object edges, where the outliers are hundreds of millimetres rather than a few.

## Gate behavior

- Measurement is always on. Each surviving keypoint carries the height measured at its own pixel, which `RobotKeypointTracker` and `FrontBackKeypointConverter` project with in place of the configured `keypoint_height_meters` constant. A keypoint that cannot be measured keeps NaN and falls back to the constant, so there is no false-negative risk.
- Rejection is on by default and uses the local relief: top percentile inside the inscribed circle minus the floor percentile of a ring around it.
- The gate abstains rather than guesses. With no depth, no field transform, no detection box, or too few valid samples in either region, the group passes through untouched.

## Default threshold

Rejection at the default threshold removes 91.1% of arena-logo detections on the replay, keeps 89.8% of moving opponent detections, and keeps 91.1% of the frames that had any opponent detection at all.

Minimum elevation sweep, end to end (logo removed / moving kept):

| Threshold (m) | Logo removed | Moving kept |
| --- | --- | --- |
| 0.025 | 71.0% | 95.2% |
| 0.030 | 88.4% | 89.6% |
| 0.035 (default) | 91.1% | 89.8% |
| 0.040 | 94.2% | 85.6% |
| 0.045 | 96.4% | 75.3% |

0.035 dominates 0.030 on both axes. Past 0.035 the cost to real detections climbs steeply for a few more points of logo.
