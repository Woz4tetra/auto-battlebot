# Static detection gate (2026-08-29, MassD replay)

Measurements behind `StaticDetectionGate` and its config defaults. The gate ships off by default.

## What the replay showed

Arena floor graphics are fixed in the field frame; robots are not. The two logo blobs sit a median 0.043 m and 0.049 m from their own centre. A moving detection travels 0.127 m over 4 s even at the 10th percentile. The gate reads that gap.

- Logo blobs measure 0.043 to 0.049 m mean deviation from their running centre.
- Logo p99 deviation is 0.174 m, which sets the cluster-association radius: wide enough to hold a logo blob together without reaching across the arena.
- The static test uses the mean deviation, not the maximum: one noisy frame must not disqualify a logo, and one frame of a robot passing through must not either.

## Known failure direction

The gate cannot tell a genuinely immobilised robot from a painted one. A robot that drives into a suppressed cluster raises its spread and switches suppression off there until it leaves, which is the safe direction to fail.

## Bake-off against the height gate

End to end on the same replay it is a worse trade than the height gate at every setting tried, and stacks badly on top of it.

| Configuration | Logo removed | Moving kept |
| --- | --- | --- |
| Height gate alone | 91.1% | 89.8% |
| Height gate plus static gate (static_radius 0.08, dwell 4 s) | 95.1% | 75.5% |
| Static gate alone (0.06 m, 4 s) | 57.5% | 89.3% |
| Static gate alone (0.05 m, 8 s) | 34.7% | 95.9% |

Enable it for an arena where the height gate underperforms, and re-measure.
