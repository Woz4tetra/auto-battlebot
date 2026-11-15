# Opponent coast horizon (2026-10-15 sysid evaluation)

`opponent_max_coast_s` is the age since the last accepted measurement past which an opponent track holds position and heading. It is separate from `max_coast_s` because the our-robot EKF predicts from the commands it issued and an opponent track has none.

0.1 s is the best trade on the 2026-10-15 sysid evaluation.

- Longer horizons overshoot robots that stop or reverse: median error at 0.35 s is 98 mm with a 0.1 s horizon against 244 mm with 0.2 s.
- Shorter horizons lose the fresh-estimate gain: p90 error is 87 mm with 0.1 s against 176 mm with no estimate.
