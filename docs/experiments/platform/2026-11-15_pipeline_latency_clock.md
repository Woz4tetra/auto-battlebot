# 2026-11-15 pipeline latency clock

## Latency clock

Measure end-to-end latency from when the image was sampled (camera frame timestamp) rather than from `robots.header.stamp`, which gets reused across cache substitutions and so under-reports latency on substituted ticks. Differenced against the logical clock, not the wall clock. Both give the same answer on hardware, where SystemClock is the wall clock and the frame stamps are wall-clock stamps. Off a recording only the logical clock shares a timeline with the stamps: the wall-clock form read -790 ms on a 70 s replay, reporting the age of the recording rather than the age of the frame.
