# 2026-11-15 pipeline latency clock

These facts were source comments in `src/` and moved here on 2026-11-15 so the code carries names and tests instead of prose. Each section names the file and the code it described.

### `src/runner.cpp`

Beside `const double pipeline_latency_ms =`:

Measure end-to-end latency from when the image was sampled (camera frame timestamp) rather than from `robots.header.stamp`, which gets reused across cache substitutions and so under-reports latency on substituted ticks. Differenced against the logical clock, not the wall clock. Both give the same answer on hardware, where SystemClock is the wall clock and the frame stamps are wall-clock stamps. Off a recording only the logical clock shares a timeline with the stamps: the wall-clock form read -790 ms on a 70 s replay, reporting the age of the recording rather than the age of the frame.
