# Lockstep playback determinism (2026-11-22)

I ran SVO clips through the app twice and diffed the output MCAPs topic by topic to find what
still varies between runs. This was on pathfinder (RTX 4080 Laptop, sm89, TensorRT 10.14, ZED SDK
5.5), machine class `x86_64_sm89_trt10.14_zed5.5`. How the tier works now:
`docs/reference/tools/playback_regression.md`.

## Sources found, in the order they showed up

| Step | Config | Result on two runs |
| --- | --- | --- |
| 1 | `svo_real_time_mode = false`, `SteppedClock`, `SteppedControlLoop` 250 Hz, `max_loop_rate_hz = 0`, `rebase_stamps = false` | `/tf` `camera_world -> camera` first differs at frame 108 of `clyde` (300-frame clip), by about 0.4 mm; everything downstream follows |
| 2 | step 1 plus `position_tracking = false` | every non-diagnostic topic identical over 300 frames; diagnostics differ only in `age_ms`, `*elapsed_ms`, `runner.rate`, `runner.uptime_s`, relay counters, health |
| 3 | step 2, 1,800 frames, wall-time fields stripped | `held` diagnostics sections appear in one run and not the other (they expire on wall time); `/status/*` topics differ in count (10-20 Hz wall-rate limits); `/diagnostics/foxglove_publisher` differs in its window split |
| 4 | step 3, those excluded | all 3 clips identical on 23 topics |

`position_tracking = false` broke `wreck_creation`: 0.0 track coverage over 1,800 frames, against
0.95 with tracking on in real-time mode. The field fit put the camera 0.13 m under the floor after
3 plane-fit failures. The cause was the seek. With `svo_real_time_mode = false`, the frames right
after `set_svo_position(3000)` decode with garbage depth; the real-time SDK skips past them while
the app starts. Decoding sequentially to the start frame with depth off
(`svo_decode_to_start = true`) gave 0 plane failures and 0.95 coverage. It costs about 10 s for
3,000 frames and 45 s for 13,000.

Holding the first tracked pose instead of turning tracking off kept `/tf` identical but not the
height gate: with tracking on, ZED depth itself differs between runs.

## Remaining source

One `wreck_creation` run out of 8 with the final config differed from the baseline starting at
message 1,580 of `/diagnostics/front_back_keypoint_converter`. `MRS_BUFF_MK3_BACK` moved from
x 0.0580914 to 0.0580903 m. YOLO confidences in that run matched to full float precision, and the
height gate diagnostics matched, so the difference enters through `height_above_plane_m`, which
the height gate reads from ZED `NEURAL_LIGHT` depth. The other 7 runs matched bit for bit. The
lockstep check reruns a differing clip once and passes only on an exact match.

## Clip metrics (lockstep baseline)

| Clip | Opponent coverage | Our coverage | Opponent jumps/min | Our jumps/min | Mean abs linear | Moving fraction |
| --- | --- | --- | --- | --- | --- | --- |
| `clyde` | 0.893 | 0.721 | 7 | 5 | 0.593 | 0.967 |
| `sphinx` | 0.877 | 0.932 | 6 | 10 | 0.601 | 1.000 |
| `wreck_creation` | 0.952 | 0.985 | 0 | 14 | 0.578 | 1.000 |

A lockstep check of all three clips takes 2 min 15 s, most of it in decoding to the start frames.

## Real-time A/A baseline

5 runs per clip of the same code, idle box:

| Clip | Loop latency p95 (ms) | Frame drop rate |
| --- | --- | --- |
| `clyde` | 3.50-3.53 | 0.22% |
| `sphinx` | 3.56-3.58 | 0.17% |
| `wreck_creation` | 3.52-3.56 | 0.28% |

The spread is under 0.05 ms, so the 5 ms latency floor and 0.02 drop-rate floor set the bounds.

## Planted nav change

Multiplying `MotionProfileNavigation`'s `speed_kp` by 1.05 failed all three lockstep clips on both
the run and the rerun. `/robot_markers`, `/nav_markers`, and the transmitter, filter, and nav
diagnostics differed; `linear_mean_abs` moved by 0.0004-0.0005, which no metric tolerance would
have caught. Reverting the change passed `--playback` (136 s) and `--realtime` (189 s).
