# Playback regression tiers

`scripts/verify --playback` and `scripts/verify --realtime` replay fixed clips of the NHRL May
eval recordings through the full app and compare the output MCAP with a stored baseline. The code
is `auto_battlebot/verify/playback.py`; tests are `auto_battlebot/verify/tests/test_playback.py`.

## Commands

```bash
scripts/verify --playback                 # build the app, lockstep clips against the baseline
scripts/verify --realtime                 # build the app, real-time clips against A/A bounds
scripts/verify --fast --playback          # lint and the fast suites, then the lockstep clips
venv/bin/python -m auto_battlebot.verify.playback check --tier lockstep --clip sphinx
venv/bin/python -m auto_battlebot.verify.playback baseline --tier lockstep
venv/bin/python -m auto_battlebot.verify.playback baseline --tier realtime --runs 5
venv/bin/python -m auto_battlebot.verify.playback machine    # this box's machine class
```

A tier flag alone runs only that tier. With `--fast`, `--changed`, or `--suite` the suites run
too, in parallel with the tier. Each clip's app log, scratch profile, and MCAP land in
`data/verify/playback/<stamp>_<tier>/<clip>/`; the newest 5 runs are kept. `results.json` in the
run directory holds every clip's hashes and metrics.

`baseline` writes `auto_battlebot/verify/playback_baselines.json`. `check` never writes it, so a
baseline change always shows in the diff next to the code that caused it.

## Clips

`auto_battlebot/verify/playback_clips.toml` lists one 1,800-frame (60 s) clip per eval recording.
The path and start frame come from `config/playback/recordings.toml`; a clip can set its own
`start_frame`.

| Clip | Recording | Start frame | Profile base |
| --- | --- | --- | --- |
| `clyde` | `2026-05-02T10-06-06` | 8000 | `mrs_buff_mk3` |
| `sphinx` | `2026-05-02T11-45-08` | 13000 | `mrs_buff_mk3` |
| `wreck_creation` | `2026-05-02T14-12-27` | 3000 | `mrs_buff_mk3` |

The harness writes a scratch profile per clip that extends `playback/<robot>_lockstep` or
`playback/<robot>_playback` and sets `svo_file_path`, `svo_start_frame`, `svo_max_frames`, and
`[mcap] recordings_dir`. The app closes after `svo_max_frames` frames.

## Lockstep tier

`config/playback/_lockstep.toml` extends `_playback` and removes every run-to-run difference
found so far:

| Setting | Why |
| --- | --- |
| `svo_real_time_mode = false` | the SDK hands over every frame instead of skipping when the pipeline is slow |
| `svo_decode_to_start = true` | grabs with depth off up to the start frame; a seek lands on a non-keyframe and the next frames decode with garbage depth |
| `position_tracking = false` | ZED positional tracking drifts apart between runs (first difference at frame 108 of `clyde`); with it on, depth differs too |
| `rebase_stamps = false` | stamps stay on the recording clock instead of a wall-clock offset |
| `[clock] type = "SteppedClock"` | the clock follows frame stamps until the control loop steps it |
| `SteppedControlLoop`, `rate_hz = 250` | control cycles run inline, one 4 ms period per cycle |
| `max_loop_rate_hz = 0` | no sleep pacing |

`SteppedControlLoop::advance_to(stamp)` runs `floor(dt / period)` cycles and calls
`ControlLoop::run_cycle_at(t)` with `t = last + k * period` for each one. `run_cycle_at` calls
`ClockInterface::step_to(t)` before the cycle. `SteppedClock` takes that time and ignores later
`set()` calls from the runner, so the filter, navigation, and transmitter see a clock that
advances one period per cycle and coasts between frames. `ManualClock` ignores `step_to`, so the
simulation keeps one `now` for every catch-up cycle.

### What gets hashed

Every topic in the output MCAP is hashed in log order (SHA-256 over length-prefixed payloads),
except wall-time data:

- Topics: `/log`, `/diagnostics/health`, `/diagnostics/foxglove_publisher`, and every
  `/status/*` topic (rate-limited on wall time)
- Diagnostics entries with `held: true` (a held section expires on wall time) and the
  `relay_publish_link` entry
- Diagnostics fields `age_ms`, `rate`, `uptime_s`, `viz_dropped`, and every `*elapsed_ms`

What remains is 23 topics per clip, including `/robot_markers`, `/nav_markers`, `/tf`,
`/keypoint_detections`, `/hazard_markers`, and the filter, navigation, and transmitter
diagnostics.

### Pass rule

- Same machine class: every topic hash equals the baseline, and the red liveness checks match.
  When a clip differs, it runs once more and passes only if the rerun matches exactly; the report
  names the flake. A code change that moves any output differs on both runs.
- Another machine class only: metrics within `CROSS_CLASS_TOLERANCE` of the first class that has
  a baseline, and no new red checks. The report says which class it compared against.

The machine class is `<arch>_sm<compute capability>_trt<major.minor>_zed<major.minor>`, for
example `x86_64_sm89_trt10.14_zed5.5`. TensorRT engines and ZED depth are both tied to it.

### Remaining source of difference

ZED `NEURAL_LIGHT` depth occasionally changes a keypoint's height above the plane in the fifth
decimal, which moves its projected position by about 1 µm. One `wreck_creation` run in 8 showed
it; YOLO confidences matched to full float precision in that run. The rerun rule absorbs it.
Measurements: `docs/experiments/platform/2026-11-22_lockstep_playback_determinism.md`.

## Real-time tier

The clips run under the normal playback profile (`playback/<robot>_playback`: real-time SDK,
`ThreadedControlLoop`, `CameraFollowingClock`, positional tracking on). It scores:

| Metric | From |
| --- | --- |
| `latency_p95_ms` | `/diagnostics/runner` `pipeline.latency_ms` |
| `frame_drop_rate` | gaps in `/camera/frame_meta` `video_frame_index` |
| red checks | `/diagnostics/performance_monitor` entries at level 2 |

The baseline holds each metric's min and max over 5 A/A runs of the same code. A run passes when
each metric is inside `[min - m, max + m]` with `m = max(max - min, floor)`; the floors are 5 ms
and 0.02. A red check no A/A run saw fails. Run it on an idle box: other GPU or CPU load moves
latency more than any code change in the tier's range.

## Metrics

Both tiers record the same metrics; lockstep fails on hashes, the cross-class check uses these:

| Metric | Definition |
| --- | --- |
| `opponent_coverage`, `our_coverage` | fraction of `/robot_markers` messages with a `their_robot*` / `our_robot*` label |
| `opponent_jumps_per_min`, `our_jumps_per_min` | frame-to-frame moves over 0.3 m of the same track, per minute of clip |
| `linear_mean_abs`, `angular_mean_abs` | mean absolute `linear_out` / `angular_out` of `/diagnostics/playback_transmitter` |
| `moving_fraction` | fraction of transmitter samples with either command over 0.05 |
