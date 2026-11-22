# PerformanceMonitor and verify_mcap

`PerformanceMonitor` (`include/health/performance_monitor.hpp`) watches every link in the loop and
the command chain while the app runs. It replaces a pre-session checklist. The same class grades a
recording offline through `scripts/verify_mcap`.

## Where it runs

- `ControlLoop` owns it when `[performance_monitor] enable = true` (the default). `main.cpp` builds
  it and gives it the viz sink's sent-frame counter for the relay link.
- Every control cycle, `ControlLoop::monitor_cycle` copies the transmitter's
  `TransmitterLinkReport` into it, records one `MonitorTick`, evaluates every
  `evaluate_period_s` (0.1 s), and publishes every `publish_period_s` (0.5 s).
- The Runner calls `note_camera_frame()` per camera frame. `submit_measurement` counts perception
  results. Both are atomic adds from the runner thread; everything else runs on the control thread.
- Nothing runs before the first camera frame, because `CameraFollowingClock` has no time yet.

## Output

Diagnostics module `performance_monitor`, so `/diagnostics/performance_monitor` in the MCAP and the
dashboard's `WarningStrip` banner (it shows the worst module).

| Subsection | Level | Values |
| --- | --- | --- |
| A firing check, e.g. `calibration` | WARN, or ERROR for red | the values that triggered it, `start_s`, `duration_s` |
| A check that just cleared | OK, message `cleared` | `start_s`, `end_s` |
| Each expected link with no warning, e.g. `espnow_link` | OK | `total`, `age_s`, `rate_hz` once a window has passed |

Red is `WarningSeverity::RED`, published as ERROR. A severity change ends the warning and starts a
new one, so a link that goes silent and then comes back slow records a red span and a warn span.

## Links

A link is judged only while it is expected. Silence past its max age, after `link_grace_s` (3 s)
from the moment it became expected, is red. A rate under the floor over the `window_s` (5 s)
window warns; a zero rate waits for the silence check.

| Link | Counted from | Expected while | Min rate, max age |
| --- | --- | --- | --- |
| `camera_frames` | `Runner::tick` after `camera_->get` | always | 10 Hz, 0.5 s |
| `perception` | `ControlLoop::submit_measurement` | field initialized | 10 Hz, 0.5 s |
| `filter` | control cycles with a field | field initialized | 10 Hz, 0.5 s |
| `transmit` | `TransmitterLinkReport::commands_sent` (successful writes) | field initialized and dashboard autonomy on | 10 Hz, 0.25 s |
| `crsf_telemetry` | `CrsfParser::crc_valid_frames()` in `OpenTxTransmitter` | the radio's autonomy switch (SA) on | 0.1 Hz, 10 s |
| `espnow` | ESP-NOW telemetry frames | field initialized and dashboard autonomy on | 50 Hz, 0.25 s |
| `imu` | telemetry frames with a finite `yaw_rate` | same as `espnow` | 50 Hz, 0.25 s |
| `relay_publish` | `VizSink::frames_sent()` | a viz sink exists | 5 Hz, 1 s |

CRSF link statistics only flow while the robot's receiver is connected, so silence is judged only
while SA is on. The `espnow` check also warns when the dongle's ack percent drops under 90 or the
robot's `espnow_age_ms` exceeds 100 ms during autonomy.

## Rolling checks

Fractions are over autonomy ticks (field initialized, dashboard autonomy on, and the radio's SA on
when the transmitter has one) in the last `window_s`, and need `min_window_ticks` (25) samples.

| Check | Fires when | Severity |
| --- | --- | --- |
| `slew_binding` | shaped stick and the transmitter's post-slew stick differ by more than 0.2 on over 10% of ticks | WARN |
| `guard_binding` | telemetry `guard_scale` below 0.9 on over 5% of ticks | WARN |
| `calibration` | our robot outside the centered field box on over 2% of ticks | WARN |
| `phantom_track` | a track with position sigma above 1 m and speed under 0.05 m/s for 1 s | WARN |
| `yaw_rate_disagreement` | correlation of the IMU yaw rate and the tracked heading derivative under 0.5, over at least 20 pairs, while the IMU rate's std is at least 0.5 rad/s | WARN |
| `thermal_trip` | telemetry `guard_state` is 4 (TRIPPED) | RED |
| `firmware_protocol` | the robot's `MSG_FIRMWARE_INFO` protocol version differs from the host's | RED |
| `our_tracking`, `opponent_tracking` | track present on under 85% of ticks; opponent only when the opponent count is above 0 | WARN |

- The IMU rate is converted to the field's counter-clockwise convention:
  `-yaw_rate_cw` upright, `+yaw_rate_cw` upside down (`espnow_yaw_rate_ccw_radps`).
- Headings are sampled only on cycles that consumed a measurement, so the derivative follows the
  tracker rather than the filter's IMU-driven prediction.
- A coasting opponent renders with zero velocity, which is how a phantom reads as not moving.

Every threshold is a defaulted field of `PerformanceMonitorConfiguration`
(`include/health/performance_monitor_config.hpp`); a profile overrides it under
`[performance_monitor]`.

## Command chain at load

`resolve_command_chain` (`include/health/command_chain.hpp`) turns the profile's transmitter into
key/value pairs:

| Key | EspNowTransmitter | OpenTxTransmitter |
| --- | --- | --- |
| `transmitter` | `EspNowTransmitter` | `OpenTxTransmitter` |
| `drive_processor` | config value | config value |
| `max_motor_rpm_per_s` | config value, `0` when absent | same |
| `protocol_version` | `espnow_protocol::PROTOCOL_VERSION` | `none` |

Playback, sim and noop transmitters drive no robot and have no chain.

`RlPolicyNavigation::load` compares it with the bundle's `[command_chain]` table
(`auto_battlebot/rl/export/bundle.py` writes `MK2_ESPNOW_COMMAND_CHAIN` by default):

- Any differing or missing key throws `PolicyBundleError` listing each key as
  `key: bundle X, profile Y`, so the policy does not load.
- A bundle without the table loads, and the `rl_policy_nav` subsection `command_chain` warns that
  the chain is unchecked, with the profile hash.
- A match logs both FNV-1a 64 hashes of the sorted `key=value` lines at OK.

`drive_mode` (nav plan 5.4) and a guard mode join the chain when they exist.

## Firmware identity

The Mk2 sends `MSG_FIRMWARE_INFO` (`FirmwareInfoPayload`: protocol version and a 16-byte build
string) once per second after a telemetry frame. `firmware/mr_stabs_mk2/scripts/firmware_build.py`
sets the build string to the 8-character git hash, with `-dirty` when tracked files differ.
`EspNowTransmitter` logs it under the `espnow_transmitter` subsection `firmware` when it changes,
at ERROR when the protocol differs. Firmware from before this frame sends none, and the
`firmware_protocol` check stays quiet.

## verify_mcap

```bash
scripts/verify_mcap data/recordings/<file>.mcap          # struct defaults
scripts/verify_mcap -c mr_stabs_mk2_fid_zed_box <files>   # a profile's [performance_monitor]
```

It runs `build/verify_mcap` (or `build-test/verify_mcap`), which replays the recording through
`grade_mcap` (`include/health/mcap_grader.hpp`) in log-time order and evaluates every
`evaluate_period_s` of log time. Each file prints its warnings with start and end times, the
total events counted per link, and notes on what it could not judge. Exit status: 0 no red, 1 a
red warning, 3 unreadable.

| Monitor input | Recorded source |
| --- | --- |
| transmitter type | `/status/config` field `transmitter.type`, else inferred from topics |
| camera frames | `/camera/frame_meta`, else `/camera/video` |
| perception | `/keypoint_detections` |
| filter, our pose, tracks, field box | `/status/tracks` (20 Hz) |
| autonomy, initialized | `/status/system`; the SA switch also from `opentx_transmitter` `switch_states` |
| CRSF telemetry | non-held `link_statistics`, `battery`, `attitude`, `flight_mode` in `/diagnostics/opentx_transmitter` |
| transmit | `opentx_transmitter` `send`, or a change of `espnow_seq` in telemetry |
| ESP-NOW, IMU, guard, command age, sent stick | `/robot/espnow_telemetry` |
| ack percent | `/diagnostics/espnow_transmitter` `link` |
| policy stick | `/diagnostics/rl_policy_nav` `policy.linear` |
| track sigma | `/diagnostics/kalman_motion_estimator` `opponent_<FRAME>` |

- A link whose source topic is absent is noted and not judged. The relay link is never recorded.
- Without `/status/system`, the field counts as initialized at the first detection, and transmit,
  ESP-NOW and IMU are not judged. Without an SA state, CRSF is not judged.
- Warnings that start in the last second are dropped as shutdown.
- Diagnostics publish once per runner tick (about 30 Hz), so offline transmit and CRSF rates top
  out there; the 10 Hz transmit floor holds for both sources.
