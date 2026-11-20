# Recording format: Foxglove schemas

Every MCAP the C++ app records, every MCAP the Python tools write, and every converted legacy
recording follows this layout. The C++ publisher (`src/foxglove_adapters/`), the Python readers
(`auto_battlebot/recording/mcap_io.py`, `auto_battlebot/recording/diag_io.py`), the Python writers
(`auto_battlebot/recording/mcap_write.py`) and the legacy converter (`scripts/convert_ros1_mcap.py`) are
all written against this document. Change it here first.

`McapWriter` opens each file in its own `foxglove.Context`, so one topic can be written to several
files in one process. Every `log_*` method takes an explicit `log_time_ns`; the writer never reads
the wall clock. `tests/python/test_mcap_io.py::test_two_writers_same_topic_separate_files` pins the
first.

## File level

- MCAP profile string is empty. The C++ recorder writes uncompressed chunks (it records on the
  Jetson at 60 Hz; zstd on the hot path is not free). Python-written files (calibration
  captures) use zstd chunks; readers do not care.
- One metadata record named `auto_battlebot` with key `active_profile` holding the config
  profile id (or empty when the app was run with an explicit `-c`), and, on RGB-camera
  recordings, `camera_calibration`: the calibration `/camera/video` needs, as the same TOML text
  a `config/cameras/<id>.toml` file holds. Recordings from before 2026-09-24 carry only
  `calibration_id`, which playback resolves against `config/cameras/`. Recordings from
  2026-11-19 on also hold `resolved_config`: the `--print-config` text for the run, every field
  with its value and the `file.toml:line` that set it or `default`.
- The RGB camera rolls the file over past `mcap.max_size_gb`. Each segment repeats the metadata
  and re-creates every channel, so it is independently readable; segments after the first are
  named `..._partNN.mcap`.
- `log_time` and `publish_time` are equal, wall clock nanoseconds at the moment of writing.
  Message stamps inside the payloads are the pipeline stamps (double seconds converted to
  `sec`/`nsec`), which can differ from `log_time`.

## Channels

| Topic | Message encoding | Schema name | Schema encoding | Recorded |
| --- | --- | --- | --- | --- |
| `/camera/video` | `protobuf` | `foxglove.CompressedVideo` | `protobuf` | yes |
| `/camera/image` | `protobuf` | `foxglove.CompressedImage` | `protobuf` | yes (config may ignore) |
| `/camera/camera_info` | `protobuf` | `foxglove.CameraCalibration` | `protobuf` | yes |
| `/camera/frame_meta` | `json` | `auto_battlebot.FrameMeta` | `jsonschema` | yes |
| `/tf` | `protobuf` | `foxglove.FrameTransforms` | `protobuf` | yes |
| `/field_mask` | `protobuf` | `foxglove.CompressedImage` | `protobuf` | yes |
| `/field_mask/camera_info` | `protobuf` | `foxglove.CameraCalibration` | `protobuf` | yes |
| `/field_markers` | `protobuf` | `foxglove.SceneUpdate` | `protobuf` | yes |
| `/field_points` | `protobuf` | `foxglove.PointCloud` | `protobuf` | yes |
| `/hazard_markers` | `protobuf` | `foxglove.SceneUpdate` | `protobuf` | yes |
| `/robot_markers` | `protobuf` | `foxglove.SceneUpdate` | `protobuf` | yes |
| `/nav_markers` | `protobuf` | `foxglove.SceneUpdate` | `protobuf` | yes |
| `/blob_detections` | `json` | `auto_battlebot.Detections` | `jsonschema` | yes |
| `/keypoint_detections` | `json` | `auto_battlebot.Detections` | `jsonschema` | yes |
| `/blob_detections/annotations` | `protobuf` | `foxglove.ImageAnnotations` | `protobuf` | no, live only |
| `/keypoint_detections/annotations` | `protobuf` | `foxglove.ImageAnnotations` | `protobuf` | no, live only |
| `/diagnostics/<module>` | `json` | `auto_battlebot.Diagnostics` | `jsonschema` | yes |
| `/robot/esp32_diagnostics` | `json` | `auto_battlebot.Esp32Diagnostics` | `jsonschema` | yes |
| `/robot/espnow_telemetry` | `json` | `auto_battlebot.EspNowTelemetry` | `jsonschema` | yes |
| `/apriltag/robot_tags` | `json` | `auto_battlebot.AprilTagRobotTags` | `jsonschema` | yes |
| `/log` | `protobuf` | `foxglove.Log` | `protobuf` | yes |
| `/status/system` | `json` | `auto_battlebot.status.System` | `jsonschema` | yes |
| `/status/app` | `json` | `auto_battlebot.status.App` | `jsonschema` | yes |
| `/status/config` | `json` | `auto_battlebot.status.ResolvedConfig` | `jsonschema` | yes |
| `/status/sticks` | `json` | `auto_battlebot.status.Sticks` | `jsonschema` | yes |
| `/status/tracks` | `json` | `auto_battlebot.status.Tracks` | `jsonschema` | yes |
| `/status/command_ack` | `json` | `auto_battlebot.status.CommandAck` | `jsonschema` | yes |
| `/status/network` | `json` | `auto_battlebot.status.Network` | `jsonschema` | yes |
| `/command/<name>` | `json` | `auto_battlebot.command.<Name>` | `jsonschema` | yes, recorder only |

Protobuf schemas are the ones shipped with the Foxglove SDK (schema data is the serialized
`FileDescriptorSet`, exactly what `foxglove::schemas::X::schema()` in C++ and
`foxglove.schemas.X` in Python emit). The jsonschema texts live in
`include/foxglove_adapters/json_schemas.hpp` (C++) and `auto_battlebot/recording/mcap_write.py` (Python)
and must stay byte-identical.

Latched topics (the relay re-sends the last message to a newly subscribed client):
`/field_mask`, `/field_mask/camera_info`, `/field_markers`, `/field_points`, `/hazard_markers`,
`/robot_markers`, `/nav_markers`, `/status/app`, `/status/network`.

## `/status/*` and `/command/*` (JSON)

The remote UI protocol. Every topic, its message struct, and its rate is one row in
`include/remote/protocol.hpp`; the structs are in `include/remote/messages.hpp`, and each
channel's jsonschema is generated from the struct, so this doc does not repeat the fields.
`web/src/generated/protocol.ts` is generated from the same table by `remote_protocol_dump`.

| Topic | Rate | Content |
| --- | --- | --- |
| `/status/system` | 10 Hz | Camera, transmitter, loop rate, init, opponent count, autonomy, SVO and MCAP recording, temperature, compute mode, app uptime, the radio's autonomy switch and how long it has been on |
| `/status/app` | every 5 s, latched | Profiles, current profile, loop rate target and fail thresholds |
| `/status/config` | every 20 s, latched | Every config field the app parsed: `section`, `key`, `value` (TOML text), `source` (`file.toml:line` relative to `config/`, or `default`) |
| `/status/sticks` | 20 Hz | Our robot's sticks read back from the transmitter, normalized [-1, 1] |
| `/status/tracks` | 20 Hz | Field size, the field border projected into the camera image (fractions of width and height), each robot's track id, label, group, stale flag, field-frame `x`, `y`, `yaw`, and image point, and navigation's target segment |
| `/status/command_ack` | per command | `seq`, `topic`, `accepted`, `message` |
| `/status/network` | every 5 s and on change, latched | Hostname, cable and Wi-Fi addresses, Wi-Fi access |

Commands are recorded when the Runner drains them, one channel per command topic, recorder only:
the relay already has these topics as client channels. Enums are lowercase strings
(`{"action": "reboot_host"}`); optional fields are left out when empty.

`decode_command` (`include/remote/command_decode.hpp`) turns a command topic and payload into a
`RemoteCommand`. It never throws, so a bad payload from the dashboard cannot kill the remote
thread. An unknown topic logs a warning with the valid topic list. Bad JSON or a missing field logs
a warning with the schema name and the parse error. An empty payload reads as `{}`.

`CommandQueue` (`include/remote/command_queue.hpp`) carries commands from the viz sink thread to
the loop thread: the sink posts on its own thread, and the Runner drains the queue at the start of
each tick, so every command runs on the loop thread. A reboot or power-off command also invokes
the callback from `set_on_system_action` on the posting thread, so a loop thread blocked in a
camera re-open does not delay it.

## Frames

`frame_id` strings are the lower-case `FrameId` enum names: `camera`, `camera_world`,
`visual_odometry`, `field`, `our_robot_1`, `their_robot_1`, and so on. Unchanged from before.

A robot's pose is its reference point. `Keypoint::reference_offset_m` is how far that point sits ahead of the front/back keypoint midpoint, in metres along the heading. The reference point is our robot's axle, the body origin the MuJoCo plant and the RL policy use, and an opponent's footprint centre. The model that knows where its keypoints sit on the robot sets it. It is 0 for models that place the keypoints symmetrically about the reference point.

## Stamps

`Header.stamp` is a double in seconds. It maps to `Timestamp{sec, nsec}` as
`sec = floor(stamp)`, `nsec = round((stamp - sec) * 1e9)` clamped to `[0, 999999999]`.
The Python readers return `stamp_ns = sec * 1_000_000_000 + nsec`.

## `/camera/video` (protobuf)

One Annex-B H.264 access unit per frame, IDR every 30 frames, encoded with `max_b_frames = 0`.
No B-frames means decode order equals capture order, so the nth message is the nth captured
frame; `start_frame` on playback and `export_camera_transforms.py` matching dataset images by
position in the stream both depend on that.

The e-CAM25 records frames **before** rectification, so a revised lens calibration can be
applied to footage already shot; `camera_calibration` in the file metadata is the one in use, and
a `calibration_file` in the playback config overrides it. The ZED X One records the SDK's
rectified frames, and its `camera_calibration` holds the rectified intrinsics with zero
distortion. `log_time` is the capture instant the encoder carried through from the V4L2 buffer, not the
moment the packet was written.

The encoder is FFmpeg's `h264_nvenc` on the desktop and GStreamer's `nvv4l2h264enc` on the
Jetsons (`src/rgbd_camera/video_encoder.cpp` lists the order). Stock L4T FFmpeg registers
`h264_nvenc` but cannot open it without `libnvidia-encode`, which no Jetson has; every ZED Box
recording before the GStreamer backend has no `/camera/video` for that reason. With
`video_recording = true` and no encoder that opens, the app now exits at startup instead.

Seek to the last IDR at or before the target and decode forward, discarding until you reach it.
Seeking into the middle of a GOP returns smeared frames; this is the same failure
`sl::Camera::setSVOPosition` has, and it applies to any inter-coded stream.

## `/camera/frame_meta` (JSON)

```json
{"image_stamp_ns": "1788011445339499712", "video_frame_index": 1234}
```

`image_stamp_ns` is a **string** holding the decimal `uint64`. It is above 2^53, so a JSON
number would be rounded by every JavaScript consumer (Foxglove's Raw Messages panel included).
Python readers convert it with `int()`. `video_frame_index` is an integer, `-1` when video
recording is off. It is the return value of `VideoEncoder::submit()`: the position of the frame's
packet in the file (0, 1, 2... for exactly the frames that become packets), `-1` for a frame the
encoder refused, and restarted from 0 each session. Playback reads it back as that position.

Join frame_meta to `/camera/video` on `video_frame_index`, never on the timestamp. The two
channels are written by different threads: the video message's `log_time` is the capture instant
the encoder carried through, while frame_meta is logged when the publisher reaches it, so
pairing them by iteration order runs ahead by however far the encoder is behind.

Recordings written before 2026-09-09 carry `svo_frame_index` and `svo_path` instead, from when
the video lived in a separate `.svo2` file. `auto_battlebot.recording.mcap_io` reads the old
names and reports the index as `video_frame_index`, which is what it always was.

## `/blob_detections`, `/keypoint_detections` (JSON)

Same payload as before, no `std_msgs/String` wrapper:

```json
{"stamp": 1788011445.339499712, "w": 1280, "h": 720,
 "dets": [{"x1": 10.0, "y1": 20.0, "x2": 110.0, "y2": 120.0, "conf": 0.9123,
           "class_id": 0, "label": "mr_stabs_mk1", "kps": [[55.0, 70.0, 0.98]]}]}
```

`kps` is present only when the model emits keypoints. Each entry is `[x, y, confidence]` in
original-image pixels.

## `/diagnostics/<module>` (JSON)

One channel per `DiagnosticsModuleLogger` name (`runner`, `pursuit_nav`,
`opentx_transmitter`, ...). The topic is `/diagnostics/` + module name. One message per
`DiagnosticsLogger::publish()` tick that had data for that module. Payload:

```json
{"runner":     {"level": 0, "message": "", "age_ms": 3.1, "held": false, "values": {"rate": 59.7}},
 "navigation": {"level": 0, "message": "", "age_ms": 2.8, "held": false,
                "values": {"using_previous_robots": 1}},
 "tick":       {"level": 0, "message": "", "age_ms": 41.0, "held": true,
                "values": {"elapsed_ms": 12.3}}}
```

Subsections are sticky. Every message carries every subsection the module logged in the last
10 s, so a subsection logged only on change does not drop out of the messages between changes.
`held` is true when the subsection was not logged since the module's previous message: its
values, level, and message repeat an earlier one. `age_ms` is the time since it was last
logged. Readers that treat messages as samples must skip held subsections, as
`decode_diagnostic_array` does. Recordings from before 2026-10-13 have neither field and
contain only the subsections logged in that tick.

Top-level keys are the subsection names. The empty subsection (data logged straight on the
module logger) is keyed by the module name itself, which is what the old
`DiagnosticStatus.name` held for it. `level` is 0 OK, 1 WARN, 2 ERROR, 3 STALE. `values` keys
are the flattened `DiagnosticsData` keys (`/`-joined nested keys, `/N` array indexes) and the
values are typed: JSON integers for `int`, JSON numbers for `double` (NaN and infinities become
`null`), JSON strings for `std::string`. Nothing is stringified.

Foxglove plot path example: `/diagnostics/pursuit_nav.pursuit_nav.values.heading_error`.

## `/robot/esp32_diagnostics` (JSON)

Written by `Esp32DiagnosticsOpenTxTransmitter` (the `mr_stabs_mk2_sysid_zed_box` profile). One
message per 100 Hz record of the Mr Stabs Mk2 firmware's telemetry ring, which the app polls over
WiFi with `GET /telemetry` (`Esp32WifiTelemetry`). The ring holds two minutes, so a link outage
shorter than that delays rows but loses none. `log_time` is `host_receive_ns`.

Recordings made before the ring (2026-10-13) hold one message per line of the firmware's event
stream instead, every control loop in record mode, with `backfill` absent.

```json
{"host_receive_ns": 1788011445339499712, "timestamp_ms": 123457,
 "radio_connected": true, "armed": true, "a_percent": 50.0, "b_percent": 0.0,
 "button_state": false, "flip_switch": 0, "left_cmd": 48.0, "right_cmd": 52.0,
 "accel_x": 1.1, "accel_y": 0.0, "accel_z": 9.7, "is_upside_down": false,
 "loop_us": 905, "wifi_clients": 1, "orientation_x": 10.0, "orientation_y": 0.5,
 "orientation_z": -0.5, "pid_setpoint": 0.0, "pid_output": 0.0, "vbat": 15.842,
 "ibat": 3.12, "yaw_rate": 0.4, "yaw_rate_cmd": null, "seq": 88412, "backfill": true}
```

- `host_receive_ns`: app wall clock when the poll reply (or event-stream line) arrived, integer
  nanoseconds. Every row of one poll shares it. A JSON integer
  above 2^53, so a JavaScript consumer rounds it; Python reads it exactly.
- `timestamp_ms`: the robot's `millis()`. The difference to `host_receive_ns` drifts with the two
  clocks and jumps with WiFi latency; `esp32_diagnostics` diagnostics log it as `clock_offset_ms`.
- `left_cmd`, `right_cmd`: per-motor commands after the heading PID and the mixer, percent. These
  are the drivetrain's input.
- `orientation_*`: BNO055 Euler angles in degrees. `accel_*`: BNO055 acceleration.
- `vbat`: pack voltage from the INA228 in volts, `null` when the firmware predates the field or
  prints `nan` (no INA228, or a failed read).
- `ibat`: pack current from the INA228 in amps, positive while discharging, `null` under the same
  conditions as `vbat`.
- `yaw_rate`, `yaw_rate_cmd`: gyro yaw rate and the rate the yaw loop commands, deg/s clockwise.
  `yaw_rate_cmd` is `null` when the loop is not running; both are `null` on older firmware.
- `seq`: the ring's record number, +1 per 100 Hz record, starting at 0 on boot. A gap is records
  the ring overwrote before a poll reached them. On event-stream recordings it counts sent lines
  instead. `null` on older firmware.
- `backfill`: true when the row waited in the ring, either for the next poll or for the link to
  come back; only the newest row of a poll that caught up is false. A backfilled row's
  `host_receive_ns` is late by that wait, so clock fits skip it. `null` on event-stream rows.
- The motor guard fields (`motor_rise_left` through `erpm_crc_errors`) mean what they do on
  `/robot/espnow_telemetry`, and are `null` on older firmware.
- `wifi_clients` is not in the ring and reads 0.

Link health (connected, rows per second, polls and failed polls, parse errors, backfilled and
lost records, records still waiting in the ring, ring capacity, robot reboots, largest robot-clock
gap, clock offset) goes to `/diagnostics/esp32_diagnostics` once a second.

## `/robot/espnow_telemetry` (JSON)

Written by `EspNowTransmitter` (the Mr Stabs Mk2 ZED Box profiles). One message per telemetry frame
the robot sends over ESP-NOW (`publish_diag` in `firmware/mr_stabs_mk2/src/main.cpp`), at most
100 Hz. The binary layout is `TelemetryPayload` in
`firmware/shared/espnow_protocol/src/espnow_protocol.h`. `log_time` is `host_receive_ns`. See
`firmware/shared/espnow_protocol/`.

```json
{"host_receive_ns": 1788011445339499712, "timestamp_ms": 123457, "loop_us": 905,
 "a_percent": 50.0, "b_percent": 0.0, "left_cmd": 48.0, "right_cmd": 52.0,
 "accel_x": 1.1, "accel_y": 0.0, "accel_z": 9.7, "orientation_x": 10.0,
 "orientation_y": 0.5, "orientation_z": -0.5, "pid_setpoint": 0.0, "pid_output": 0.0,
 "vbat": 15.842, "ibat": 3.12, "yaw_rate": 0.4, "yaw_rate_cmd": null,
 "linear_accel_x": 0.42, "linear_accel_y": -0.05, "linear_accel_z": 0.11, "gyro_x": 1.2,
 "gyro_y": -0.8, "imu_calib_sys": 3, "imu_calib_gyro": 3, "imu_calib_accel": 1, "imu_calib_mag": 0,
 "radio_connected": true, "armed": true, "autonomy_enabled": true, "button_state": false,
 "flip_switch": 0, "is_upside_down": false, "espnow_active": false, "espnow_channel": 1,
 "espnow_age_ms": 3, "espnow_seq": 4211, "jetson_tx_us": 912034511, "espnow_lq": 100,
 "telemetry_seq": 30517, "motor_rise_left": 12.4, "motor_rise_right": 11.9,
 "motor_speed_left": 410.2, "motor_speed_right": -398.7, "motor_amps_left": 6.1,
 "motor_amps_right": 5.8, "guard_scale": 1.0, "guard_gain": 1.04, "guard_state": 1,
 "guard_sensors": 0, "stall_flags": 0, "erpm_crc_errors": 0, "round_trip_us": 6120}
```

- The fields shared with `/robot/esp32_diagnostics` mean the same thing. Every float is `null`
  when the firmware sends NaN.
- `vbat`, `ibat`: INA238 pack volts and amps, `ibat` positive while discharging. Both are `null`
  when the INA238 is absent. `yaw_rate`, `yaw_rate_cmd`: deg/s clockwise; `yaw_rate_cmd` is `null`
  when the yaw loop is not running.
- `accel_x`..`accel_z` are the BNO055's fused gravity vector, not total acceleration.
  `linear_accel_x`..`linear_accel_z` are the fused acceleration with gravity removed, m/s^2. Both
  are in the chip frame, as are `gyro_x` and `gyro_y`: the gyro rates about the chip's x and y
  axes, deg/s, right-handed. `yaw_rate` is the third axis, projected onto up.
- `imu_calib_sys`, `imu_calib_gyro`, `imu_calib_accel`, `imu_calib_mag`: the BNO055 CALIB_STAT
  levels, 0 uncalibrated to 3 fully calibrated. The robot rereads the register once a second.
  The firmware runs IMUPLUS, which leaves the magnetometer out, so `imu_calib_mag` stays 0. All
  four are 0 until the sensor first comes up.
- `autonomy_enabled`: ch5 (SA) as the robot decoded it. ESP-NOW substitutes the drive command
  only while this is true.
- `espnow_active`: ESP-NOW supplied this tick's `a_percent`/`b_percent`. Always false in Phase 1
  (shadow mode).
- `espnow_channel`: the channel the robot listens on, 0 while it sweeps.
- `espnow_age_ms`: since the robot last accepted a command frame, a dongle repeat included;
  `null` before the first.
- `espnow_seq`, `jetson_tx_us`: the last accepted command, echoed.
- `espnow_lq`: robot telemetry send ACK success over its last 32 frames, percent.
- `telemetry_seq`: +1 per telemetry frame the robot sends, starting at 0 on boot. A gap is frames
  lost between the robot and the host; a drop back toward 0 is a reboot.
- `motor_rise_*`, `motor_speed_*`, `motor_amps_*`, `guard_*`, `stall_flags`, `erpm_crc_errors`:
  the motor stall guard (`docs/plans/motor_stall_protection_plan.md`). `motor_rise_*` is the
  thermal model's winding rise above ambient, C. `motor_speed_*` is shaft speed, rad/s, signed.
  `motor_amps_*` is the estimated winding current, A. `guard_scale` is 1 while commands pass
  through and below 1 while limiting; `guard_gain` is the shunt correction on the model current.
  `guard_state` is 0 OFF, 1 NOMINAL, 2 HOT, 3 LIMITING, 4 TRIPPED (thermal trip latched until disarm). `guard_sensors` is 0 FUSED, 1 SHUNT_ONLY,
  2 ERPM_ONLY, 3 NONE. `stall_flags` bit 0 is the left motor, bit 1 the right. `erpm_crc_errors`
  sums both ESCs since boot. Recordings before 2026-10-22 lack all twelve.
- `round_trip_us`: host monotonic arrival minus `jetson_tx_us`. Each sample also holds the robot's
  wait for its next 10 ms telemetry slot and the age of a repeated command, so the per-second
  minimum is the transport estimate.

Link health (dongle open, dongle channel, ACK percent, telemetry per second, minimum round trip,
CRC errors) goes to `/diagnostics/espnow_transmitter` once a second.

## `/apriltag/robot_tags` (JSON)

Written by `AprilTagKeypointModel`, one message per camera frame, including frames with no
detection so gaps are explicit. The corners are the measurement: the video is lossy H.264, so they
let PnP be re-solved offline with a revised tag size or intrinsics. `log_time` is
`image_stamp_ns`.

```json
{"image_stamp_ns": 1788011445339499776, "frame_id": "camera",
 "camera": {"fx": 665.0, "fy": 716.0, "cx": 960.0, "cy": 600.0, "width": 1920, "height": 1200},
 "tag_size_m": 0.064, "roi": [820, 410, 240, 236],
 "detections": [{"id": 76,
                 "corners": [[900.1, 480.2], [960.3, 481.0], [959.8, 540.9], [899.7, 540.1]],
                 "decision_margin": null,
                 "solutions": [{"rvec": [3.1, 0.02, 0.1], "tvec": [-0.05, 0.02, 0.71],
                                "reprojection_error_px": 0.12},
                               {"rvec": [2.9, -0.3, 0.1], "tvec": [-0.05, 0.02, 0.71],
                                "reprojection_error_px": 0.35}]}]}
```

- `image_stamp_ns`: the frame's `Header.stamp` times 1e9, rounded. The stamp is a double, so this
  is within about 250 ns of `/camera/frame_meta`'s exact string, not equal to it; join on the
  nearest frame.
- `roi`: `[x, y, w, h]` of the window that was searched, `null` when the full frame was. A window
  that finds nothing falls back to the full frame in the same tick.
- `corners`: pixels, OpenCV aruco order, clockwise from the marker's top-left.
- `decision_margin`: always `null`. OpenCV's aruco detector does not expose one.
- `solutions`: both `SOLVEPNP_IPPE_SQUARE` solutions as `solvePnPGeneric` returns them, sorted
  by reprojection error ascending. `rvec`/`tvec` map the marker frame (origin at the tag centre, x
  right and y up in the printed image, z out of the face) into the camera frame, metres.

## `/log` (`foxglove.Log`)

Levels map spdlog `trace`/`debug` to `DEBUG`, `info` to `INFO`, `warn` to `WARNING`, `err` to
`ERROR`, `critical` to `FATAL`. `name` is the spdlog logger name, `file`/`line` the source
location when spdlog has one.

## `/field_points` (`foxglove.PointCloud`)

- `frame_id`: the field description header frame (the camera frame at field init).
- `pose`: identity.
- `point_stride`: 12.
- `fields`: `x` offset 0, `y` offset 4, `z` offset 8, all `FLOAT32`.
- `data`: little-endian packed xyz, one triple per inlier point. No color field. The 3D panel
  colors on the `z` field.

The values are the PCL cloud's own float32 coordinates. The legacy converter narrows the
`float64` marker points back to float32 and asserts every value round-trips exactly.

## `/field_markers`, `/hazard_markers`, `/robot_markers`, `/nav_markers` (`foxglove.SceneUpdate`)

One `SceneUpdate` per publish. Each old `visualization_msgs/Marker` becomes one `SceneEntity`
holding one primitive, or one `SceneEntityDeletion`. This is the rule both the C++ adapters and
the converter follow:

- `SceneEntity.id` = `<ns>/<id>` (`"robot_bounds/4"`, `"field/0"`, `"nav_target/1"`).
- `SceneEntity.timestamp` = marker header stamp. `frame_id` = marker header frame.
- `SceneEntity.lifetime` = marker lifetime when non-zero, otherwise unset.
- `SceneEntity.frame_locked` = marker `frame_locked`.
- Marker `color` (`ColorRGBA` float) becomes `Color` (double) on the primitive.
- `LINE_STRIP` and `LINE_LIST` become one `LinePrimitive` with the same `type`, `thickness` =
  `scale.x`, `scale_invariant` = false, `pose` = marker pose, `points` = marker points.
- `CUBE` becomes one `CubePrimitive` with `pose` = marker pose, `size` = marker scale.
- `SPHERE` becomes one `SpherePrimitive` with `pose` = marker pose, `size` = marker scale.
- `ARROW` in pose form (no points): `ArrowPrimitive` with `pose` = marker pose,
  `head_length` = 0.23 x `scale.x`, `shaft_length` = 0.77 x `scale.x`,
  `shaft_diameter` = `scale.y`, `head_diameter` = `scale.z`.
- `ARROW` in two-point form: `pose.position` = first point, `pose.orientation` = the rotation
  taking +x onto (second - first), `shaft_diameter` = `scale.x`, `head_diameter` = `scale.y`,
  `head_length` = `scale.z` when non-zero else 0.23 x length, `shaft_length` = length minus
  `head_length`.
- `TEXT_VIEW_FACING` becomes one `TextPrimitive` with `pose` = marker pose, `billboard` = true,
  `font_size` = `scale.z`, `scale_invariant` = false, `text` = marker text.
- `POINTS` only ever occurred in `/field_markers` (`ns == "field_inliers"`) and is not a
  SceneEntity. The converter moves it to `/field_points`. The C++ adapters never emit it.
- `action == DELETE` becomes `SceneEntityDeletion{type: MATCHING_ID, id: "<ns>/<id>"}` with the
  marker header stamp. `DELETEALL` becomes `{type: ALL}`.

Entity ids the readers rely on:

- `/field_markers`: `field/0` is the border, a `LINE_STRIP` with five points (the four corners in
  order and the first repeated). `diag_io.load_field_size` reads the first four.
- `/robot_markers`: `robot_bounds/<n>` where `<n>` is the `FrameId` enum index of the robot.
  `robot_poses/<n>`, `robot_labels/<n>`, `robot_keypoint_lines/<n>` share that index.
  `robot_uncertainty/<n>` is a `LINE_STRIP` ring of radius 2 x `position_sigma_m` around the
  robot, present only when the estimator reports a finite sigma (Kalman opponents today). When
  the heading sigma is finite, `robot_labels/<n>` text ends in ` ±<deg>°`, one sigma rounded to a
  degree.
  `robot_keypoints/<k>` counts keypoints across all robots in the message.
- `/hazard_markers`: `hazards/<i>` per keep-out disc, in `FieldDescription::hazards` order. Every
  live update starts with a deletion of type `ALL` so rings from the previous cycle clear.
- `/robot_markers` live updates likewise start with an `ALL` deletion. Legacy conversions carry
  no such deletion (the source marker arrays never cleared either).
- `/nav_markers`: `nav_pursuit_line`, `nav_target`, `nav_velocity`, `nav_angular`,
  `nav_angular_head`, each `/<i>` with a running counter, and deletions for every absent
  namespace at id 0.

## `/blob_detections/annotations`, `/keypoint_detections/annotations` (`foxglove.ImageAnnotations`)

Live only. Per detection: a `PointsAnnotation` of type `LINE_LOOP` with the four box corners,
`thickness` 2, `outline_color` from the label group; a `CircleAnnotation` per keypoint with
`diameter` 8; a `TextAnnotation` at the box top-left with `"<label> <conf:.2f>"`,
`font_size` 14. The timestamp is the detections stamp so the Image panel pairs them with
`/camera/image`. Numeric fields of a detection are never copied into `metadata`; the structured
channel carries them.

## Python readers and the AprilTag capture format

Notes on the Python code that reads these recordings, and on the separate MCAP layout the AprilTag ground-truth tools write.

### `calibration/apriltag/apriltag_mcap.py`

MCAP layout for the AprilTag ground-truth pipeline: raw camera images in, poses out (offline).

apriltag_track.py records the camera images here; analyze_apriltag_mcap.py replays them, re-runs the
AprilTag detection, and solves the field-plane pose. Recording images instead of detections means
the floor lock, the detector tuning, the intrinsics, and the yaw offset can all be corrected and re-
run offline without re-driving the robot.

A recording holds:
  - /calibration/metadata : one JSON message (intrinsics, image size, robot tag + floor board
    params).
  - /floor/image          : the frames captured during the one-time floor-board lock (board
    visible).
  - /camera/image         : the driving frames to track the robot tag in.
  - /transmitter/channels : one JSON message per driving frame, stamped with that frame's time,
    holding the transmitter stick axes the driver was commanding (read-only; present only when the
    OpenTX radio is connected).
  - /drive/command        : one JSON message per issued command (~50 Hz), stamped with its send
    time, holding the scripted excitation command (present only when run with --drive).

Images are stored either as foxglove.CompressedImage (JPEG, the default) or foxglove.RawImage
(uncompressed bgr8, lossless) depending on the capture format; both open in Foxglove. JPEG is lossy
at the marker edges the subpixel corner refinement keys on, so its stored corners differ slightly
from the live frame; raw is bit-identical to what the camera produced. MCAP chunk compression is
chosen per format: JPEG uses ZSTD (frames are already small, so it is cheap and shrinks them
further), while raw uses no compression. Compressing 6 MB raw frames per frame is the capture
bottleneck (ZSTD ~26 ms, LZ4 ~13 ms, both cap fps below 60 and barely shrink noisy sensor data), so
raw stores uncompressed (~3.6 ms/frame) and relies on the NVMe.

Timestamps are CLOCK_MONOTONIC seconds; each frame's monotonic time is the MCAP log_time, and
analysis reads the pose timestamps straight back from it. With --drive the issued commands share
that same clock (one process), so the command log and the solved poses align with no time
correction.

#### `CHANNELS_SCHEMA`

The JSON schema for the transmitter stick axes recorded per driving frame. The MCAP log_time is
the image timestamp (for alignment); sample_t is when the snapshot was actually decoded off the
radio, so analysis can tell how stale the sticks are relative to the frame. channels holds the raw
OpenTX values.

#### `COMMAND_SCHEMA`

JSON schema for the scripted excitation command issued per send (~50 Hz) in a run with --drive.
The MCAP log_time is the command's CLOCK_MONOTONIC send time. cmd_lin/cmd_ang are the normalized
[-1, 1] commands; trainer_lin/trainer_ang are the integer [-500, 500] values actually written to
the trainer link; label tags the protocol phase so the fitter can slice the run.

#### `CaptureWriter`

Writes the calibration metadata and the floor / camera image streams to an MCAP file.

Metadata is written lazily, just before the first image, so its image_width/image_height come
straight from a real frame rather than from possibly-unreliable capture properties. Call
set_metadata() once before recording any frames.

#### `write_channels`

Record the driver's stick axes for one frame, stamped (log_time) with image time
`image_t`.

`sample_t` is when the snapshot was decoded off the radio; its difference from `image_t` is
the staleness of the sticks relative to the frame. No metadata gate: channels only ever
accompany the /camera/image frames, which write the metadata first.

#### `write_command`

Record one scripted excitation command (--drive), stamped (log_time) with its send time
`t`.

Commands run on their own ~50 Hz clock independent of the camera; analysis zero-order-holds
them onto the frame times. No metadata gate: commands only ever accompany the /camera/image
frames, which write the metadata first.

#### Foxglove overlay recording

`analyze_apriltag_mcap.py` can emit a second MCAP that overlays the solved poses on the camera
frames, so the geometry can be checked by eye in Foxglove. It carries everything Foxglove needs to
project the 3D scene onto the image in the 3D panel: the camera frames (copied verbatim from the
source), a CameraCalibration so the projection is calibrated, a TF tree (field to camera fixed,
field to robot per frame), and scene geometry (a body cube and heading arrow at the robot, plus
the full trajectory line).

All frames use the OpenCV camera convention, so there is no extra optical rotation:

- `field`: the floor GridBoard frame, z = 0 on the floor. The fixed world frame.
- `camera`: the camera optical frame (x right, y down, z forward).
- `robot`: the robot tag projected onto the field plane (x, y, 0) and rotated by yaw about field z.

Everything is a Foxglove schema (foxglove.FrameTransforms, foxglove.CameraCalibration,
foxglove.SceneUpdate), the same ones the live stack records.

#### `WORLD_FRAME`

The calibration field frame (the GridBoard frame) can have +z pointing either way depending on
board orientation; when it points toward the floor the camera renders below the field in
Foxglove's z-up world. WORLD is a display-only root that flips the field (identity or 180deg about
x, chosen from the camera's z) so the rig shows upright. It does not affect the image overlay,
which depends only on the field -> camera relative transform.

#### Overlay camera calibration

The overlay writer undistorts frames before writing, and the published calibration carries zero
distortion, so Foxglove's pinhole projection of the 3D markers lands exactly on the (now
rectified) image. The ZED's full distortion model has too many coefficients for Foxglove to
apply, so the writer bakes it out instead. Keeping the same K as the new camera matrix preserves
focal length and centre.

### `recording/diag_io.py`

#### `_TRANSMITTER_COLUMNS`

`_TRANSMITTER_COLUMNS` maps each transmitter channel to a column. values/15 is the auto switch; values/0 and values/1 are the
actual transmitted drive command (post trainer-mode mix of navigation and driver sticks),
which is the true plant input.

#### `load_field_size`

Arena (width, height) in metres, recovered from the field border marker.

The C++ encoder builds the four corners as (+-size.x/2, +-size.y/2) in the
field-center frame and then rigid-transforms them, so the rectangle edge
lengths recover the true size regardless of the frame they are expressed in.

The field is published a few times (initial + re-inits) and early detections
can be bad. The arena is fixed, so the largest-area detection is the best
estimate; return that.

### `recording/foxglove_ws.py`

Subscribe-only client for the Foxglove WebSocket protocol, as ``viz_relay`` serves it.

Yields the same ``(topic, log_time_ns, MessageBytes)`` tuples as ``mcap_io.iter_messages``, so
one consumer serves a live relay and a recording. The relay advertises channels with their
schemas; protobuf schemas are registered with ``mcap_io`` so ``decode_tf_message`` and friends
work on live payloads unchanged.

There is no publish path: the client never advertises a client channel and never sends a
message frame. It sends ``subscribe`` and ``unsubscribe`` and nothing else.

Wire format (foxglove.websocket.v1, which the SDK's ``foxglove.sdk.v1`` shares):

- server text frames: JSON ``{"op": "serverInfo" | "advertise" | "unadvertise" | ...}``
- client text frames: ``{"op": "subscribe", "subscriptions": [{"id", "channelId"}]}``
- server binary frames, opcode 0x01: ``u8 opcode, u32 subscription id, u64 log time ns``
  (little endian), then the payload.

### `recording/mcap_io.py`

#### `_PROTO_DECODERS`

`_PROTO_DECODERS` holds protobuf decoders keyed by schema name, filled from the schema records of every file read
through ``iter_messages``. Foxglove schema names are stable, so a decoder learned from one
recording serves any other.

#### `decode_frame_meta`

This decodes ``/camera/frame_meta``. ``image_stamp_ns`` is a decimal string on the wire (it is above
2^53); the legacy bare-number form is accepted too.

Recordings written before video moved into the MCAP carry ``svo_frame_index`` and ``svo_path``
instead. The index reads back as ``video_frame_index``, which is what it always was: the
ordinal of the frame in whatever video stream the run was writing. The path is dropped, since
with one file there is no second file to join to.

#### `decode_diagnostic_array`

One dict per diagnostic status: ``{level, name, message, hardware_id, values}``.

For a ``/diagnostics/<module>`` JSON message ``hardware_id`` is the module (from the topic),
``name`` is the section key and ``values`` are typed. Held sections, which repeat values
logged before this message, are skipped, so each logged value is read once.

#### `iter_camera_frames`

Frames from whichever camera topic the recording carries.

A label run records ``/camera/image`` because its config clears ``ignored_topics``; an ordinary
match recording carries ``/camera/video`` and no JPEG at all. Tools that used to need
``scripts/combine_mcap_svo.py`` to reach match frames can point straight at the recording.

#### `match_stamps`

Match each reference stamp to the nearest candidate stamp within tolerance.

Playback replays of the same SVO produce identical frame stamps, but the stamp reaches
different topics through different double->ns conversions, so allow a small tolerance
(default 1 ms; frames are more than 15 ms apart). It returns {reference_stamp: candidate_stamp}.

### `recording/sysid_io.py`

Readers for the drivetrain sysid topics: robot AprilTags, robot telemetry, sticks, field pose.

Contracts (written by the C++ app, see ``docs/guides/sysid_guide.md``):

- ``/apriltag/robot_tags`` (JSON): one message per camera frame, frames without detections
  included. Each detection carries its four corners in OpenCV aruco order and every IPPE
  solution as (rvec, tvec, reprojection error). rvec/tvec map marker-frame points (origin at the
  tag center, x right, y up in the printed image, z out of the face) into the OpenCV camera
  frame (x right, y down, z forward).
- ``/robot/espnow_telemetry`` (JSON): one flat object per 10 ms telemetry frame over the ESP-NOW
  dongle. ``host_receive_ns`` is the app clock at receipt (also the MCAP log_time);
  ``timestamp_ms`` is the robot clock.
- ``/robot/esp32_diagnostics`` (JSON): the same drive fields polled over WiFi, in recordings made
  before the sysid profile moved to ESP-NOW (2026-10-25). Its ``seq`` is ESP-NOW's
  ``telemetry_seq``.
- ``/diagnostics/opentx_transmitter``, subsection ``channels``: the 16 stick channels the radio
  echoes over USB, as ``values/0`` .. ``values/15``. Logged on every channels frame since
  2026-10-13, only when they changed before that.
- ``/tf`` and ``/tf_static``: ``field -> camera_world`` and ``camera_world -> camera``. The app
  republishes both on ``/tf`` every cycle; older recordings put the first on ``/tf_static``.
  ``diag_io.load_camera_in_field`` reads only ``/tf_static`` for it and returns an empty frame on
  current recordings, and it keeps only the translation, so this module has its own loader that
  returns the full transform.

#### `ESP32_FIELDS`

Field order of one robot telemetry event, with the pandas dtype each column gets. ESP-NOW
frames carry more fields than these; the extras are dropped, and wifi_clients reads as 0.
Booleans stay bool; vbat and ibat are float with NaN where the firmware had no INA228 reading
(null, or a recording older than the field). yaw_rate, yaw_rate_cmd, and the gyro_x/gyro_y chip
rates (deg/s, ESP-NOW telemetry only) are NaN when absent, and
seq is a nullable integer, <NA> in recordings older than the field. backfill is False on
event-stream rows and in recordings older than the telemetry ring.

#### `load_transmitter_channels`

Stick channels as the radio reported them: one row per diagnostics publish tick that read
a channels frame, columns ``stamp_ns, ch0 .. ch15`` (raw CRSF units, 172..1811, center 992).

Recordings before 2026-10-13 log channels only when they change, so hold each row until the
next one. The stamp is the diagnostics publish tick, which trails the serial read by up to one
tick.

### `recording/video_import.py`

Wrap a plain video file into a recording shaped like the RGB camera's.

The camera records H.264 on ``/camera/video`` inside the same MCAP as the pipeline output
(``docs/reference/recording/foxglove_recording_format.md``). Fixed-camera footage from elsewhere, such as NHRL's
``Cage-N-Overhead-High`` feed, is the same shape of data from a different sensor, so re-wrapping it
gives ``VideoPlaybackCamera`` something to replay before the camera hardware exists.

What comes out is a recording, not a match: it carries the three camera channels and nothing else,
because everything else in a real file is pipeline *output* and replay exists to regenerate it.

The video is re-encoded rather than remuxed. A general MP4 uses B-frames and whatever GOP its
encoder chose; the camera writes ``max_b_frames = 0`` and an IDR every 30 frames, and both
``start_frame`` seeking and reading the nth message as the nth captured frame depend on that.

Two CLIs sit on this: ``scripts/video_to_mcap.py`` converts videos you already have, and
``scripts/nhrl_to_recordings.py`` runs the whole NHRL flow from fight to replayable recording set.

#### `iter_access_units`

Annex-B access units, one per frame, re-encoded to the camera's constraints.

ffmpeg writes a raw H.264 elementary stream to stdout and libavcodec's own parser splits it
back into one packet per coded picture. Hand-rolling that split from start codes gets the
SPS-before-IDR case wrong in ways that only show up as a frame count that is quietly short.

## Relay socket framing (app to `viz_relay`)

Not part of the recording, but the same encoded bytes flow here. `SOCK_STREAM` unix socket at
`/tmp/auto_battlebot_viz.sock` (override with `viz_relay --socket` and `[publisher] socket_path`).
Every frame is `[u32 len][u8 kind][body]`, little-endian, `len` counting `kind` plus body.

- kind 0 `ADVERTISE`: `u32 channel_id`, `u8 latch`, then five length-prefixed byte strings
  (`u32 len` + bytes): topic, message encoding, schema name, schema encoding, schema data.
- kind 1 `MESSAGE`: `u32 channel_id`, `u64 log_time_ns`, payload to end of frame.
- kind 2 `SUBSCRIBER_COUNT` (relay to app): `u32 channel_id`, `u32 count`.
- kind 3 `CLIENT_MESSAGE` (relay to app): length-prefixed topic, then the payload to the end of
  the frame. A Foxglove client published it; the relay advertises `clientPublish` with the
  `json` encoding and forwards every client message. The app decodes `/command/<name>` against
  `RemoteTopics::commands` in `include/remote/protocol.hpp` and refuses a payload with a missing
  field or a wrong type. `/command/reinit_field` takes `{}` (or an empty payload) and
  reinitializes the field, same as the UI tile or the init button.

A frame whose `len` exceeds `kMaxFrameBytes` (64 MiB, `include/viz/frame.hpp`) is a corrupt
stream: the relay drops the app connection and the app's sink reconnects. A single
`/field_points` message is a few MB, well under the cap.

`channel_id` is assigned by the app and is only meaningful for one socket connection. The app
re-sends every `ADVERTISE` after a reconnect. The relay keys its Foxglove channels by topic and
keeps them across app reconnects when the schema is unchanged. It serves one app at a time: a
second app connecting while one is attached is accepted and closed at once (the app's sink
retries quietly every second), so a dev replay next to a service instance cannot fight over the
relay.

## Design notes from the headers

### Frame identity

`FrameIdentity` says which camera frame a piece of pipeline output came from. `Header::stamp` is a double, so it cannot carry a nanosecond stamp losslessly; `image_stamp_ns` keeps the raw integer, which relates a video frame to the pipeline messages around it without decoding anything. `video_frame_index` is the ordinal of the frame within the video stream, so `start_frame` on playback has an unambiguous meaning across a rollover boundary. There is no path field: video lives on `/camera/video` inside the same MCAP as the pipeline output, so there is no second file to join to.

### Detections

Raw detector output (`/blob_detections`, `/keypoint_detections`) is captured in original-image pixel coordinates before the blob/keypoint collapse, so offline tools can score the model itself. The JSON payload is `{"stamp":<sec>,"w":<px>,"h":<px>,"dets":[{"x1","y1","x2","y2","conf","class_id","label","kps"}]}`.

### Behavior mode and targets

The control loop snapshot carries the behavior mode that produced the target, so the UI and MCAP recordings can tell a run-away target from an attack selector gone haywire. The target segment on the remote status topic is where navigation is steering this cycle: an opponent in ATTACK, a safe point in RUN_AWAY, or the dashboard's held point while auto tracking is off. The segment runs from where navigation placed our robot, which may be a held stale track.

### Hazards topic

Keep-out discs are published every cycle at the inflated radius the controller steers on, unlike the field border which is published once, because tracked hazards move and a replay has to be able to say what the controller knew.

### Diagnostics values

A diagnostics value stays typed: numbers stay numbers all the way to the recording, and only the UI stringifies at render time. The Foxglove backend publishes one JSON channel per module, `/diagnostics/<module>`, keyed by subsection so a Foxglove plot path cannot silently retarget when a module first logs mid-run.

Subsections are sticky. Each publish window (the span between two `clear()` calls) starts a subsection over on its first log, then merges later logs into it. A subsection not logged in the current window keeps its last values and is reported as held, with its age, until `HELD_SECTION_EXPIRY` (10 s) passes without a log. A module that logs one subsection per send and another only on change therefore publishes both in every message. A module logger has two writers (the control thread logging, and the Runner thread clearing during `DiagnosticsLogger::publish()`), so a mutex guards the sections.

### ESP32 diagnostics rows

- The CSV line the host parses (`parse_esp32_diagnostics_csv`) is the firmware's printf format in `firmware/mr_stabs_mk2/src/diagnostics_server.cpp`: the parser accepts 20, 21, 22, 24, or 37 fields and rejects every other count. 37 is the current layout: the 24-field layout, 12 motor guard columns, then `seq`. `tests/esp32_diagnostics/test_esp32_diagnostics.cpp` holds one literal per layout; change the firmware format and those literals together.
- `seq` is the firmware's row counter, +1 per row sent. A gap means rows were dropped between the robot and the host. It is empty on firmware that predates the field.
- `backfill` is set on rows read from the firmware's telemetry ring (`GET /telemetry`): true when the row arrived later than it would have live, because it waited in the ring for the poll or for the link to come back. Its `host_receive_ns` then overstates the delivery delay, so clock fits skip it. It is empty on rows from the event stream.
- `OpenTxTransmitter` with diagnostics wraps the radio side untouched. An `Esp32WifiTelemetry` worker polls the firmware's telemetry ring, and `update()` drains what arrived since the last tick into one `/robot/esp32_diagnostics` JSON message per 100 Hz record, logged at the host arrival time. Link health goes to the `esp32_diagnostics` diagnostics module once a second.

### Camera world rebase

Field filters that fit once and rebase as the camera moves share one rebase definition. `compute_field` runs at init and answers in the camera frame of that instant, which the filter pins as camera-world; every later frame rebases through the camera's motion since. Each field re-init redefines field to camera_world, and when two filters carried their own copy the sparse mask topics let Foxglove project a stale mask through the new camera pose. `scripts/fix_field_mask_frames.py` repairs recordings that hit it. On a clamped RGB camera `tf_visodom_from_camera` is identity and the rebase is a no-op.

### AprilTag recording

The AprilTag keypoint model searches a window around the last detection (the full frame when there is none or the window finds nothing), solves both IPPE poses for each robot tag, and records the raw measurement on `/apriltag/robot_tags`, frames without a detection included, so the corners can be re-solved offline. It then composes the chosen tag pose with the tag's mounting, projects the configured front and back body points into the image, and returns them labeled MR_STABS_MK2 with their height above the floor, the same shape the YOLO model produces.

### Recorder

- `McapRecorder` writes already-encoded messages with the Foxglove SDK writer. Callers open a channel once per topic and write bytes against it. The recorder is the single gate on what lands in the file: `enabled_` (toggled from the UI) and the configured ignored topics. Producers can ask `records_topic()` before doing expensive encoding.
- Every file operation runs on one writer thread. `write()` copies the message into a queue and returns, and `set_enabled()` queues the open or close. No caller waits on the disk, so a file closing, old recordings being deleted, or a slow write cannot stall the tick, the control thread, or a thread that logs. `mutex_` only guards the queue and the gate, never I/O, and no code logs while holding it. The writer thread touches `writer_`, `context_` and `channels_`, which only it uses, and takes `mutex_` only to copy shared state.
- `enabled_hint_` mirrors `enabled_` so `write()` can skip the copy without the lock while recording is off.
- Queue cap: queued bytes past which `write()` drops a message instead of growing the queue. Messages over `kLargeMessageBytes` (video, images) hit the lower cap, so a disk that falls behind costs frames before it costs status, state and log messages. At the ZED Box's 30 Mbps the lower cap is about 30 s of video.
- Rollover: when the current file reaches `max_size_gb` the runner rolls over by turning recording off and on, so the camera's encoder restarts with it: the new file opens on an IDR with frame indices from zero. Switching files under a running encoder started mid-GOP.
- Lock order: the app's logger copies every log line into the MCAP through `McapLogSink`. Toggling recording logs "Recording to ..." while the recorder's lock is held, and that line comes back into `write()`. On one thread it deadlocked the app on the ZED Box. Across threads it deadlocks when one thread logs (holding the sink's lock, waiting for the recorder's) while the runner toggles (holding the recorder's lock, logging). `TogglingWithTheMcapLogSinkDoesNotDeadlock` and `LoggingFromAnotherThreadWhileTogglingDoesNotDeadlock` cover both.
- File names: two toggles inside one second both pick a file name before the writer thread creates either, so the second cannot rely on the first existing on disk to pick a new name. `QuickTogglesEachGetTheirOwnFile` covers it.
- The queue cap counts every queued byte, status messages included, so one 1 MiB frame fewer than `kMaxQueuedLargeBytes / 1 MiB` fits. Video over `kLargeMessageBytes` stops at the lower cap and is dropped. Status and log messages fit under the higher cap and all arrive.
- File age counts from the current file's start and restarts with each file, because the web page's recording timer shows it.
- A camera that cannot be re-opened ends the run. The runner closes the recordings before that, because the camera's destructor may have to exit the process (`CameraFailureExitClosesRecordings`).
- File-level metadata sits alongside `active_profile`. `calibration_id` says which calibration rectified a recording's frames so a revised one can be applied later.
- MCAP metadata key (under the `auto_battlebot` record) holds the calibration a recording's `/camera/video` needs, as the same TOML text a `config/cameras/` file holds. With it the recording replays without a sidecar file.

### Video playback camera

`VideoPlaybackCamera` replays `/camera/video` out of a pipeline recording. It is synchronous like the SVO playback camera: `get()` decodes on the caller's thread, so every frame reaches the pipeline in order and none are skipped. Reproducible replay is what regression testing rests on. It reads three channels and ignores the rest: everything else in the file (markers, detections, diagnostics, log) is the previous run's output, which replay exists to regenerate. Reading it would be wrong, not just wasteful. MCAP indexes channels, so filtering at the reader means those messages are never deserialized. `depth` comes back empty, `tf_visodom_from_camera` is identity, and `pose_tracking_ready` is true.

A recording that carries only `/camera/image` opens fine and then produces nothing, which reads as a broken camera. The camera names the missing channel at startup so a wrong recording is not mistaken for a broken one.

### Publisher

- `AsyncPublisher` queue: a tick makes seven publish calls, so the queue holds about nine ticks.
- `AsyncPublisher` runs another publisher's calls on one worker thread, in call order, so publishing stays off the Runner's tick. The inner publisher only ever sees the worker thread. Destroying it drains the queue before joining, so queued frames still reach the sinks. Each call copies its arguments into the queue. Camera frames are shallow `cv::Mat` copies, which is safe because every camera hands out a freshly allocated frame per grab. The queue holds 64 calls (`kMaxPendingCalls`). A full queue blocks the caller instead of dropping calls, so a recording never loses a message. The tick pays for publishing only when the worker has fallen a full queue behind, and the wait is logged as `blocked_ms`.
- `PublisherInterface::flush()` returns once every earlier publish call has reached the sinks. Shutdown calls it before closing the recording, so a publisher that defers its work does not lose the last frames.
- `FoxglovePublisher` declares its async workers last, so they stop before the channels they log to are destroyed.
- The `AsyncPublisher` worker frees each call's frame when it is done with it. A pinned frame's free can be slow, so the tick's next enqueue must not wait behind it (`EnqueueDoesNotWaitForAFrameBeingFreed`).
- A full `AsyncPublisher` queue makes the caller wait for the worker instead of dropping calls, so recordings stay complete.
- The recording toggle flushes from its own thread while the tick keeps publishing. A flush that waited for an empty queue could wait as long as the tick kept up, so `flush()` waits only for the calls queued before it.
- `FoxglovePublisher` with neither sink nor recorder accepts every publish. The app runs this way when the relay is down and recording is off.
- `/camera/preview` is a downscaled JPEG for the web dashboard. Live only, never recorded.

### Status bus

`StatusBus` publishes the status topics in `RemoteTopics::status` to the relay and the MCAP recorder. Every status channel is advertised at construction, so a client sees the full list before the first message. Throttling is UI pacing on `std::chrono::steady_clock` and never feeds control, so it does not follow the playback clock. `/status/app` repeats every 5 s so a relay that connects late still gets it. `/status/network` is paced by `HostServices` (after each change and every 5 s) and `/status/battery` by `BatteryMonitor` (once a second), so the bus does not throttle either.

### Relay socket framing

- Client channels are per-connection on the relay side, so a frame from a Foxglove client carries the topic rather than a channel id.
- The default socket path is a fixed `/tmp` path rather than `$XDG_RUNTIME_DIR`, so the systemd unit for the relay and the one for the app agree without sharing environment, and so the docker playback container (which runs both) needs no extra mount.

### Recording compression

The MCAP recorder writes with no compression. The Jetson writes this at 60 Hz next to the control loop, so the CPU goes to the loop.

### JPEG preview encoding

JPEG compression costs about 10 ms of CPU on the Jetson. It runs on worker threads, and only when a client subscribed or the recorder records the topic. The loop pays one copy of the frame, or one resize for the preview.

`ImageEncoderWorker` (`include/viz/image_encoder_worker.hpp`) runs the encoding off the perception loop, modeled on `VideoEncoder`. `submit()` never blocks: the worker keeps one pending frame and a newer frame replaces it, since a preview wants the newest frame, not every frame. The thread starts on the first submit, so a topic nobody subscribes to never costs a thread.

`auto_battlebot_nvjpg_encode` (`include/viz/nvjpg_plugin.h`) takes one 4:2:0 frame in I420 layout: contiguous full-range (JFIF) Y, Cb and Cr planes, with even width and height (it returns 0 for odd ones). It points the `jpeg_until_next_call` out-parameter at the result, valid until the next call on the same encoder, and returns the size, or 0 on failure.

### NVJPG plugin

`NvjpgJpegEncoder` encodes JPEG on the Jetson's NVJPG engine through `NvJPEGEncoder` from the Jetson Multimedia API. It is compiled only when CMake finds the Multimedia API headers and L4T's `libnvjpeg` (`HAVE_NVJPG`, `AUTO_BATTLEBOT_HAVE_NVJPG`). That `libnvjpeg` is NVIDIA's libjpeg-compatible library with the NVJPG extensions, not the CUDA toolkit's GPU nvJPEG. NVJPG takes YCbCr 4:2:0, so each frame is converted from BGR on the CPU first, and odd widths and heights lose their last column or row.

- L4T's `libnvjpeg` exports the whole libjpeg API without symbol versions. Once it is in the global symbol scope it answers OpenCV's, libtiff's and gdk-pixbuf's `jpeg_*` calls with its own, larger structs, and `cv::imencode(".jpg")` and `cv::imdecode` crash (SIGBUS/SEGV).
- `NvJPEGEncoder` and `libnvjpeg` therefore live in `libauto_battlebot_nvjpg.so`, a CMake MODULE next to `libauto_battlebot.so`. `NvjpgJpegEncoder` loads it with `RTLD_LOCAL` so its symbols stay out of the global scope, and `RTLD_DEEPBIND` so its own `jpeg_*` calls reach `libnvjpeg` instead of OpenCV's libjpeg.
- `auto_battlebot_lib`, `auto_battlebot`, `viz_relay` and `auto_battlebot_test` never link `libnvjpeg` or the plugin. CTest `NoNvjpegLink` (`scripts/check_no_nvjpeg_link.sh`) reads their direct `NEEDED` entries with `readelf -d` and fails on either name.
- The ZED SDK's `libsl_zed` pulls `libnvjpeg` in transitively after `libjpeg.so.8`. That is not a direct `NEEDED` entry of our binaries and the check does not cover it.

### H.264 parameter sets

`gst_h264_backend.cpp` sets `insert-sps-pps` on the hardware encoder so SPS and PPS arrive in band ahead of every IDR. `foxglove.CompressedVideo` needs them, and they make a mid-file seek decode. `x264enc` writes SPS/PPS once, so `h264parse` runs with `config-interval = -1` to repeat them ahead of every IDR.

### RGB camera recording

- The video is recorded pre-rectification on purpose: a calibration can be revised later and re-applied to footage already shot. Replay needs the calibration, so `v4l2_rgb_camera.cpp` embeds it as MCAP metadata (`kCameraCalibrationMetadataKey`). A recording then replays without the calibration file, and a revised file can still override it through config.
- The encoder numbers the frames it takes. `frame_meta` carries that number, so it matches the video packet in whichever file the frame lands in. It is -1 when the encoder took none.
- `ZedOneRgbCamera` submits the BGRA frame, not the BGR copy, because the Jetson's `nvvidconv` takes 4-channel input directly.
- A recording started from the remote UI after launch goes through `start_encoder()`, not `initialize()`. A failure there is not fatal: the UI reports the failed toggle, and killing a running match over it is worse.

### Video playback parsing

`video_playback_camera.cpp` reads the `data` field of a `foxglove.CompressedVideo` without linking protobuf. The message has four fields: timestamp (1), frame_id (2), data (3), format (4). Pulling one length-delimited field out of a wire-format message is a dozen lines, and a protobuf dependency would be the larger thing to maintain.

Playback reads the video channel only. The stamps are already indexed, and `camera_info` is not read at all: the recording is pre-rectification, so its `camera_info` describes a rectified stream that playback regenerates from the calibration file. Everything else in the file is the previous run's output, which replay exists to regenerate.

### SVO rollover

`ZedRgbdCamera` calls `SvoRecorder::on_frame_grabbed()` for each frame. It rolls the SVO file over when it passes the size cap. The returned `FrameRef` says where the frame landed, which lets a recording be joined back to its SVO frames.

### Opponent measurement logging

`KalmanMotionEstimator` logs every opponent measurement as the estimator received it, in either mode. A replay recording can then run back through the Python mirror offline.

### viz_relay process

`viz_relay` owns the Foxglove WebSocket server. `auto_battlebot` connects over a unix socket and streams already-encoded messages. The relay re-publishes them on `ws://<host>:8765` with `foxglove::WebSocketServer`.

- Foxglove stays connected across app restarts. Channels are keyed by topic and reused when the app reconnects with the same schema, so panels never blink.
- The relay latches. The last message on a topic advertised with `latch=1` is re-sent whenever a new client subscribes. That lets Foxglove attach mid-run and still see the field border and inlier cloud, which are published once at startup.
- The relay serves the web dashboard (`web/dist`) over HTTP with Crow. Both servers live in this process because it outlives app restarts: when the app crashes, the page still loads and shows the app as down.
- Framing is in `include/viz/frame.hpp`, and the layout is under Relay socket framing above.

## Exporting camera transforms and video

### Camera transforms for the eval dataset

`training/model_eval/export_camera_transforms.py` reads each sub dataset's `data.yaml` for its `source_mcap`. It writes next to the existing `images/` and `labels/`, which it does not modify:

```text
<sub-dataset>/camera_info.json                     intrinsics, constant over the recording
<sub-dataset>/camera_transforms/<stamp_ns>.json    one per image
```

```bash
python training/model_eval/export_camera_transforms.py [--dataset-dir <dir>] [--dry-run]
```

The transform is `tf_field_from_camera`: multiply a point in the camera frame by it to get that point in the field frame. It composes two recorded transforms:

```text
tf_field_from_camera = tf_field_from_cameraworld @ tf_cameraworld_from_camera
```

Both edges come from `/tf`, published once per frame the perception loop processed. `field -> camera_world` only changes on a field re-init but is restamped every cycle, so the applicable one is the latest at or before the image. Recordings made before that edge moved carry it once on `/tf_static`. The exporter reads both.

Frame association:

- Dataset images are named by the `/camera/image` header stamp, which the SVO recorder wrote. `/tf` and `/camera/camera_info` carry the perception pipeline's own stamps.
- A pipeline stamp lands about 0.4 to 0.9 of the way through the SVO frame interval it belongs to. This is a stable sub-frame capture lag, not clock skew. It never crosses an interval boundary, so "the pipeline stamp inside [svo_stamp, next_svo_stamp)" identifies the frame exactly.
- Each image and each `/tf` sample resolves to a position in the `/camera/image` stream that way, and the two are matched on that position. It is a position within this MCAP, not a `.svo2` frame index. `ZED_SVO_Editor` drops a few frames per file, so the two differ by a small offset. The offset is the same on both sides, so it cancels.
- The pipeline ran at about 25 fps against the camera's 30 fps, so it skipped frames. For a skipped frame no live transform was computed, and the result is interpolated from the neighbouring processed frames.

Every image gets a file. Its `method` field says which case applies, so a consumer can filter on it:

| `method` | Meaning |
| --- | --- |
| `exact` | One processed frame owns this image's frame index |
| `interpolated` | The frame was skipped; slerp/lerp between the nearest processed frames |
| `ambiguous` | Two processed frames claim this index (only the 05-01 recording) |
| `unavailable` | No field frame yet, or no processed frame to interpolate from |

### Left-eye JPEGs from an SVO2

`iter_left_jpegs()` in `auto_battlebot/recording/svo2.py` decodes the concatenated access units with ffmpeg and re-encodes the cropped left half to MJPEG. Output frames come out one for one with the messages fed in, so the Nth JPEG is SVO frame N (`test_left_jpegs_one_per_message`).

- `-r` is passed because a raw elementary stream carries no timing. Without an input rate, the mjpeg encoder in the 22.04 ffmpeg (4.4) rejected the second frame with "Invalid pts (0) <= last (0)" and the export stopped after a handful of frames. ffmpeg 6.1 does not reproduce that, so the test does not pin it.
- ZED records intra-refresh H.264 with no IDR frames, so the first frames of a recording decode partially until the refresh completes. They are emitted anyway to keep frame indices aligned with the SVO.
- Close the iterator (`contextlib.closing`) if you stop early, so ffmpeg is torn down instead of blocking on a full pipe.
- `_iter_jpegs` splits the MJPEG stream on the SOI marker (`FFD8FF`). Entropy-coded data byte-stuffs 0xFF as `FF00`, so the marker only appears at the start of an image. An image carrying an EXIF thumbnail would break this, and ffmpeg's MJPEG output carries none (`test_iter_jpegs_splits_on_soi_only`).

### Extracting video without the ZED SDK

`training/svo_rgb_extract.py` extracts left or right RGB video from a `.svo2` recording with no `pyzed` or CUDA. A `.svo2` is an MCAP container holding H.264 side-by-side stereo frames on the `<camera>/side_by_side` topic. Each message is one access unit: an 8-byte length prefix followed by an Annex-B H.264 stream. The script reads the frames with the `mcap` library, crops the requested eye, and pipes the raw H.264 into ffmpeg to write an MP4.

Depth is not available through this path. The ZED SDK computes depth from the stereo pair on the device and does not store it in the recording. Use the SDK (`svo_export.py`) for the depth channel.

```bash
python training/svo_rgb_extract.py recording.svo2                  # left eye
python training/svo_rgb_extract.py recording.svo2 --channel 1      # right eye
python training/svo_rgb_extract.py a.svo2 b.svo2 --output out.mp4
```

`training/video_mcap_extract.py` is the counterpart for a pipeline recording made with the RGB camera. That recording carries one Annex-B H.264 access unit per frame on `/camera/video`, in the same file as the pipeline output, so there is no second file to find and no join to do. Frames come out pre-rectification, which is what lets a revised lens calibration apply to footage already shot.

The MP4 path pipes the raw access units into ffmpeg without decoding, which is fast and lossless. The frames path decodes and writes PNGs for labelling. `to_frames()` names each file `<index>_<stamp_ns>.png`: the index first so filenames sort in capture order, and the stamp so a frame can be tied back to the pipeline messages around it.

```bash
venv/bin/python training/video_mcap_extract.py recording.mcap
venv/bin/python training/video_mcap_extract.py recording.mcap --frames out_dir/
venv/bin/python training/video_mcap_extract.py recording.mcap --frames out_dir/ --stride 10
```

## Recording readers and exporters

### `playground/calibration/check_sysid_recordings.py`

`stick_at`:

Linear and angular stick fraction held at each stamp (channels log only on change), with each channel's resting offset removed.

### `playground/mcap_auto_percentage.py`

`mcap_auto_percentage.py`:

Report what percentage of a match recording the robot spent in autonomous mode.

The match window runs from the first time values/15 == 1024 (auto engaged) to the last auto->manual transition. Auto mode is detected from the diagnostics: opentx_transmitter / channels / values/15 == 1024.

### `playground/opponent_prediction/plot_dropout_samples.py`

- `replay_session`: The diagnostics re-send a channel's last value with held = true; only fresh ones count.

### `playground/opponent_prediction/render_dropout_videos.py`

`replay_clock_offset_ns`:

Rebased pipeline stamp minus the SVO image stamp.

`/keypoint_detections` is logged at its own (rebased) stamp and each tick's `/camera/frame_meta` follows it by the camera grab time, so a detection pairs with the next frame_meta.

### `playground/opponent_prediction/run_replays.py`

- `run`: The recorder splits video into a second file; keep the main one (the diagnostics).

### `playground/radio/channel_streaming.py`

`channel_streaming.py`:

OpenTX Channel Data Streaming Test Script This script connects to the OpenTX radio via USB serial and parses the custom channel data packets along with CRSF telemetry data.

```
Channel packet format:
- Sync: 0xA5
- Length: packet length excluding sync byte
- Data: channel values as 16-bit signed integers (little-endian)
- Checksum: XOR of all bytes excluding sync
```

```
Usage:
    python3 test_channel_streaming.py [--port /dev/ttyACM0] [--baudrate 115200]
```

### `playground/radio/crsf/crsf_link_statistics.py`

- `CrsfLinkStatistics.up_rssi_ant1`: Uplink RSSI Antenna 1 (dBm * -1) Uplink RSSI Antenna 2 (dBm * -1) Uplink Package success rate / Link quality (%) Uplink SNR (dB) number of currently best antenna enum {4fps = 0 , 50fps, 150fps} enum {0mW = 0, 10mW, 25mW, 100mW, 500mW, 1000mW, 2000mW, 250mW, 50mW} Downlink RSSI (dBm * -1) Downlink Package success rate / Link quality (%) Downlink SNR (dB)

### `simulation/protocol.py`

- `GT_POSE_SIZE`: Ground truth poses appended after depth buffer: uint32 num_robots, then num_robots * (x, y, yaw) as float64. First entry = our robot, rest = opponents in config order. x, y, yaw per robot (3 x float64 = 24 bytes)
- `MEAS_HEADER_SIZE`: After the ground truth: what perception delivered this tick, from the sensing model (auto_battlebot/perception/sensing_model.py). One header, then one pose per ground-truth slot in the same order. Flags travel as 0.0 / 1.0 so the block stays all float64 like the rest. delivered, capture_time (sim seconds)

## Python diagnostics loader (`diag_io`)

`auto_battlebot/recording/diag_io.py` turns `/diagnostics` and the marker topics into tick tables.

- Navigation implementations log the same tick schema (`our_x`, `target_x`, `distance`, `angle_error_deg`, `facing_target`, `linear_x`, `angular_z`, ...) under their own `hardware_id`. Only one navigation runs per fight, so any id in `NAV_HW_IDS` is the navigation stage for that recording. A navigation implementation with a different schema (`rl_policy_nav`) stays out of the set.
- `FRAME_IDS` follows the `FrameId` enum order in `include/enums/frame_id.hpp`. Robot markers set `marker.id` to the enum index of the frame, so the index maps back to the name there.
- `load_robot_tracks` reads the CUBE body markers (`ns == "robot_bounds"`). It does not carry `is_stale`, because markers do not encode it.
- Transmitter channels are logged only on change. They are latched physical state, so the loader forward-fills them. Defaulting the gaps would invent auto<->manual transitions and zero-command ticks.
- `_merge_status` indexes its `defaultdict` only where a status contributes a column. Touching it unconditionally would mint an all-NaN row for every message that carries nothing the loader wants.

## Tooling notes

### MCAP reader

`CMakeLists.txt` fetches mcap because the Foxglove SDK writes MCAP but does not read it, and `VideoPlaybackCamera` has to read `/camera/video` back out of a recording.

### Relay design

`viz_relay` owns the Foxglove WebSocket server (`ws://0.0.0.0:8765`) and stays up across app restarts, so Foxglove keeps its connection while `auto_battlebot` restarts. The app never waits on the relay: with no relay it still drives and records, and it reconnects on its own. The relay links only the Foxglove SDK and CLI11, never `auto_battlebot_lib`, so it starts without the ZED SDK.

`scripts/run_viz_relay.sh` starts `viz_relay` in the background if none is serving the default socket. It forwards its arguments to `viz_relay` (for example `--port 8766`). Set `AUTO_BATTLEBOT_BUILD_DIR` to pick a build tree other than `build/`.

### App-side sink (`VizSink`)

`VizSink` (`include/viz/viz_sink.hpp`) is the app's client of the relay socket.

- `publish()` never blocks and the sink never fails startup. It copies the encoded frame into a bounded queue and returns.
- One background thread owns the socket. It connects (reconnecting on a one second timer), replays every advertise after a reconnect, drains the queues, and reads subscriber counts and client messages coming back from the relay.
- With no relay, queued messages are dropped and `dropped_messages()` counts them.
- The socket is non-blocking and the thread polls it for reading and writing together, so a relay that stops reading cannot hold up commands coming the other way, or the destructor. A frame that makes no progress for 2 s (`kSendStallTimeout`) drops the connection and reconnects.
- Image, video and point cloud channels go in a bulk queue. Everything else (status, logs, scene updates) goes in a priority queue that is sent first, so status never waits behind seconds of camera frames. Each channel stays in one queue, so its own messages keep their order.
- A frozen relay's listen backlog still completes `connect()`, and the socket buffers the first sends. That is not a relay, so the sink logs that it connected only once the relay reads (`VizSinkTest.ConnectedIsLoggedOnlyOnceTheRelayReads`).
- Client messages a Foxglove client publishes (for example `{}` on `/command/reinit_field`) reach the client message handler on the sink thread.

The relay unit owns the WebSocket server and restarts on its own. The app unit only orders `After=` it and never waits on it.

### Diagnostics stage names

The diagnostics backend sets `name = hardware_id` when a message has no subsection. `scripts/mcap_latency_report.py` builds stage names as `hw_id` in that case, and `hw_id.name` otherwise, to avoid `x.x` stages.

### Latency report startup trim

`scripts/mcap_latency_report.py` trims the startup ticks up to and including the first field initialization sample by default. The one-shot field computation costs hundreds of ms and skews every summary stat. `--include-field-init` reports the whole recording.

### Browser decode of `/camera/video` and images

`web/src/lib/image.ts` decodes `foxglove.CompressedImage` and `foxglove.CompressedVideo`. The relay advertises protobuf channels with a base64 `FileDescriptorSet` as the schema. The page builds the message type from that at runtime, the same way Foxglove Studio does, so it never ships its own copy of the `.proto` files.

### Autonomy seconds

`/status/system` publishes at 10 Hz, so `scripts/download_recordings.sh` computes autonomous seconds as messages with `autonomy_switch_on` divided by 10.

### `scripts/combine_mcap_svo.py`

Combines an MCAP recording with the SVO2 files it references (in `/log` or `/camera/frame_meta`). For each SVO2 it writes `<input_mcap_stem>__<svo_stem>.mcap` with one cropped left-eye image per SVO frame, decoded out of the `.svo2` (itself an MCAP container of H.264 frames), plus the slice of the original MCAP messages whose `log_time` falls inside that SVO's range (intersect mode). It needs `ffmpeg` on `PATH` and the Python `mcap` package. It uses neither the ZED SDK nor `pyzed`.

- The SVO stores a horizontally concatenated `[left | right]` H.264 frame. The script decodes it, crops to the left half, and publishes it as `foxglove.CompressedImage` on `/camera/image`, stamped with the frame's own time.
- An input that does not read cleanly is repaired first: every record that still parses is copied into `<output dir>/recovered/<same name>`, and the run continues from that copy. This covers the input MCAP and the `.svo2` files it references, which are MCAPs too, so a killed recorder or a Jetson that lost power, which tears off the recording and the SVO it was filling at the same moment, still yields everything written before the tear.
- Damage detection runs two checks because they fail on different damage. The summary lookup needs the footer at the end of the file, which a recording that never finalized does not have. The record scan reads the body from the front and stops wherever the bytes stop making sense, which catches a torn final chunk and mid-file corruption that an intact footer would hide. The scan costs 0.06 s for a 36 MB recording.
- The recovered copy keeps the source name, so every output named after the file, combined MCAPs and the `svo_file` field on `/camera/svo_frame` alike, comes out the same as for an undamaged input.
- The SVO's IMU data is dropped. It lives on the `sensors` and `integrated_sensors` topics as an opaque base64 blob that only the SDK decodes, and nothing downstream of the combined MCAPs reads it: the model_eval and model_compare tools use `/camera/image` and `/camera/camera_info` only. Use `ZED_SVO_Editor -export-to-mcap` if you need those channels.
- `/camera/svo_frame` carries one message per frame, alongside the image it describes, saying where that image came from. It is the export-side companion to `/camera/frame_meta`, which the pipeline writes with the SVO frame index straight from the SDK. See `docs/experiments/platform/2026-08-28_svo_pipeline_stamp_offset.md` for when stamps alone identify a frame.
- The log_time to header.stamp mapping corrects the roughly 50 ms lag between capture and the recorder write in the C++ publisher. The original MCAP's log_times are wall-clock-stamped, and the SVO MCAP uses the camera-frame-stamped timeline.

### `scripts/convert_ros1_mcap.py`

Converts a legacy ROS 1 recording into this layout. It never writes in place and never deletes anything. With a directory pair, every `*.mcap` under the input tree is converted to the mirrored path under the output tree. Existing outputs are skipped unless `--overwrite` is given. Files that are not `profile: ros1` are reported and skipped.

Payload bytes that are already the right thing (JPEGs, the detections JSON) pass through untouched. `log_time` and `publish_time` are preserved, and the `auto_battlebot` metadata record (with `active_profile`) is copied. Four topics change shape, as the C++ stack's own output did:

- `/field_markers` splits into the border `SceneUpdate` and a `/field_points` float32 `PointCloud`. Every coordinate is asserted to round-trip through float32 unchanged.
- `/diagnostics` splits into one JSON channel per module with typed values, using the same string-to-number rule `diag_io` applied to legacy recordings.
- `/camera/frame_meta` re-emits `image_stamp_ns` as a decimal string (asserted equal).
- `/blob_detections` and `/keypoint_detections` lose the `std_msgs/String` wrapper.

`--check` re-reads input and output and compares per-topic message counts (accounting for the splits) and first and last log times.

Chunk compression is copied from the source: the C++ recorder writes none, the JPEG-heavy Python-written captures use zstd chunks, and rewriting those uncompressed would grow them. Channels that are already Foxglove-readable pass through unchanged, because the calibration recordings mix JSON channels with ROS 1 images.

The ROS 1 wire format is little-endian. A header is `uint32 seq`, `uint32 stamp_secs`, `uint32 stamp_nsecs`, `string frame_id`. A string is `uint32 length` plus raw bytes, with no null terminator. This script is the only consumer left of the pre-migration format. The readers in `auto_battlebot` refuse legacy input.
