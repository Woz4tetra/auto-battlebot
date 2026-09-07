# Recording format: Foxglove schemas

Every MCAP the C++ app records, every MCAP the Python tools write, and every converted legacy
recording follows this layout. The C++ publisher (`src/foxglove_adapters/`), the Python readers
(`auto_battlebot/mcap_io.py`, `auto_battlebot/diag_io.py`), the Python writers
(`auto_battlebot/mcap_write.py`) and the legacy converter (`scripts/convert_ros1_mcap.py`) are
all written against this document. Change it here first.

## File level

- MCAP profile string is empty. The C++ recorder writes uncompressed chunks (it records on the
  Jetson at 60 Hz; zstd on the hot path is not free). Python-written files (calibration
  captures, `combine_mcap_svo.py` outputs) use zstd chunks; readers do not care.
- `combine_mcap_svo.py` outputs add one JSON channel, `/camera/svo_frame`
  (`auto_battlebot.SvoFrame`), mapping each image to its SVO frame index.
- One metadata record named `auto_battlebot` with key `active_profile` holding the config
  profile id (or empty when the app was run with an explicit `-c`).
- `log_time` and `publish_time` are equal, wall clock nanoseconds at the moment of writing.
  Message stamps inside the payloads are the pipeline stamps (double seconds converted to
  `sec`/`nsec`), which can differ from `log_time`.

## Channels

| Topic | Message encoding | Schema name | Schema encoding | Recorded |
| --- | --- | --- | --- | --- |
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
| `/log` | `protobuf` | `foxglove.Log` | `protobuf` | yes |

Protobuf schemas are the ones shipped with the Foxglove SDK (schema data is the serialized
`FileDescriptorSet`, exactly what `foxglove::schemas::X::schema()` in C++ and
`foxglove.schemas.X` in Python emit). The jsonschema texts live in
`include/foxglove_adapters/json_schemas.hpp` (C++) and `auto_battlebot/mcap_write.py` (Python)
and must stay byte-identical.

Latched topics (the relay re-sends the last message to a newly subscribed client):
`/field_mask`, `/field_mask/camera_info`, `/field_markers`, `/field_points`, `/hazard_markers`,
`/robot_markers`, `/nav_markers`.

## Frames

`frame_id` strings are the lower-case `FrameId` enum names: `camera`, `camera_world`,
`visual_odometry`, `field`, `our_robot_1`, `their_robot_1`, and so on. Unchanged from before.

## Stamps

`Header.stamp` is a double in seconds. It maps to `Timestamp{sec, nsec}` as
`sec = floor(stamp)`, `nsec = round((stamp - sec) * 1e9)` clamped to `[0, 999999999]`.
The Python readers return `stamp_ns = sec * 1_000_000_000 + nsec`.

## `/camera/frame_meta` (JSON)

```json
{"image_stamp_ns": "1788011445339499712", "svo_frame_index": 1234, "svo_path": "data/svo/x.svo2"}
```

`image_stamp_ns` is a **string** holding the decimal `uint64`. It is above 2^53, so a JSON
number would be rounded by every JavaScript consumer (Foxglove's Raw Messages panel included).
Python readers convert it with `int()`. `svo_frame_index` is an integer, `-1` when SVO
recording is off. `svo_path` is a string, empty when SVO recording is off.

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
{"runner":     {"level": 0, "message": "", "values": {"rate": 59.7}},
 "navigation": {"level": 0, "message": "", "values": {"using_previous_robots": 1}},
 "tick":       {"level": 0, "message": "", "values": {"elapsed_ms": 12.3}}}
```

Top-level keys are the subsection names. The empty subsection (data logged straight on the
module logger) is keyed by the module name itself, which is what the old
`DiagnosticStatus.name` held for it. `level` is 0 OK, 1 WARN, 2 ERROR, 3 STALE. `values` keys
are the flattened `DiagnosticsData` keys (`/`-joined nested keys, `/N` array indexes) and the
values are typed: JSON integers for `int`, JSON numbers for `double` (NaN and infinities become
`null`), JSON strings for `std::string`. Nothing is stringified.

Foxglove plot path example: `/diagnostics/pursuit_nav.pursuit_nav.values.heading_error`.

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

## Relay socket framing (app to `viz_relay`)

Not part of the recording, but the same encoded bytes flow here. `SOCK_STREAM` unix socket at
`/tmp/auto_battlebot_viz.sock` (override with `viz_relay --socket` and `[publisher] socket_path`).
Every frame is `[u32 len][u8 kind][body]`, little-endian, `len` counting `kind` plus body.

- kind 0 `ADVERTISE`: `u32 channel_id`, `u8 latch`, then five length-prefixed byte strings
  (`u32 len` + bytes): topic, message encoding, schema name, schema encoding, schema data.
- kind 1 `MESSAGE`: `u32 channel_id`, `u64 log_time_ns`, payload to end of frame.
- kind 2 `SUBSCRIBER_COUNT` (relay to app): `u32 channel_id`, `u32 count`.

`channel_id` is assigned by the app and is only meaningful for one socket connection. The app
re-sends every `ADVERTISE` after a reconnect. The relay keys its Foxglove channels by topic and
keeps them across app reconnects when the schema is unchanged. It serves one app at a time: a
second app connecting while one is attached is accepted and closed at once (the app's sink
retries quietly every second), so a dev replay next to a service instance cannot fight over the
relay.
