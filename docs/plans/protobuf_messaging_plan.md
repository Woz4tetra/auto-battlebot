# Protobuf messaging plan

Goal: every message that crosses a process, language, or device boundary is defined once, in a
`.proto` file under `proto/`, and every producer and consumer uses code generated from it. That
covers the JSON recording topics, the remote UI's status and command topics, the diagnostics
topics, the ESP-NOW link between the Jetson, the dongle, and the robot, the robot's WiFi
telemetry batch, the lockstep sim protocol, and the AprilTag calibration tool's topics.

Today the same message is written out by hand in up to three places. `EspNowTelemetry` is a
packed struct in `firmware/shared/espnow_protocol/src/espnow_protocol.h`, a hand-built JSON
string in `src/transmitter/espnow_transmitter.cpp:61-117`, and a jsonschema text kept
byte-identical in `include/foxglove_adapters/json_schemas.hpp` and
`auto_battlebot/recording/mcap_write.py`. The sim protocol is two hand-synced struct layouts
(`simulation/protocol.py`, `src/simulation/sim_connection.cpp:23-37`). Changing one field means
editing every copy and hoping a parity test covers it.

## Status, 2026-10-17

Plan only. Nothing is implemented.

## Scope

| Path | In scope | Reason |
| --- | --- | --- |
| JSON recording topics: `/camera/frame_meta`, `/blob_detections`, `/keypoint_detections`, `/apriltag/robot_tags` | yes | Hand-built JSON in C++, schema texts duplicated |
| `/diagnostics/<module>` | yes, behind gate D1 | Free-form tree; Foxglove plot paths may get worse |
| `/status/*`, `/command/*` | yes | Already single-source (`AB_JSON_MESSAGE`), but only in C++; the web gets generated TS through a dump binary |
| `/robot/espnow_telemetry`, `/robot/esp32_diagnostics` | yes | Three hand copies of one message |
| ESP-NOW air and serial payloads: Command, Telemetry, TunableSet, LinkStats | yes | Packed structs, memcpy decode |
| WiFi `GET /telemetry` batch body | yes | Same Telemetry record as ESP-NOW |
| ESP-NOW OTA messages (`MSG_OTA_*`) | no, frozen | The chunk has 1 byte of margin under 250, and OTA must keep working against robots still on the old firmware |
| Serial framing (`[7B B0][type][len][payload][crc16]`) and air header (`BWB\x01` + type) | no | Transport framing; protobuf has no framing or type tag of its own |
| Lockstep sim TCP protocol | yes | Two hand-synced struct layouts |
| Calibration tool topics (`/calibration/metadata`, `/transmitter/channels`, `/drive/command`) | yes | Python-only JSON with inline jsonschema dicts |
| `viz` unix socket framing (`include/viz/frame.hpp`) | no | Container for already-encoded messages, like MCAP itself |
| Foxglove SDK topics (`foxglove.*`) | no | Already protobuf |
| CRSF, BLHeli MSP | no | External standard protocols |
| Firmware web UI (`/events` SSE CSV, `/status`, `/wifi` JSON) | no | Browser-only, deleted by ESP-NOW Phase 2 (`espnow_link_plan.md`) |
| `velocity_jig` ASCII console protocol | no | Standalone tool, human-typed commands, never reaches the app |
| `viz_relay` `/healthz` JSON | no | One-field HTTP probe for the kiosk |
| RTNeural model JSON (`src/rl_policy/runtimes/rtneural_runtime.cpp`) | no | A file format for weights, not a message |

## What exists

- **C++ JSON writers**: all hand-built `std::string` through `OutputChannel::log`
  (`include/publisher/output_channel.hpp:40`). They are `to_frame_meta_json`
  (`src/foxglove_adapters/camera_info.cpp:43`), `to_detections_json`
  (`src/foxglove_adapters/detections.cpp:10`), `diagnostics_module_json`
  (`src/diagnostics_logger/foxglove_diagnostics_backend.cpp:42`), `to_espnow_telemetry_json`
  (`src/transmitter/espnow_transmitter.cpp:61`), `to_esp32_diagnostics_json`
  (`src/esp32_diagnostics/esp32_diagnostics_event.cpp:147`), and
  `AprilTagKeypointModel::to_json` (`src/keypoint_model/apriltag_keypoint_model.cpp:443`).
- **C++ JSON reader**: `video_playback_camera.cpp:207-215` string-scans `/camera/frame_meta`.
  The same file hand-decodes protobuf `CompressedVideo` to avoid linking protobuf (`:36`).
- **Remote protocol**: structs with `AB_JSON_MESSAGE` (`include/remote/messages.hpp`), topic table
  in `include/remote/protocol.hpp:45-66` (8 status, 12 command), encode/decode/schema in
  `include/remote/json_message.hpp`. `tools/remote_protocol_dump.cpp` feeds
  `web/scripts/gen-protocol.mjs`, which writes `web/src/generated/protocol.ts`.
- **Python**: almost every reader goes through `mcap_io.iter_messages` (`mcap_io.py:133`), then
  either a typed decoder (`decode_frame_meta :225`, `decode_diagnostic_array :247`,
  `decode_detections :552`, `sysid_io.parse_robot_tags :162`, `sysid_io.load_esp32_diagnostics
  :239`) or a raw `decode_json(data)[...]`. About 20 raw callers exist. `mcap_io._proto`
  (`:101`) already decodes protobuf from the file's embedded schema through `mcap_protobuf`.
- **Web**: `@foxglove/ws-protocol`, JSON decode in `web/src/lib/connection.ts:215`. `protobufjs`
  already decodes `CompressedImage` and `CompressedVideo` from runtime descriptors
  (`web/src/lib/image.ts:7-59`).
- **viz_relay**: forwards bytes and never parses payloads, but advertises
  `supported_encodings = {"json"}` for client publishing (`src/viz_relay/main.cpp:247`).
- **Firmware**: packed structs in `espnow_protocol.h:46-143` with size `static_assert`s. The host
  compiles that header (`CMakeLists.txt:309-310`) and memcpys payloads after an exact length check
  (`espnow_transmitter.cpp:177-197`, `esp32_telemetry_batch.cpp:14-68`).
- **Dependencies**: no libprotobuf, protoc, or nanopb anywhere. The Foxglove C++ SDK encodes its
  own types in Rust and ships pre-serialized descriptors, so it brings no protobuf runtime. The
  venv has Python `protobuf 7.34.1` and `mcap-protobuf-support`, both transitive and unpinned.

## Decisions

### 1. One tree of `.proto` files

```
proto/auto_battlebot/msg/
  common.proto        ImagePoint, FieldPoint, enums shared by several files
  camera.proto        CameraSettings, CameraSettingRange, CameraSettingLimits, AntiBanding
  recording.proto     FrameMeta, Detections, AprilTagRobotTags
  diagnostics.proto   Diagnostics
  telemetry.proto     RobotTelemetry (host wrapper around link.Telemetry)
  status.proto        package auto_battlebot.msg.status
  command.proto       package auto_battlebot.msg.command
  sim.proto           package auto_battlebot.msg.sim
  calibration.proto   package auto_battlebot.msg.calibration
  link/link.proto     package auto_battlebot.msg.link (firmware, nanopb)
  link/link.options   nanopb sizes
```

The proto package is `auto_battlebot.msg`, not `auto_battlebot`. Generated C++ lands in
`namespace auto_battlebot::msg`. Without that, generated `CameraSettings`, `BehaviorMode`, and
`SystemAction` collide with the existing `auto_battlebot::CameraSettings` struct and the
`include/enums/` enum classes.

The schema names change with the package: `auto_battlebot.FrameMeta` becomes
`auto_battlebot.msg.FrameMeta`, and `auto_battlebot.status.System` becomes
`auto_battlebot.msg.status.System`. The message encoding changes anyway, so every reader must
tell old files from new ones by `message_encoding` already. Readers that match on schema name
accept both.

### 2. Toolchain

| Consumer | Library | Codegen | Generated code |
| --- | --- | --- | --- |
| C++ app, relay, tests | libprotobuf, release pinned in `.protobuf-version` | `protoc` from the same build, CMake `protobuf_generate()` | build tree, not committed |
| Python | `protobuf` runtime pinned in `pyproject.toml` to the same release | same `protoc`, `--python_out --pyi_out` | committed under `auto_battlebot/msg/`, regenerated by `scripts/lint` |
| Web | `@bufbuild/protobuf` (protobuf-es v2) | `protoc-gen-es`, npm devDependency | committed under `web/src/generated/msg/`, regenerated by `scripts/lint` |
| Firmware (robot, dongle, native tests) | nanopb, exact version pinned in `lib_deps` | nanopb's PlatformIO integration (`custom_nanopb_protos`) | build tree |
| Schema checks | `buf` CLI, pinned | `buf lint`, `buf format`, `buf breaking --against .git#branch=main` | none |

- **libprotobuf release.** Pick the release that matches the venv's Python runtime: 7.34.1 is the
  Python side of protobuf release 34.1. Gate T1 confirms that pairing and that 7.34 installs on
  Python 3.10 (JetPack 6). If it doesn't, pin the newest release that supports 3.10 on every box.
- **Install the C++ side the way the LLVM pin works.** `install/install_protobuf.sh` builds
  protobuf plus its bundled abseil from source into `/opt/protobuf-<version>` once per box, and
  every platform install calls it. CMake finds it with `find_package(Protobuf CONFIG)` and fails
  configure with the install command when it is missing.
  - Rejected, distro packages: 3.12 on 22.04 and 3.21 on 24.04 generate different code, the same
    problem `.llvm-version` exists to solve.
  - Rejected, FetchContent: adds protobuf plus abseil to every `clean_build.sh`, and clean builds
    are routine here.
- **Python generated code is committed** because megamind and the other Python-only boxes have no
  C++ build. `scripts/lint` regenerates it and fails when the committed copy is stale, the same
  way it handles `protocol.ts`. ruff skips `*_pb2.py`; mypy reads the `.pyi`.
- **Rejected: nanopb on the host too.** Variable-length fields (Tracks, Diagnostics, Detections)
  would need fixed maximum counts or callbacks, and nanopb gives no descriptor for the MCAP
  schema.
- **Rejected: hand-written encoders**, like the `CompressedVideo` parser. That puts back the
  duplication this plan removes.

### 3. MCAP schema data

Each protobuf channel's schema is `encoding = protobuf`, `data` = a serialized
`FileDescriptorSet` holding the message's file and its transitive imports. That is the same thing
the Foxglove SDK emits for its own types.

- C++ builds it once per channel from `T::descriptor()->file()`.
- Python builds it from `<module>_pb2.DESCRIPTOR`.
- A test checks that both produce the same set for every message.

Foxglove, `mcap_protobuf`, and `foxglove_ws.py` (`:137-148`) all decode from it with no
generated code.

### 4. Old recordings stay readable, forever

`data/` is immutable, and the eval ground truth and replay setups are tied to recordings that
hold JSON topics. So every reader decodes both encodings, chosen by `message_encoding`. There is
no converter.

- **Python**: each typed decoder returns today's types from either encoding. A new
  `mcap_io.decode_dict(msg)` replaces the raw `decode_json` callers and returns the dict today's
  JSON gives. Its rules:
  - `None` for absent optional fields and for non-finite floats, matching `json_number()`
  - Python `int` for 64-bit fields
  - lowercase enum names with the prefix stripped (`"reboot_host"`)
  - map fields as dicts

  It is about 40 lines over `ListFields()` and the descriptor. It must not use
  `json_format.MessageToDict`, which turns 64-bit ints into strings and drops zero-valued
  fields.
- **C++**: `video_playback_camera.cpp` is the only C++ reader. It parses `FrameMeta` when the
  channel is protobuf and keeps the string scan for JSON channels.

### 5. Boundary types

Two rules decide which C++ type a message uses:

- **A struct that exists only to be sent becomes the generated message.** That covers everything
  in `include/remote/messages.hpp` except `CameraSettings`, plus `FrameMeta`, `Detections`,
  `AprilTagRobotTags`, and `RobotTelemetry`. Producers fill the generated message directly.
  `json_message.hpp` and `AB_JSON_MESSAGE` are deleted.
- **A domain type the C++ code uses for its own work stays a C++ type**, with one `to_msg`/
  `from_msg` pair per type and a round-trip test. That covers `CameraSettings` (camera code and
  the TOML profile store), the `include/enums/` enums, `DiagnosticsData`, and
  `Esp32DiagnosticsEvent`.

`include/remote/protocol.hpp` stays the topic table (topic, latch, rate) and its rows name the
generated types: `StatusTopic<msg::status::System>{"/status/system", Latch::NO, 10.0}`. Its
`static_assert`s stay. `remote_protocol_dump` shrinks to dumping that table (topic, type name,
latch, rate) for the web, which takes its types from `protoc-gen-es`.

### 6. Enums

- The `include/enums/` enum classes stay canonical for config parsing (`PARSE_ENUM`). Each one
  that appears in a message gets a proto mirror and a `static_assert`. Using magic_enum, the
  assert checks that both enums have the same names once the proto prefix is stripped, so a
  value added on one side fails the build.
- **Command enums get `*_UNSPECIFIED = 0` and the decoder rejects it.** Today `SystemAction`'s
  first value is `REBOOT_HOST`. Under proto3, a command that omits the field would decode as a
  reboot. `CameraProfileAction` (zero would be `APPLY`) and `TunableId` get the same treatment.
- Status enums map one to one, with the C++ default as value 0 (`BEHAVIOR_MODE_ATTACK`,
  `ANTI_BANDING_OFF`).

### 7. Presence, defaults, and NaN

- `optional` replaces both null-in-JSON and the in-band sentinels:
  - NaN `vbat`/`ibat`/`yaw_rate_cmd` on the robot
  - `espnow_age_ms == UINT32_MAX`
  - absent `jetson_temperature_c`

  Message-typed fields always track presence and need no keyword.
- Proto3 has no custom defaults. Every sender sets every field it means, including fields whose
  C++ default is not zero: `SetAutoTracking.enabled = true` and `SetOpponentCount.count = 1`.
  The web's command builders set them explicitly.
- Floats that are not `optional` carry NaN as NaN. Only the dict shim maps it to `None`.
- Host-side messages use `double` and the robot's messages use `float`, matching what each side
  holds today. No precision changes in recordings.

### 8. Evolution rules

These go in `proto/README.md` and `CLAUDE.md`, and `buf breaking` enforces them in lint:

- **Add a field**: take the next unused number.
- **Delete a field**: mark its number and name `reserved`.
- **Never change**: a field's type or number.
- **Rename** only when nothing outside the tree reads the name. Foxglove message paths and the
  dict shim both use field names.

### 9. ESP-NOW link: keep the framing, new type IDs, freeze OTA

The serial frame and air header stay byte-for-byte. Protobuf replaces only the payload of four
message types, and they get **new type IDs**:

| Message | Old ID | New ID | Direction |
| --- | --- | --- | --- |
| Command | 0x01 | 0x21 | Jetson to robot |
| Telemetry | 0x02 | 0x22 | robot to Jetson |
| TunableSet | 0x03 | 0x23 | Jetson to robot |
| LinkStats | 0x04 | 0x24 | dongle to Jetson |
| Ping, OTA_* | 0x05, 0x10-0x14 | unchanged | |

- **Mismatched builds fail closed.** An old robot ignores 0x21, so a new host sends commands
  that the robot never applies. An old host ignores 0x22, so it records no telemetry. Neither
  side misreads a struct as protobuf.
- **`AIR_MAGIC` keeps version byte `0x01`**, so the new host's `espnow_ota.py` can still flash a
  robot on old firmware. That robot is the one that needs flashing.
- The old IDs are deleted in the same change and left as a comment: "0x01-0x04 retired
  2026-xx, do not reuse".
- **The dongle stays thin.** It stops checking `len == 16` for the priority slot and checks the
  type byte instead. It forwards every payload untouched, and encodes only its own LinkStats with
  nanopb.

### 10. Hot paths

- **Main loop**: generated C++ messages allocate on first fill. Producers keep one message per
  channel and `Clear()` it each cycle; `Clear()` keeps the capacity. `OutputChannel` gets a
  `log_proto(const google::protobuf::Message&)` that serializes into its existing reused
  `buffer_`. Gate L1 checks loop time.
- **Firmware**: nanopb encodes into a stack buffer with no heap.
  - The ring (`lib/telemetry_ring/telemetry_ring.h`) stores the nanopb-generated
    `auto_battlebot_msg_link_Telemetry` C struct. It is fixed-size because the message has no
    strings or repeated fields.
  - The ring encodes on HTTP serve, on the web server task, not in `loop()`.

## Message specs

Comments on each field carry the units and meaning that today live in the jsonschema
descriptions and struct comments. Field numbers 1 to 15 encode in one byte; the hot link fields
take them.

### `common.proto`

```proto
syntax = "proto3";
package auto_battlebot.msg;

// A point in the camera image as a fraction of its width and height, 0 to 1 on screen.
message ImagePoint {
  double u = 1;
  double v = 2;
}

// A point on the field floor in the field frame, meters, origin at the field center.
message FieldPoint {
  double x = 1;
  double y = 2;
}

// Mirrors include/enums/behavior_mode.hpp.
enum BehaviorMode {
  BEHAVIOR_MODE_ATTACK = 0;
  BEHAVIOR_MODE_RUN_AWAY = 1;
}

// Mirrors include/enums/system_action.hpp. Zero is rejected by the command decoder.
enum SystemAction {
  SYSTEM_ACTION_UNSPECIFIED = 0;
  SYSTEM_ACTION_REBOOT_HOST = 1;
  SYSTEM_ACTION_POWEROFF_HOST = 2;
}

// Mirrors include/enums/camera_profile_action.hpp. Zero is rejected by the command decoder.
enum CameraProfileAction {
  CAMERA_PROFILE_ACTION_UNSPECIFIED = 0;
  CAMERA_PROFILE_ACTION_APPLY = 1;
  CAMERA_PROFILE_ACTION_SAVE = 2;
  CAMERA_PROFILE_ACTION_DELETE = 3;
}
```

### `camera.proto`

```proto
syntax = "proto3";
package auto_battlebot.msg;

// Mirrors include/enums/anti_banding.hpp.
enum AntiBanding {
  ANTI_BANDING_OFF = 0;
  ANTI_BANDING_AUTO = 1;
  ANTI_BANDING_HZ_50 = 2;
  ANTI_BANDING_HZ_60 = 3;
}

// Live image controls in the ZED SDK's own units. Boundary type: the C++ CameraSettings struct
// stays, with to_msg/from_msg.
message CameraSettings {
  // On: the SDK's AEC/AGC owns exposure and both gains, and the three values below are what it
  // last chose. Off: they are applied as given.
  bool auto_exposure = 1;
  int32 exposure_us = 2;
  int32 analog_gain_mdb = 3;
  int32 digital_gain = 4;
  bool auto_white_balance = 5;
  int32 white_balance_k = 6;
  int32 saturation = 7;
  int32 gamma = 8;
  int32 sharpness = 9;
  AntiBanding anti_banding = 10;
}

// One control's range as the open camera reports it. Unsupported controls are hidden.
message CameraSettingRange {
  bool supported = 1;
  int32 min = 2;
  int32 max = 3;
}

// Ranges for the numeric fields of CameraSettings. Exposure is capped at the frame period.
message CameraSettingLimits {
  CameraSettingRange exposure_us = 1;
  CameraSettingRange analog_gain_mdb = 2;
  CameraSettingRange digital_gain = 3;
  CameraSettingRange white_balance_k = 4;
  CameraSettingRange saturation = 5;
  CameraSettingRange gamma = 6;
  CameraSettingRange sharpness = 7;
  CameraSettingRange anti_banding = 8;
}
```

### `recording.proto`

```proto
syntax = "proto3";
package auto_battlebot.msg;

// /camera/frame_meta, one per captured frame.
message FrameMeta {
  // Raw camera image stamp, ns. JSON carried it as a decimal string because it is above 2^53.
  uint64 image_stamp_ns = 1;
  // Frame index within the /camera/video stream; -1 when video recording is off.
  sint64 video_frame_index = 2;
}

// Keypoint in image pixels.
message Keypoint {
  double x = 1;
  double y = 2;
  double confidence = 3;
}

message Detection {
  double x1 = 1;
  double y1 = 2;
  double x2 = 3;
  double y2 = 4;
  double conf = 5;
  int32 class_id = 6;
  string label = 7;
  // Empty for detectors without keypoints. JSON carried each as [x, y, confidence].
  repeated Keypoint kps = 8;
}

// /blob_detections and /keypoint_detections.
message Detections {
  // Frame stamp, s.
  double stamp = 1;
  // Image size, pixels.
  int32 w = 2;
  int32 h = 3;
  repeated Detection dets = 4;
}

message CameraIntrinsics {
  double fx = 1;
  double fy = 2;
  double cx = 3;
  double cy = 4;
  int32 width = 5;
  int32 height = 6;
}

// Pixel rectangle.
message PixelRoi {
  int32 x = 1;
  int32 y = 2;
  int32 w = 3;
  int32 h = 4;
}

message PixelPoint {
  double u = 1;
  double v = 2;
}

// One solvePnPGeneric IPPE_SQUARE solution; rvec/tvec map tag to camera.
message PnpSolution {
  repeated double rvec = 1;  // 3 values, Rodrigues
  repeated double tvec = 2;  // 3 values, m
  double reprojection_error_px = 3;
}

message TagDetection {
  int32 id = 1;
  // 4 corners in OpenCV aruco order: clockwise from the marker's top-left.
  repeated PixelPoint corners = 2;
  optional double decision_margin = 3;
  // Sorted by reprojection error, ascending.
  repeated PnpSolution solutions = 4;
}

// /apriltag/robot_tags: robot AprilTag detections for one frame, with both IPPE solutions.
message AprilTagRobotTags {
  // Frame image stamp, ns.
  uint64 image_stamp_ns = 1;
  string frame_id = 2;
  CameraIntrinsics camera = 3;
  double tag_size_m = 4;
  // Region searched; absent when the full frame was searched.
  PixelRoi roi = 5;
  repeated TagDetection detections = 6;
}
```

`corners` and `roi` change shape (an array of `[u, v]` pairs and a `[x, y, w, h]` array in JSON).
`sysid_io.parse_robot_tags` is the one decoder every reader goes through, so it absorbs the
difference.

### `diagnostics.proto`

```proto
syntax = "proto3";
package auto_battlebot.msg;

enum DiagnosticLevel {
  DIAGNOSTIC_LEVEL_OK = 0;
  DIAGNOSTIC_LEVEL_WARN = 1;
  DIAGNOSTIC_LEVEL_ERROR = 2;
  DIAGNOSTIC_LEVEL_STALE = 3;
}

// The C++ DiagnosticScalar: int, double, or string.
message DiagnosticValue {
  oneof kind {
    double number = 1;
    int64 integer = 2;
    string text = 3;
  }
}

message DiagnosticSection {
  DiagnosticLevel level = 1;
  string message = 2;
  // Milliseconds since the subsection was last logged.
  double age_ms = 3;
  // Not logged since the previous message for this module; the values repeat it.
  bool held = 4;
  // Flattened keys, "/"-joined, as flatten_diagnostics_data produces them.
  map<string, DiagnosticValue> values = 5;
}

// /diagnostics/<module>.
message Diagnostics {
  // One entry per subsection; the empty subsection is keyed by the module name.
  map<string, DiagnosticSection> sections = 1;
}
```

`DiagnosticsData` stays the C++ input type. Only `diagnostics_module_json` is replaced.

### `telemetry.proto`

One host message for both robot telemetry topics. It wraps the record the robot sends, so the
robot's fields are defined only in `link.proto`.

```proto
syntax = "proto3";
package auto_battlebot.msg;

import "auto_battlebot/msg/link/link.proto";

enum TelemetryTransport {
  TELEMETRY_TRANSPORT_UNSPECIFIED = 0;
  TELEMETRY_TRANSPORT_ESPNOW = 1;  // /robot/espnow_telemetry
  TELEMETRY_TRANSPORT_WIFI = 2;    // /robot/esp32_diagnostics, polled GET /telemetry.pb
}

message RobotTelemetry {
  link.Telemetry robot = 1;
  // Host wall clock when the frame arrived, ns; also the MCAP log time.
  int64 host_receive_ns = 2;
  TelemetryTransport transport = 3;
  // ESP-NOW only: host monotonic arrival minus robot.jetson_tx_us. Absent before the first
  // accepted command.
  optional int64 round_trip_us = 4;
  // WiFi only: the row waited in the robot's ring for a poll or for the link, so
  // host_receive_ns is late and clock fits skip it.
  bool backfill = 5;
}
```

Both topics keep their names. What changes:

- Foxglove paths gain one level: `/robot/espnow_telemetry.vbat` becomes
  `/robot/espnow_telemetry.robot.vbat`.
- `/robot/esp32_diagnostics` loses `wifi_clients`, which the binary batch path cannot carry
  today (`TelemetryPayload` has no such field).
- Its `seq` becomes `robot.telemetry_seq`, the rename `calibration/sysid/telemetry.py` already
  does.

### `link/link.proto` (firmware and host)

```proto
syntax = "proto3";
package auto_battlebot.msg.link;

// Type 0x21, Jetson to robot through the dongle. The dongle repeats the last one every 4 ms.
message Command {
  // Higher is new, equal is a dongle repeat, lower is dropped unless 500 ms of silence passed.
  fixed32 seq = 1;
  // Host monotonic send time, us, echoed in Telemetry.jetson_tx_us.
  fixed32 jetson_tx_us = 2;
  // Normalized [-1, 1]: linear/angular or left/right depending on the drive processor.
  float channel_a = 3;  // ch0
  float channel_b = 4;  // ch1
}

// Type 0x22, robot to Jetson at 100 Hz over ESP-NOW; also the record in TelemetryBatch.
message Telemetry {
  // Robot clock, ms.
  uint32 timestamp_ms = 1;
  uint32 loop_us = 2;
  // Arbitrated drive command, after the flip, percent.
  float a_percent = 3;
  float b_percent = 4;
  // Motor commands after PID and mixer, percent.
  float left_cmd = 5;
  float right_cmd = 6;
  // deg/s clockwise.
  float yaw_rate = 7;
  // deg/s clockwise; absent when the yaw loop is not running.
  optional float yaw_rate_cmd = 8;
  // +1 per frame sent; a gap means dropped frames. In the WiFi ring copy this is the ring seq.
  uint32 telemetry_seq = 9;
  float pid_setpoint = 10;
  float pid_output = 11;
  // Pack voltage, V, and current, A, positive discharging; absent without the INA228.
  optional float vbat = 12;
  optional float ibat = 13;
  bool armed = 14;
  bool radio_connected = 15;

  // BNO055 fused gravity vector.
  float accel_x = 16;
  float accel_y = 17;
  float accel_z = 18;
  // BNO055 Euler angles, degrees.
  float orientation_x = 19;
  float orientation_y = 20;
  float orientation_z = 21;
  // BNO055 fused acceleration with gravity removed, m/s^2, chip frame.
  float linear_accel_x = 22;
  float linear_accel_y = 23;
  float linear_accel_z = 24;
  // Gyro rate about the chip axis, deg/s, right-handed.
  float gyro_x = 25;
  float gyro_y = 26;
  // BNO055 CALIB_STAT, 0 uncalibrated to 3 fully calibrated. Split from the packed byte.
  // IMUPLUS mode reaches sys 3 without the magnetometer, and mag stays 0.
  uint32 imu_calib_sys = 27;
  uint32 imu_calib_gyro = 28;
  uint32 imu_calib_accel = 29;
  uint32 imu_calib_mag = 30;
  // ch5 (SA) as the robot saw it; gates espnow_active.
  bool autonomy_enabled = 31;
  bool button_state = 32;
  sint32 flip_switch = 33;
  bool is_upside_down = 34;
  // ESP-NOW supplied this tick's drive command.
  bool espnow_active = 35;
  // Channel the robot listens on; 0 while sweeping.
  uint32 espnow_channel = 36;
  // Since the robot last accepted a command frame; absent before the first.
  optional uint32 espnow_age_ms = 37;
  // Last accepted Command.seq, and its jetson_tx_us echoed for round-trip measurement.
  uint32 espnow_seq = 38;
  fixed32 jetson_tx_us = 39;
  // Telemetry send ACK success, percent over the last 32 frames.
  uint32 espnow_lq = 40;
}

enum TunableId {
  TUNABLE_ID_UNSPECIFIED = 0;  // rejected by the robot
  TUNABLE_ID_LEFT_ESC_DEADZONE = 1;
  TUNABLE_ID_RIGHT_ESC_DEADZONE = 2;
}

// Type 0x23, Jetson to robot.
message TunableSet {
  TunableId id = 1;
  float value = 2;
}

// Type 0x24, dongle to Jetson every 100 ms, serial only.
message LinkStats {
  uint32 sent = 1;
  uint32 acked = 2;
  uint32 failed = 3;
  // esp_now_send to send callback for the newest unicast frame, us.
  uint32 last_rtt_us = 4;
  uint32 channel = 5;
  // False while the dongle broadcasts, true once it has learned the robot MAC.
  bool robot_known = 6;
}

// Body of GET /telemetry.pb?from=<seq>&max=<n>. Replaces TelemetryBatchHeader plus raw records;
// the new path replaces the magic, version, and record_size checks.
message TelemetryBatch {
  // Random per boot; a change means seqs restarted at 0.
  fixed32 boot_id = 1;
  // Records the ring holds.
  uint32 capacity = 2;
  // Seq the next record gets; head - 1 is the newest.
  uint32 head = 3;
  // Oldest seq still in the ring.
  uint32 oldest = 4;
  // Seq of records[0].
  uint32 first_seq = 5;
  repeated Telemetry records = 6;
}
```

`link.options` sets `int_size:IS_8` on the `imu_calib_*`, `espnow_channel`, `espnow_lq`, and
`flip_switch` fields so the generated C struct stays small. It also makes
`TelemetryBatch.records` a callback that encodes straight from the ring. Ping (0x05) has no
payload and keeps none. The OTA structs stay in `espnow_protocol.h`, unchanged.

### `status.proto`

Schema names move from `auto_battlebot.status.<Name>` to `auto_battlebot.msg.status.<Name>`.

```proto
syntax = "proto3";
package auto_battlebot.msg.status;

import "auto_battlebot/msg/common.proto";
import "auto_battlebot/msg/camera.proto";

// /status/system, 10 Hz.
message System {
  bool camera_ok = 1;
  bool transmitter_connected = 2;
  bool transmitter_receiving = 3;
  double loop_rate_hz = 4;
  bool initialized = 5;
  int32 selected_opponent_count = 6;
  bool autonomy_enabled = 7;
  bool camera_recording = 8;
  bool mcap_recording = 9;
  optional double jetson_temperature_c = 10;
  string compute_mode = 11;
  // Seconds since the app started.
  double uptime_s = 12;
  // The transmitter's autonomy (trainer) switch; absent when the transmitter has none.
  optional bool autonomy_switch_on = 13;
  // Seconds since that switch last went on; absent while it is off.
  optional double autonomy_on_s = 14;
  // Seconds since the current MCAP file started; absent while MCAP recording is off.
  optional double mcap_file_s = 15;
  BehaviorMode behavior_mode = 16;
  // The target selector picks the target. Off, navigation steers to the dashboard's held point.
  bool auto_tracking = 17;
}

// A model label and the color it is drawn in, from [ui.label_colors].
message LabelColor {
  string label = 1;
  string color = 2;  // "#rrggbb"
}

// /status/app, every 5 s, latched.
message App {
  repeated string available_profiles = 1;
  string current_profile = 2;
  double max_loop_rate_hz = 3;
  double rate_fail_threshold = 4;
  double rate_fail_duration_sec = 5;
  repeated LabelColor label_colors = 6;
  // Short git hash, "-dirty" for uncommitted changes; "unknown" without a stamp.
  string version = 7;
}

// /status/sticks, 20 Hz: our robot's sticks as the transmitter read them back, [-1, 1].
message Sticks {
  double linear = 1;
  double angular = 2;
}

// One tracked robot in the field frame, meters and radians.
message TrackedRobot {
  // Track slot, e.g. their_robot_2. Unique per robot even when two share a label.
  string id = 1;
  string label = 2;
  bool ours = 3;
  bool stale = 4;
  double x = 5;
  double y = 6;
  double yaw = 7;
  // Absent when the robot projects behind the camera.
  ImagePoint image = 8;
}

// Where navigation is steering this cycle.
message Target {
  double from_x = 1;
  double from_y = 2;
  ImagePoint from_image = 3;
  double x = 4;
  double y = 5;
  // The targeted robot's label; empty for a point that is not a robot.
  string label = 6;
  BehaviorMode mode = 7;
  ImagePoint image = 8;
  // The dashboard placed this point; the target selector did not pick it.
  bool manual = 9;
}

// Protobuf has no nested repeated, so one polyline is a message.
message ImagePolyline {
  repeated ImagePoint points = 1;
}

// /status/tracks, 20 Hz.
message Tracks {
  bool our_robot_seen = 1;
  int32 opponents_seen = 2;
  // Field size, m. Zero before init.
  double field_x = 3;
  double field_y = 4;
  repeated TrackedRobot robots = 5;
  // Field border projected into the image, split where it passes behind the camera.
  repeated ImagePolyline field_outline = 6;
  // Top-down view's camera triangle in the image: tip, then base corners. Empty when any corner
  // is behind the camera.
  repeated ImagePoint camera_marker = 7;
  // Absent before navigation has a target.
  Target target = 8;
}

// /status/command_ack, per command.
message CommandAck {
  int64 seq = 1;
  string topic = 2;
  bool accepted = 3;
  string message = 4;
}

// /status/network, every 5 s and on change, latched.
message Network {
  string hostname = 1;
  // IPv4 link-local address on the cable, empty when no port has one.
  string cable_address = 2;
  string wifi_interface = 3;
  optional string wifi_address = 4;
  bool wifi_access = 5;
}

// /status/battery, latched. Never published when the battery source is NONE.
message Battery {
  // False until the first good read, and again after invalid_read_grace failed reads.
  bool valid = 1;
  // Estimated state of charge, 0 to 100.
  double percent = 2;
  // The last read's measurements; absent when that read failed.
  optional double bus_voltage_v = 3;
  optional double current_a = 4;
  // Current is flowing into the battery.
  bool charging = 5;
}

// /status/camera_settings, latched.
message CameraSettings {
  // False on cameras without live controls.
  bool supported = 1;
  // DEFAULT first, then the saved profiles.
  repeated string profiles = 2;
  string active_profile = 3;
  // Live values changed since the active profile was applied or saved.
  bool modified = 4;
  auto_battlebot.msg.CameraSettings settings = 5;
  auto_battlebot.msg.CameraSettingLimits limits = 6;
}
```

### `command.proto`

```proto
syntax = "proto3";
package auto_battlebot.msg.command;

import "auto_battlebot/msg/common.proto";
import "auto_battlebot/msg/camera.proto";

message ReinitField {}

message SetOpponentCount {
  int32 count = 1;  // senders always set it; the C++ default of 1 does not exist on the wire
}

message SetAutonomy {
  bool enabled = 1;
}

message SetBehaviorMode {
  BehaviorMode mode = 1;
}

// On hands targeting back to the target selector. Off holds the target at the field center
// until a SetManualTarget moves it.
message SetAutoTracking {
  bool enabled = 1;  // senders always set it; the C++ default of true does not exist on the wire
}

// Moves the held target while auto tracking is off; refused while it is on. Replaces two
// optionals with a oneof, so "exactly one" is enforced by the decoder.
message SetManualTarget {
  oneof point {
    FieldPoint field = 1;  // a tap on the top-down view
    ImagePoint image = 2;  // a tap on the camera view, projected onto the floor
  }
}

// Turns camera (SVO or H.264) and MCAP recording on or off together.
message SetRecording {
  bool enabled = 1;
}

// Picks the profile for the next launch.
message SelectProfile {
  string name = 1;
}

message SystemAction {
  auto_battlebot.msg.SystemAction action = 1;  // UNSPECIFIED is rejected
}

// Opens or closes the dashboard ports on the Wi-Fi interface.
message SetWifiAccess {
  bool enabled = 1;
}

// Changes the camera's live controls. The saved profile is untouched until a SAVE.
message SetCameraSettings {
  auto_battlebot.msg.CameraSettings settings = 1;
}

// APPLY a saved profile or DEFAULT, SAVE the live values under name, or DELETE name.
message CameraProfile {
  auto_battlebot.msg.CameraProfileAction action = 1;  // UNSPECIFIED is rejected
  string name = 2;
}
```

### `sim.proto`

Replaces `REQUEST_FMT`, `RESPONSE_HEADER_FMT`, and the GT and measurement blocks in
`simulation/protocol.py`, and the packed structs in `src/simulation/sim_connection.cpp`. Each
message on the TCP stream is prefixed by a little-endian `u32` length.

```proto
syntax = "proto3";
package auto_battlebot.msg.sim;

// Client to server, one per tick.
message StepRequest {
  double linear_x = 1;
  double linear_y = 2;
  double angular_z = 3;
}

message Pose2D {
  double x = 1;
  double y = 2;
  double yaw = 3;
}

// What perception delivered for one ground-truth slot, from the sensing model.
message MeasuredPose {
  bool measured = 1;
  Pose2D pose = 2;
}

message Measurement {
  bool delivered = 1;
  // Sim seconds.
  double capture_time = 2;
  // One per ground-truth slot, same order.
  repeated MeasuredPose poses = 3;
}

// Server to client, one per tick.
message StepResponse {
  uint32 width = 1;
  uint32 height = 2;
  // 16 values, the same layout tf_matrix has today.
  repeated double tf_matrix = 3;
  double fx = 4;
  double fy = 5;
  double cx = 6;
  double cy = 7;
  // The sim's logical time, s; the sim owns the only dt.
  double sim_time = 8;
  // The same bytes the response carries today after the header.
  bytes rgb = 9;
  bytes depth = 10;
  // Our robot first, then opponents in config order.
  repeated Pose2D ground_truth = 11;
  Measurement measurement = 12;
}
```

The flags that travel as `0.0`/`1.0` doubles today become `bool`.

### `calibration.proto`

```proto
syntax = "proto3";
package auto_battlebot.msg.calibration;

enum ImageFormat {
  IMAGE_FORMAT_JPEG = 0;  // lossy
  IMAGE_FORMAT_RAW = 1;   // lossless bgr8
}

// Floor grid board parameters for the lock; the arguments of make_floor_board.
message FloorBoard {
  int32 cols = 1;
  int32 rows = 2;
  double marker_size = 3;
  double marker_sep = 4;
  int32 first_id = 5;
}

// /calibration/metadata, once per recording.
message Metadata {
  // CLOCK_MONOTONIC seconds at recording start.
  double t = 1;
  // 3x3 K, row-major, 9 values.
  repeated double camera_matrix = 2;
  repeated double dist_coeffs = 3;
  int32 image_width = 4;
  int32 image_height = 5;
  // Robot tag edge length, m.
  double tag_size = 6;
  int32 tag_id = 7;
  double yaw_offset_deg = 8;
  FloorBoard floor = 9;
  ImageFormat image_format = 10;
  string clock = 11;
}

// /transmitter/channels, per driving frame. log_time is the image stamp.
message TransmitterChannels {
  // CLOCK_MONOTONIC seconds the channel snapshot was decoded.
  double sample_t = 1;
  // Raw OpenTX channel values, about [-1024, 1024]; stick axes on the low channels.
  repeated int32 channels = 2;
}

// /drive/command, per scripted excitation send (about 50 Hz) in a --drive run.
message DriveCommand {
  // Normalized [-1, 1].
  double cmd_lin = 1;
  double cmd_ang = 2;
  // Values written to the trainer link, [-500, 500].
  int32 trainer_lin = 3;
  int32 trainer_ang = 4;
  // Excitation protocol phase, so the fitter can slice the run.
  string label = 5;
}
```

## Link size budget

Worst case: every field present, every varint at its widest. Today's sizes come from the
`static_assert`s in `espnow_protocol.h:176-183`.

| Message | Today, bytes | Protobuf worst case, bytes | Air frame worst (+5) | Limit |
| --- | --- | --- | --- | --- |
| Command | 16 | 20 | 25 | 250 |
| Telemetry | 118 | 191 | 196 | 250 |
| TunableSet | 5 | 7 | 12 | 250 |
| LinkStats | 18 | 28 | serial only, 34 | 261 serial |
| TelemetryBatch, 256 records | 32 + 30,208 | about 49,000 | HTTP | none |

Telemetry worst case adds up as follows:

- 40 tags: 15 at one byte plus 25 at two, so 65 bytes.
- 21 floats at 4 bytes each: 84.
- 5 `uint32` at up to 5 bytes each: 25.
- `jetson_tx_us`, fixed32: 4.
- 13 small varints at 1 byte each: 13.

Typical frames are smaller, since proto3 omits zero and false fields. At the dongle's 6 Mbps PHY
rate, the extra 73 bytes cost at most 97 us of air time per telemetry frame, and 4 bytes cost
5 us per command. Per-frame MAC overhead is unchanged. nanopb's generated
`*_size` macros replace the hand `static_assert`s:
`static_assert(auto_battlebot_msg_link_Telemetry_size <= AIR_MAX_PAYLOAD)`.

**Ring memory.** The generated C struct comes to about 128 bytes once `int_size` is applied,
against 118 today. 12000 records grow from 1.42 MB to about 1.54 MB of the QT Py's 2 MB PSRAM.
Gate F4 checks free PSRAM after the allocation. If it is short, `capacity` drops to keep the
byte budget: about 11,000 records, 110 s at 100 Hz instead of 120 s.

## Steps

Each phase lands on its own and leaves the tree working. Old recordings stay readable after
every phase.

### Phase 0: toolchain (gate T1)

1. `.protobuf-version`, `install/install_protobuf.sh`, called from every platform install.
   Build with `-j6` under `systemd-run MemoryMax=16G` (see the build-job cap).
2. Pin `protobuf==<matching>` in `pyproject.toml`, keeping the platform-conditional deps as they
   are. Add `@bufbuild/protobuf` and `@bufbuild/protoc-gen-es` to `web/package.json`. Add the
   exact nanopb version to the robot, dongle, and native `lib_deps`.
3. A throwaway `proto/auto_battlebot/msg/hello.proto` compiled and round-tripped in C++, Python,
   TS, and the native firmware test env on every platform.

### Phase 1: infrastructure plus FrameMeta end to end (gate R1)

1. `proto/` tree, `proto/README.md` (evolution rules), `buf.yaml`. Add a `scripts/lint` step that
   runs `buf format`, `buf lint`, `buf breaking`, and regenerates Python and TS, failing when
   either is stale.
2. CMake: `protobuf_generate()` into a static `auto_battlebot_msg` library that
   `auto_battlebot_lib` links.
3. C++: `OutputChannel::log_proto`; a `proto_schema<T>()` that builds the `FileDescriptorSet`.
   `VizSchema::protobuf(name, data)`.
4. Python: `McapWriter.log_proto(topic, message, ns)`; `mcap_io.decode_dict`; schema builder from
   `DESCRIPTOR`.
5. FrameMeta:
   - `to_frame_meta_json` becomes `to_frame_meta`.
   - `video_playback_camera.cpp` decodes both encodings.
   - `decode_frame_meta` handles both, returning `image_stamp_ns` as `int` in both cases.
   - The Python writers move over: `video_import.py:218`, `combine_mcap_svo.py:470`,
     `convert_ros1_mcap.py:237`.
   - The raw readers move to `decode_dict`: `match_windows.py:104`,
     `render_dropout_videos.py:160`, `render_match_prediction_camera.py:72`,
     `check_sysid_recordings.py:176`, `combine_mcap_svo.py:108,128`.

### Phase 2: the other recording topics

1. Detections: `to_detections_json`, `decode_detections`, `read_detections`, the raw reader in
   `render_dropout_videos.py:164`, and `CameraView.svelte`'s hand-written `Detections` interface
   (the web subscribes to `/keypoint_detections`).
2. AprilTagRobotTags: `AprilTagKeypointModel::to_json`, `sysid_io.parse_robot_tags`, and
   `calibration/sysid/synthetic.py:330`.
3. Calibration tool topics: `apriltag_mcap.py` writers (`:175,205,230,578`) and readers
   (`:239,272,286`), `analyze_apriltag_mcap.py`.

### Phase 3: diagnostics (gate D1 first)

1. Record one replay with protobuf diagnostics on a branch and run gate D1.
2. If it passes, make these changes:
   - `diagnostics_module_json` becomes `to_diagnostics_msg`.
   - `decode_diagnostic_array` handles both encodings.
   - The raw readers move to `decode_dict`: `compare_replays.py:62,90`,
     `fit_opponent_sensing_model.py:124`, `plot_dropout_samples.py:73`,
     `fit_sensing_model.py:260`, `fit_provisional_ekf_error.py:91,147`.
   - The writers move over: `McapWriter.log_diagnostics` and `convert_ros1_mcap.py:125-142`.
   - The web's `diagnostics.svelte.ts` moves over.
3. If it fails, diagnostics stays JSON. Its schema text moves to a single file that both
   languages load, the fallback from the earlier discussion, and this plan records why.

### Phase 4: remote protocol (gates W1, L1)

1. Generated types replace `messages.hpp`, and `protocol.hpp` rows name them.
   `json_message.hpp` is deleted. `CameraSettings` and the enums get `to_msg`/`from_msg` with
   round-trip tests and the name `static_assert`s.
2. `StatusBus` and `CommandLog` use `log_proto`. `decode_command` parses by topic and rejects
   `UNSPECIFIED` enums.
3. `viz_relay`: `supported_encodings = {"json", "protobuf"}`. JSON stays until the web is moved
   over, then goes.
4. Web:
   - protobuf-es types replace `protocol.ts`, `gen-protocol.mjs`, and the schema half of
     `remote_protocol_dump`.
   - `connection.ts` decodes by topic and advertises command channels as protobuf, with the
     `FileDescriptorSet` from the generated file.
   - The command builders set every field.
5. Python readers of `/status/tracks` move to `decode_dict`: `compare_replays.py:81-82`,
   `fit_sensing_model.py:280-281`. `field_outline` comes back as lists of points either way.

### Phase 5: ESP-NOW link and robot telemetry (gates F1 to F4)

1. `link.proto` and `link.options`.
2. Firmware:
   - nanopb in the robot and dongle builds.
   - `publish_diag()` fills `auto_battlebot_msg_link_Telemetry`.
   - The ring stores the generated struct.
   - `/telemetry` becomes `/telemetry.pb`, encoding `TelemetryBatch` with the `records`
     callback.
   - The new type IDs go in.
   - The packed payload structs are deleted from `espnow_protocol.h`, leaving the framing, the
     CRC, `SerialFrameParser`, and the OTA structs.
   - Native googletest env tests encode golden fixtures.
3. Dongle: priority slot keyed on type 0x21; LinkStats via nanopb.
4. Host:
   - `espnow_transmitter.cpp` decodes 0x22 and 0x24 with libprotobuf, sends 0x21, and records
     `RobotTelemetry`.
   - `esp32_wifi_telemetry.cpp` polls `/telemetry.pb`.
   - `esp32_telemetry_batch.cpp` decodes `TelemetryBatch`.
   - `to_espnow_telemetry_json` and `to_esp32_diagnostics_json` are deleted, along with the CSV
     parser that only tests use.
5. Python:
   - `calibration/sysid/telemetry.py` takes its columns from the `link.Telemetry` descriptor
     plus the `RobotTelemetry` host fields.
   - `sysid_io.load_esp32_diagnostics` and `sources.py:81-83` decode both encodings.
   - `synthetic.py:328` writes `RobotTelemetry`.
6. Firmware, dongle, and host flash and deploy together. They fail closed if they are not.

### Phase 6: sim protocol (gate S1)

`simulation/protocol.py`, `kinematic_sim_server.py`, `rl_eval_server.py`, and
`sim_connection.cpp` move to length-prefixed `StepRequest`/`StepResponse`.

### Phase 7: cleanup

1. Delete:
   - `include/foxglove_adapters/json_schemas.hpp`
   - the schema texts and `JSON_TOPIC_SCHEMAS` in `mcap_write.py`
   - `tests/python/test_json_schema_parity.py`
   - the C++ `json_number`/`json_escape` helpers once nothing uses them
   - `McapWriter.log_json`, if nothing outside the legacy-read tests calls it
2. Rewrite `docs/foxglove_recording_format.md`:
   - the channel table and per-topic sections point at the `.proto` files
   - a "Reading older files" section lists the JSON layouts readers still accept
3. `CLAUDE.md`, Conventions:
   - add "Messages that cross a process, language, or device boundary are defined in `proto/`.
     Add a field with the next free number; mark deleted numbers `reserved`."
   - update the remote UI line to point at `proto/auto_battlebot/msg/status.proto`
4. Update the ESP-NOW protocol tables in `docs/plans/espnow_link_plan.md`.

## Gates

| Gate | Phase | Passes when |
| --- | --- | --- |
| T1 | 0 | The hello message round-trips in C++, Python, TS, and native firmware on dev x86 (22.04 and 24.04), the JP7 Orin NX, and the JP6 ZED Box Mini. The Python runtime installs on 3.10 and 3.12. The protobuf build time on the ZED Box Mini is recorded here |
| R1 | 1 | A new C++-written MCAP decodes in Python and in the Foxglove app. Every typed decoder returns identical results before and after the change on the NHRL May replay set and the four `nhrl_keypoints_eval_test` recordings |
| D1 | 3 | In Foxglove, a diagnostics value plots from a message path a person can type, e.g. `/diagnostics/nav.sections[:]{key=="nav"}.value.values[:]{key=="speed"}.value.number`. If that is the best Foxglove offers and it is unusable in practice, diagnostics stays JSON |
| W1 | 4 | On the ZED Box, every dashboard panel renders and every command is acked against the live app. A command with an `UNSPECIFIED` enum is refused |
| L1 | 1, 2, 4 | Loop time p99 from `scripts/mcap_latency_report.py` on the Jetson regresses by no more than 0.5 ms against a baseline taken the same day on the same build box |
| F1 | 5 | The nanopb `*_size` `static_assert`s hold. Golden bytes encoded by nanopb decode identically in libprotobuf and Python, and the reverse for Command |
| F2 | 5 | `pb_encode` of a full Telemetry on the ESP32-S3 takes under 100 us, measured with `esp_timer` over 10,000 frames |
| F3 | 5 | Over a 5-minute bench session, `round_trip_us` p50 and p99 are within 0.2 ms of a baseline from the current firmware. This waits until ESP-NOW is re-enabled on the robot (`ESPNOW_ENABLED = false` today); the WiFi batch path validates first |
| F4 | 5 | Free PSRAM after the ring allocation is at least what it is today, or `capacity` is cut to match |
| S1 | 6 | The lockstep smoke test (Mr Stabs Mk2, pursuit, 450 ticks, `mr_stabs_mk2_kinematic_sim_sensing.toml`) gives the same result as before: the same tick count to first command and the same final distance |

## Tests

- **Golden fixtures** under `tests/fixtures/msg/`, one encoded message per type, checked from
  C++, Python, and (for `link.proto`) the native firmware env. A fixture change shows up in the
  diff.
- **Descriptor parity**: C++ and Python build the same `FileDescriptorSet` for every message.
  This replaces `test_json_schema_parity.py`.
- **Legacy decode**: the existing JSON-text tests (`test_detections.cpp`, `test_camera_info.cpp`,
  `test_foxglove_diagnostics_backend.cpp`, `test_espnow_transmitter.cpp`,
  `test_esp32_diagnostics.cpp`, `test_esp32_wifi_telemetry.cpp`, `test_video_playback_camera.cpp`,
  `test_status_bus.cpp`) become protobuf tests. A JSON message from each kept as a fixture
  checks that the readers still accept old files.
- **`decode_dict`**: for each topic, the JSON and protobuf forms of the same message give equal
  dicts. That covers NaN, absent optionals, 64-bit ints, and enums.
- **Enum mirrors**: compile-time, through the magic_enum name `static_assert`s.
- **Command rejection**: `UNSPECIFIED` enums, a `SetManualTarget` with no point, and an unknown
  topic.

## Risks

- **Python 3.10 support.** If protobuf 7.34 does not support Python 3.10, JetPack 6 decides the
  release for every box. T1 finds out before anything else depends on it.
- **libprotobuf build on the Jetsons.** The protobuf and abseil build is large and has to run
  under the memory cap. It runs once per box, not per clean build, but a JetPack reflash means
  running it again.
- **Foxglove ergonomics for maps** (gate D1). Diagnostics is the one topic where JSON's keyed
  objects read better than protobuf's map entries.
- **Lenient parsing.** Random bytes can parse as a valid protobuf message, because unknown fields
  are skipped. The serial CRC, the air magic, and the MCAP schema name guard the paths that could
  feed garbage in. A size check against the nanopb maximum guards the link.
- **64-bit fields in the web.** protobuf-es gives `int64` as `bigint`. `CommandAck.seq` is the
  only 64-bit field the dashboard reads. Mark it `[jstype = JS_NUMBER]` or make it `int32`; W1
  catches whichever breaks.
- **Mixed robot and host builds during Phase 5** fail closed, and the operator sees it as
  `LinkStats.robot_known` with no telemetry. That's visible, but a field session with a
  mismatched pair has no autonomy until both are reflashed.
- **Foxglove paths change.** The telemetry topics gain `.robot.`, and the schema names gain
  `.msg.`. There are no committed layouts. Paths quoted in prose (`rl_policy_navigation_plan.md:378`,
  `sensing_model_plan.md:451`, `control_loop_thread_plan.md:423`, `combine_mcap_svo.py:65`) get
  updated in the phase that changes them.

## Open questions

1. Should `/robot/esp32_diagnostics` keep its name, or become `/robot/wifi_telemetry` now that it
   carries `RobotTelemetry`? This plan keeps the name. ESP-NOW Phase 2 deletes the topic anyway.
2. Is dropping `wifi_clients` from `/robot/esp32_diagnostics` fine? Only the legacy CSV path ever
   filled it.
3. Should Phase 6 (sim) wait until the RL work in `rl_policy_navigation_plan.md` is not using the
   lockstep server daily?

## Next steps

1. Answer the open questions.
2. Run Phase 0 and gate T1, starting with the protobuf install on the ZED Box Mini and Python
   3.10, since those two decide the release for everything else.
3. Phase 1 with FrameMeta, then gate R1 on the NHRL May replay set.
