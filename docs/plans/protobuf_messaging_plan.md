# Protobuf messaging plan

Goal: every message that crosses a process, language, or device boundary is defined once, in a
`.proto` file under `proto/`, and every producer and consumer uses code generated from it. That
covers the JSON recording topics, the remote UI's status and command topics, the diagnostics
topics, the ESP-NOW link between the Jetson, the dongle, and the robot, the robot's WiFi
telemetry batch, the lockstep sim protocol, and the AprilTag calibration tool's topics.

Today the same message is written out by hand in up to four places. `EspNowTelemetry` is a
packed struct in `firmware/shared/espnow_protocol/src/espnow_protocol.h`, a hand-built JSON
string in `to_espnow_telemetry_json` (`src/transmitter/espnow_transmitter.cpp:61`), a jsonschema
text kept byte-identical in `include/foxglove_adapters/json_schemas.hpp` and
`auto_battlebot/recording/mcap_write.py`, and a table of type IDs and payload sizes with its own
`struct.unpack` decoders in `firmware/espnow_dongle/scripts/espnow_diag.py`. The sim protocol is
two hand-synced struct layouts (`simulation/protocol.py`,
`src/simulation/sim_connection.cpp:24-38`). Changing one field means editing every copy and
hoping a parity test covers it.

## Status, 2026-11-18

Plan only. Nothing is implemented: there is no `proto/`, `.protobuf-version`,
`install/install_protobuf.sh`, or `buf.yaml`, and no nanopb or protobuf-es dependency.

The plan was written on 2026-10-17 and checked against the tree on 2026-11-18. That check
changed these parts:

- `link.Telemetry` is nested. The motor stall guard added 12 fields, and the flat layout no
  longer fits one ESP-NOW frame (see Link size budget).
- `.proto` field comments are exempt from the comment policy (decision 11).
- ESP-NOW is live (`ESPNOW_ENABLED = true`), so Phase 5 changes the drive path and the host
  reports a robot on the retired type IDs (decision 9).
- `espnow_diag.py` is in scope, and the robot's HTTP routes have a scope row.
- The link rules for libprotobuf are decided (decision 2), and gates T1, L1, S1, and W1 check
  more.
- Line-number lists of callers are replaced by a grep check per phase.

## Scope

| Path | In scope | Reason |
| --- | --- | --- |
| JSON recording topics: `/camera/frame_meta`, `/blob_detections`, `/keypoint_detections`, `/apriltag/robot_tags` | yes | Hand-built JSON in C++, schema texts duplicated |
| `/diagnostics/<module>` | yes, behind gate D1 | Free-form tree; Foxglove plot paths may get worse |
| `/status/*`, `/command/*` | yes | Already single-source (`AB_JSON_MESSAGE`), but only in C++; the web gets generated TS through a dump binary |
| `/robot/espnow_telemetry` | yes | Hand copies of one message in C++ and Python |
| ESP-NOW air and serial payloads: Command, Telemetry, TunableSet, LinkStats | yes | Packed structs, memcpy decode |
| `firmware/espnow_dongle/scripts/espnow_diag.py` | yes | Its own copy of the type IDs, the payload sizes, and `struct.unpack` decoders for Command, Telemetry, TunableSet, and LinkStats; it also builds Command frames |
| `firmware/mr_stabs_mk2/scripts/espnow_ota.py` | no | Speaks only the serial framing and the frozen OTA messages |
| WiFi `GET /telemetry` batch body | yes | Same Telemetry record as ESP-NOW |
| ESP-NOW OTA messages (`MSG_OTA_*`) | no, frozen | The chunk has 1 byte of margin under 250, and OTA must keep working against robots still on the old firmware |
| Serial framing (`[7B B0][type][len][payload][crc16]`) and air header (`BWB\x01` + type) | no | Transport framing; protobuf has no framing or type tag of its own |
| Lockstep sim TCP protocol | yes | Two hand-synced struct layouts |
| Calibration tool topics (`/calibration/metadata`, `/transmitter/channels`, `/drive/command`) | yes | Python-only JSON with inline jsonschema dicts |
| `viz` unix socket framing (`include/viz/frame.hpp`) | no | Container for already-encoded messages, like MCAP itself |
| Foxglove SDK topics (`foxglove.*`) | no | Already protobuf |
| CRSF, BLHeli MSP | no | External standard protocols |
| Robot HTTP routes other than `/telemetry`: `/events` SSE CSV, `/status`, `/wifi*`, `/tuning`, `/record/*`, `/crash/erase`, `/i2c/scan` | no | Served by the robot for its own web page (`firmware/mr_stabs_mk2/web/`); nothing on the host reads them |
| Robot ESC tuning routes: `/esc/status`, `/esc/tuning`, `/esc/4way` | no | `/esc/4way` relays BLHeli 4-way bytes, an external protocol. `/esc/status` is a small JSON poll with one host client, `auto_battlebot/blheli32/http_port.py` |
| `velocity_jig` ASCII console protocol | no | Standalone tool, human-typed commands, never reaches the app |
| `viz_relay` `/healthz` JSON | no | One-field HTTP probe for the kiosk |
| RTNeural model JSON (`src/rl_policy/runtimes/rtneural_runtime.cpp`) | no | A file format for weights, not a message |

## What exists

- **C++ JSON writers**: all hand-built `std::string` through `OutputChannel::log`
  (`include/publisher/output_channel.hpp:32`). They are `to_frame_meta_json`
  (`src/foxglove_adapters/camera_info.cpp:43`), `to_detections_json`
  (`src/foxglove_adapters/detections.cpp:10`), `diagnostics_module_json`
  (`src/diagnostics_logger/foxglove_diagnostics_backend.cpp`), `to_espnow_telemetry_json`
  (`src/transmitter/espnow_transmitter.cpp:61`), and
  `AprilTagKeypointModel::to_json` (`src/keypoint_model/apriltag_keypoint_model.cpp:446`).
  No C++ code writes `/robot/esp32_diagnostics`; only recordings before 2026-11-20 carry it.
- **C++ JSON reader**: `video_playback_camera.cpp:199-200` string-scans `/camera/frame_meta`.
  The same file hand-decodes protobuf `CompressedVideo` to avoid linking protobuf.
- **Remote protocol**: structs with `AB_JSON_MESSAGE` (`include/remote/messages.hpp`), topic table
  in `include/remote/protocol.hpp:38-63` (8 status, 12 command), encode/decode/schema in
  `include/remote/json_message.hpp`. `tools/remote_protocol_dump.cpp` feeds
  `web/scripts/gen-protocol.mjs`, which writes `web/src/generated/protocol.ts`.
- **Python**: almost every reader goes through `mcap_io.iter_messages` (`mcap_io.py:113`), then
  either a typed decoder (`decode_frame_meta :195`, `decode_diagnostic_array :205`,
  `decode_detections :471`, `sysid_io.parse_robot_tags :134`,
  `sysid_io.load_esp32_diagnostics`) or a raw `decode_json(data)[...]`. There are 25 raw
  `decode_json(` call sites in 14 files. `mcap_io._proto` (`:85`) already decodes protobuf from
  the file's embedded schema through `mcap_protobuf`.
- **Web**: `@foxglove/ws-protocol`, JSON decode in `web/src/lib/connection.ts:215`. `protobufjs`
  (`^8.8.0`) already decodes `CompressedImage` and `CompressedVideo` from runtime descriptors
  (`web/src/lib/image.ts`).
- **viz_relay**: forwards bytes and never parses payloads, but advertises
  `supported_encodings = {"json"}` for client publishing (`src/viz_relay/main.cpp:238`).
- **Firmware**: packed structs in `espnow_protocol.h:34-133` with size `static_assert`s
  (`:162-169`). `TelemetryPayload` is 155 bytes. The host compiles that header
  (`CMakeLists.txt:268-269`) and memcpys payloads after an exact length check
  (`espnow_transmitter.cpp:259-266`).
- **Dongle diagnostics tool**: `firmware/espnow_dongle/scripts/espnow_diag.py` reads the dongle's
  serial stream and serves a live page. It holds the type IDs and payload sizes (`:35-57`) and
  decodes payloads with `struct.unpack`. `firmware/espnow_dongle/scripts/tests/test_espnow_diag.py` pins its constants to
  the header.
- **Dependencies**: no libprotobuf, protoc, or nanopb anywhere. The Foxglove C++ SDK encodes its
  own types in Rust and ships pre-serialized descriptors, so it brings no protobuf runtime. The
  venv has Python `protobuf 7.34.1` (its metadata says `Requires-Python >=3.10`) and
  `mcap-protobuf-support`, both transitive and unpinned. `onnx`, `onnxruntime`, and
  `mcap-protobuf-support` set lower bounds on it (4.25.x) and no upper bound.
- **Linking**: `auto_battlebot_lib` is a shared library (`CMakeLists.txt:271`). The dev box's
  OpenCV build (`/usr/local/lib/libopencv_dnn.so.410`) bundles its own protobuf. It exports 22
  symbols that mention `google::protobuf`; 19 are OpenCV and libstdc++ templates, and 3 are
  protobuf's own (`internal::byteswap<1>`, `<4>`, `<8>`).

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
| C++ app, tools, tests | libprotobuf, release pinned in `.protobuf-version` | `protoc` from the same build, CMake `protobuf_generate()` | build tree, not committed |
| Python | `protobuf` runtime pinned in `pyproject.toml` to the same release | same `protoc`, `--python_out --pyi_out` | committed under `auto_battlebot/msg/`, regenerated by `scripts/lint` |
| Web | `@bufbuild/protobuf` (protobuf-es v2) | `protoc-gen-es`, npm devDependency | committed under `web/src/generated/msg/`, regenerated by `scripts/lint` |
| Firmware (robot, dongle, native tests) | nanopb, exact version pinned in `lib_deps` | nanopb's PlatformIO integration (`custom_nanopb_protos`) | build tree |
| Schema checks | `buf` CLI, pinned | `buf lint`, `buf format`, `buf breaking --against .git#branch=main` | none |

- **libprotobuf release.** Pick the release that matches the venv's Python runtime: 7.34.1 is the
  Python side of protobuf release 34.1. The 7.34.1 wheel declares `Requires-Python >=3.10`, so
  it should install on JetPack 6. Gate T1 confirms that pairing and the install on the JetPack 6
  venv, where a platform-conditional dependency could still cap protobuf lower. If one does, pin
  the newest release every box accepts.
- **Install the C++ side the way the LLVM pin works.** `install/install_protobuf.sh` builds
  protobuf plus its bundled abseil from source into `/opt/protobuf-<version>` once per box, and
  every platform install calls it. `docker/playback.Dockerfile` picks its install scripts by
  hand, so it gets its own layer that runs the script. CMake finds the install with
  `find_package(Protobuf CONFIG)` and fails configure with the install command when it is
  missing.
  - Rejected, distro packages: 3.12 on 22.04 and 3.21 on 24.04 generate different code, the same
    problem `.llvm-version` exists to solve.
  - Rejected, FetchContent: adds protobuf plus abseil to every `clean_build.sh`, and clean builds
    are routine here.
- **Link rules.** `auto_battlebot_lib` is a shared library, and the app, the tests, and
  `remote_protocol_dump` all link it. A process must hold one libprotobuf and one copy of the
  generated code, or every descriptor registers twice. So:
  - The install builds protobuf and abseil as shared libraries
    (`protobuf_BUILD_SHARED_LIBS=ON`) under `/opt/protobuf-<version>/lib`, and CMake sets the
    rpath to it. A static libprotobuf linked `PUBLIC` through a shared library also lands on
    each executable's link line, which is how the second copy gets in.
  - `auto_battlebot_msg` is an object library compiled into `auto_battlebot_lib`. Nothing else
    compiles the generated sources.
  - `viz_relay` forwards bytes and does not link protobuf at all.
  - OpenCV's bundled protobuf stays out of the way: of the symbols `libopencv_dnn` exports, only
    `google::protobuf::internal::byteswap<1|4|8>` are protobuf's own. Gate T1 still runs the
    full test suite and one playback replay, since a hello round trip alone never loads both.
- **The web keeps `protobufjs` for `image.ts`.** That file decodes Foxglove's own types from the
  descriptors the server advertises. protobuf-es handles the `auto_battlebot.msg` types from
  generated code. Moving `image.ts` over is a separate change.
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

Foxglove, `mcap_protobuf`, and `foxglove_ws.py` (`:119-124`) all decode from it with no
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
  the TOML profile store), the `include/enums/` enums, and `DiagnosticsData`.

`include/remote/protocol.hpp` stays the topic table (topic, latch, rate) and its rows name the
generated types: `StatusTopic<msg::status::System>{"/status/system", Latch::NO, 10.0}`. Its
`static_assert`s stay. `remote_protocol_dump` shrinks to dumping that table (topic, type name,
latch, rate) for the web, which takes its types from `protoc-gen-es`.

### 6. Enums

- The `include/enums/` enum classes stay canonical for config parsing (`PARSE_ENUM`). Each one
  that appears in a message gets a proto mirror and a `static_assert`. Using magic_enum, the
  assert checks that both enums have the same names once the proto prefix is stripped and
  `UNSPECIFIED` is skipped, so a value added on one side fails the build. `to_msg`/`from_msg`
  map by name, never by cast: `UNSPECIFIED = 0` shifts the command enums' numbers by one.
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
  C++ default is not zero: `SetAutoTracking.enabled = true`, `SetOpponentCount.count = 1`, and
  seven controls in `SetCameraSettings.settings` (`auto_exposure`, `auto_white_balance`,
  `digital_gain`, `white_balance_k`, `saturation`, `gamma`, `sharpness` all default non-zero in
  `include/rgbd_camera/camera_settings.hpp`). The web's command builders set them explicitly,
  and the camera settings builder always sends the whole struct.
- Floats that are not `optional` carry NaN as NaN. Only the dict shim maps it to `None`.
- Host-side messages use `double` and the robot's messages use `float`, matching what each side
  holds today. No precision changes in recordings.

### 8. Evolution rules

These go in `proto/README.md` and `CLAUDE.md`, and `buf breaking` enforces them in lint:

- **Add a field**: take the next unused number. In `link.Telemetry` and its sub-messages, check
  the size budget first; the message has 8 bytes of room (see Link size budget).
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

ESP-NOW is live: `ESPNOW_ENABLED = true` (`firmware/mr_stabs_mk2/src/main.cpp:96`) and the robot
drives from it under the autonomy switch. This change swaps the payload of the drive path, so
the robot, the dongle, and the host change in one flash-and-deploy.

- **Mismatched builds fail closed.** An old robot ignores 0x21, so a new host sends commands
  that the robot never applies. An old host ignores 0x22, so it records no telemetry. Neither
  side misreads a struct as protobuf.
- **The new host says so.** The dongle forwards every frame, so a new host still receives an old
  robot's 0x02 telemetry and an old dongle's 0x04 stats. It counts frames on retired IDs and
  reports "robot firmware uses the retired link protocol, reflash it" (or the dongle) at ERROR
  in the ESP-NOW transmitter's diagnostics section and in the log. Without that, a mismatched
  pair looks like a dead link.
- **`AIR_MAGIC` keeps version byte `0x01`**, so the new host's `espnow_ota.py` can still flash a
  robot on old firmware. That robot is the one that needs flashing.
- **The old IDs stay in the enum as retired entries**, not as a comment: `MSG_RETIRED_01` to
  `MSG_RETIRED_04`. The names keep the numbers from being reused, and the host's mismatch report
  switches on them. Nothing encodes or decodes their payloads.
- **The dongle stays thin.** It stops checking `length != sizeof(CommandPayload)` for the
  priority slot (`firmware/espnow_dongle/src/main.cpp:177`) and checks the type byte instead. It
  forwards every payload untouched, and encodes only its own LinkStats with nanopb.

### 10. Hot paths

- **Main loop**: generated C++ messages allocate on first fill. Producers keep one message per
  channel and `Clear()` it each cycle; `Clear()` keeps the capacity. `OutputChannel` gets a
  `log_proto(const google::protobuf::Message&)` that serializes into its existing reused
  `buffer_`. Gate L1 checks loop time.
- **Diagnostics is the exception.** `Clear()` keeps capacity for scalar and repeated fields, but
  it frees a `map` field's nodes and their strings, and `Diagnostics` is two levels of maps
  rebuilt for every module every cycle. The diagnostics backend builds each message on a
  `google::protobuf::Arena` with a reused initial block and resets the arena after each publish,
  so a steady-state cycle does not call the allocator. Gate L1 covers Phase 3 for this reason.
- **Firmware**: nanopb encodes into a stack buffer with no heap.
  - The ring (`lib/telemetry_ring/telemetry_ring.h`) stores the nanopb-generated
    `auto_battlebot_msg_link_Telemetry` C struct. It is fixed-size because the message and its
    three sub-messages have no strings or repeated fields.
  - The ring encodes on HTTP serve, on the web server task, not in `loop()`.

### 11. Comments in `.proto` files

`CLAUDE.md` bans code comments, and `scripts/comment_budget.py` enforces a per-file budget. The
`.proto` files are the one place a field's units and meaning can sit next to the field for every
language at once: `protoc` copies them into the generated C++, Python stubs, and TypeScript. So
`.proto` field and message comments are exempt.

- `CLAUDE.md`, Comments: add a prose exception beside `DATASHEET:`. A comment in a `.proto`
  file may state a field's units, range, meaning, and when it is absent. It may not carry
  history, plan checkpoints, or the reason for a change; those rules are unchanged.
- `scripts/comment_budget.py` has no `.proto` entry in `SUFFIX_LANGUAGES`, so it skips these
  files today. Phase 1 makes that explicit with a test that a commented `.proto` passes `check`.
- The exemption stops at `.proto`. Committed generated files (`*_pb2.py`, `*_pb.ts`) carry a
  "Generated by" or `@generated` header in their first 3 lines and are skipped by the existing
  rule; `link.options` and `buf.yaml` get no comments.
- The `protocol.ts` special case at `comment_budget.py:450` goes when `protocol.ts` does
  (Phase 4).
- `espnow_protocol.h` is not exempt, which is why the retired type IDs are enum entries
  (decision 9).

## Message specs

Comments on each field carry the units and meaning that today live in the jsonschema
descriptions and struct comments (decision 11). Field numbers 1 to 15 encode in one byte; the
hot link fields take them.

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
}

message RobotTelemetry {
  link.Telemetry robot = 1;
  // Host wall clock when the frame arrived, ns; also the MCAP log time.
  int64 host_receive_ns = 2;
  TelemetryTransport transport = 3;
  // ESP-NOW only: host monotonic arrival minus robot.espnow.jetson_tx_us. Absent before the
  // first accepted command.
  optional int64 round_trip_us = 4;
}
```

`/robot/espnow_telemetry` keeps its name, and `EspNowTransmitter` writes this message.
What changes:

- Foxglove paths gain one level, or two for the grouped fields:
  `/robot/espnow_telemetry.vbat` becomes `/robot/espnow_telemetry.robot.vbat`, and
  `.accel_x` becomes `.robot.imu.accel_x`.
- Every leaf field of `link.Telemetry` keeps the name its JSON key has today
  (`espnow_active`, `imu_calib_sys`, `guard_scale`), and no two leaves share a name. Python
  loaders flatten the record by dropping the `robot`, `imu`, `guard`, and `espnow` levels, so old
  JSON files and new protobuf files give the same columns. A test checks that the leaf names are
  unique and equal the JSON key set.
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

// BNO055 readings. Every field number is below 16, so each tag is one byte.
message Imu {
  // Fused gravity vector.
  float accel_x = 1;
  float accel_y = 2;
  float accel_z = 3;
  // Euler angles, degrees.
  float orientation_x = 4;
  float orientation_y = 5;
  float orientation_z = 6;
  // Fused acceleration with gravity removed, m/s^2, chip frame.
  float linear_accel_x = 7;
  float linear_accel_y = 8;
  float linear_accel_z = 9;
  // Gyro rate about the chip axis, deg/s, right-handed.
  float gyro_x = 10;
  float gyro_y = 11;
  // CALIB_STAT, 0 uncalibrated to 3 fully calibrated. Split from the packed byte.
  // IMUPLUS mode reaches sys 3 without the magnetometer, and mag stays 0.
  uint32 imu_calib_sys = 12;
  uint32 imu_calib_gyro = 13;
  uint32 imu_calib_accel = 14;
  uint32 imu_calib_mag = 15;
}

// Motor stall guard (lib/motor_guard).
message MotorGuard {
  // Winding temperature rise above ambient from the thermal model, C.
  float motor_rise_left = 1;
  float motor_rise_right = 2;
  // Motor shaft speed, rad/s, signed.
  float motor_speed_left = 3;
  float motor_speed_right = 4;
  // Estimated winding current, A.
  float motor_amps_left = 5;
  float motor_amps_right = 6;
  // Command scale; 1 passes commands through, below 1 is limiting.
  float guard_scale = 7;
  // Shunt correction gain on the model current.
  float guard_gain = 8;
  // motor_guard::State: 0 OFF, 1 NOMINAL, 2 HOT, 3 LIMITING, 4 TRIPPED.
  uint32 guard_state = 9;
  // motor_guard::Sensors: 0 FUSED, 1 SHUNT_ONLY, 2 ERPM_ONLY, 3 NONE.
  uint32 guard_sensors = 10;
  // Bit 0 left motor stalled, bit 1 right.
  uint32 stall_flags = 11;
  // eRPM telemetry CRC errors, both ESCs summed since boot.
  uint32 erpm_crc_errors = 12;
}

// The robot's view of the ESP-NOW link.
message EspNowStatus {
  // ESP-NOW supplied this tick's drive command.
  bool espnow_active = 1;
  // Channel the robot listens on; 0 while sweeping.
  uint32 espnow_channel = 2;
  // Since the robot last accepted a command frame; absent before the first.
  optional uint32 espnow_age_ms = 3;
  // Last accepted Command.seq, and its jetson_tx_us echoed for round-trip measurement.
  fixed32 espnow_seq = 4;
  fixed32 jetson_tx_us = 5;
  // Telemetry send ACK success, percent over the last 32 frames.
  uint32 espnow_lq = 6;
}

// Type 0x22, robot to Jetson at 100 Hz over ESP-NOW; also the record in TelemetryBatch.
// Leaf field names are unique across this message and its sub-messages, and equal the JSON keys
// of the recordings made before protobuf.
message Telemetry {
  // Robot clock, ms.
  fixed32 timestamp_ms = 1;
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
  Imu imu = 14;
  MotorGuard guard = 15;
  EspNowStatus espnow = 16;
  bool armed = 17;
  bool radio_connected = 18;
  // ch5 (SA) as the robot saw it; gates espnow_active.
  bool autonomy_enabled = 19;
  bool button_state = 20;
  sint32 flip_switch = 21;
  bool is_upside_down = 22;
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

`Telemetry` is grouped into three sub-messages because a proto tag costs one byte only for field
numbers 1 to 15. Flat, 37 of the 52 fields would pay two, and the message would not fit one air
frame (see Link size budget). The groups follow the firmware modules that fill them.

`guard_state` and `guard_sensors` stay integers, not proto enums. Recordings hold them as
integers today, and the dict shim would turn an enum into a name.

`link.options` sets `int_size:IS_8` on the `imu_calib_*`, `espnow_channel`, `espnow_lq`,
`flip_switch`, `guard_state`, `guard_sensors`, and `stall_flags` fields, and `int_size:IS_16` on
`erpm_crc_errors`, so the generated C struct stays small. It also makes `TelemetryBatch.records`
a callback that encodes straight from the ring. Ping (0x05) has no payload and keeps none. The
OTA structs stay in `espnow_protocol.h`, unchanged.

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
`static_assert`s in `espnow_protocol.h:162-169` and the payload struct definitions. The worst
case uses the widths nanopb's `*_size` macros are expected to use: 4 bytes for a float or
fixed32, 1 for a bool, 5 for a `uint32`, 2 for an `IS_8` integer (128 to 255 take two varint
bytes), and 3 for an `IS_16` one. Gate F1 records the real macro values here.

| Message | Today, bytes | Protobuf worst case, bytes | Air frame worst (+5) | Limit |
| --- | --- | --- | --- | --- |
| Command | 16 | 20 | 25 | 250 |
| Telemetry | 155 | 237 | 242 | 250 |
| TunableSet | 5 | 7 | 12 | 250 |
| LinkStats | 18 | 32 | serial only, 38 | 261 serial |
| TelemetryBatch, 256 records | 32 + 39,680 | about 61,500 | HTTP | none |

Telemetry worst case adds up as follows:

- Top-level scalars, 86 bytes:
  - `timestamp_ms`, fixed32 with a one-byte tag: 5.
  - `loop_us` and `telemetry_seq`, `uint32` with one-byte tags: 12.
  - 10 floats with one-byte tags: 50.
  - 5 bools with two-byte tags: 15.
  - `flip_switch`, `IS_8` with a two-byte tag: 4.
- `imu`, 69 bytes: 11 floats (55) and 4 `IS_8` calibration levels (12), plus a tag and a length
  byte.
- `guard`, 55 bytes: 8 floats (40), 3 `IS_8` fields (9), and `erpm_crc_errors` (4), plus a tag
  and a length byte.
- `espnow`, 27 bytes: a bool (2), two `IS_8` fields (6), `espnow_age_ms` (6), and two fixed32
  (10), plus a two-byte tag and a length byte.

The same 52 fields laid out flat come to 263 bytes, 18 over the 245-byte `AIR_MAX_PAYLOAD`: 37
fields pay a two-byte tag. Grouping saves 26 bytes.

**Headroom is 8 bytes.** The packed struct has 90 bytes to spare today; protobuf spends one tag
byte per field, 52 in all, plus the sub-message framing. A new float in a sub-message costs 5
bytes, so one more fits and two do not. When the next field does not fit, the slow fields
(`imu_calib_*`, `erpm_crc_errors`, `guard_state`, `guard_sensors`, `espnow_channel`,
`espnow_lq`) move to a second message on a new type ID sent at 10 Hz. That split is not part of
this plan.

Typical frames are smaller, since proto3 omits zero and false fields. At the dongle's 6 Mbps PHY
rate, the extra 82 bytes cost at most 109 us of air time per telemetry frame, and 4 bytes cost
5 us per command. Per-frame MAC overhead is unchanged. nanopb's generated
`*_size` macros replace the hand `static_assert`s:
`static_assert(auto_battlebot_msg_link_Telemetry_size <= AIR_MAX_PAYLOAD)`.

**Ring memory.** The ring holds 9000 records (`TELEMETRY_RING_PSRAM_RECORDS`,
`firmware/mr_stabs_mk2/src/main.cpp:63`), 90 s at 100 Hz, in 1.40 MB of the QT Py's 2 MB PSRAM.
The generated C struct is larger than the packed 155 bytes: it adds a presence flag for each
`optional` field and each sub-message, and it is aligned. By hand count it comes to about 192
bytes, or about 166 with `packed_struct:true` in `link.options`.

| Ring record | Bytes | 9000 records | Records in today's 1.40 MB |
| --- | --- | --- | --- |
| `TelemetryPayload` today | 155 | 1.40 MB | 9000, 90 s |
| Generated struct, aligned | about 192 | 1.73 MB | about 7,260, 73 s |
| Generated struct, `packed_struct` | about 166 | 1.49 MB | about 8,400, 84 s |

Gate F4 measures `sizeof` and free PSRAM after the allocation. `packed_struct` is the first
choice, since `TelemetryPayload` is already packed and the ESP32-S3 reads it unaligned today.
`TELEMETRY_RING_PSRAM_RECORDS` then drops to whatever keeps free PSRAM where it is.

## Steps

Each phase lands on its own and leaves the tree working. Old recordings stay readable after
every phase.

**Reader check.** `iter_messages` already yields a `MessageBytes` tagged with the channel's
encoding and schema name, so `decode_dict` and the typed decoders can pick the decoder from the
payload. A reader that calls `decode_json(`, `decode_string(`, or `json.loads(` on a message
skips that tag and breaks on a protobuf file. Each phase below names the files it moves, without
line numbers, and is done when this prints nothing for that phase's topics:

```bash
grep -rIn "decode_json(\|decode_string(\|json\.loads(" --include=*.py \
  auto_battlebot scripts training simulation \
  | grep -v "^auto_battlebot/recording/mcap_io.py"
```

Hits that parse a file, not a message, stay. Today the command finds 25 `decode_json(` call
sites in 14 files, plus `json.loads(` on message bytes in at least `match_windows.py`,
`render_match_prediction_camera.py`, and `score_field_session.py`. Phase 1 classifies the
unchecked `json.loads(` hits in files that call `iter_messages`
(`auto_battlebot/nav/repro_1025.py`, `auto_battlebot/nav/step_test.py`,
`auto_battlebot/rl/lockstep_eval.py`, `scripts/calibration/render_mujoco_overlay.py`,
`auto_battlebot/calibration/sim_stop_test.py`, `auto_battlebot/perception/sysid_sessions.py`) and
adds each message reader to the phase that owns its topic.

### Phase 0: toolchain (gate T1)

1. `.protobuf-version`, `install/install_protobuf.sh`, called from every platform install and
   from its own layer in `docker/playback.Dockerfile`. It builds shared libraries (decision 2).
   Build with `-j6` under `systemd-run MemoryMax=16G` (see the build-job cap).
2. Pin `protobuf==<matching>` in `pyproject.toml`, keeping the platform-conditional deps as they
   are. Add `@bufbuild/protobuf` and `@bufbuild/protoc-gen-es` to `web/package.json`. Add the
   exact nanopb version to the robot, dongle, and native `lib_deps`.
3. A throwaway `proto/auto_battlebot/msg/hello.proto` compiled and round-tripped in C++, Python,
   TS, and the native firmware test env on every platform. The C++ side is compiled into
   `auto_battlebot_lib`, not a standalone binary, so the full test suite and a playback replay
   run with libprotobuf and OpenCV's dnn module loaded together.

### Phase 1: infrastructure plus FrameMeta end to end (gates R1, L1)

1. `proto/` tree, `proto/README.md` (evolution rules, the Telemetry size budget), `buf.yaml`.
   Add a `scripts/lint` step that runs `buf format`, `buf lint`, `buf breaking`, and regenerates
   Python and TS, failing when either is stale.
2. Comment policy (decision 11): the `.proto` exception in `CLAUDE.md`, and a
   `comment_budget.py` test that a commented `.proto` file passes `check`.
3. CMake: `protobuf_generate()` into an `auto_battlebot_msg` object library compiled into
   `auto_battlebot_lib` (decision 2).
4. C++: `OutputChannel::log_proto`; a `proto_schema<T>()` that builds the `FileDescriptorSet`.
   `VizSchema::protobuf(name, data)`.
5. Python: `McapWriter.log_proto(topic, message, ns)`; `mcap_io.decode_dict`; schema builder from
   `DESCRIPTOR`.
6. FrameMeta:
   - `to_frame_meta_json` becomes `to_frame_meta`.
   - `video_playback_camera.cpp` decodes both encodings.
   - `decode_frame_meta` handles both, returning `image_stamp_ns` as `int` in both cases.
   - The Python writers move over: `video_import.py`, `combine_mcap_svo.py`,
     `convert_ros1_mcap.py`.
   - The raw readers move to `decode_frame_meta` or `decode_dict`:
     `auto_battlebot/calibration/match/match_windows.py`,
     `playground/opponent_prediction/render_dropout_videos.py`,
     `auto_battlebot/calibration/match/render_match_prediction_camera.py`,
     `auto_battlebot/calibration/sysid/check_sysid_recordings.py`, `scripts/combine_mcap_svo.py`.

### Phase 2: the other recording topics (gate L1)

1. Detections: `to_detections_json`, `decode_detections`, `read_detections`, the raw reader in
   `render_dropout_videos.py`, and `CameraView.svelte`'s hand-written `Detections` interface
   (the web subscribes to `/keypoint_detections`).
2. AprilTagRobotTags: `AprilTagKeypointModel::to_json`, `sysid_io.parse_robot_tags`, and the
   writer in `calibration/sysid/synthetic.py`.
3. Calibration tool topics: the writers and readers in
   `auto_battlebot/calibration/apriltag/apriltag_mcap.py` (4 `log_json` calls, 3 `decode_json`
   calls), and `auto_battlebot/calibration/apriltag/analyze_apriltag_mcap.py`.

### Phase 3: diagnostics (gate D1 first, then L1)

1. Record one replay with protobuf diagnostics on a branch and run gate D1.
2. If it passes, make these changes:
   - `diagnostics_module_json` becomes `to_diagnostics_msg`, built on a reused arena
     (decision 10).
   - `decode_diagnostic_array` handles both encodings.
   - The raw readers move to `decode_dict`: `compare_replays.py`,
     `fit_opponent_sensing_model.py`, `plot_dropout_samples.py`, `fit_sensing_model.py`,
     `fit_provisional_ekf_error.py`.
   - The writers move over: `McapWriter.log_diagnostics` and `convert_ros1_mcap.py`.
   - The web's `diagnostics.svelte.ts` moves over.
   - Gate L1 runs on the Jetson with every diagnostics module publishing.
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
   - `comment_budget.py` loses its `protocol.ts` special case.
5. Python readers of `/status/tracks` move to `decode_dict`:
   `playground/opponent_prediction/compare_replays.py`,
   `auto_battlebot/rl/fit_sensing_model.py`, and `auto_battlebot/rl/score_field_session.py`
   (which calls `json.loads` on the bytes today). `field_outline` comes back as lists of points
   either way.

### Phase 5: ESP-NOW link and robot telemetry (gates F1 to F4)

ESP-NOW drives the robot today, so this phase starts with the F3 baseline on the current
firmware and ends with the robot, the dongle, and the host changed together.

1. Take the F3 baseline: `round_trip_us` p50 and p99 over a 5-minute bench session.
2. `link.proto` and `link.options`.
3. Firmware:
   - nanopb in the robot and dongle builds.
   - `publish_diag()` fills `auto_battlebot_msg_link_Telemetry` and its three sub-messages.
   - The ring stores the generated struct, with `TELEMETRY_RING_PSRAM_RECORDS` set by gate F4.
   - `/telemetry` becomes `/telemetry.pb`, encoding `TelemetryBatch` with the `records`
     callback.
   - The new type IDs go in, and 0x01 to 0x04 become `MSG_RETIRED_*` entries.
   - The packed payload structs are deleted from `espnow_protocol.h`, leaving the framing, the
     CRC, `SerialFrameParser`, and the OTA structs.
   - Native googletest env tests encode golden fixtures.
4. Dongle: priority slot keyed on type 0x21; LinkStats via nanopb.
5. Host:
   - `espnow_transmitter.cpp` decodes 0x22 and 0x24 with libprotobuf, sends 0x21, and records
     `RobotTelemetry`.
   - It counts frames on the retired IDs and reports the mismatch (decision 9).
   - `to_espnow_telemetry_json` is deleted.
6. Python:
   - `calibration/sysid/telemetry.py` takes its columns from the `link.Telemetry` descriptor
     plus the `RobotTelemetry` host fields, flattening the sub-messages by leaf name.
   - `sysid_io.load_esp32_diagnostics` and `calibration/sysid/sources.py` decode both encodings.
   - `calibration/sysid/synthetic.py` writes `RobotTelemetry`.
   - `firmware/espnow_dongle/scripts/espnow_diag.py` takes its type IDs from one table and
     decodes Command, Telemetry, TunableSet, and LinkStats with `auto_battlebot.msg.link`. Its
     payload-size table and `struct.unpack` decoders for those four go; the OTA ones stay.
     `firmware/espnow_dongle/scripts/tests/test_espnow_diag.py` checks the new IDs against the header.
7. Firmware, dongle, and host flash and deploy together. They fail closed if they are not, and
   the host reports which side is old.

### Phase 6: sim protocol (gate S1)

`simulation/protocol.py`, `kinematic_sim_server.py`, `rl_eval_server.py`, `sim_connection.cpp`,
and `auto_battlebot/kinematic_sim/tests/test_sim_protocol.py` move to length-prefixed `StepRequest`/`StepResponse`.

Today the client receives `rgb` and `depth` straight into the `cv::Mat` buffers
(`sim_connection.cpp:159-167`). As `bytes` fields they are copied once on serialize and once on
parse: 640x360 RGB plus float depth is 1.6 MB per tick on each side. Gate S1 checks the tick
rate for that reason.

### Phase 7: cleanup

1. Delete:
   - `include/foxglove_adapters/json_schemas.hpp`
   - the schema texts and `JSON_TOPIC_SCHEMAS` in `mcap_write.py`
   - `auto_battlebot/recording/tests/test_json_schema_parity.py`
   - the C++ `json_number`/`json_escape` helpers once nothing uses them
   - `McapWriter.log_json`, if nothing outside the legacy-read tests calls it
2. Rewrite `docs/reference/recording/foxglove_recording_format.md`:
   - the channel table and per-topic sections point at the `.proto` files
   - a "Reading older files" section lists the JSON layouts readers still accept
3. `CLAUDE.md`, Conventions:
   - add "Messages that cross a process, language, or device boundary are defined in `proto/`.
     Add a field with the next free number; mark deleted numbers `reserved`."
   - update the remote UI line to point at `proto/auto_battlebot/msg/status.proto`
4. Update the ESP-NOW protocol tables in `firmware/shared/espnow_protocol/`, and the payload
   descriptions in `firmware/mr_stabs_mk2/README.md`, `firmware/espnow_dongle/README.md`, and
   `docs/reference/robot_link/firmware_telemetry.md`.

## Gates

| Gate | Phase | Passes when |
| --- | --- | --- |
| T1 | 0 | The hello message round-trips in C++, Python, TS, and native firmware on dev x86 (22.04 and 24.04), the JP7 Orin NX, and the JP6 ZED Box Mini. With hello compiled into `auto_battlebot_lib`, `./scripts/build_and_test.sh` passes and one playback replay runs to the end. `ldd` on the app, the test binary, and `remote_protocol_dump` shows one libprotobuf, and `viz_relay` shows none. The Python runtime installs in the 3.10 and 3.12 venvs with the pin. The playback Docker image builds. The protobuf build time on the ZED Box Mini is recorded here |
| R1 | 1 | A new C++-written MCAP decodes in Python and in the Foxglove app. Every typed decoder returns identical results before and after the change on the NHRL May replay set and the four `nhrl_keypoints_eval_test` recordings |
| D1 | 3 | In Foxglove, a diagnostics value plots from a message path a person can type, e.g. `/diagnostics/nav.sections[:]{key=="nav"}.value.values[:]{key=="speed"}.value.number`. If that is the best Foxglove offers and it is unusable in practice, diagnostics stays JSON |
| W1 | 4 | On the ZED Box, every dashboard panel renders and every command is acked against the live app. A command with an `UNSPECIFIED` enum is refused. Open question 4 is answered before JSON command channels are removed |
| L1 | 1, 2, 3, 4 | Loop time p99 from `scripts/mcap_latency_report.py` on the Jetson regresses by no more than 0.5 ms against a baseline taken the same day on the same build box. For Phase 3 the run has every diagnostics module publishing |
| F1 | 5 | The nanopb `*_size` `static_assert`s hold, and the macro values are written into the Link size budget table. Golden bytes encoded by nanopb decode identically in libprotobuf and Python, and the reverse for Command |
| F2 | 5 | `pb_encode` of a full Telemetry on the ESP32-S3 takes under 100 us, measured with `esp_timer` over 10,000 frames |
| F3 | 5 | Over a 5-minute bench session, `round_trip_us` p50 and p99 are within 0.2 ms of the baseline taken from the current firmware in Phase 5 step 1. With an old robot and a new host, and with an old dongle and a new host, the host reports the mismatch within 1 s |
| F4 | 5 | `sizeof(auto_battlebot_msg_link_Telemetry)` is recorded in the Ring memory table. Free PSRAM after the ring allocation is at least what it is today, with `TELEMETRY_RING_PSRAM_RECORDS` cut to match |
| S1 | 6 | The lockstep smoke test (Mr Stabs Mk2, MotionProfile, 450 ticks, `config/simulation/mr_stabs_mk2_kinematic_sim_sensing.toml`) gives the same result as before: the same tick count to first command and the same final distance. Its wall-clock tick rate is within 5% of a baseline taken the same day |

## Tests

- **Golden fixtures** under `tests/fixtures/msg/`, one encoded message per type, checked from
  C++, Python, and (for `link.proto`) the native firmware env. A fixture change shows up in the
  diff.
- **Descriptor parity**: C++ and Python build the same `FileDescriptorSet` for every message.
  This replaces `test_json_schema_parity.py`.
- **Legacy decode**: the existing JSON-text tests (`test_detections.cpp`, `test_camera_info.cpp`,
  `test_foxglove_diagnostics_backend.cpp`, `test_espnow_transmitter.cpp`,
  `test_video_playback_camera.cpp`,
  `test_status_bus.cpp`) become protobuf tests. A JSON message from each kept as a fixture
  checks that the readers still accept old files.
- **`decode_dict`**: for each topic, the JSON and protobuf forms of the same message give equal
  dicts. That covers NaN, absent optionals, 64-bit ints, and enums.
- **Enum mirrors**: compile-time, through the magic_enum name `static_assert`s.
- **Command rejection**: `UNSPECIFIED` enums, a `SetManualTarget` with no point, and an unknown
  topic.
- **Telemetry leaf names**: the leaf field names of `link.Telemetry` are unique and equal the
  key set of a JSON `/robot/espnow_telemetry` fixture, minus the host fields.
- **Link mismatch**: a frame on a retired type ID raises the host's mismatch report and is
  never decoded.
- **`espnow_diag.py`**: decodes the `link.proto` golden fixtures, and its type IDs match the
  header.
- **Comment policy**: a commented `.proto` file passes `comment_budget.py check`.

## Risks

- **Python protobuf pin on JetPack 6.** The 7.34.1 wheel supports Python 3.10 by its metadata.
  What is left is a platform-conditional dependency on the JetPack 6 venv that caps protobuf
  lower; the dev venv's dependents set lower bounds only. T1 finds out before anything else
  depends on it.
- **Telemetry has 8 bytes of headroom.** The stall guard added 37 packed bytes in the month after
  this plan was written. One more float fits; the one after that needs the slow-field split
  described under Link size budget. The `static_assert` on the nanopb size fails the firmware
  build when a field does not fit, so it cannot ship by accident.
- **Two protobuf copies in one process.** OpenCV's dnn module bundles its own. Its exported
  symbols do not overlap libprotobuf's in any way that matters on the dev box, but the Jetson
  OpenCV builds are different binaries. T1 runs the test suite and a replay on each box.
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
- **Mixed robot and host builds during Phase 5** fail closed. ESP-NOW is the live drive path, so
  a field session with a mismatched pair has no autonomy until both are reflashed. The new host
  names the old side in diagnostics and the log (decision 9); an old host with new firmware shows
  only `LinkStats.robot_known` with no telemetry.
- **Foxglove paths change.** The telemetry topics gain `.robot.` (and `.imu.`, `.guard.`, or
  `.espnow.` for the grouped fields), and the schema names gain `.msg.`. There are no committed
  layouts. Paths quoted in prose under `docs/` and in script help text get updated in the phase
  that changes them; find them by grepping for the topic name.

## Open questions

1. Should `/robot/esp32_diagnostics` keep its name, or become `/robot/wifi_telemetry` now that it
   carries `RobotTelemetry`? This plan keeps the name.
2. Is dropping `wifi_clients` from `/robot/esp32_diagnostics` fine? Only the legacy CSV path ever
   filled it.
3. Should Phase 6 (sim) wait until the nav v2 work in `nav_scoring_and_rl_v2_plan.md` is not
   using the lockstep server daily?
4. Does anyone send commands from the Foxglove app's Publish panel? `protocol.hpp` says a
   Foxglove panel can use them. If so, check that the panel can publish to a protobuf channel
   before Phase 4 drops JSON from `supported_encodings`; if it cannot, the relay keeps JSON
   command channels and `decode_command` keeps a JSON path.

## Next steps

1. Answer the open questions.
2. Run Phase 0 and gate T1, starting with the protobuf install on the ZED Box Mini and the
   JetPack 6 venv, since those two decide the release for everything else.
3. Phase 1 with FrameMeta and the `.proto` comment exception, then gate R1 on the NHRL May replay
   set.
