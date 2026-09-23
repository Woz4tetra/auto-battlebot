# Web UI plan

Goal: a browser UI for the ZED Box, with no screen on the robot system. At events an iPad
connects over a wired USB-C Ethernet link. During testing an Android phone opens the same page
over the shop Wi-Fi, so the iPad can stay put away. The page is a Svelte app that is a Foxglove
WebSocket client of `viz_relay`. It subscribes to status topics and publishes commands, and it
has a wide layout for the tablet and a narrow one for the phone. `viz_relay` also serves the page
over HTTP from an embedded Crow server.

The backend changes make the remote protocol one table: every status and command topic maps to
a C++ struct, and that table drives JSON encoding, JSON Schema, MCAP recording, command dispatch,
and the generated TypeScript types. Adding a command or a status field touches one struct, one
table row, and one handler, and the compiler flags any handler you forget.

Written 2026-09-22 against `benw/jetpack-7` at `9e3f5b7f`.

## What exists

| Piece | State | Where |
| --- | --- | --- |
| Relay | Owns `ws://0.0.0.0:8765`, advertises `clientPublish` with `json` encoding, forwards client messages to the app as `CLIENT_MESSAGE` frames | `src/viz_relay/main.cpp` |
| App sink | Non-blocking unix socket client; `set_client_message_handler` receives client messages on the sink thread | `include/viz/viz_sink.hpp` |
| Fan-out | `OutputChannel` encodes once, sends to relay and MCAP | `include/publisher/output_channel.hpp` |
| Remote commands | `enum class RemoteCommand { REINIT_FIELD }`, topic `/command/<lowercase name>`, payload ignored | `include/enums/remote_command.hpp`, `src/remote_command.cpp` |
| Command handling | `Runner::post_remote_command` queues; `handle_remote_commands` switches on the enum | `src/runner.cpp:163` |
| LVGL commands | Six separate request atomics on `UIState` (`reinit_requested`, `opponent_count_requested`, `autonomy_toggle_requested`, `recording_toggle_requested`, `system_action_requested`, `requested_profile`), each with its own `Runner::handle_*_request` | `include/ui/ui_state.hpp:52`, `src/runner.cpp:83-200` |
| Status | `SystemStatus` built in `Runner::publish_system_status`, which returns early when `ui_state_` is null, so a headless box publishes no status | `src/runner.cpp:52` |
| JSON channels | Hand-written JSON strings and schema literals | `include/foxglove_adapters/json_schemas.hpp` |
| Camera image | `/camera/image` is full-resolution JPEG `foxglove.CompressedImage` from `cv::imencode`, run inline on the perception loop when a client subscribes or the MCAP recorder writes the topic. `_common.toml` ignores it in MCAP, so normally only subscribers trigger it. The code comment puts it at about 10 ms on the Jetson | `src/foxglove_adapters/image.cpp`, `src/publisher/foxglove_publisher.cpp:86` |
| H.264 | `VideoEncoder` runs FFmpeg on its own thread and picks `h264_nvenc`, then `h264_nvv4l2m2m`, then `libx264`. Only `V4l2RgbCamera` uses it, for recorded `/camera/video`. The ZED path records SVO2 through the ZED SDK and publishes no `/camera/video` | `include/rgbd_camera/video_encoder.hpp`, `src/rgbd_camera/v4l2_rgb_camera.cpp:276` |

Two problems this plan fixes on the way:

- Commands have two paths. LVGL writes atomics, and Foxglove goes through `RemoteCommand`. A new
  command means a new atomic, a new `handle_*_request`, a new enum value, and a new UI hook.
- Status only exists when the LVGL UI is on. The ZED Box runs with `[ui] enable = false`, so
  today it would publish nothing a remote UI could show.

## Architecture

```
iPad Safari (cable) / Android Chrome (Wi-Fi)
  |  HTTP  :80    -> Crow in viz_relay -> web/dist/ (index.html, hashed assets)
  |  WS    :8765  -> foxglove::WebSocketServer in viz_relay
                       |  subscribe /status/*, /camera/preview(_video), /keypoint_detections, /diagnostics/*
                       |  clientPublish /command/*
                       v
                  unix socket (viz frames, unchanged)
                       v
auto_battlebot
  VizSink  --CLIENT_MESSAGE-->  decode_command()  -->  CommandQueue  <--  LVGL UI (desktop)
                                                          |
                                                    Runner::handle_commands()  (std::visit)
                                                          |
  StatusBus.publish(SystemStatusMessage{...})  -->  OutputChannel  -->  relay + MCAP
```

Both servers live in `viz_relay` because it outlives app restarts. If the app crashes, the page
still loads, still connects, and shows "app disconnected" from the missing status stream.

## Backend

### Remote protocol table

New directory `include/remote/`, `src/remote/`, `tests/remote/`.

`include/remote/protocol.hpp` is the one place that maps topics to types:

```cpp
namespace auto_battlebot::remote {

enum class Latch : bool { NO = false, YES = true };

/** App -> clients. `max_rate_hz` throttles StatusBus; 0 publishes every call. */
template <typename Message>
struct StatusTopic {
    using message_type = Message;
    std::string_view topic;
    Latch latch;
    double max_rate_hz;
};

/** Clients -> app. The payload struct is also the command's type in RemoteCommand. */
template <typename Payload>
struct CommandTopic {
    using payload_type = Payload;
    std::string_view topic;
};

struct RemoteTopics {
    static constexpr auto status = std::tuple{
        StatusTopic<SystemStatusMessage>{"/status/system", Latch::NO, 10.0},
        StatusTopic<AppInfoMessage>{"/status/app", Latch::YES, 0.0},
        StatusTopic<SticksMessage>{"/status/sticks", Latch::NO, 20.0},
        StatusTopic<TracksMessage>{"/status/tracks", Latch::NO, 20.0},
        StatusTopic<CommandAckMessage>{"/status/command_ack", Latch::NO, 0.0},
        StatusTopic<NetworkMessage>{"/status/network", Latch::YES, 1.0},
    };
    static constexpr auto commands = std::tuple{
        CommandTopic<ReinitFieldCommand>{"/command/reinit_field"},
        CommandTopic<SetOpponentCountCommand>{"/command/set_opponent_count"},
        CommandTopic<SetAutonomyCommand>{"/command/set_autonomy"},
        CommandTopic<SetRecordingCommand>{"/command/set_recording"},
        CommandTopic<SelectProfileCommand>{"/command/select_profile"},
        CommandTopic<SystemActionCommand>{"/command/system_action"},
        CommandTopic<SetWifiAccessCommand>{"/command/set_wifi_access"},
    };
};

/** One alternative per command payload, derived from RemoteTopics::commands. */
using RemoteCommand = command_variant_t<decltype(RemoteTopics::commands)>;

}  // namespace auto_battlebot::remote
```

Rules the header enforces with `static_assert`:

- Every topic string is unique across both tuples.
- Every message and payload type appears once, so code can publish or handle by type alone
  (`status_bus.publish(SystemStatusMessage{...})`) and never repeat the topic string.
- Status topics start with `/status/`, command topics with `/command/`.

`RemoteCommand` replaces `enum class RemoteCommand`. `include/enums/remote_command.hpp`,
`include/remote_command.hpp`, `src/remote_command.cpp`, and `tests/test_remote_command.cpp` are
deleted. `/command/reinit_field` keeps its topic, and a `{}` payload still works, so existing
Foxglove layouts with a publish button keep working.

### Messages

`include/remote/messages.hpp` holds the structs. Each struct declares its fields once with a
macro that produces the nlohmann (de)serializers and the JSON Schema description together:

```cpp
struct SetOpponentCountCommand {
    int count = 1;
    AB_JSON_MESSAGE(SetOpponentCountCommand, "auto_battlebot.command.SetOpponentCount", count)
};

struct SystemActionCommand {
    SystemAction action = SystemAction::REBOOT_HOST;
    AB_JSON_MESSAGE(SystemActionCommand, "auto_battlebot.command.SystemAction", action)
};

struct ReinitFieldCommand {
    AB_JSON_EMPTY_MESSAGE(ReinitFieldCommand, "auto_battlebot.command.ReinitField")
};
```

`AB_JSON_MESSAGE(Type, schema_name, fields...)` expands to:

- `static constexpr std::string_view kSchemaName = schema_name;`
- `NLOHMANN_DEFINE_TYPE_INTRUSIVE(Type, fields...)`. Missing fields throw on parse, so a command
  with no `count` is rejected instead of silently using the default.
- `static void describe(SchemaFields& f)`, which calls `f.add<decltype(field)>("field")` per
  field through `NLOHMANN_JSON_PASTE`. `decltype` of a member name is legal in an unevaluated
  context inside a static member function, so the per-field macro needs no type argument.

`AB_JSON_EMPTY_MESSAGE` exists because `NLOHMANN_DEFINE_TYPE_INTRUSIVE` needs at least one
field. It writes `{}` and accepts any object.

`src/remote/json_schema.cpp` builds the schema from `describe()`:

| C++ type | JSON Schema |
| --- | --- |
| `bool` | `boolean` |
| integral (not `uint64_t`) | `integer` |
| floating point | `number` |
| `std::string` | `string` |
| enum | `string` with `enum` = lowercase `magic_enum` names |
| `std::vector<T>` | `array` of `T` |
| `std::optional<T>` | `T`, left out of `required` |
| struct with `describe` | nested `object` |

`uint64_t` fails a `static_assert`. Stamps above 2^53 go as strings, like `FrameMeta`, and the
builder forces you to decide.

Enums serialize as lowercase names. `AB_JSON_ENUM(SystemAction)` next to the enum defines
non-template `to_json`/`from_json` through `magic_enum`, which beat nlohmann's integer enum
overloads in overload resolution. Enums live in `include/enums/` per the repo convention.
`UISystemAction` moves there as `SystemAction { REBOOT_HOST, POWEROFF_HOST }`. `NONE` goes away
because a command either exists or does not.

New dependency: `nlohmann/json` v3.11.3 by `FetchContent`, linked to `auto_battlebot_lib`. The
existing hand-written JSON channels (`/blob_detections`, `/keypoint_detections`,
`/camera/frame_meta`, `/diagnostics/*`) are left alone. Their schemas have byte-identical copies
in `auto_battlebot/mcap_write.py`, and porting them is separate work.

### Status messages

Fields come from what LVGL shows today. Rows in this table become rows in
`docs/foxglove_recording_format.md`.

| Topic | Type | Rate | Fields |
| --- | --- | --- | --- |
| `/status/system` | `SystemStatusMessage` | 10 Hz | `camera_ok`, `transmitter_connected`, `transmitter_receiving`, `loop_rate_hz`, `initialized`, `selected_opponent_count`, `autonomy_enabled`, `svo_recording`, `mcap_recording`, `jetson_temperature_c` (optional), `compute_mode` |
| `/status/app` | `AppInfoMessage` | latched, on change | `available_profiles`, `current_profile`, `max_loop_rate_hz`, `rate_fail_threshold`, `rate_fail_duration_sec` |
| `/status/sticks` | `SticksMessage` | 20 Hz | `linear`, `angular` for our robot, normalized [-1, 1] |
| `/status/tracks` | `TracksMessage` | 20 Hz | `our_robot_seen`, `opponents_seen`, and per robot `label`, `x`, `y`, `yaw` in the field frame |
| `/status/command_ack` | `CommandAckMessage` | per command | `seq`, `topic`, `accepted`, `message` |
| `/status/network` | `NetworkMessage` | latched, at most 1 Hz | `hostname`, `cable_address`, `wifi_interface`, `wifi_address` (optional), `wifi_access` |

`CommandAckMessage` replaces `UIState::profile_notice`. The profile switch acks with
`"Selected X. Reboot to apply."`, and the page shows the message of any ack with one.
`seq` increases per ack, and the page matches acks by `topic` within a 1 s window. A command
with no ack in that window shows "not delivered", which covers the relay dropping a message
while the app is down.

`/status/network` comes from `HostServices` (see "Wi-Fi access" below), which reads addresses
with `getifaddrs()` every 5 s rather than spawning `nmcli`. The System tab shows it, and the QR
code for the phone encodes `http://<wifi_address>` from it.

Battery is not in the first cut. The SOC estimator is polled from LVGL code
(`src/ui/lvgl_platform_bound/lvgl_ui_battery.cpp`), and the ZED Box may have no Waveshare UPS.
If it needs one, the estimator moves to a service owned by `main.cpp` and publishes
`/status/battery` at 1 Hz.

### StatusBus

`include/remote/status_bus.hpp`:

```cpp
class StatusBus {
   public:
    StatusBus(std::shared_ptr<VizSink> sink, std::shared_ptr<McapRecorder> recorder);

    /** Serializes and publishes on the topic RemoteTopics maps to Message. Drops the call when
     *  the topic's max_rate_hz says it is too soon. Never blocks. */
    template <typename Message>
    void publish(const Message& message);
};
```

- The constructor walks `RemoteTopics::status` and builds one `OutputChannel` per topic with
  `VizSchema::jsonschema(kSchemaName, schema_for<Message>())`. Every status channel is
  advertised at startup, so a client sees the full list before the first message.
- Throttling uses `std::chrono::steady_clock`. This is UI pacing and never feeds control, so it
  does not need the `ClockInterface` playback clock.
- Status topics are recorded to MCAP, so a recording shows what the driver saw.
- Cost per publish: one small `nlohmann::json` dump (tens of microseconds) and a queue copy in
  `VizSink::publish`, which never blocks.

`main.cpp` constructs it after `viz_sink` and `mcap_recorder` and passes it to `Runner`.
`Runner::publish_system_status` builds the status every tick whether or not `ui_state_` exists,
sets it on `ui_state_` when present, and calls `status_bus_->publish(...)`. `/status/sticks` is
published where `set_command_feedback` is called (`src/runner.cpp:537`), and `/status/tracks`
where robots reach `ui_state_`.

### Commands: one queue, one dispatch

`include/remote/command_queue.hpp` is a mutex and a vector with `post(RemoteCommand)` and
`drain()`. `main.cpp` owns it and hands it to three places:

1. The `VizSink` client message handler, through `decode_command`.
2. `UIState`, so LVGL tiles post commands.
3. `Runner`, which drains it at the start of each tick.

`src/remote/command_decode.cpp`:

```cpp
/** Matches the topic against RemoteTopics::commands and parses the payload into that command's
 *  struct. Unknown topics warn with the valid list; bad JSON or missing fields warn with the
 *  schema name and the parse error. Never throws. */
std::optional<RemoteCommand> decode_command(std::string_view topic, const std::byte* data,
                                            size_t len);
```

`Runner::handle_commands` replaces `handle_remote_commands` and the five `handle_*_request`
functions:

```cpp
for (auto& command : command_queue_->drain()) {
    std::visit(overloaded{
        [&](const ReinitFieldCommand&) { should_reinit_field = true; },
        [&](const SetOpponentCountCommand& c) { set_opponent_count(c.count); },
        [&](const SetAutonomyCommand& c) { set_autonomy(c.enabled); },
        [&](const SetRecordingCommand& c) { set_recording(c.enabled); },
        [&](const SelectProfileCommand& c) { select_profile(c.name); },
        [&](const SystemActionCommand& c) { keep_running = run_system_action(c.action); },
        [&](const SetWifiAccessCommand& c) { host_services_->set_wifi_access(c.enabled); },
    }, command);
    command_log_.record(command);
}
```

There is no `auto` catch-all overload. A new alternative in `RemoteCommand` without a handler
fails to compile, which is the same guarantee `-Wswitch` gave the old enum.

Every drained command goes through `command_log_` into MCAP on its own `/command/*` topic with
the command's schema, recorder only and no relay sink, so the app never advertises a topic
the relay already has as a client channel. After a match you can see who pressed reboot and
when. Each handler posts a `CommandAckMessage`.

Toggles become explicit sets. `autonomy_toggle_requested` (+1/-1/0) and
`recording_toggle_requested` become `SetAutonomyCommand{enabled}` and
`SetRecordingCommand{enabled}`. An explicit set is safe to resend when the iPad retries after
a reconnect, and a toggle would flip twice. LVGL tiles read the current `SystemStatus` and post
the negation.

`UIState` loses the six request members. `quit_requested` stays: it is window-close for the
local process, not a remote command. `manual_target` stays: it is a continuous press-and-hold
stream, not a discrete command, and the web UI does not do it in this plan.

### Wi-Fi access

The dashboard is cable-only by default: the firewall table (see "Dashboard URL") accepts the
dashboard ports on loopback and the Ethernet port only. `SetWifiAccessCommand{enabled}` opens or
closes them on the Wi-Fi interface, so the phone can reach the box during testing.

- `include/host/host_services.hpp`: `HostServices` owns one worker thread. `set_wifi_access`
  queues the request and returns, and the worker runs
  `systemctl start|stop auto-battlebot-dashboard-wifi.service`. The perception loop never waits
  on a subprocess.
- The app user may start and stop that one unit through a polkit rule the install script writes.
  The app never gets root and never runs `nft` itself.
- The setting persists across reboots: the worker writes it to
  `$HOME/.local/state/auto_battlebot/wifi_access` and applies it again at startup. State lives
  under `$HOME`, not `config/`, because a deploy overwrites the repo tree. Persisting is the
  point: in the shop the phone has to work after a reboot without getting the iPad out. The cost
  is that it stays open at an event until someone turns it off, so both layouts show an amber
  `WI-FI` tag in the header whenever it is on.
- The worker also refreshes `/status/network` after each change.
- Any client can send the command. Turning it on from Wi-Fi can't happen, since Wi-Fi clients are
  refused while it is off. Turning it off from Wi-Fi works, and disconnects that client.

### Adding a command after this lands

1. Add the payload struct with `AB_JSON_MESSAGE` to `include/remote/messages.hpp`.
2. Add one `CommandTopic<...>` row to `RemoteTopics::commands`.
3. Build. `Runner::handle_commands` fails to compile until you add the overload.
4. Run `./scripts/lint`, which regenerates `web/src/generated/protocol.ts`. The page can now
   call `send("/command/<name>", {...})` with the payload type-checked.

Adding a status message is the same, with `StatusTopic` and a `status_bus_->publish(...)` call at
the producer.

### Camera preview and hardware encoding

`/camera/image` costs about 10 ms of CPU on the perception loop whenever any client subscribes,
which is a sixth of the 60 ms budget. The iPad makes that the normal case instead of a
debugging case. Two changes fix it: encoding moves off the loop onto worker threads, and
the Orin NX's fixed-function engines do the encoding. NVJPG handles JPEG and NVENC handles
H.264. The threads remove the latency, and the engines remove the CPU load. Both are needed:
a worker thread running `cv::imencode` still takes CPU from the perception threads.

The Orin NX has both engines. The Orin Nano has no NVENC, so the fallbacks below are not only
for the desktop.

#### Encoder threads

`include/viz/image_encoder_worker.hpp`, modeled on `VideoEncoder`:

- `submit(const cv::Mat&, uint64_t log_time_ns)` never blocks. It keeps one pending frame and
  replaces it when a newer one arrives. The preview wants the newest frame, not every frame.
- The worker encodes and calls a sink that logs to the `OutputChannel`.
- It counts `encoded_frames`, `replaced_frames`, and `encode_ms` (last and mean) for the
  diagnostics module.
- The loop's remaining cost is one clone of the source frame, or one resize to the preview size
  before cloning, which copies less.

`FoxglovePublisher::publish_camera_data` submits to workers instead of calling
`to_compressed_image`. The workers are `/camera/image` at full resolution and `/camera/preview`
at preview size, each started lazily when the first client subscribes.

#### NVJPG for JPEG

`include/viz/jpeg_encoder.hpp`:

```cpp
class JpegEncoderInterface {
   public:
    virtual ~JpegEncoderInterface() = default;
    /** BGR in, JPEG bytes out. Called only from the worker thread. */
    virtual bool encode(const cv::Mat& bgr, int quality, std::vector<std::byte>& out) = 0;
    virtual std::string_view name() const = 0;
};
std::unique_ptr<JpegEncoderInterface> make_jpeg_encoder();  // NVJPG, then OpenCV
```

- `NvjpgJpegEncoder` uses `NvJPEGEncoder` from the Jetson Multimedia API
  (`/usr/src/jetson_multimedia_api`, `libnvjpeg.so` under `/usr/lib/aarch64-linux-gnu/nvidia`).
  It takes YUV 4:2:0, so the worker converts BGR to I420 first. That is `cv::cvtColor` on the
  CPU, about 0.3 ms at 640x360. At full resolution it moves to VPI or `NvBufSurfTransform` on
  the VIC, but only if the measurement below says the conversion matters.
- `OpenCvJpegEncoder` wraps `cv::imencode` and is the fallback on the desktop and wherever
  NVJPG fails to open.
- Selection works the same as `VideoEncoder`: try the hardware encoder, fall back, and log which
  one started. There is no config field. On the one platform with NVJPG, the hardware encoder is
  always the right choice, and a flag would only add a way to pick the wrong one.
- CMake: `find_path` for the Multimedia API headers. When found, `NvjpgJpegEncoder` compiles in
  and links `nvjpeg` from the L4T library directory. Otherwise it is left out. This follows how
  `BUILD_WITH_ZED` tracks whether the SDK is installed.
- The `libnvjpeg.so` in L4T is NVIDIA's libjpeg-compatible library with the NVJPG extensions.
  It is not the CUDA toolkit's nvJPEG, and the two must not both land on the link line. The CUDA
  nvJPEG does encode on the GPU, but the GPU is the resource the detectors need, so it is not an
  option here.

`/field_mask` goes through the same encoder. It is published per field fit, not per frame, so
it is not urgent.

#### NVENC for the preview stream

JPEG at 10 Hz is enough to aim at the field and confirm tracking, and it is the first cut. A
smoother preview uses H.264:

- New channel `/camera/preview_video`, `foxglove.CompressedVideo`, from a second `VideoEncoder`
  instance: `preview_width` wide, 30 fps, 1.5 Mbps, keyframe every 30 frames.
  `input_is_uyvy = false`, because it takes the BGR frame the loop already has.
- The Orin NX NVENC handles the recording stream and a 640-wide stream at the same time. The
  recording encoder is only in use on the V4L2 camera, and the ZED SDK uses its own session for
  SVO2.
- A new subscriber cannot decode until the next IDR. `VideoEncoder` gains
  `request_keyframe()`, which sets `pict_type = AV_PICTURE_TYPE_I` on the next frame.
  `FoxglovePublisher` calls it when the channel's subscriber count goes from 0 to 1, so the
  iPad shows video within one frame instead of up to one second.
- The page decodes with WebCodecs `VideoDecoder` (`avc1` codec string built from the SPS),
  which iPadOS supports from 16.4. When `VideoDecoder` is missing, or `/camera/preview_video` is
  not advertised, the page falls back to `/camera/preview` JPEG.
- Encoder selection must be checked on JetPack 7. Stock Ubuntu 24.04 FFmpeg has no
  `h264_nvv4l2m2m`; that encoder comes from NVIDIA's patched FFmpeg. On the box, run
  `ffmpeg -hide_banner -encoders | grep -E 'nvv4l2|v4l2m2m|nvenc|x264'` and read which codec
  `VideoEncoder` logs at startup. If it falls through to `libx264`, the preview stream stays off,
  because software H.264 on the Orin CPU is worse than NVJPG. The fix then is a
  `NvVideoEncoder` (Multimedia API) backend next to FFmpeg, or NVIDIA's FFmpeg build in the
  install scripts. Pick one after checking what JetPack 7 ships.

#### Config and scope

- `FoxglovePublisherConfiguration` gains `preview_width` (default 640), `preview_rate_hz`
  (default 10, JPEG only), `preview_jpeg_quality` (default 70), and `preview_video_bitrate_kbps`
  (default 1500).
- Both preview channels are live only and never recorded. `/camera/video` and SVO2 already
  record the frames.
- Nothing encodes unless someone subscribes. `has_consumers()` gates the workers, the same as
  `/camera/image` today.

#### Measurements that gate merging

Run on the ZED Box with the playback config, with a diagnostics recording for each case:

| Case | Pass |
| --- | --- |
| No clients | Loop rate matches the rate before this change |
| iPad on `/camera/preview` JPEG | Loop rate within 1 Hz of no clients; NVJPG `encode_ms` under 2 ms at 640 wide |
| Foxglove on `/camera/image` at full resolution | Loop rate within 1 Hz of no clients (today it costs about 10 ms per frame) |
| iPad on `/camera/preview_video` | Loop rate within 1 Hz of no clients; `VideoEncoder` codec is not `libx264`; time from subscribe to first decoded frame under 100 ms |

Report the numbers in the commit message for each step.

The page draws `/keypoint_detections` boxes over whichever preview it shows, scaling from the
message's `w` and `h`.

### viz_relay: Crow HTTP server

Crow runs inside `viz_relay` next to the Foxglove server.

- Flags: `--http-port` (default 8080, 0 disables) and `--web-root`. The default web root is the
  first of `<exe_dir>/web` (installed) and `<exe_dir>/../web/dist` (the `build/` tree in the
  repo) that exists. The dev box needs no copy step.
- Routes:
  - `/` serves `index.html` with `Cache-Control: no-cache`.
  - `/assets/<path>` serves Vite's hashed files with
    `Cache-Control: public, max-age=31536000, immutable`.
  - `/manifest.webmanifest` and icons are served from the root.
  - `/healthz` returns `{"app_connected": bool}` as JSON.
- Path resolution goes through `resolve_static_path(root, url_path)` in
  `include/viz/static_files.hpp`. It rejects `..` segments and anything that resolves outside the
  root, and it is unit-tested apart from Crow.
- When the web root is missing, `/` returns 503 with one line saying to run `scripts/build_web.sh`.
- Threads: `app.bindaddr(host).port(http_port).concurrency(2).run_async()`, stopped in the
  existing shutdown path next to `server_->stop()`. Crow's logger is routed to spdlog at warning
  level.
- HTTP and WebSocket share `--host`, which stays `0.0.0.0` everywhere. Binding to the Ethernet
  address instead would fail at boot whenever the cable is unplugged, because the address only
  exists with carrier. On the ZED Box, a firewall table limits the dashboard ports to the
  Ethernet port instead (see "Dashboard URL" below).
- Port 80 on the ZED Box, set by the install script's `viz_relay.service` drop-in. The dev box
  keeps the default 8080, which needs no capability.

Dependencies: Crow v1.2.x and standalone Asio by `FetchContent`, linked to `viz_relay` only.
The app binary does not get Crow. How Crow's CMake finds Asio is unverified. If it wants
`find_package(asio)`, add a small INTERFACE target for the fetched headers, or install
`libasio-dev` from the platform install scripts.

## Frontend

### Stack

- Svelte 5 with TypeScript, built by Vite. It is a single-page app and does not use SvelteKit:
  there is no routing or server rendering to justify it.
- `@foxglove/ws-protocol` for the WebSocket client.
- `qrcode-generator` (no dependencies) for the phone QR code on the System tab.
- `protobufjs` to decode `foxglove.CompressedImage`, built at runtime from the
  `FileDescriptorSet` the relay advertises, the same way Foxglove itself decodes channels.
- Dev only: `json-schema-to-typescript`, `svelte-check`, `prettier` with
  `prettier-plugin-svelte`.

### Layout

```
web/
  package.json, package-lock.json
  vite.config.ts            # VITE_WS_URL defaults to ws://<page host>:8765
  index.html
  public/manifest.webmanifest, icons
  scripts/gen-protocol.mjs  # remote_protocol_dump output -> src/generated/protocol.ts
  src/
    main.ts, App.svelte
    generated/protocol.ts   # committed; regenerated by scripts/lint
    lib/connection.ts       # FoxgloveClient wrapper
    lib/status.svelte.ts    # $state per status topic, connection state
    lib/image.ts            # CompressedImage decode -> createImageBitmap
    components/
      ConnectionBanner.svelte
      StatusBar.svelte      # camera, transmitter, loop rate, temperature, recording
      CameraView.svelte     # /camera/preview + detection overlay
      ControlTiles.svelte   # autonomy, opponent count, recording, reinit field
      ProfilePicker.svelte
      SystemMenu.svelte     # hold-to-confirm reboot and power off
      PhoneAccess.svelte    # Wi-Fi access toggle, addresses, QR code (wide layout)
      ViewSwitch.svelte     # camera / top-down toggle (narrow layout)
      TopDown.svelte        # /status/tracks on the field outline
      Diagnostics.svelte    # /diagnostics/* list, plot, values
```

`web/node_modules/` and `web/dist/` are gitignored.

### Generated types

A small executable `remote_protocol_dump` (built with the app, in `tools/`) walks both tuples and
prints `{status: [...], commands: [...]}` with each topic, schema name, schema, and latch.
`web/scripts/gen-protocol.mjs` turns that into `src/generated/protocol.ts`:

```ts
export interface SystemStatusMessage { camera_ok: boolean; loop_rate_hz: number; ... }
export type StatusMessages = {
  "/status/system": SystemStatusMessage;
  "/status/app": AppInfoMessage;
  ...
};
export type CommandPayloads = {
  "/command/set_opponent_count": SetOpponentCountCommand;
  ...
};
export const COMMAND_SCHEMAS = { "/command/set_opponent_count": "auto_battlebot.command.SetOpponentCount", ... } as const;
```

`connection.ts` exposes only typed calls:

```ts
subscribe<T extends keyof StatusMessages>(topic: T, cb: (m: StatusMessages[T]) => void): () => void;
send<T extends keyof CommandPayloads>(topic: T, payload: CommandPayloads[T]): Promise<CommandAck>;
```

A renamed field or topic on the C++ side is a `svelte-check` error.

### Connection behavior

- On connect: map `advertise` events by topic, subscribe to what mounted components asked for,
  and `clientAdvertise` each command topic with `json` encoding and its schema name.
- Reconnect with backoff from 0.5 s to 5 s. Safari drops the socket when the screen locks, so
  this is the normal path, not an error path.
- `ConnectionBanner` shows three states: relay unreachable, relay up but no `/status/system` for
  2 s (app down), and connected.
- `send` resolves on the matching `/status/command_ack`, or rejects after 1 s.

### Layouts

The visual spec is direction B ("Race program") on the design canvas: off-white paper, heavy
black rules, Archivo (wide cuts for state words) and IBM Plex Mono for data, amber for warnings,
red for recording, errors, and power off. Dark mode inverts the palette and uses the inverted
logo.

Two layouts, picked with `matchMedia('(min-width: 1000px)')`:

- **Wide** (iPad landscape, 1180 px). Header with tabs, metrics, link state, and the dark-mode
  switch. On Main, a 380 px control column on the left (rows 01 to 06), then the camera and
  top-down views side by side, the status tables, and the warning strip.
- **Narrow** (phones, and iPad portrait at 820 px). One column, scrolling. Header with the logo,
  link state, and dark-mode switch, then a three-cell metrics strip and full-width tabs
  (MAIN, DIAG, SYSTEM). On Main, a camera / top-down switch shows one view at a time at full
  width, then the same control rows, status list, and warning link. Diagnostics shows the module
  list with the selected module's plot and values below it. System shows the connection, profile,
  appearance, and power, without the Wi-Fi toggle or QR code.

Both layouts share every component and the same sizes in the control rows: 12 px labels and row
numbers, 24 px state values, 14 px control text, touch targets at least 44 px.

Other details:

- Reboot and power off need hold-to-confirm (800 ms). Re-init field is a single tap.
- The header's link state says `CABLE` or `WI-FI`. `/healthz` returns which one the request came
  in on, from the client's source address (`169.254.0.0/16` is the cable).
- The dark-mode choice is kept in `localStorage` per device, and defaults to the system setting.
- Add to Home Screen:
  - iPadOS Safari opens the page standalone with the manifest's `display: standalone`.
  - Android Chrome only installs a standalone web app from a secure context. Over plain
    `http://` it adds a shortcut that opens in a normal tab, which is fine for testing.

## Build, lint, deploy

- `scripts/build_web.sh`: `npm ci` when `package-lock.json` is newer than `node_modules`, then
  `npm run build` to `web/dist/`. `scripts/build.sh` calls it when `node` is on `PATH`, and warns
  and continues otherwise.
- Node: Ubuntu 24.04 apt ships Node 18, and current Vite needs 20.19 or newer. Add
  `install/install_node.sh` (NodeSource 22.x) to every platform install, so the ZED Box builds
  the page the same way the dev box does.
- `scripts/lint`:
  - Regenerates `protocol.ts` from `remote_protocol_dump`, and fails under `--dry-run` if it
    changed.
  - Runs `prettier --write web/src` (`--check` under `--dry-run`).
  - Runs `svelte-check` in the slow set, skipped by `--quick`.
- `install(DIRECTORY web/dist/ DESTINATION <bin dir>/web)` when `web/dist` exists.
- ZED Box platform config sets `[ui] enable = false`.

### Dashboard URL

Every client opens `http://auto-battlebot-dashboard.local`. The WebSocket URL comes from
`location.hostname`, so it follows as `ws://auto-battlebot-dashboard.local:8765`.

Addressing on the cable uses IPv4 link-local, not DHCP:

- The box takes the fixed address `169.254.42.1/16` on its Ethernet port. The iPad gets no DHCP
  answer and self-assigns a `169.254.x.x` address a few seconds after the cable goes in. Both
  ends are on `169.254.0.0/16`, so they reach each other with no server on the link.
- I chose this over a DHCP server (dnsmasq, or NetworkManager's `shared` mode) because a DHCP
  server on this port would hand out addresses to every client on a shop LAN the day the box is
  plugged into a router. Link-local can't do that.
- If the iPad's self-assign wait is too slow, set its Ethernet adapter to manual
  `169.254.42.2/16` once, and it connects instantly after that.

The name is the box's hostname, `auto-battlebot-dashboard`, published by Avahi over mDNS:

- Avahi answers a hostname query on each interface with that interface's own address:
  `169.254.42.1` to the iPad on the cable, and the DHCP address to the phone on Wi-Fi. That is
  why this uses the hostname, not an alias. An `avahi-publish -a` alias has one fixed address on
  every interface, so a phone on Wi-Fi would resolve to the unreachable cable address.
- SSH to the box also becomes `auto-battlebot-dashboard.local`. The ZED Box is new, so no
  existing setup depends on its old name.
- iPadOS resolves `.local` natively. Android resolves it from Android 12 on, through the DNS
  resolver Mainline module shipped in Google Play system updates since November 2021.
  Android's Private DNS set to a provider hostname can stop `.local` from resolving. Set it to
  Automatic or Off.
- If a phone still can't resolve the name, the QR code on the tablet's System tab encodes
  `http://<wifi address>` directly.

Android over the cable is possible but not the default path. Android doesn't assign itself an
IPv4 link-local address on Ethernet, so it needs a static address once:
`169.254.42.3/16` with gateway `169.254.42.1`. Only some phones expose Ethernet settings (Samsung
does, under "Configure Ethernet device", and the Ethernet toggle has to be off to save). Over
Wi-Fi it needs no setup.

#### `install/install_dashboard_network.sh`

A new install script with an `install_dashboard_network` function. It is sourced and called from
`scripts/install_jetson.sh` after `install_uvcvideo_rt`, and can also be run on its own, the same
way `install/install_llvm_toolchain.sh` can.

Arguments:
- `--interface <dev>`: the Ethernet port the iPad uses. Defaults to the first `ethernet` device
  from `nmcli -t -f DEVICE,TYPE device status`.
- `--wifi-interface <dev>`: the interface the Wi-Fi access toggle opens. Defaults to the first
  `wifi` device from `nmcli`.
- `--allow-all-interfaces`: skip the firewall table, so the dashboard and Foxglove are open on
  every interface all the time.

Steps:

1. **Checks.** Require `nmcli`. Refuse any interface that carries the default route, since
   converting it would cut the box off the network and drop any SSH session on it. The message
   says to pick another port with `--interface` or move the uplink to Wi-Fi first.
2. **Packages.** `apt-get install -y avahi-daemon avahi-utils nftables`. NetworkManager and
   polkit already ship with JetPack.
3. **Hostname.** `hostnamectl set-hostname auto-battlebot-dashboard`, and the `127.0.1.1` line in
   `/etc/hosts` updated to match so `sudo` doesn't warn that it can't resolve the host. The
   script prints the old name when it changes one.
4. **NetworkManager profile** `auto-battlebot-dashboard` on the interface:
   - `ipv4.method manual`, `ipv4.addresses 169.254.42.1/16`, `ipv4.never-default yes`
   - `ipv6.method link-local`
   - `connection.mdns no`, so systemd-resolved stays off port 5353 and Avahi answers
   - `connection.autoconnect yes`, `connection.autoconnect-priority 100`, which beats the distro's
     "Wired connection 1" at priority 0

   The script deletes and re-adds the profile, so rerunning it is safe. `nmcli connection up`
   may fail with no cable plugged in, and that's fine: NetworkManager brings the profile up on
   carrier.
5. **Avahi.** `systemctl enable --now avahi-daemon`. If `/etc/avahi/avahi-daemon.conf` sets
   `allow-interfaces` without both interfaces, the script warns and does not edit the file.
6. **Firewall**, unless `--allow-all-interfaces`:
   - `/etc/auto-battlebot/dashboard.nft` defines its own table,
     `inet auto_battlebot_dashboard`, so no other ruleset is touched. The table has a set
     `allowed_ifaces` (type `ifname`) holding `lo` and the Ethernet port. Its input chain accepts
     anything arriving on `@allowed_ifaces` and drops TCP 80 and 8765 from everything else.
   - The file starts with the declare-then-delete idiom, so reloading replaces the table
     instead of failing on one that already exists.
   - Loaded by the oneshot `auto-battlebot-dashboard-firewall.service`:
     `Before=network-pre.target`, `ExecStart=nft -f ...`, and `ExecStop` deletes the table.
   - This keeps reboot and power off unreachable over Wi-Fi until someone turns Wi-Fi access on.
   - `auto-battlebot-dashboard-wifi.service`, a oneshot with `RemainAfterExit=yes`,
     `Requires=` and `After=` the firewall service. `ExecStart` adds the Wi-Fi interface to
     `allowed_ifaces` with `nft add element`, and `ExecStop` removes it. The unit is not enabled:
     the app starts it at boot when the saved setting says so.
   - `/etc/polkit-1/rules.d/50-auto-battlebot-dashboard.rules` lets the app user run
     `org.freedesktop.systemd1.manage-units` with the verbs `start` and `stop` on that one unit
     and nothing else. JetPack 7 is Ubuntu 24.04, whose polkit reads JavaScript rules files.
   - With `--allow-all-interfaces`, the script disables both services, removes the polkit rule,
     and deletes the table if an earlier run installed them.
7. **Port 80 for viz_relay.** The drop-in
   `/etc/systemd/system/viz_relay.service.d/dashboard.conf` sets
   `AmbientCapabilities=CAP_NET_BIND_SERVICE` and replaces `ExecStart` with
   `~/.local/bin/viz_relay --http-port 80`, using the invoking user's home from `SUDO_USER`.
   - The capability lives in the unit, so rebuilding or reinstalling `viz_relay` doesn't drop
     it. `setcap` on the binary would, because `cmake --install` copies a fresh file.
   - The relay never runs as root.
   - The drop-in is only written when `viz_relay --help` lists `--http-port`. Before Crow lands,
     an unknown flag would put the relay in a crash loop, so until then the script skips this
     step and says to rerun it.
8. **Enable and restart.** `daemon-reload`, enable and restart the firewall service, restart
   `avahi-daemon` so it picks up the new hostname, and restart `viz_relay` if it's running.
9. **Print verification commands:**
   - `avahi-resolve -4 -n auto-battlebot-dashboard.local` prints the box's addresses.
   - `sudo nft list set inet auto_battlebot_dashboard allowed_ifaces` shows `lo` and the
     Ethernet port.
   - `sudo -u <user> systemctl start auto-battlebot-dashboard-wifi.service` succeeds without a
     password prompt, which proves the polkit rule.
   - On the iPad: Settings > Ethernet shows a self-assigned `169.254.x.x` address, and Safari
     opens the URL.

The script only sets up the network. It does not build the page or install Node. Those belong to
`scripts/build_web.sh` and `install/install_node.sh`.

Dev loop:

```bash
./scripts/build_and_run.sh -c config/playback/<config>.toml   # app + relay
cd web && npm run dev -- --host                                # http://localhost:5173, hot reload
```

`npm run dev -- --host` also serves the dev page to the iPad or phone over the LAN for early
testing. Chrome's device toolbar at 390 px wide covers the narrow layout on the desktop.

## Tests

C++ (GoogleTest, `tests/remote/`, `tests/viz/`):

- `test_protocol.cpp`: topics are unique and prefixed; every command round-trips through
  `to_json`/`from_json`; `schema_for<T>()` matches a golden string for one message of each field
  kind (bool, int, double, string, enum, vector, optional, nested).
- `test_command_decode.cpp`: valid payload decodes; `{}` on `/command/reinit_field` decodes;
  missing required field, wrong type, malformed JSON, and unknown topic all return `nullopt`
  without throwing.
- `test_host_services.cpp`: `set_wifi_access` returns before the fake `systemctl` finishes (a
  fake that sleeps 200 ms); the setting is written to the state file and applied again by a new
  `HostServices` started against that file.
- `test_status_bus.cpp`: throttling drops calls inside the rate window; a latched topic keeps
  its last message; payload bytes parse back to the struct.
- `test_runner_commands.cpp`: each command changes Runner state the same way the old atomic did.
- `test_static_files.cpp`: `resolve_static_path` rejects `..`, encoded `%2e%2e`, absolute paths,
  and symlinks out of the root.
- `test_image_encoder_worker.cpp`: `submit` returns without waiting on a slow encoder (a fake that
  sleeps 50 ms); a burst of submits encodes the newest frame and counts the rest as replaced.
- `test_jpeg_encoder.cpp`: whichever encoder `make_jpeg_encoder()` returns produces bytes that
  `cv::imdecode` reads back at the input size, with mean absolute error under 3 against the
  input at quality 90. On the Jetson this runs NVJPG, and on the desktop OpenCV.
- `test_video_encoder.cpp` (extend if it exists): after `request_keyframe()`, the next packet
  reports `keyframe = true`.

Frontend: `svelte-check` only. Add vitest if `connection.ts` grows logic worth testing apart
from a live relay.

End to end, on a playback run and then on the ZED Box with the iPad over the dongle:

1. Every tile's command shows up in `/status/system` within 200 ms.
2. Kill and restart the app: the banner says app down, then recovers without reloading.
3. Kill and restart the relay: the page reloads its socket and recovers.
4. Lock and unlock the iPad: reconnects within 5 s.
5. Wi-Fi off, airplane mode on: everything still works.
6. Plug the cable in after the box has booted: `http://auto-battlebot-dashboard.local` loads
   within 15 s. From a laptop on the box's Wi-Fi, ports 80 and 8765 are refused.
7. Turn on Wi-Fi access from the iPad. An Android phone on the same Wi-Fi opens
   `http://auto-battlebot-dashboard.local` in Chrome, and the header says `WI-FI`. The phone's
   commands show up in `/status/system` within 200 ms.
8. Reboot the box. The phone reconnects without the iPad, and the amber `WI-FI` tag is still
   shown. Turn Wi-Fi access off: the phone is refused within 1 s.
9. Scan the System tab's QR code with the phone, with Private DNS set to a provider: the page
   loads by IP address.
10. Narrow layout at 390 px: no horizontal scroll on any tab, and every control row is fully
    usable.
11. The MCAP from the run has `/status/*` and `/command/*` channels, and
   `auto_battlebot/recording/` loaders still read it.
12. The encoder measurements table above passes: loop rate with the iPad connected and a
    preview subscribed is within 1 Hz of the rate with nothing connected. Repeat with the iPad
    and the phone both connected.

## Order of work

Each step is one commit, and the tree builds and passes tests after each.

1. **Protocol core.** nlohmann, `AB_JSON_MESSAGE`, schema builder, `RemoteTopics`,
   `decode_command`, `CommandQueue`. Port `reinit_field` and delete the `RemoteCommand` enum.
   Nothing visible changes.
2. **One command path.** Move the six `UIState` request members to commands, switch LVGL tiles
   to posting commands, replace the `handle_*_request` functions with `handle_commands`, and
   record commands to MCAP. Check the desktop LVGL UI behaves the same on a playback run.
3. **Status topics.** `StatusBus`, status published with or without the UI, acks.
   Update `docs/foxglove_recording_format.md`.
4. **Encoder threads.** `ImageEncoderWorker` with `OpenCvJpegEncoder`, moving `/camera/image` off
   the loop, and adding `/camera/preview`. This already removes the 10 ms from the loop on every
   platform.
5. **NVJPG.** `NvjpgJpegEncoder`, the CMake detection, and the ZED Box measurements.
6. **Crow in viz_relay.** Static serving, `/healthz`, `resolve_static_path` tests.
7. **Web scaffold.** `web/`, codegen, `connection.ts`, banner and status bar only, build and
   lint wiring, `install/install_node.sh`.
8. **Screens.** Controls, JPEG camera with overlay, top-down view, profile picker, system menu,
   diagnostics, dark mode. Wide and narrow layouts together, so neither lags.
9. **ZED Box.** Platform config, `HostServices` and `SetWifiAccessCommand`,
   `install/install_dashboard_network.sh` wired into `scripts/install_jetson.sh`, and the iPad
   and phone test checklist above.
10. **NVENC preview.** Check the JetPack 7 FFmpeg encoders first. Then `/camera/preview_video`,
    `request_keyframe()`, and WebCodecs decode with JPEG fallback. This comes last because the
    JPEG preview is enough to use the UI, and this step has the most unknowns.

Update `CLAUDE.md` in step 3 with one line under Conventions: remote UI topics and commands are
added in `include/remote/protocol.hpp`.

## Open questions

- Manual target (press-and-hold on the camera feed to pick a target) needs pixel to field-frame
  projection in the browser or a pixel-space command. It is left out until the rest works.
- Battery on the ZED Box: is there a UPS or battery monitor to read? That decides whether the
  estimator moves.
- Does the ZED Box keep an LVGL build at all? This plan keeps LVGL for the desktop and leaves it
  disabled on the box.
- Crow's Asio discovery, noted above.
- Does JetPack 7's FFmpeg have `h264_nvv4l2m2m`? If not, is it better to add a `NvVideoEncoder`
  backend or install NVIDIA's FFmpeg build? The same answer decides whether e-CAM25 recording
  on JetPack 7 is quietly running `libx264` today.
- Is `NvJPEGEncoder` in the JetPack 7 Multimedia API unchanged from JetPack 6? The samples
  moved to `NvBufSurface` in JetPack 5, and the plan assumes the JetPack 5/6 API.
- Anyone on the Ethernet link can reboot the box, and so can anyone on the Wi-Fi while Wi-Fi
  access is on. That is acceptable on a direct cable and on the shop network. At an event, turn
  Wi-Fi access off, which is why the amber `WI-FI` tag stays in the header while it is on.
- Two boxes with this install on the same Wi-Fi both claim `auto-battlebot-dashboard.local`, and
  Avahi renames the second to `auto-battlebot-dashboard-2.local`. Is the ZED Box the only box
  that gets it, or does the Orin NX bench box need a flag to keep its own name?
- Which Android phone? Android 12 or newer is needed for `.local`. Whether it exposes Ethernet
  settings decides if the cable path works for it too.
- Does the ZED Box have one Ethernet port or two? With one, the dashboard takes it, and the box's
  uplink has to be Wi-Fi. The install script refuses to convert the port that has the default
  route, so this shows up at install time instead of as a lost SSH session.

## Implementation notes

Where the code differs from the plan above, and why:

- **Enums need no `AB_JSON_ENUM`.** `include/remote/json_message.hpp` encodes and decodes every
  field itself (`detail::encode`/`detail::decode`), so enums become lowercase strings without a
  per-enum macro. Decoding is strict: a string never becomes a number, `2.5` never becomes `2`,
  and `1` never becomes `true`.
- **Rates.** `/status/app` repeats every 5 s (latched) instead of only on change, so a relay that
  connects late still gets it. `/status/network` has no bus throttle: `HostServices` publishes it
  after each change and every 5 s, and a 1 Hz throttle would have swallowed the post-change
  message.
- **`TracksMessage`** also carries `field_x`/`field_y` for the top-down outline, and each robot's
  `ours` and `stale` flags.
- **`UIState::profile_notice` stays** alongside the ack, because the LVGL dialog still reads it.
- **Crow and Asio** are fetched as headers only (`SOURCE_SUBDIR` with no `CMakeLists.txt`), which
  sidesteps Crow's `find_package(asio)`. `ASIO_STANDALONE` is defined on `viz_relay`.
- **`/healthz`** returns `{"app_connected", "link"}` with `link` one of `cable`, `wifi`, `local`.
- **The firewall** also drops 8080, the relay's default HTTP port, in case the port 80 drop-in is
  missing. `install_jetson.sh` treats a refused network setup (the uplink is the only Ethernet
  port) as a warning, not a failed install.
- **`config/_zed_box.toml`** is the platform base with `[ui] enable = false`. No robot profile
  extends it yet: the ZED X One S camera path does not exist in the tree.
- **`request_keyframe()`** rides with the queued frame, not a flag the encoder thread reads,
  because the thread may still be encoding older frames when the request arrives.
- **`/camera/preview_video`** is only advertised when FFmpeg has `h264_nvenc` or `h264_nvv4l2m2m`,
  and turns itself off if the encoder that opens is `libx264`.
- **NVJPG** (`src/viz/nvjpg_jpeg_encoder.cpp`) is written against the JetPack 5/6 Multimedia API
  and has not been compiled yet: the dev box has no Multimedia API, and the Jetson was not
  reachable while this landed. CMake leaves it out unless it finds the headers and L4T's
  `libnvjpeg`, so the desktop build is unaffected.

