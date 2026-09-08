# Plan: replace ROS 1 wire formats with Foxglove

## Goal

Delete miniroscpp, the `roscore` + `foxglove_bridge` container, and every `ros1msg` schema
from the repository. Keep live Foxglove viewing, keep the Foxglove connection alive across
app restarts, and convert the 137 existing recordings so no legacy decoder survives the
migration.

The end state: `grep -ri ros` over `src/`, `include/`, and `auto_battlebot/` returns nothing
but the word "across".

## What ROS 1 does today

Three jobs, and only the first is visible from the outside.

1. **Live transport.** `src/main.cpp:54` calls `miniros::init`, and `src/publisher/config.cpp`
   advertises 12 topics. A `roscore` and `foxglove_bridge` run in the `auto-battlebot-ros`
   container (`docker/ros-connector.Dockerfile`, based on `ros:noetic-robot`). Foxglove
   connects to that bridge.
2. **Message definitions and serialization.** 16 `.hxx` message headers, reduced to 7 published
   top-level types. `McapRecorder::write` (`include/mcap_recorder/mcap_recorder.hpp:52`)
   serializes with `miniros::serialization` and stamps `ros1msg` schemas into every recording.
3. **Odds and ends.** `miniros::Time::now()` for MCAP log times, `miniros::ok()` as a shutdown
   check at `src/runner.cpp:186,208,342`.

18 C++ files touch miniros. The MCAP side has the wider blast radius: `auto_battlebot/mcap_io.py`
hand-decodes the ros1 wire format for 6 message types, 17 Python files sit on those helpers, and
`data/` holds 137 recordings totalling 27 GB, every one of them `profile: ros1`.

## Target architecture

```
auto_battlebot  --unix socket-->  viz_relay  --ws://:8765-->  Foxglove app
      |
      +--> foxglove::McapWriter --> data/recordings/*.mcap (protobuf)
```

Two processes, same split as today, no ROS in either.

**The relay** owns a `foxglove::WebSocketServer` and stays up across app restarts. The app
connects over a unix socket and forwards already-encoded messages. The relay keys its
`RawChannel` objects by topic and reuses them when an app reconnects with a matching schema,
so Foxglove sees a continuous channel instead of an unadvertise/re-advertise pair. Panels do
not blink on restart.

**The app** links no Foxglove WebSocket code. It writes MCAP directly with
`foxglove::McapWriter` and pushes the same encoded bytes at the socket. The SDK's WebSocket
half only ever gets linked into `viz_relay`.

### Why the relay also fixes the startup hang

The 2026-08-29 "app doesn't start" incidents were a ROS master race: `run_ros_connector.sh`
is a bare `docker compose up -d` with no readiness gate, miniros `retry_timeout` defaults to
zero so `wait_for_master` retries forever (`master_link.cpp:351`), and
`setup_rosout_publisher(nh)` runs before the UIManager is constructed, so a stalled master
leaves a black screen.

A unix socket `connect()` cannot hang. It returns `ENOENT` or `ECONNREFUSED` immediately.
Encode that as a rule and hold to it:

- The viz sink never blocks and never fails startup. Non-blocking connect, warn, continue.
- Background reconnect on a timer. Messages drop while disconnected.
- Construct the UIManager before the viz sink, so failures are visible on screen.
- `viz_relay` gets its own systemd unit with `Restart=always`. `auto_battlebot.service`
  declares `After=`, never `ExecStartPre=`, so the app never waits on it.

If the relay is down the app still drives and still records. Live viz is the only loss, and it
reconnects on its own.

## Measured baseline

Every size target below comes from `scripts/mcap_topic_sizes.py` on
`data/saved_recordings/MassD_2026-08-29/auto_battlebot_mrs_buff_mk3_massd_ns_jetson_2026-08-29_09-50-45.mcap`,
111.1 s, 20,104 messages, 81.2 MB:

```
/field_markers           visualization_msgs/MarkerArray        5    63.8 MB   12.8 MB/msg   78.6%
/diagnostics             diagnostic_msgs/DiagnosticArray   2,550     7.6 MB    3.1 KB/msg    9.4%
/hazard_markers          visualization_msgs/MarkerArray    1,617     4.1 MB    2.6 KB/msg    5.1%
/nav_markers             visualization_msgs/MarkerArray    1,617     1.9 MB    1.2 KB/msg    2.3%
/robot_markers           visualization_msgs/MarkerArray    1,617   913.2 KB    578.0 B       1.1%
/camera/camera_info      sensor_msgs/CameraInfo            2,549   876.2 KB    352.0 B       1.1%
/field_mask              sensor_msgs/CompressedImage           5   779.1 KB  155.8 KB/msg    0.9%
/tf                      tf2_msgs/TFMessage                4,166   553.0 KB    135.0 B       0.7%
/camera/frame_meta       std_msgs/String                   2,549   351.9 KB    141.0 B       0.4%
/blob_detections         std_msgs/String                   1,617   187.9 KB    118.0 B       0.2%
/keypoint_detections     std_msgs/String                   1,617   137.7 KB     87.0 B       0.2%
/rosout                  rosgraph_msgs/Log                   190    19.8 KB    106.0 B       0.0%
/field_mask/camera_info  sensor_msgs/CameraInfo                5     1.7 KB    358.0 B       0.0%
```

Two things follow from this table, and they set the priorities for the whole migration.

**`/field_markers` is 78.6% of the file across five messages.** It carries the field inlier
cloud as `Marker::POINTS`, and `ros_marker.cpp:110-124` pushes a per-point color alongside
every point. The wire cost is `geometry_msgs/Point` at 3 float64 plus `std_msgs/ColorRGBA` at
4 float32, so 40 bytes per point and roughly 320,000 points per message. Two of those bytes
budgets are pure waste: the source cloud is PCL, where x, y, z are already float32 and
`geometry_msgs/Point` widens them for nothing, and the color is a pure function of z (the
blue-to-red gradient at `ros_marker.cpp:117-120`) that the 3D panel can compute itself by
coloring on a field.

**The three `std_msgs/String` topics are 0.8% of the file combined.** Their encoding is a
usability decision, not a bandwidth one. Pick whatever is easiest to read and hardest to get
wrong.

## Schema mapping

Topic names stay the same except where noted, so saved Foxglove layouts mostly survive. The
3D panel needs reconfiguring for `SceneUpdate` and `PointCloud`.

| Topic | Today | After |
| --- | --- | --- |
| `/camera/image` | `sensor_msgs/CompressedImage` | `foxglove.CompressedImage` |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | `foxglove.CameraCalibration` |
| `/camera/frame_meta` | `std_msgs/String` (JSON payload) | JSON channel, stamp as string |
| `/tf` | `tf2_msgs/TFMessage` | `foxglove.FrameTransforms` |
| `/field_mask` | `sensor_msgs/CompressedImage` | `foxglove.CompressedImage` |
| `/field_mask/camera_info` | `sensor_msgs/CameraInfo` | `foxglove.CameraCalibration` |
| `/field_markers` | `visualization_msgs/MarkerArray` | `foxglove.SceneUpdate` (border only) |
| `/field_points` | part of `/field_markers` | `foxglove.PointCloud` (new topic) |
| `/hazard_markers` | `visualization_msgs/MarkerArray` | `foxglove.SceneUpdate` |
| `/robot_markers` | `visualization_msgs/MarkerArray` | `foxglove.SceneUpdate` |
| `/nav_markers` | `visualization_msgs/MarkerArray` | `foxglove.SceneUpdate` |
| `/blob_detections` | `std_msgs/String` (JSON payload) | JSON channel, real jsonschema |
| `/keypoint_detections` | `std_msgs/String` (JSON payload) | JSON channel, real jsonschema |
| `/blob_detections/annotations` | none | `foxglove.ImageAnnotations` (live only) |
| `/keypoint_detections/annotations` | none | `foxglove.ImageAnnotations` (live only) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | one JSON channel per module |
| `/rosout` | `rosgraph_msgs/Log` | `foxglove.Log` on `/log` |

### The inlier cloud gets its own topic

`to_ros_field_marker` (`src/ros/ros_message_adapters/ros_marker.cpp:27`) returns two markers:
a `LINE_STRIP` border and a `POINTS` inlier cloud. Split them. The border is four corners and
belongs in `SceneUpdate` with the rest of the geometry. The cloud belongs in
`foxglove.PointCloud` on its own topic, with float32 x, y, z and no color field, at 12 bytes
per point.

Expected result: `/field_markers` plus `/field_points` together drop from 63.8 MB to about
19 MB, and the recording from 81.2 MB to about 36 MB. No precision is lost, since float32 is
what the PCL cloud already holds, and the rendering is the same once the panel colors on z.

This is the single largest change in the migration by bytes, and it is a concrete number
stage 2 can be checked against.

### Marker to SceneUpdate is otherwise close to one-to-one

`ros_marker.cpp` is 21 KB of marker construction using 7 marker types. `LINE_STRIP` and
`LINE_LIST` map to `LinePrimitive`, `CUBE`/`SPHERE`/`ARROW` to their primitives,
`TEXT_VIEW_FACING` to `TextPrimitive` with `billboard` set, the `DELETE` action to
`SceneUpdate.deletions`, and namespace plus id to `SceneEntity.id`. Marker lifetime maps to
`SceneEntity.lifetime`. With `POINTS` moved to `PointCloud`, no marker type is left without a
target.

### Detections: structured for analysis, annotated for viz

Publish both, and record only one.

The structured channel stays the recorded truth: the same JSON `to_ros_detections`
(`src/ros/ros_message_adapters/ros_detections.cpp`) already builds by hand, minus the
`std_msgs/String` wrapper, plus a real jsonschema. `mcap_io.decode_detections` becomes
`json.loads`.

`/blob_detections/annotations` and `/keypoint_detections/annotations` carry
`foxglove.ImageAnnotations`, which draws boxes and keypoints on the Image panel. Today you
cannot see detections on the camera image at all; you read JSON in a Raw Messages panel and
correlate by eye. `PointsAnnotation` with `type: LINE_LOOP` gives the box, `CircleAnnotation`
the keypoints, `TextAnnotation` the label.

Annotations do not go in the recording. `ImageAnnotations` has no numeric field for
`confidence` or `class_id` (its `metadata` is `KeyValuePair[]`, so values would be strings
again, which is the exact wart being removed from `/diagnostics`), so it is a rendering of
data the structured channel already holds. Add both annotation topics to
`mcap_recorder.ignored_topics`.

### Diagnostics: real numbers, one channel per module

Two ROS impositions get fixed here.

**Values become numbers.** `diagnostic_msgs/KeyValue` is string-to-string, which is the entire
reason `auto_battlebot/diag_io.py:188` exists:

```python
def _coerce_numeric(df: pd.DataFrame) -> None:
    """Diagnostics values arrive as strings; convert the numeric columns in place."""
```

what to coerce. Emit numbers as numbers and both the function and the column list go away.

**One channel per module, not one array.** `DiagnosticsLogger::loggers_` is a `std::map`
(`src/diagnostics_logger/diagnostics_logger.cpp:4`), so iteration is alphabetical and array
order is stable, but `get_logger` inserts on demand. A module that first logs mid-run shifts
every index after it, and a Foxglove plot path of `status[3].values.foo` silently retargets to
a different module. `/diagnostics/perception`, `/diagnostics/nav` and so on make the plot path
stable and turn `diag_io` into a straight column read.

### frame_meta: the stamp must be a string

This one is a live bug, not just a format preference.

`image_stamp_ns` is `uint64_t` (`include/data_structures/camera.hpp:33`), and
`to_ros_frame_identity` (`src/ros/ros_message_adapters/ros_camera_info.cpp:59`) emits it as a
bare JSON number via `snprintf("%llu")`. A real value is 1788011445339499712, past the 2^53
exact-integer range of a double.

Python's `json.loads` returns an exact int, so today's analysis is correct. Anything parsing
that JSON in JavaScript, Foxglove's Raw Messages panel included, rounds it to a 256 ns quantum
at that magnitude. `auto_battlebot/calibration/match_windows.py:104` and
`auto_battlebot/track_overlay.py:97` both do exact joins on this value, and the whole point of
`image_stamp_ns` is that it is the original-clock stamp that never went through the replay
rebase.

Emit it as a JSON string. That is what protobuf's own canonical JSON mapping does for 64-bit
integers, so it is the idiomatic form rather than a workaround.

### Why JSON and not custom protobuf

For the structured channels the choice is JSON with a real jsonschema, not a hand-written
`.proto`. The three String topics are 0.8% of a recording, so compactness buys nothing
measurable, while a custom protobuf costs a codegen step in CMake for C++ and a generated
module for Python. JSON keeps `json.loads`, stays greppable in a Raw Messages panel, and has
one known sharp edge (64-bit integers) that the frame_meta rule above handles.

`/diagnostics` is the one topic where this could be revisited. It is 9.4% of the file today,
and once `/field_points` lands it becomes roughly 21% of a 36 MB recording. Measure it after
stage 2 before deciding it is a problem.

## Stage 1: teach Python to read Foxglove protobuf

Land this first. It changes no behavior and de-risks every stage after it, because once it
merges, a new-format recording is readable the day C++ starts producing one.

- `pyproject.toml`: add `foxglove-sdk` and `mcap-protobuf-support`. Leave `mcap-ros1-support`
  in place for now. The platform-conditional dependency blocks stay as they are.
- `auto_battlebot/mcap_io.py`: dispatch on the channel's message encoding. `ros1` goes to the
  existing hand-rolled decoders, `protobuf` and `json` to the new paths. Same public API
  (`decode_compressed_image`, `decode_camera_info`, `decode_tf_message`, `decode_detections`,
  `iter_messages`), so no caller changes.
- `auto_battlebot/diag_io.py`: same treatment for `decode_diagnostic_array`, plus a
  per-module channel reader for the new layout. `NAV_HW_IDS` gating is unaffected.
  `_coerce_numeric` has to stay until stage 5, because it is still correct for ros1 input;
  the new path returns typed values and skips it.
- `auto_battlebot/mcap_io.py`: the new `/field_points` reader decodes `foxglove.PointCloud`.
  Nothing reads the inlier cloud from a recording today, so this is new surface rather than a
  port, and it can be deferred if nothing needs it.
- `training/model_eval/export_camera_transforms.py` is the only first-party file importing
  `mcap_ros1` directly. Route it through `mcap_io` instead.

Verify by writing a tiny protobuf MCAP from Python with the same topics and confirming the
decoders return equal values for equivalent content.

## Stage 2: C++ emits Foxglove schemas over the relay

The big one. The relay and the schema swap land together, on one branch. Doing the relay first
with `ros1msg` passthrough would work, but it means writing about 400 lines of ros1
serialization that stage 2 immediately deletes. With months of runway, skip the throwaway.

**New**

- `include/viz/frame.hpp`: the socket framing, shared by app and relay.
  `[u32 len][u8 kind]`, kind 0 ADVERTISE (topic, message encoding, schema name, schema
  encoding, schema bytes), kind 1 MESSAGE (channel id, log time ns, payload), kind 2
  SUBSCRIBER_COUNT flowing back from the relay.
- `include/viz/viz_sink.hpp`, `src/viz/viz_sink.cpp`: the app-side client. `SOCK_SEQPACKET`,
  raised `SO_SNDBUF`, non-blocking, drop on `EAGAIN` with a drop counter in diagnostics.
- `src/viz_relay/main.cpp`: the relay. `WebSocketServer`, a topic-keyed `RawChannel` map,
  retained-message latching, `onSubscribe`/`onUnsubscribe` counting.
- `src/publisher/foxglove_publisher.cpp` and header, replacing `ros_publisher.*`.
- `src/foxglove/message_adapters/`, replacing `src/ros/ros_message_adapters/`. The geometry in
  `ros_marker.cpp` carries over; only the output type changes.

**Do `/field_points` first.** It is the largest win, it is self-contained, and it gives the
stage a measurable pass mark before the rest of the adapters land. Split
`to_ros_field_marker` into a border `SceneUpdate` and a `PointCloud`, drop the per-point
colors, keep float32. Then replay and check `mcap_topic_sizes.py`: the two topics together
should come in near 19 MB where `/field_markers` alone was 63.8 MB.

**Do `/diagnostics` second.** Splitting one array into a channel per module touches
`DiagnosticsBackend` and both its implementations, and it is the change most likely to have
knock-on effects in `diag_io`. Get it done while there is room to iterate.

**Latching.** Six topics use `latch=true` today: `/field_mask`, `/field_mask/camera_info`,
`/field_markers`, `/hazard_markers`, `/robot_markers`, `/nav_markers`. `/field_points` joins
them, and it is the one that most needs latching: it is published five times in a 111 s run. `RawChannel` has no
latching, so the relay retains the last message per latched topic and re-sends it on
`callbacks.onSubscribe`. Note that `log()` broadcasts to every subscriber, so a second Foxglove
client connecting re-sends to the first. Harmless for marker arrays.

This matters more with a persistent relay than it did with the bridge: it is what lets you
attach Foxglove mid-run and immediately see field geometry that was published once at startup.

**Subscriber gating.** `src/publisher/ros_publisher.cpp:44` skips JPEG encoding for
`/camera/image` when nothing subscribes. The relay knows the count and reports it back over
frame kind 2. Keep the optimization; it is real work saved on the Jetson.

**Rewrites**

- `src/main.cpp:54-56,72`: drop `miniros::init` and `NodeHandle`. Construct the UIManager
  before the viz sink.
- `src/runner.cpp:186,208,342`: delete `miniros::ok()` and `miniros::shutdown()`.
  `quit_requested_` and the existing `Quittable` signal handling already cover shutdown.
- `include/mcap_recorder/mcap_recorder.hpp`: swap `mcap::McapWriter` for
  `foxglove::McapWriter`, drop the `miniros::serialization` template body, replace
  `miniros::Time::now()` with the existing `ClockInterface`. Keep the `auto_battlebot`
  metadata record carrying `active_profile`; the analysis tooling reads it.
- `src/logging/logging.cpp`: `/rosout` becomes `foxglove.Log` on `/log`. The
  `#undef DEBUG/ERROR/INFO/WARN/FATAL` block at the top of the file exists only to dodge
  `rosgraph_msgs/Log.hxx` and goes away with it.
- `src/diagnostics_logger/ros_diagnostics_backend.*` becomes `foxglove_diagnostics_backend.*`.
  `tests/diagnostics_logger/test_diagnostics_logger.cpp` currently does
  `reinterpret_pointer_cast<miniros::Publisher>` on a mock in 6 places; an interface makes
  that honest.
- `include/publisher/config.hpp`: `uses_ros()` becomes `uses_viz()`. The config type string
  `"RosPublisher"` becomes `"FoxglovePublisher"`, set in exactly one place,
  `config/_common.toml:269`, since everything else inherits through the extends chain.
- `tests/ros/test_ros_publisher.cpp` is a placeholder whose TODO asks for exactly this
  refactor. It becomes a real test against the sink interface.

**Build**

- `CMakeLists.txt`: remove the `miniroscpp` `FetchContent_Declare` (line 29), drop it from both
  `FetchContent_MakeAvailable` calls (lines 104 and 106), and remove `miniros::roscxx` from
  `target_link_libraries` (line 243). Add an imported target for `third_party/foxglove` and a
  second executable target for `viz_relay`.
- `install/install_foxglove_sdk.sh` already exists (v0.20.0, x86_64 and aarch64) and nothing
  sources it. Wire `install_foxglove_sdk` into `scripts/install_ubuntu_22.sh`,
  `scripts/install_ubuntu_24.sh`, and `scripts/install_jetson.sh`, next to the existing
  `install_mcap_cli` calls. Add `third_party/` to `.gitignore` and `.deployignore`.

**Delete**

```
docker/ros-connector.Dockerfile
docker/ros-connector.launch
docker/launch_ros_connector.sh
docker/docker-compose.ros-connector.yml
install/install_ros_connector.sh
scripts/run_ros_connector.sh
include/ros/          src/ros/          tests/ros/
```

Plus the `include:` and `depends_on: auto-battlebot-ros` in `docker/docker-compose.playback.yml`,
replaced by a `viz-relay` service with `restart: unless-stopped`; the `run_ros_connector.sh`
call at `scripts/run.sh:9`, replaced by `scripts/run_viz_relay.sh`; and the `ExecStartPre` at
`service/auto_battlebot.service:10`, replaced by a separate unit and an `After=`.

**Verify.** Replay the same SVO before and after.

- `mcap info`: same topic names plus `/field_points`, same message counts within a frame or
  two.
- `mcap_topic_sizes.py`: `/field_markers` plus `/field_points` near 19 MB, total file near
  36 MB where it was 81.2 MB. A number far off that means the point cloud is still carrying
  colors or float64.
- Foxglove renders every panel, with the 3D panel coloring `/field_points` on z.
- Restart the app while watching and confirm no reconnect.

## Stage 3: Python writers

Two files write `ros1msg` MCAPs by hand and must produce protobuf instead.

- `auto_battlebot/calibration/apriltag_mcap.py` (1045 lines). Writes the calibration overlay:
  camera frames, CameraInfo, TF, pose markers. The `/tf_static` latching comment at line 851
  is worth re-reading, since Foxglove latching semantics differ from ROS.
- `scripts/combine_mcap_svo.py` (1054 lines). Merges SVO frames into a recording.

Both produce derived artifacts. Regenerate them from source rather than converting the outputs,
and regenerate in place: those directories hold hand-made `validation_state.json` files that
must not be lost.

## Stage 4: convert the corpus

137 recordings, 27 GB, across `data/recordings` (23) and `data/saved_recordings` (114).
176 GB free on `/`, so the extra 27 GB is not a problem.

`scripts/convert_ros1_mcap.py`:

- Takes an input path and an output path. **Never writes in place.** CLAUDE.md forbids
  modifying files under `data/`, and a half-converted 27 GB corpus is unrecoverable.
- Reuses the ros1 decoders already in `mcap_io.py` for reading and the Foxglove protobuf
  writers from stage 1 for writing.
- Passes JPEG payloads through as bytes. No re-encode, so those topics are lossless and the
  conversion is IO-bound.
- Preserves `logTime` and `publishTime` exactly, and copies the `auto_battlebot` metadata
  record with `active_profile`.
- `--check` mode re-reads both files and compares per-topic message counts and first/last
  timestamps.

**Four topics are not a re-wrap.** The converter has to redo the same semantic changes stage 2
made in C++, and each is a place a bug hides:

- `/field_markers` splits into a border `SceneUpdate` and a `/field_points` `PointCloud`,
  dropping the per-point colors and narrowing float64 back to float32. Narrowing is safe
  because the values originated as float32 in the PCL cloud, but the converter is the one
  place that assumption is not locally visible, so assert it: every coordinate must round-trip
  through float32 unchanged, and the run should fail loudly if one does not.
- `/diagnostics` splits into per-module channels, and its string values become numbers.
  Reuse `diag_io._coerce_numeric`'s rules for deciding what is numeric rather than writing a
  second copy of that judgement. Anything that does not parse stays a string.
- `/camera/frame_meta` re-emits `image_stamp_ns` as a string. Since the source is ros1 bytes
  read into a Python int, no precision was ever lost on this path, but the check is cheap:
  compare the decoded value before and after as integers.
- `/blob_detections` and `/keypoint_detections` lose the `std_msgs/String` wrapper. The JSON
  inside is unchanged.

Expect converted files to come out roughly 55% smaller, dominated by the point cloud change.
A file that does not shrink means `/field_markers` did not convert.

Convert into a parallel tree, verify, then let the originals be deleted by hand. Do not have
the script delete anything.

**The verification that actually counts** is not message counts, it is running a real analysis
script against both. `scripts/mcap_latency_report.py` and `playground/analyze_nav_diagnostics.py`
both produce numeric output from a recording. Run each on an original and its conversion and
diff. If the latency percentiles and nav diagnostics match, the conversion preserved what the
tooling cares about.

Do the MassD and NHRL sets last, after the process has been proven on `data/recordings`.

## Stage 5: delete the ros1 decoders

Once the corpus is converted and verified:

- Strip the `ros1` branches from `mcap_io.py` and `diag_io.py`, along with `_read_string`,
  `_read_uint32`, `_read_int8`, and `_read_header`. The file loses roughly half its length.
- Delete `diag_io._coerce_numeric` and `_NUMERIC_DIAG_COLS`. Every recording now carries typed
  numbers, so there is nothing left to coerce.
- Drop `mcap-ros1-support` from `pyproject.toml`.
- Delete `scripts/convert_ros1_mcap.py`. It is the last ros1 reader in the tree, and git
  history keeps it if a stray recording turns up. This is the step that makes the goal true
  rather than approximately true, and it is also the one worth pausing on: keeping the
  converter costs nothing but a file, and removing it means an unconverted recording found
  later needs a `git show` to read.
- Update `README.md:50` and `docs/docker_playback.md`, which both describe the bridge setup.

## Risks

**The 27 GB conversion is the only irreversible step,** and only if the originals get deleted.
Mitigation is in the plan already: separate output tree, script never deletes, originals go by
hand after the analysis-script diff passes.

**The converter does semantic transforms, not just re-encoding,** on four topics. The point
cloud narrowing is the one to watch, since it is the only place a value changes width. The
float32 round-trip assertion in stage 4 is what makes that safe.

**The relay is new code on the viz path.** It is not on the control path, and the drop-on-
backpressure rule keeps it off the latency budget, but it is still a process that can crash.
`Restart=always` and the app's tolerance for a missing relay cover it.

**Foxglove layout rework.** Most topic names are unchanged, so image, plot, and raw-message
panels carry over. The 3D panel needs reconfiguring for `SceneUpdate` and for the new
`/field_points` cloud, and the diagnostics plots need repointing from `status[i].values.foo`
to per-module paths. That repointing is the payoff, since the new paths cannot silently
retarget. Budget an afternoon and save the new layout somewhere durable.

## Sequencing

Stage 1 is independent and safe to merge alone. Stage 2 is one branch and one merge. Stages 3
through 5 follow in order. Nothing in stages 1 and 3 blocks stage 2, so they can be worked in
parallel if convenient.

## Next steps

1. Run `install/install_foxglove_sdk.sh` by hand and confirm `third_party/foxglove` has the
   headers and static lib for this machine.
2. Start stage 1: add the deps, add protobuf dispatch to `mcap_io.py`, and prove it against a
   hand-written protobuf MCAP.
3. When stage 2 opens, do `/field_points` before anything else and check the size against the
   measured baseline. It is the largest win in the migration and the fastest thing to verify.
