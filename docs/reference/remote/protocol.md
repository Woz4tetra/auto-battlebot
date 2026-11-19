# Remote protocol

`include/remote/protocol.hpp` is the one place remote topics map to types. Every status topic and command a browser or a Foxglove panel can use is a row there. The rows drive JSON encoding, the JSON Schema each channel advertises, MCAP recording, command dispatch, and `web/src/generated/protocol.ts`.

Payload structs live in `include/remote/messages.hpp`. A field name is the JSON key and the TypeScript field name, so a rename changes the wire key and breaks old recordings. Units that the name lacks are listed below.

## Adding a topic

- Command: a payload struct in `messages.hpp`, one `CommandTopic` row, then a handler in `Runner::handle_commands`. The build fails until the handler exists.
- Status topic: a message struct, one `StatusTopic` row, and a `status_bus->publish(...)` call at the producer.

## Decoding

- `AB_JSON_MESSAGE` declares a message's fields once. It expands to the schema name, `to_json`/`from_json`, and a `describe()` that feeds the JSON Schema builder.
- A missing required field throws on parse. A command with no `count` is refused instead of using the default. `std::optional` fields may be absent.
- `decode` is stricter than nlohmann's `get<T>`: a string never becomes a number, 1.5 never becomes 1, and a number never becomes a bool. A command with a wrong type is refused, not coerced.
- `tests/remote/test_json_message.cpp` pins both.

## Publishing

`StatusBus::publish` serializes on the topic that `RemoteTopics` maps the message type to. It drops the call when the topic's `max_rate_hz` says it is too soon, and returns whether it published. `StatusBusTest.ThrottlesInsideTheRateWindow` pins it.

## Status fields

| Field | Meaning |
| --- | --- |
| `System.uptime_s` | Seconds since the app started |
| `System.autonomy_on_s` | Seconds since the autonomy switch last went on. Absent while it is off |
| `System.mcap_file_s` | Seconds since the current MCAP file started. Absent while MCAP recording is off |
| `LabelColor.color` | `#rrggbb`. `[ui.label_colors]` rejects anything else at config parse |
| `Sticks.linear`, `Sticks.angular` | Our robot's sticks as the transmitter read them back, normalized to [-1, 1] |
| `ImagePoint.u`, `ImagePoint.v` | Fractions of the camera image width and height, 0 to 1 on screen |
| `Target.manual` | The dashboard placed this point. The target selector did not pick it |
| `Tracks.field_x`, `Tracks.field_y` | Field size in meters, so the top-down view can draw the outline. Zero before init |
| `Battery.valid` | False until the first good read, and again after `invalid_read_grace` failed reads |
| `Battery.percent` | Estimated state of charge, 0 to 100 |
| `Battery.charging` | Current is flowing into the battery |
| `CameraSettings.profiles` | `DEFAULT` first, then the saved profiles |
| `CameraSettings.modified` | The live values changed since the active profile was applied or saved |

## Commands

| Command | Behavior |
| --- | --- |
| `SetAutoTracking` | On hands targeting back to the target selector. Off holds the target at the field center until a `SetManualTarget` moves it. `RunnerCommandsTest.SetAutoTrackingOffHoldsFieldCenter` pins it |
| `SetManualTarget` | Moves the held target while auto tracking is off. Refused while it is on, so a stray touch on a view never takes targeting away from the selector. Exactly one of `field` (a tap on the top-down view) or `image` (a tap on the camera view, projected onto the floor) is set |
| `CameraProfile` | `APPLY` a saved profile or `DEFAULT`, `SAVE` the live values under `name`, or `DELETE` `name` |

## Field projection

`include/remote/field_projection.hpp` projects field geometry into the camera image as fractions of the image size, so the dashboard draws over any preview size. It is a pinhole model on the rectified image.

- `unproject_image_point` casts an image point onto the field floor (z = 0) and inverts `project_field_point`. It returns nullopt when the ray points at or above the horizon, or the field or intrinsics are missing. The result can land outside the field.
- `project_field_outline` samples each border side so a side that passes behind the camera is cut where it does, not dropped whole. Every returned polyline has at least two points. Empty when the field or intrinsics are missing.
- `project_camera_marker` projects the top-down view's camera triangle, which sits below the field's -y edge pointing away from it. Order is tip, then the two base corners. It has the same field-frame placement as the triangle in `web/src/components/TopDown.svelte`, and `FieldProjectionTest.CameraMarkerMatchesTheTopDownTriangle` reads that file, so moving the triangle there fails the test. Empty when any corner is behind the camera or the field or intrinsics are missing.
