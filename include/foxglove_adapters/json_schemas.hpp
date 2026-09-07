#pragma once

// jsonschema texts for the JSON channels. Must stay byte-identical to the copies in
// auto_battlebot/mcap_write.py; docs/foxglove_recording_format.md is the reference.

namespace auto_battlebot {
namespace foxglove_adapters {

constexpr const char *kFrameMetaSchemaName = "auto_battlebot.FrameMeta";
constexpr const char *kFrameMetaSchema =
    R"({"type":"object","title":"auto_battlebot.FrameMeta","properties":{"image_stamp_ns":{"type":"string","description":"Raw camera image stamp in nanoseconds as a decimal string; above 2^53 so not a JSON number"},"svo_frame_index":{"type":"integer","description":"Frame index within svo_path, -1 when SVO recording is off"},"svo_path":{"type":"string","description":"Active SVO file, empty when SVO recording is off"}},"required":["image_stamp_ns","svo_frame_index","svo_path"]})";

constexpr const char *kDetectionsSchemaName = "auto_battlebot.Detections";
constexpr const char *kDetectionsSchema =
    R"({"type":"object","title":"auto_battlebot.Detections","properties":{"stamp":{"type":"number","description":"Frame stamp in seconds"},"w":{"type":"integer","description":"Image width in pixels"},"h":{"type":"integer","description":"Image height in pixels"},"dets":{"type":"array","items":{"type":"object","properties":{"x1":{"type":"number"},"y1":{"type":"number"},"x2":{"type":"number"},"y2":{"type":"number"},"conf":{"type":"number"},"class_id":{"type":"integer"},"label":{"type":"string"},"kps":{"type":"array","description":"Keypoints as [x, y, confidence] in image pixels","items":{"type":"array","items":{"type":"number"},"minItems":3,"maxItems":3}}},"required":["x1","y1","x2","y2","conf","class_id","label"]}}},"required":["stamp","w","h","dets"]})";

constexpr const char *kDiagnosticsSchemaName = "auto_battlebot.Diagnostics";
constexpr const char *kDiagnosticsSchema =
    R"({"type":"object","title":"auto_battlebot.Diagnostics","description":"One key per subsection of a diagnostics module; the empty subsection is keyed by the module name","additionalProperties":{"type":"object","properties":{"level":{"type":"integer","description":"0 OK, 1 WARN, 2 ERROR, 3 STALE"},"message":{"type":"string"},"values":{"type":"object","additionalProperties":{"type":["number","string","null"]}}},"required":["level","message","values"]}})";

}  // namespace foxglove_adapters
}  // namespace auto_battlebot
