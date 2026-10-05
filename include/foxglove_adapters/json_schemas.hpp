#pragma once

// jsonschema texts for the JSON channels. Must stay byte-identical to the copies in
// auto_battlebot/mcap_write.py; docs/foxglove_recording_format.md is the reference.

namespace auto_battlebot {
namespace foxglove_adapters {

constexpr const char *kFrameMetaSchemaName = "auto_battlebot.FrameMeta";
constexpr const char *kFrameMetaSchema =
    R"({"type":"object","title":"auto_battlebot.FrameMeta","properties":{"image_stamp_ns":{"type":"string","description":"Raw camera image stamp in nanoseconds as a decimal string; above 2^53 so not a JSON number"},"video_frame_index":{"type":"integer","description":"Frame index within the /camera/video stream, -1 when video recording is off"}},"required":["image_stamp_ns","video_frame_index"]})";

constexpr const char *kDetectionsSchemaName = "auto_battlebot.Detections";
constexpr const char *kDetectionsSchema =
    R"({"type":"object","title":"auto_battlebot.Detections","properties":{"stamp":{"type":"number","description":"Frame stamp in seconds"},"w":{"type":"integer","description":"Image width in pixels"},"h":{"type":"integer","description":"Image height in pixels"},"dets":{"type":"array","items":{"type":"object","properties":{"x1":{"type":"number"},"y1":{"type":"number"},"x2":{"type":"number"},"y2":{"type":"number"},"conf":{"type":"number"},"class_id":{"type":"integer"},"label":{"type":"string"},"kps":{"type":"array","description":"Keypoints as [x, y, confidence] in image pixels","items":{"type":"array","items":{"type":"number"},"minItems":3,"maxItems":3}}},"required":["x1","y1","x2","y2","conf","class_id","label"]}}},"required":["stamp","w","h","dets"]})";

constexpr const char *kDiagnosticsSchemaName = "auto_battlebot.Diagnostics";
constexpr const char *kDiagnosticsSchema =
    R"({"type":"object","title":"auto_battlebot.Diagnostics","description":"One key per subsection of a diagnostics module; the empty subsection is keyed by the module name","additionalProperties":{"type":"object","properties":{"level":{"type":"integer","description":"0 OK, 1 WARN, 2 ERROR, 3 STALE"},"message":{"type":"string"},"values":{"type":"object","additionalProperties":{"type":["number","string","null"]}}},"required":["level","message","values"]}})";

constexpr const char *kEsp32DiagnosticsSchemaName = "auto_battlebot.Esp32Diagnostics";
constexpr const char *kEsp32DiagnosticsSchema =
    R"({"type":"object","title":"auto_battlebot.Esp32Diagnostics","description":"One Mr Stabs Mk2 firmware diagnostics event","properties":{"host_receive_ns":{"type":"integer","description":"Host wall clock when the line arrived, ns; also the MCAP log time"},"timestamp_ms":{"type":"integer","description":"Robot clock, ms"},"radio_connected":{"type":"boolean"},"armed":{"type":"boolean"},"a_percent":{"type":"number"},"b_percent":{"type":"number"},"button_state":{"type":"boolean"},"flip_switch":{"type":"integer"},"left_cmd":{"type":"number","description":"Left motor command after PID and mixer, percent"},"right_cmd":{"type":"number","description":"Right motor command after PID and mixer, percent"},"accel_x":{"type":"number"},"accel_y":{"type":"number"},"accel_z":{"type":"number"},"is_upside_down":{"type":"boolean"},"loop_us":{"type":"integer"},"wifi_clients":{"type":"integer"},"orientation_x":{"type":"number","description":"BNO055 Euler angle, degrees"},"orientation_y":{"type":"number","description":"BNO055 Euler angle, degrees"},"orientation_z":{"type":"number","description":"BNO055 Euler angle, degrees"},"pid_setpoint":{"type":"number"},"pid_output":{"type":"number"},"vbat":{"type":["number","null"],"description":"Pack voltage, V; null when the firmware sends none"},"ibat":{"type":["number","null"],"description":"Pack current, A, positive discharging; null when the firmware sends none"}},"required":["host_receive_ns","timestamp_ms","radio_connected","armed","a_percent","b_percent","button_state","flip_switch","left_cmd","right_cmd","accel_x","accel_y","accel_z","is_upside_down","loop_us","wifi_clients","orientation_x","orientation_y","orientation_z","pid_setpoint","pid_output","vbat","ibat"]})";

constexpr const char *kAprilTagRobotTagsSchemaName = "auto_battlebot.AprilTagRobotTags";
constexpr const char *kAprilTagRobotTagsSchema =
    R"({"type":"object","title":"auto_battlebot.AprilTagRobotTags","description":"Robot AprilTag detections for one frame, with both IPPE PnP solutions","properties":{"image_stamp_ns":{"type":"integer","description":"Frame image stamp, ns"},"frame_id":{"type":"string"},"camera":{"type":"object","properties":{"fx":{"type":"number"},"fy":{"type":"number"},"cx":{"type":"number"},"cy":{"type":"number"},"width":{"type":"integer"},"height":{"type":"integer"}},"required":["fx","fy","cx","cy","width","height"]},"tag_size_m":{"type":"number"},"roi":{"type":["array","null"],"description":"[x, y, w, h] searched, null when the full frame was searched","items":{"type":"integer"}},"detections":{"type":"array","items":{"type":"object","properties":{"id":{"type":"integer"},"corners":{"type":"array","description":"[u, v] pixels, OpenCV aruco order: clockwise from the marker's top-left","items":{"type":"array","items":{"type":"number"},"minItems":2,"maxItems":2},"minItems":4,"maxItems":4},"decision_margin":{"type":["number","null"]},"solutions":{"type":"array","description":"solvePnPGeneric IPPE_SQUARE, sorted by reprojection error ascending; rvec/tvec map tag to camera","items":{"type":"object","properties":{"rvec":{"type":"array","items":{"type":"number"}},"tvec":{"type":"array","items":{"type":"number"}},"reprojection_error_px":{"type":"number"}},"required":["rvec","tvec","reprojection_error_px"]}}},"required":["id","corners","decision_margin","solutions"]}}},"required":["image_stamp_ns","frame_id","camera","tag_size_m","roi","detections"]})";

}  // namespace foxglove_adapters
}  // namespace auto_battlebot
