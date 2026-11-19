#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "enums/behavior_mode.hpp"
#include "enums/camera_profile_action.hpp"
#include "enums/system_action.hpp"
#include "remote/json_message.hpp"
#include "rgbd_camera/camera_settings.hpp"

// Payloads of every remote status topic and command. Topics map to these in
// include/remote/protocol.hpp; field names are the JSON keys and the TypeScript field names in
// web/src/generated/protocol.ts.
namespace auto_battlebot::remote {

struct SystemStatusMessage {
    bool camera_ok = false;
    bool transmitter_connected = false;
    bool transmitter_receiving = false;
    double loop_rate_hz = 0.0;
    bool initialized = false;
    int selected_opponent_count = 1;
    bool autonomy_enabled = true;
    bool camera_recording = false;
    bool mcap_recording = false;
    std::optional<double> jetson_temperature_c;
    std::string compute_mode;
    /** Seconds since the app started. */
    double uptime_s = 0.0;
    std::optional<bool> autonomy_switch_on;
    /** Seconds since that switch last went on; absent while it is off. */
    std::optional<double> autonomy_on_s;
    /** Seconds since the current MCAP file started; absent while MCAP recording is off. */
    std::optional<double> mcap_file_s;
    BehaviorMode behavior_mode = BehaviorMode::ATTACK;
    bool auto_tracking = true;
    AB_JSON_MESSAGE(SystemStatusMessage, "auto_battlebot.status.System", camera_ok,
                    transmitter_connected, transmitter_receiving, loop_rate_hz, initialized,
                    selected_opponent_count, autonomy_enabled, camera_recording, mcap_recording,
                    jetson_temperature_c, compute_mode, uptime_s, autonomy_switch_on, autonomy_on_s,
                    mcap_file_s, behavior_mode, auto_tracking)
};

struct LabelColor {
    std::string label;
    /** "#rrggbb". */
    std::string color;
    AB_JSON_MESSAGE(LabelColor, "auto_battlebot.status.LabelColor", label, color)
};

struct AppInfoMessage {
    std::vector<std::string> available_profiles;
    std::string current_profile;
    double max_loop_rate_hz = 0.0;
    double rate_fail_threshold = 0.0;
    double rate_fail_duration_sec = 0.0;
    std::vector<LabelColor> label_colors;
    std::string version;
    AB_JSON_MESSAGE(AppInfoMessage, "auto_battlebot.status.App", available_profiles,
                    current_profile, max_loop_rate_hz, rate_fail_threshold, rate_fail_duration_sec,
                    label_colors, version)
};

struct ResolvedConfigField {
    std::string section;
    std::string key;
    std::string value;
    std::string source;
    AB_JSON_MESSAGE(ResolvedConfigField, "auto_battlebot.status.ResolvedConfigField", section, key,
                    value, source)
};

struct ResolvedConfigMessage {
    std::string profile;
    std::vector<ResolvedConfigField> fields;
    AB_JSON_MESSAGE(ResolvedConfigMessage, "auto_battlebot.status.ResolvedConfig", profile, fields)
};

/** Our robot's sticks as the transmitter read them back, normalized [-1, 1]. */
struct SticksMessage {
    double linear = 0.0;
    double angular = 0.0;
    AB_JSON_MESSAGE(SticksMessage, "auto_battlebot.status.Sticks", linear, angular)
};

/** A point in the camera image, as a fraction of its width and height (0 to 1 on screen). */
struct ImagePoint {
    double u = 0.0;
    double v = 0.0;
    AB_JSON_MESSAGE(ImagePoint, "auto_battlebot.status.ImagePoint", u, v)
};

struct FieldPoint {
    double x = 0.0;
    double y = 0.0;
    AB_JSON_MESSAGE(FieldPoint, "auto_battlebot.status.FieldPoint", x, y)
};

struct TrackedRobot {
    std::string id;
    std::string label;
    bool ours = false;
    bool stale = false;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    std::optional<ImagePoint> image;
    AB_JSON_MESSAGE(TrackedRobot, "auto_battlebot.status.TrackedRobot", id, label, ours, stale, x,
                    y, yaw, image)
};

struct TargetMessage {
    double from_x = 0.0;
    double from_y = 0.0;
    std::optional<ImagePoint> from_image;
    double x = 0.0;
    double y = 0.0;
    std::string label;
    BehaviorMode mode = BehaviorMode::ATTACK;
    std::optional<ImagePoint> image;
    /** The dashboard placed this point; the target selector did not pick it. */
    bool manual = false;
    AB_JSON_MESSAGE(TargetMessage, "auto_battlebot.status.Target", from_x, from_y, from_image, x, y,
                    label, mode, image, manual)
};

struct TracksMessage {
    bool our_robot_seen = false;
    int opponents_seen = 0;
    /** Field size in meters, so the top-down view can draw the outline. Zero before init. */
    double field_x = 0.0;
    double field_y = 0.0;
    std::vector<TrackedRobot> robots;
    std::vector<std::vector<ImagePoint>> field_outline;
    std::vector<ImagePoint> camera_marker;
    std::optional<TargetMessage> target;
    AB_JSON_MESSAGE(TracksMessage, "auto_battlebot.status.Tracks", our_robot_seen, opponents_seen,
                    field_x, field_y, robots, field_outline, camera_marker, target)
};

struct CommandAckMessage {
    int64_t seq = 0;
    std::string topic;
    bool accepted = false;
    std::string message;
    AB_JSON_MESSAGE(CommandAckMessage, "auto_battlebot.status.CommandAck", seq, topic, accepted,
                    message)
};

struct NetworkMessage {
    std::string hostname;
    std::string cable_address;
    std::string wifi_interface;
    std::optional<std::string> wifi_address;
    bool wifi_access = false;
    AB_JSON_MESSAGE(NetworkMessage, "auto_battlebot.status.Network", hostname, cable_address,
                    wifi_interface, wifi_address, wifi_access)
};

struct BatteryMessage {
    /** False until the first good read, and again after `invalid_read_grace` failed reads. */
    bool valid = false;
    /** Estimated state of charge, 0 to 100. */
    double percent = 0.0;
    std::optional<double> bus_voltage_v;
    std::optional<double> current_a;
    /** Current is flowing into the battery. */
    bool charging = false;
    AB_JSON_MESSAGE(BatteryMessage, "auto_battlebot.status.Battery", valid, percent, bus_voltage_v,
                    current_a, charging)
};

struct CameraSettingsMessage {
    bool supported = false;
    /** DEFAULT first, then the saved profiles. */
    std::vector<std::string> profiles;
    std::string active_profile;
    /** Live values were changed since the active profile was applied or saved. */
    bool modified = false;
    CameraSettings settings;
    CameraSettingLimits limits;
    AB_JSON_MESSAGE(CameraSettingsMessage, "auto_battlebot.status.CameraSettings", supported,
                    profiles, active_profile, modified, settings, limits)
};

struct ReinitFieldCommand {
    AB_JSON_EMPTY_MESSAGE(ReinitFieldCommand, "auto_battlebot.command.ReinitField")
};

struct SetOpponentCountCommand {
    int count = 1;
    AB_JSON_MESSAGE(SetOpponentCountCommand, "auto_battlebot.command.SetOpponentCount", count)
};

struct SetAutonomyCommand {
    bool enabled = false;
    AB_JSON_MESSAGE(SetAutonomyCommand, "auto_battlebot.command.SetAutonomy", enabled)
};

struct SetBehaviorModeCommand {
    BehaviorMode mode = BehaviorMode::ATTACK;
    AB_JSON_MESSAGE(SetBehaviorModeCommand, "auto_battlebot.command.SetBehaviorMode", mode)
};

/** On hands targeting back to the target selector. Off holds the target at the field center
 *  until a SetManualTarget moves it. */
struct SetAutoTrackingCommand {
    bool enabled = true;
    AB_JSON_MESSAGE(SetAutoTrackingCommand, "auto_battlebot.command.SetAutoTracking", enabled)
};

/** Moves the held target while auto tracking is off; refused while it is on, so a stray touch on
 *  a view never takes targeting away from the selector. Exactly one of `field` (a tap on the
 *  top-down view) or `image` (a tap on the camera view, projected onto the floor) is set. */
struct SetManualTargetCommand {
    std::optional<FieldPoint> field;
    std::optional<ImagePoint> image;
    AB_JSON_MESSAGE(SetManualTargetCommand, "auto_battlebot.command.SetManualTarget", field, image)
};

struct SetRecordingCommand {
    bool enabled = false;
    AB_JSON_MESSAGE(SetRecordingCommand, "auto_battlebot.command.SetRecording", enabled)
};

struct SelectProfileCommand {
    std::string name;
    AB_JSON_MESSAGE(SelectProfileCommand, "auto_battlebot.command.SelectProfile", name)
};

struct SystemActionCommand {
    SystemAction action = SystemAction::REBOOT_HOST;
    AB_JSON_MESSAGE(SystemActionCommand, "auto_battlebot.command.SystemAction", action)
};

struct SetWifiAccessCommand {
    bool enabled = false;
    AB_JSON_MESSAGE(SetWifiAccessCommand, "auto_battlebot.command.SetWifiAccess", enabled)
};

struct SetCameraSettingsCommand {
    CameraSettings settings;
    AB_JSON_MESSAGE(SetCameraSettingsCommand, "auto_battlebot.command.SetCameraSettings", settings)
};

/** APPLY a saved profile or DEFAULT, SAVE the live values under `name`, or DELETE `name`. */
struct CameraProfileCommand {
    CameraProfileAction action = CameraProfileAction::APPLY;
    std::string name;
    AB_JSON_MESSAGE(CameraProfileCommand, "auto_battlebot.command.CameraProfile", action, name)
};

}  // namespace auto_battlebot::remote
