#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "enums/system_action.hpp"
#include "remote/json_message.hpp"

// Payloads of every remote status topic and command. Topics map to these in
// include/remote/protocol.hpp; field names are the JSON keys and the TypeScript field names in
// web/src/generated/protocol.ts.
namespace auto_battlebot::remote {

// ---- Status: app -> clients ----

struct SystemStatusMessage {
    bool camera_ok = false;
    bool transmitter_connected = false;
    bool transmitter_receiving = false;
    double loop_rate_hz = 0.0;
    bool initialized = false;
    int selected_opponent_count = 1;
    bool autonomy_enabled = true;
    bool svo_recording = false;
    bool mcap_recording = false;
    std::optional<double> jetson_temperature_c;
    std::string compute_mode;
    AB_JSON_MESSAGE(SystemStatusMessage, "auto_battlebot.status.System", camera_ok,
                    transmitter_connected, transmitter_receiving, loop_rate_hz, initialized,
                    selected_opponent_count, autonomy_enabled, svo_recording, mcap_recording,
                    jetson_temperature_c, compute_mode)
};

struct AppInfoMessage {
    std::vector<std::string> available_profiles;
    std::string current_profile;
    double max_loop_rate_hz = 0.0;
    double rate_fail_threshold = 0.0;
    double rate_fail_duration_sec = 0.0;
    AB_JSON_MESSAGE(AppInfoMessage, "auto_battlebot.status.App", available_profiles,
                    current_profile, max_loop_rate_hz, rate_fail_threshold, rate_fail_duration_sec)
};

/** Our robot's sticks as the transmitter read them back, normalized [-1, 1]. */
struct SticksMessage {
    double linear = 0.0;
    double angular = 0.0;
    AB_JSON_MESSAGE(SticksMessage, "auto_battlebot.status.Sticks", linear, angular)
};

/** One tracked robot in the field frame (meters, radians). */
struct TrackedRobot {
    std::string label;
    bool ours = false;
    bool stale = false;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    AB_JSON_MESSAGE(TrackedRobot, "auto_battlebot.status.TrackedRobot", label, ours, stale, x, y,
                    yaw)
};

struct TracksMessage {
    bool our_robot_seen = false;
    int opponents_seen = 0;
    /** Field size in meters, so the top-down view can draw the outline. Zero before init. */
    double field_x = 0.0;
    double field_y = 0.0;
    std::vector<TrackedRobot> robots;
    AB_JSON_MESSAGE(TracksMessage, "auto_battlebot.status.Tracks", our_robot_seen, opponents_seen,
                    field_x, field_y, robots)
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
    /** IPv4 link-local address on the cable, empty when no port has one. */
    std::string cable_address;
    std::string wifi_interface;
    std::optional<std::string> wifi_address;
    bool wifi_access = false;
    AB_JSON_MESSAGE(NetworkMessage, "auto_battlebot.status.Network", hostname, cable_address,
                    wifi_interface, wifi_address, wifi_access)
};

// ---- Commands: clients -> app ----

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

/** Turns SVO and MCAP recording on or off together. */
struct SetRecordingCommand {
    bool enabled = false;
    AB_JSON_MESSAGE(SetRecordingCommand, "auto_battlebot.command.SetRecording", enabled)
};

/** Picks the profile for the next launch. */
struct SelectProfileCommand {
    std::string name;
    AB_JSON_MESSAGE(SelectProfileCommand, "auto_battlebot.command.SelectProfile", name)
};

struct SystemActionCommand {
    SystemAction action = SystemAction::REBOOT_HOST;
    AB_JSON_MESSAGE(SystemActionCommand, "auto_battlebot.command.SystemAction", action)
};

/** Opens or closes the dashboard ports on the Wi-Fi interface. */
struct SetWifiAccessCommand {
    bool enabled = false;
    AB_JSON_MESSAGE(SetWifiAccessCommand, "auto_battlebot.command.SetWifiAccess", enabled)
};

}  // namespace auto_battlebot::remote
