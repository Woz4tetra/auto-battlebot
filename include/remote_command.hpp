#pragma once

#include <optional>
#include <string_view>

#include "enums/remote_command.hpp"

namespace auto_battlebot {

// A Foxglove client sends a RemoteCommand by publishing on /command/<name>, where <name> is the
// enum value in lowercase: /command/reinit_field. The payload is ignored, so `{}` is enough.
inline constexpr std::string_view kRemoteCommandTopicPrefix = "/command/";

// Returns nullopt for topics outside the prefix. Logs the valid names for an unknown command.
std::optional<RemoteCommand> parse_remote_command_topic(std::string_view topic);

}  // namespace auto_battlebot
