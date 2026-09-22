#include "remote_command.hpp"

#include <spdlog/spdlog.h>

#include <magic_enum.hpp>
#include <string>

namespace auto_battlebot {

std::optional<RemoteCommand> parse_remote_command_topic(std::string_view topic) {
    if (!topic.starts_with(kRemoteCommandTopicPrefix)) return std::nullopt;
    const std::string_view name = topic.substr(kRemoteCommandTopicPrefix.size());
    auto command = magic_enum::enum_cast<RemoteCommand>(name, magic_enum::case_insensitive);
    if (!command) {
        std::string valid;
        for (auto value : magic_enum::enum_names<RemoteCommand>()) {
            if (!valid.empty()) valid += ", ";
            valid += std::string(kRemoteCommandTopicPrefix) + std::string(value);
        }
        spdlog::warn("Unknown remote command topic {} (valid, case-insensitive: {})", topic, valid);
    }
    return command;
}

}  // namespace auto_battlebot
