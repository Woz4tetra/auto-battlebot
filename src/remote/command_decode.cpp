#include "remote/command_decode.hpp"

#include <spdlog/spdlog.h>

#include <string>

namespace auto_battlebot::remote {

std::optional<RemoteCommand> decode_command(std::string_view topic, const std::byte* data,
                                            size_t len) {
    std::optional<RemoteCommand> result;
    bool matched = false;
    for_each_command_topic([&](const auto& row) {
        if (matched || row.topic != topic) return;
        matched = true;
        using Payload = typename std::remove_cvref_t<decltype(row)>::payload_type;
        try {
            const auto* text = reinterpret_cast<const char*>(data);
            nlohmann::json j =
                len == 0 ? nlohmann::json::object() : nlohmann::json::parse(text, text + len);
            Payload payload;
            from_json(j, payload);
            result = std::move(payload);
        } catch (const std::exception& e) {
            spdlog::warn("Refused {} ({}): {}", topic, Payload::kSchemaName, e.what());
        }
    });
    if (!matched) {
        std::string valid;
        for_each_command_topic([&](const auto& row) {
            if (!valid.empty()) valid += ", ";
            valid += row.topic;
        });
        spdlog::warn("Unknown command topic {} (valid: {})", topic, valid);
    }
    return result;
}

}  // namespace auto_battlebot::remote
