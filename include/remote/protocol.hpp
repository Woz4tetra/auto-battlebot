#pragma once

#include <array>
#include <cstddef>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>

#include "remote/messages.hpp"

// The one place remote topics map to types. Every status topic and command a browser, a Foxglove
// panel, or the LVGL UI can use is a row here. The rows drive JSON encoding, the JSON Schema each
// channel advertises, MCAP recording, command dispatch, and web/src/generated/protocol.ts.
//
// Adding a command: a payload struct in messages.hpp, one CommandTopic row, then a handler in
// Runner::handle_commands (the build fails until it exists). Adding a status topic: a message
// struct, one StatusTopic row, and a status_bus->publish(...) call at the producer.
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

// /status/app repeats every 5 s so a relay that connects late still gets it. /status/network is
// paced by HostServices itself (after each change and every 5 s), so the bus does not throttle it.
struct RemoteTopics {
    static constexpr auto status = std::tuple{
        StatusTopic<SystemStatusMessage>{"/status/system", Latch::NO, 10.0},
        StatusTopic<AppInfoMessage>{"/status/app", Latch::YES, 0.2},
        StatusTopic<SticksMessage>{"/status/sticks", Latch::NO, 20.0},
        StatusTopic<TracksMessage>{"/status/tracks", Latch::NO, 20.0},
        StatusTopic<CommandAckMessage>{"/status/command_ack", Latch::NO, 0.0},
        StatusTopic<NetworkMessage>{"/status/network", Latch::YES, 0.0},
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

namespace detail {

template <typename Tuple>
struct command_variant;
template <typename... Payloads>
struct command_variant<std::tuple<CommandTopic<Payloads>...>> {
    using type = std::variant<Payloads...>;
};

template <typename Tuple>
struct row_types;
template <template <typename> typename Row, typename... Ts>
struct row_types<std::tuple<Row<Ts>...>> {
    template <typename T>
    static constexpr std::size_t count = (std::size_t{std::is_same_v<T, Ts>} + ... + 0);
    static constexpr bool all_unique = ((count<Ts> == 1) && ...);
};

template <typename Tuple>
constexpr auto topic_names(const Tuple& rows) {
    return std::apply(
        [](const auto&... row) {
            return std::array<std::string_view, sizeof...(row)>{row.topic...};
        },
        rows);
}

constexpr bool all_prefixed(const auto& names, std::string_view prefix) {
    for (auto name : names) {
        if (!name.starts_with(prefix) || name.size() == prefix.size()) return false;
    }
    return true;
}

constexpr bool topics_unique() {
    auto status = topic_names(RemoteTopics::status);
    auto commands = topic_names(RemoteTopics::commands);
    std::array<std::string_view, status.size() + commands.size()> all{};
    std::size_t n = 0;
    for (auto name : status) all[n++] = name;
    for (auto name : commands) all[n++] = name;
    for (std::size_t i = 0; i < all.size(); ++i) {
        for (std::size_t j = i + 1; j < all.size(); ++j) {
            if (all[i] == all[j]) return false;
        }
    }
    return true;
}

}  // namespace detail

using StatusRows = std::remove_cvref_t<decltype(RemoteTopics::status)>;
using CommandRows = std::remove_cvref_t<decltype(RemoteTopics::commands)>;

/** One alternative per command payload, derived from RemoteTopics::commands. */
using RemoteCommand = typename detail::command_variant<CommandRows>::type;

static_assert(detail::topics_unique(), "every remote topic string must be unique");
static_assert(detail::row_types<StatusRows>::all_unique,
              "a status message type may map to one topic only");
static_assert(detail::row_types<CommandRows>::all_unique,
              "a command payload type may map to one topic only");
static_assert(detail::all_prefixed(detail::topic_names(RemoteTopics::status), "/status/"),
              "status topics start with /status/");
static_assert(detail::all_prefixed(detail::topic_names(RemoteTopics::commands), "/command/"),
              "command topics start with /command/");

/** The status row for a message type. */
template <typename Message>
constexpr const StatusTopic<Message>& status_topic() {
    return std::get<StatusTopic<Message>>(RemoteTopics::status);
}

/** The command row for a payload type. */
template <typename Payload>
constexpr const CommandTopic<Payload>& command_topic() {
    return std::get<CommandTopic<Payload>>(RemoteTopics::commands);
}

/** Topic of whichever command the variant holds. */
inline std::string_view command_topic_of(const RemoteCommand& command) {
    return std::visit(
        [](const auto& payload) {
            return command_topic<std::remove_cvref_t<decltype(payload)>>().topic;
        },
        command);
}

template <typename F>
constexpr void for_each_status_topic(F&& f) {
    std::apply([&](const auto&... row) { (f(row), ...); }, RemoteTopics::status);
}

template <typename F>
constexpr void for_each_command_topic(F&& f) {
    std::apply([&](const auto&... row) { (f(row), ...); }, RemoteTopics::commands);
}

/** std::visit helper: one lambda per alternative. */
template <typename... Fs>
struct overloaded : Fs... {
    using Fs::operator()...;
};

}  // namespace auto_battlebot::remote
