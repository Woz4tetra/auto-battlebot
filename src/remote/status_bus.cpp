#include "remote/status_bus.hpp"

namespace auto_battlebot::remote {

StatusBus::StatusBus(std::shared_ptr<VizSink> sink, std::shared_ptr<McapRecorder> recorder) {
    std::size_t index = 0;
    for_each_status_topic([&](const auto& row) {
        using Message = typename std::remove_cvref_t<decltype(row)>::message_type;
        slots_[index++].channel = std::make_unique<OutputChannel>(
            std::string(row.topic), "json",
            VizSchema::jsonschema(std::string(Message::kSchemaName), schema_for<Message>()),
            row.latch == Latch::YES, sink, recorder);
    });
}

CommandLog::CommandLog(std::shared_ptr<McapRecorder> recorder) {
    if (!recorder) return;
    std::size_t index = 0;
    for_each_command_topic([&](const auto& row) {
        using Payload = typename std::remove_cvref_t<decltype(row)>::payload_type;
        channels_[index++] = std::make_unique<OutputChannel>(
            std::string(row.topic), "json",
            VizSchema::jsonschema(std::string(Payload::kSchemaName), schema_for<Payload>()), false,
            nullptr, recorder);
    });
}

void CommandLog::record(const RemoteCommand& command) {
    auto& channel = channels_[command.index()];
    if (!channel) return;
    std::visit([&](const auto& payload) { channel->log(to_json_string(payload), wall_time_ns()); },
               command);
}

}  // namespace auto_battlebot::remote
