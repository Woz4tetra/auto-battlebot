
#include <iostream>

#include "remote/protocol.hpp"

int main() {
    using namespace auto_battlebot::remote;
    nlohmann::json out = {{"status", nlohmann::json::array()},
                          {"commands", nlohmann::json::array()}};
    for_each_status_topic([&](const auto& row) {
        using Message = typename std::remove_cvref_t<decltype(row)>::message_type;
        out["status"].push_back({{"topic", row.topic},
                                 {"schema_name", Message::kSchemaName},
                                 {"schema", schema_of<Message>()},
                                 {"latch", row.latch == Latch::YES},
                                 {"max_rate_hz", row.max_rate_hz}});
    });
    for_each_command_topic([&](const auto& row) {
        using Payload = typename std::remove_cvref_t<decltype(row)>::payload_type;
        out["commands"].push_back({{"topic", row.topic},
                                   {"schema_name", Payload::kSchemaName},
                                   {"schema", schema_of<Payload>()}});
    });
    std::cout << out.dump(2) << "\n";
    return 0;
}
