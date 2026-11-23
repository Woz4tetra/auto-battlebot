#include <iostream>
#include <magic_enum.hpp>
#include <nlohmann/json.hpp>
#include <string>

#include "publisher/viz_topics.hpp"

int main() {
    using namespace auto_battlebot;
    nlohmann::json topics = nlohmann::json::array();
    for (const VizTopic &row : kVizTopics) {
        topics.push_back({{"id", std::string(magic_enum::enum_name(row.id))},
                          {"topic", std::string(row.topic)},
                          {"encoding", std::string(row.encoding)},
                          {"latch", row.latch},
                          {"rate", std::string(magic_enum::enum_name(row.rate))},
                          {"recorded", row.recorded}});
    }
    nlohmann::json out = {{"topics", topics},
                          {"diagnostics_prefix", std::string(kDiagnosticsTopicPrefix)}};
    std::cout << out.dump(2) << "\n";
    return 0;
}
