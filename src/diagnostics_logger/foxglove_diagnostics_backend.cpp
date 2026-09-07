#include "diagnostics_logger/foxglove_diagnostics_backend.hpp"

#include <map>
#include <vector>

#include "foxglove_adapters/common.hpp"
#include "foxglove_adapters/json_schemas.hpp"

namespace auto_battlebot {

FoxgloveDiagnosticsBackend::FoxgloveDiagnosticsBackend(std::shared_ptr<VizSink> sink,
                                                       std::shared_ptr<McapRecorder> mcap_recorder)
    : sink_(std::move(sink)), mcap_recorder_(std::move(mcap_recorder)) {}

OutputChannel &FoxgloveDiagnosticsBackend::channel_for(const std::string &module) {
    auto it = channels_.find(module);
    if (it == channels_.end()) {
        it = channels_
                 .emplace(module,
                          std::make_unique<OutputChannel>(
                              "/diagnostics/" + module, "json",
                              VizSchema::jsonschema(foxglove_adapters::kDiagnosticsSchemaName,
                                                    foxglove_adapters::kDiagnosticsSchema),
                              false, sink_, mcap_recorder_))
                 .first;
    }
    return *it->second;
}

void FoxgloveDiagnosticsBackend::receive(const std::vector<DiagnosticStatusSnapshot> &snapshots) {
    if (snapshots.empty()) return;
    // Group by module, preserving the (alphabetical) order the logger emits.
    std::map<std::string, std::vector<const DiagnosticStatusSnapshot *>> by_module;
    for (const auto &snap : snapshots) by_module[snap.name].push_back(&snap);

    const uint64_t log_time = wall_time_ns();
    for (const auto &[module, module_snapshots] : by_module) {
        channel_for(module).log(diagnostics_module_json(module, module_snapshots), log_time);
    }
}

std::string diagnostics_module_json(
    const std::string &module, const std::vector<const DiagnosticStatusSnapshot *> &snapshots) {
    using foxglove_adapters::json_escape;
    using foxglove_adapters::json_number;
    std::string json = "{";
    bool first_section = true;
    for (const auto *snap : snapshots) {
        if (!first_section) json += ',';
        first_section = false;
        const std::string &section = snap->subsection.empty() ? module : snap->subsection;
        json += '"' + json_escape(section) + "\":{\"level\":" + std::to_string(snap->level) +
                ",\"message\":\"" + json_escape(snap->message) + "\",\"values\":{";
        bool first_value = true;
        for (const auto &[key, value] : snap->values) {
            if (!first_value) json += ',';
            first_value = false;
            json += '"' + json_escape(key) + "\":";
            std::visit(
                [&](auto &&arg) {
                    using T = std::decay_t<decltype(arg)>;
                    if constexpr (std::is_same_v<T, std::string>) {
                        json += '"' + json_escape(arg) + '"';
                    } else if constexpr (std::is_same_v<T, double>) {
                        json += json_number(arg);
                    } else {
                        json += std::to_string(arg);
                    }
                },
                value);
        }
        json += "}}";
    }
    json += '}';
    return json;
}

}  // namespace auto_battlebot
