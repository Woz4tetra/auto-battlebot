#pragma once

#include <memory>

namespace auto_battlebot {

class McapRecorder;
class VizSink;

// Initialize spdlog with a stdout color sink and an MCAP sink writing foxglove.Log on /log.
// Must be called before any spdlog::info/warn/error calls.
void setup_logging(std::shared_ptr<McapRecorder> recorder);

// Also stream /log to the viz relay so spdlog messages show up live in Foxglove's Log panel.
// Call once the sink exists; earlier messages are only recorded.
void attach_log_viz_sink(std::shared_ptr<VizSink> sink);

}  // namespace auto_battlebot
