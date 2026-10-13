#pragma once

#include <memory>

namespace auto_battlebot {

class McapRecorder;
class VizSink;

// Initialize spdlog with a stdout color sink and an MCAP sink writing foxglove.Log on /log.
// Must be called before any spdlog::info/warn/error calls.
/** Makes the default spdlog logger asynchronous, writing to the console and to /log in the
 *  recorder and live viz. Callers never block on a sink. */
void setup_logging(std::shared_ptr<McapRecorder> recorder);

/** Flushes queued lines and swaps in a synchronous console logger. Call before main returns. */
void shutdown_logging();

// Also stream /log to the viz relay so spdlog messages show up live in Foxglove's Log panel.
// Call once the sink exists; earlier messages are only recorded.
void attach_log_viz_sink(std::shared_ptr<VizSink> sink);

}  // namespace auto_battlebot
