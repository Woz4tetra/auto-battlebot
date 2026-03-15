#pragma once

#include <memory>

namespace auto_battlebot {

class McapRecorder;

// Initialize spdlog with a stdout color sink and an MCAP sink.
// Must be called before any spdlog::info/warn/error calls.
void setup_logging(std::shared_ptr<McapRecorder> recorder);

}  // namespace auto_battlebot
