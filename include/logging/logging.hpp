#pragma once

#include <memory>

namespace auto_battlebot {

class McapRecorder;
class VizSink;

void setup_logging(std::shared_ptr<McapRecorder> recorder);

void shutdown_logging();

void attach_log_viz_sink(std::shared_ptr<VizSink> sink);

}  // namespace auto_battlebot
