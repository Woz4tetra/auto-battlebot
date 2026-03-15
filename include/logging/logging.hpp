#pragma once

#include <memory>

namespace miniros {
class NodeHandle;
}

namespace auto_battlebot {

class McapRecorder;

// Initialize spdlog with a stdout color sink and an MCAP sink.
// Must be called before any spdlog::info/warn/error calls.
void setup_logging(std::shared_ptr<McapRecorder> recorder);

// Wire up a /rosout publisher so spdlog messages are also visible on the
// live ROS topic (e.g. in Foxglove when connected directly to the node).
// Must be called after miniros::init() and NodeHandle creation.
void setup_rosout_publisher(miniros::NodeHandle& nh);

}  // namespace auto_battlebot
